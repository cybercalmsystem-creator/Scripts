# Hair Strands — рисование прядей волос (hair cards) по поверхности модели.
# Порт скрипта 3ds Max HairStrands.ms для Blender 4.2+ / 5.x.
#
# Установка: Edit > Preferences > Add-ons > (меню ˅) Install from Disk... > hair_strands_blender.py
# Панель:    3D View > N-панель > вкладка "Hair Strands"
#
# Работа:
#   1. "Модель" — указать голову/тело (mesh).
#   2. "Рисовать пряди" — зажать ЛКМ и вести по модели; отпустить — прядь создана.
#      ПКМ / Esc — выход. Колесо и СКМ — навигация во время рисования.
#   3. Либо нарисовать кривые (Curve) вручную, выделить и нажать "Из выделенных кривых".
#   4. Параметры меняют пряди на лету: выделенные или все ("Применять к").
#   5. Живое обновление: правка точек кривой-направляющей (в Edit Mode тоже) или её
#      перемещение сразу перестраивает прядь.
#   6. Текстуры: Albedo (прозрачность из альфа-канала) и Normal (tangent space).
#      Общий материал "HairStrands_Mat". У каждой пряди своя ровная UV-развёртка 0–1.
#
# Единицы: метры. Значения по умолчанию рассчитаны на голову ~20–25 см.

bl_info = {
    "name": "Hair Strands (hair cards)",
    "author": "Crocos",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "3D View > N-панель > Hair Strands",
    "description": "Рисование прядей волос (hair cards) по поверхности модели",
    "category": "Object",
}

import json
import random

import bpy
import bmesh
import gpu
from bpy.app.handlers import persistent
from bpy.props import (BoolProperty, EnumProperty, FloatProperty, IntProperty,
                       PointerProperty, StringProperty)
from bpy_extras import view3d_utils
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import interpolate_bezier

COL_STRANDS = "HairStrands"
COL_GUIDES = "HairGuides"
MAT_NAME = "HairStrands_Mat"
UV_NAME = "UVMap"

# параметры, которые хранятся в каждой пряди (obj["hc_params"], JSON)
PARAM_KEYS = ("width", "segs", "wsegs", "root", "tip", "bulge", "offset", "thick",
              "layers", "lstep", "rand", "smooth", "stick", "flip_v")
DEFAULTS = {"width": 0.012, "segs": 12, "wsegs": 3, "root": 60.0, "tip": 25.0,
            "bulge": 0.25, "offset": 0.001, "thick": 0.0, "layers": 1, "lstep": 0.0015,
            "rand": 15.0, "smooth": 3, "stick": 0.01, "flip_v": 0, "seed": 1}


class _State:
    busy = False        # идёт перестройка — обработчик depsgraph молчит
    loading = False     # UI заполняется из пряди — не применять параметры
    drawing = False
    stop = False
    bvh = None
    bvh_head = None
    bvh_dirty = True


S = _State()


# ---------------------------------------------------------------- кэш поверхности
def build_cache(head, depsgraph=None):
    S.bvh = None
    S.bvh_head = None
    S.bvh_dirty = False
    if head is None or head.type != 'MESH':
        return
    dg = depsgraph or bpy.context.evaluated_depsgraph_get()
    ev = head.evaluated_get(dg)
    me = ev.to_mesh()
    try:
        mw = ev.matrix_world
        verts = [mw @ v.co for v in me.vertices]
        polys = [tuple(p.vertices) for p in me.polygons]
        S.bvh = BVHTree.FromPolygons(verts, polys) if polys else None
    finally:
        ev.to_mesh_clear()
    S.bvh_head = head.name


def get_bvh(head, depsgraph=None):
    if head is None:
        return None
    if S.bvh is None or S.bvh_dirty or S.bvh_head != head.name:
        build_cache(head, depsgraph)
    return S.bvh


def surface_info(head, p, depsgraph=None):
    """Ближайшая точка поверхности и её нормаль: (pos, normal) или None."""
    bvh = get_bvh(head, depsgraph)
    if bvh is None:
        return None
    loc, nrm, _idx, _d = bvh.find_nearest(p)
    if loc is None:
        return None
    return loc, nrm


def bbox_center(ob):
    c = Vector()
    for v in ob.bound_box:
        c += ob.matrix_world @ Vector(v)
    return c / 8.0


# ---------------------------------------------------------------- параметры
def params_from_props(hs):
    p = {k: getattr(hs, k) for k in PARAM_KEYS}
    p["flip_v"] = int(p["flip_v"])
    return p


def parse_params(s):
    p = dict(DEFAULTS)
    if s:
        try:
            p.update(json.loads(s))
        except (ValueError, TypeError):
            pass
    return p


def new_seed():
    return random.randint(1, 999999)


# ---------------------------------------------------------------- геометрия
def resample_poly(pts, count):
    """count+1 точек, равномерно по длине ломаной."""
    lens = [0.0]
    for i in range(1, len(pts)):
        lens.append(lens[-1] + (pts[i] - pts[i - 1]).length)
    total = lens[-1]
    if total < 1e-9:
        return [pts[0].copy(), pts[-1].copy()]
    res = []
    k = 0
    for i in range(count + 1):
        d = total * i / count
        while k < len(pts) - 2 and lens[k + 1] < d:
            k += 1
        seg = lens[k + 1] - lens[k]
        f = (d - lens[k]) / seg if seg > 1e-12 else 0.0
        res.append(pts[k].lerp(pts[k + 1], f))
    return res


def spline_points_world(guide, si, res=24):
    sp = guide.data.splines[si]
    mw = guide.matrix_world
    pts = []
    if sp.type == 'BEZIER':
        bp = sp.bezier_points
        n = len(bp)
        if n == 1:
            pts = [bp[0].co.copy()]
        segs = n if sp.use_cyclic_u else n - 1
        for i in range(segs):
            a, b = bp[i], bp[(i + 1) % n]
            seg = interpolate_bezier(a.co, a.handle_right, b.handle_left, b.co, res + 1)
            pts.extend(seg if i == 0 else seg[1:])
    else:
        pts = [Vector(p.co[:3]) for p in sp.points]
    return [mw @ Vector(p) for p in pts]


def sample_path(guide, si, p):
    count = max(2, int(p["segs"]))
    raw = spline_points_world(guide, si)
    if len(raw) < 2:
        return None
    pts = resample_poly(raw, count)
    for _ in range(int(p["smooth"])):
        np_ = [v.copy() for v in pts]
        for i in range(1, len(pts) - 1):
            np_[i] = (pts[i - 1] + pts[i] * 2 + pts[i + 1]) / 4.0
        pts = np_
    return pts


def ease(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def _perp(nn, t):
    nn = nn - t * nn.dot(t)
    if nn.length < 1e-5:
        nn = t.cross(Vector((0, 0, 1)))
    if nn.length < 1e-5:
        nn = t.cross(Vector((1, 0, 0)))
    return nn.normalized()


def make_strand(pts, p, head, depsgraph=None):
    """Вершины и грани (квады) пряди. Грань: (индексы, uv по углам, группа сглаживания)."""
    n = len(pts)
    W = max(1, int(p["wsegs"]))
    flip_v = bool(p["flip_v"])
    if head is not None:
        center = bbox_center(head)
    else:
        center = sum(pts, Vector()) / n

    # локальные оси вдоль пряди
    tans, nrms = [], []
    prev_n = None
    for i in range(n):
        a, b = max(i - 1, 0), min(i + 1, n - 1)
        t = pts[b] - pts[a]
        t = t.normalized() if t.length > 1e-6 else Vector((0, 0, 1))
        nn = None
        si = surface_info(head, pts[i], depsgraph)
        if si is not None and (prev_n is None or (si[0] - pts[i]).length <= p["stick"]):
            nn = si[1].copy()
        if nn is None:
            nn = prev_n.copy() if prev_n is not None else (pts[i] - center)
        nn = _perp(nn, t)
        tans.append(t)
        nrms.append(nn)
        prev_n = nn
    for _ in range(2):
        n2 = [v.copy() for v in nrms]
        for i in range(1, n - 1):
            n2[i] = (nrms[i - 1] + nrms[i] * 2 + nrms[i + 1]).normalized()
        nrms = n2
    bins = []
    for i in range(n):
        nrms[i] = _perp(nrms[i], tans[i])
        bb = tans[i].cross(nrms[i])
        bins.append(bins[i - 1] if (i > 0 and bb.length < 1e-5) else bb.normalized())

    # UV: V — по реальной длине вдоль пряди, U — по реальной ширине ряда
    arc = [0.0]
    for i in range(1, n):
        arc.append(arc[-1] + (pts[i] - pts[i - 1]).length)
    arc_t = arc[-1]
    vs = []
    for i in range(n):
        v = arc[i] / arc_t if arc_t > 1e-9 else i / (n - 1)
        vs.append(1.0 - v if flip_v else v)
    vdir = -1.0 if flip_v else 1.0

    root_s, tip_s = p["root"] / 100.0, p["tip"] / 100.0
    thick = p["thick"]
    layers = max(1, int(p["layers"]))
    row = W + 1
    rng = random.Random(int(p.get("seed", 1)))

    verts, vuv, faces = [], [], []
    for layer in range(layers):
        lw = p["width"] * (1.0 + (p["rand"] / 100.0) * rng.uniform(-1.0, 1.0))
        shift = 0.0 if layer == 0 else rng.uniform(-0.3, 0.3) * p["width"]
        lift = p["offset"] + layer * p["lstep"]
        base = len(verts)

        def vid(i, j, base=base):
            return base + i * row + j

        for i in range(n):
            t = i / (n - 1)
            wd = lw * (root_s + (1 - root_s) * ease(t / 0.25)) * (tip_s + (1 - tip_s) * ease((1 - t) / 0.4))
            rowp = []
            for j in range(row):
                u = j / W - 0.5
                arch = p["bulge"] * wd * (1 - 4 * u * u)
                rowp.append(pts[i] + bins[i] * (u * wd + shift) + nrms[i] * (lift + arch))
            acc = [0.0]
            for j in range(1, row):
                acc.append(acc[-1] + (rowp[j] - rowp[j - 1]).length)
            tot = acc[-1]
            for j in range(row):
                verts.append(rowp[j])
                vuv.append((acc[j] / tot if tot > 1e-9 else j / W, vs[i]))

        # верх
        for i in range(n - 1):
            for j in range(W):
                q = (vid(i, j), vid(i, j + 1), vid(i + 1, j + 1), vid(i + 1, j))
                faces.append((q, [vuv[k] for k in q], 1))

        if thick > 0:
            bbase = len(verts)
            for k in range(n * row):
                verts.append(verts[base + k] - nrms[k // row] * thick)
                vuv.append(vuv[base + k])           # низ: та же развёртка, что и верх
            d = bbase - base
            # низ
            for i in range(n - 1):
                for j in range(W):
                    q = (vid(i, j) + d, vid(i + 1, j) + d, vid(i + 1, j + 1) + d, vid(i, j + 1) + d)
                    faces.append((q, [vuv[k] for k in q], 2))
            # боковины и торцы — свои узкие полоски по краям 0–1
            su = min(max(thick / max(lw, 1e-9), 0.005), 0.25)
            sv = min(max(thick / arc_t if arc_t > 1e-9 else 0.01, 0.005), 0.25)
            for i in range(n - 1):
                t1, t2 = vid(i, 0), vid(i + 1, 0)
                faces.append(((t1, t2, t2 + d, t1 + d),
                              [(0, vs[i]), (0, vs[i + 1]), (su, vs[i + 1]), (su, vs[i])], 4))
                r1, r2 = vid(i, W), vid(i + 1, W)
                faces.append(((r1, r1 + d, r2 + d, r2),
                              [(1, vs[i]), (1 - su, vs[i]), (1 - su, vs[i + 1]), (1, vs[i + 1])], 4))
            for j in range(W):
                s1, s2 = vid(0, j), vid(0, j + 1)
                u0, u0b = vuv[s1][0], vuv[s2][0]
                v0, v0i = vs[0], vs[0] + vdir * sv
                faces.append(((s1, s1 + d, s2 + d, s2), [(u0, v0), (u0, v0i), (u0b, v0i), (u0b, v0)], 8))
                e1, e2 = vid(n - 1, j), vid(n - 1, j + 1)
                u1, u1b = vuv[e1][0], vuv[e2][0]
                v1, v1i = vs[-1], vs[-1] - vdir * sv
                faces.append(((e1, e2, e2 + d, e1 + d), [(u1, v1), (u1b, v1), (u1b, v1i), (u1, v1i)], 8))
    return verts, faces


def write_mesh(me, verts, faces):
    bm = bmesh.new()
    bv = [bm.verts.new(v) for v in verts]
    uvl = bm.loops.layers.uv.new(UV_NAME)
    group = {}
    for idx, uv, g in faces:
        try:
            f = bm.faces.new([bv[i] for i in idx])
        except ValueError:
            continue
        f.smooth = True
        for loop, t in zip(f.loops, uv):
            loop[uvl].uv = t
        group[f] = g
    # жёсткие рёбра между верхом / низом / боковинами / торцами
    for e in bm.edges:
        lf = e.link_faces
        if len(lf) == 2 and group.get(lf[0]) != group.get(lf[1]):
            e.smooth = False
    bm.to_mesh(me)
    bm.free()
    me.update()


# ---------------------------------------------------------------- объекты сцены
def get_collection(scene, name):
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
    if col not in scene.collection.children_recursive:
        scene.collection.children.link(col)
    return col


def is_strand(o):
    return o is not None and o.type == 'MESH' and "hc_guide" in o


def guide_of(o):
    g = o.get("hc_guide")
    if isinstance(g, bpy.types.Object) and g.type == 'CURVE' and len(g.users_scene) > 0:
        return g
    return None


def head_for(o, scene):
    hs = scene.hair_strands
    if hs.head is not None:
        return hs.head
    h = o.get("hc_head")
    return h if isinstance(h, bpy.types.Object) and h.type == 'MESH' else None


def rebuild_strand(ob, scene, depsgraph=None):
    g = guide_of(ob)
    if g is None:
        return False
    si = int(ob.get("hc_spline", 0))
    if si < 0 or si >= len(g.data.splines):
        return False
    p = parse_params(ob.get("hc_params"))
    pts = sample_path(g, si, p)
    if pts is None:
        return False
    verts, faces = make_strand(pts, p, head_for(ob, scene), depsgraph)
    write_mesh(ob.data, verts, faces)
    ob.matrix_world = Matrix.Identity(4)
    return True


def create_strand(context, guide, si, p):
    scene = context.scene
    hs = scene.hair_strands
    me = bpy.data.meshes.new("HairStrand")
    ob = bpy.data.objects.new("HairStrand", me)
    ob.color = (0.6, 0.6, 0.6, 1.0)
    get_collection(scene, COL_STRANDS).objects.link(ob)
    ob["hc_guide"] = guide
    ob["hc_spline"] = si
    ob["hc_params"] = json.dumps(p)
    if hs.head is not None:
        ob["hc_head"] = hs.head
    mat = bpy.data.materials.get(MAT_NAME)
    if mat is not None:
        me.materials.append(mat)
    if not rebuild_strand(ob, scene):
        bpy.data.objects.remove(ob)
        bpy.data.meshes.remove(me)
        return None
    return ob


def strands_from_selection(context):
    sel = context.selected_objects
    guides = {o.name for o in sel if o.type == 'CURVE'}
    res = [o for o in sel if is_strand(o)]
    names = {o.name for o in res}
    if guides:
        for o in context.scene.objects:
            if is_strand(o) and o.name not in names:
                g = guide_of(o)
                if g is not None and g.name in guides:
                    res.append(o)
    return res


def collect_strands(context, all_=False):
    if all_:
        return [o for o in context.scene.objects if is_strand(o)]
    return strands_from_selection(context)


def update_strands(context, all_=False, apply_params=True, reseed=False):
    hs = context.scene.hair_strands
    strands = collect_strands(context, all_)
    cur = params_from_props(hs)
    S.busy = True
    try:
        for s in strands:
            if apply_params or reseed:
                old = parse_params(s.get("hc_params"))
                p = dict(cur) if apply_params else old
                p["seed"] = new_seed() if reseed else old.get("seed", 1)
                s["hc_params"] = json.dumps(p)
            rebuild_strand(s, context.scene)
    finally:
        S.busy = False
    return len(strands)


def create_guide(context, kp):
    cu = bpy.data.curves.new("HairGuide", 'CURVE')
    cu.dimensions = '3D'
    sp = cu.splines.new('BEZIER')
    sp.bezier_points.add(len(kp) - 1)
    n = len(kp)
    for i, (bp, q) in enumerate(zip(sp.bezier_points, kp)):
        a = kp[max(i - 1, 0)]
        b = kp[min(i + 1, n - 1)]
        tng = (b - a) * (0.5 if 0 < i < n - 1 else 1.0)
        bp.co = q
        bp.handle_left = q - tng / 3.0
        bp.handle_right = q + tng / 3.0
        bp.handle_left_type = 'AUTO'
        bp.handle_right_type = 'AUTO'
    ob = bpy.data.objects.new("HairGuide", cu)
    ob.color = (1.0, 0.55, 0.0, 1.0)
    get_collection(context.scene, COL_GUIDES).objects.link(ob)
    return ob


# ---------------------------------------------------------------- материал
def _abs(path):
    return bpy.path.abspath(path) if path else ""


def _load_image(path, non_color=False):
    import os
    fp = _abs(path)
    if not fp or not os.path.isfile(fp):
        return None
    img = bpy.data.images.load(fp, check_existing=True)
    if non_color:
        try:
            img.colorspace_settings.name = 'Non-Color'
        except TypeError:
            pass
    return img


def build_material(hs):
    mat = bpy.data.materials.get(MAT_NAME) or bpy.data.materials.new(MAT_NAME)
    try:
        mat.use_nodes = True
    except AttributeError:
        pass
    nt = mat.node_tree
    nodes, links = nt.nodes, nt.links
    nodes.clear()
    out = nodes.new('ShaderNodeOutputMaterial')
    out.location = (400, 0)
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.location = (100, 0)
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])

    img = _load_image(hs.albedo)
    if img is not None:
        tex = nodes.new('ShaderNodeTexImage')
        tex.name = tex.label = "Albedo"
        tex.location = (-400, 200)
        tex.image = img
        img.alpha_mode = 'STRAIGHT'
        links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
        if hs.use_alpha:
            links.new(tex.outputs['Alpha'], bsdf.inputs['Alpha'])

    nimg = _load_image(hs.normal, non_color=True)
    if nimg is not None:
        ntex = nodes.new('ShaderNodeTexImage')
        ntex.name = ntex.label = "Normal"
        ntex.location = (-700, -250)
        ntex.image = nimg
        nm = nodes.new('ShaderNodeNormalMap')
        nm.name = "HC_NormalMap"
        nm.location = (-150, -250)
        nm.space = 'TANGENT'
        nm.uv_map = UV_NAME
        nm.inputs['Strength'].default_value = hs.normal_strength
        if hs.flip_g:   # DirectX → OpenGL: G = 1 - G
            sep = nodes.new('ShaderNodeSeparateColor')
            sep.location = (-450, -250)
            inv = nodes.new('ShaderNodeMath')
            inv.operation = 'SUBTRACT'
            inv.inputs[0].default_value = 1.0
            inv.location = (-300, -300)
            comb = nodes.new('ShaderNodeCombineColor')
            comb.location = (-300, -150)
            links.new(ntex.outputs['Color'], sep.inputs[0])
            links.new(sep.outputs[0], comb.inputs[0])
            links.new(sep.outputs[1], inv.inputs[1])
            links.new(inv.outputs[0], comb.inputs[1])
            links.new(sep.outputs[2], comb.inputs[2])
            links.new(comb.outputs[0], nm.inputs['Color'])
        else:
            links.new(ntex.outputs['Color'], nm.inputs['Color'])
        links.new(nm.outputs['Normal'], bsdf.inputs['Normal'])

    # прозрачность в EEVEE
    if hasattr(mat, "surface_render_method"):
        mat.surface_render_method = 'DITHERED'
    elif hasattr(mat, "blend_method"):
        mat.blend_method = 'HASHED'
    mat.use_backface_culling = False
    return mat


def assign_material(context, mat):
    cnt = 0
    for o in context.scene.objects:
        if not is_strand(o):
            continue
        mats = o.data.materials
        if len(mats) == 0:
            mats.append(mat)
            cnt += 1
        elif mats[0] is None or mats[0] == mat or mats[0].name.startswith(MAT_NAME):
            mats[0] = mat
            cnt += 1
    return cnt


# ---------------------------------------------------------------- свойства (UI)
def _upd_param(self, context):
    if S.loading or not self.auto_apply:
        return
    update_strands(context, all_=(self.scope == 'ALL'), apply_params=True)


def _upd_head(self, context):
    S.bvh_dirty = True


def _upd_tex(self, context):
    if S.loading:
        return
    mat = build_material(self)
    assign_material(context, mat)


def _upd_nstr(self, context):
    mat = bpy.data.materials.get(MAT_NAME)
    if mat is not None and mat.node_tree is not None:
        nm = mat.node_tree.nodes.get("HC_NormalMap")
        if nm is not None:
            nm.inputs['Strength'].default_value = self.normal_strength


def _upd_guides(self, context):
    col = bpy.data.collections.get(COL_GUIDES)
    if col is not None:
        col.hide_viewport = not self.show_guides


def _head_poll(self, ob):
    return ob.type == 'MESH' and not is_strand(ob)


class HairStrandsSettings(bpy.types.PropertyGroup):
    head: PointerProperty(name="Модель", type=bpy.types.Object, poll=_head_poll, update=_upd_head)

    width: FloatProperty(name="Ширина", default=DEFAULTS["width"], min=0.00001, soft_max=1.0,
                         subtype='DISTANCE', unit='LENGTH', precision=4, update=_upd_param)
    root: FloatProperty(name="Корень, %", default=60.0, min=1, max=200, update=_upd_param)
    tip: FloatProperty(name="Кончик, %", default=25.0, min=1, max=200, update=_upd_param)
    bulge: FloatProperty(name="Выпуклость", default=0.25, min=-2, max=2, step=1, update=_upd_param)
    offset: FloatProperty(name="Отступ от модели", default=DEFAULTS["offset"], soft_min=-0.1, soft_max=0.1,
                          subtype='DISTANCE', unit='LENGTH', precision=4, update=_upd_param)
    thick: FloatProperty(name="Толщина", default=0.0, min=0.0, soft_max=0.05,
                         subtype='DISTANCE', unit='LENGTH', precision=4, update=_upd_param)
    segs: IntProperty(name="Сегм. по длине", default=12, min=2, max=200, update=_upd_param)
    wsegs: IntProperty(name="Сегм. по ширине", default=3, min=1, max=32, update=_upd_param)
    smooth: IntProperty(name="Сглаживание", default=3, min=0, max=50, update=_upd_param)

    layers: IntProperty(name="Слоёв", default=1, min=1, max=12, update=_upd_param)
    lstep: FloatProperty(name="Шаг слоёв", default=DEFAULTS["lstep"], min=0.0, soft_max=0.05,
                         subtype='DISTANCE', unit='LENGTH', precision=4, update=_upd_param)
    rand: FloatProperty(name="Разброс ширины, %", default=15.0, min=0, max=90, update=_upd_param)
    stick: FloatProperty(name="Прилипание", default=DEFAULTS["stick"], min=0.0, soft_max=1.0,
                         subtype='DISTANCE', unit='LENGTH', precision=4, update=_upd_param)
    flip_v: BoolProperty(name="Перевернуть V (корень вверху текстуры)", default=False, update=_upd_param)

    albedo: StringProperty(name="Albedo", subtype='FILE_PATH', update=_upd_tex,
                           description="Цвет волос. Прозрачность — из альфа-канала")
    use_alpha: BoolProperty(name="Прозрачность из альфа-канала", default=True, update=_upd_tex)
    normal: StringProperty(name="Normal", subtype='FILE_PATH', update=_upd_tex,
                           description="Карта нормалей, tangent space")
    flip_g: BoolProperty(name="Инвертировать G (DirectX)", default=False, update=_upd_tex)
    normal_strength: FloatProperty(name="Сила нормали", default=1.0, min=0.0, max=10.0, update=_upd_nstr)

    scope: EnumProperty(name="Применять к", items=(('SEL', "Выделенным", ""), ('ALL', "Всем прядям", "")),
                        default='SEL')
    auto_apply: BoolProperty(name="Менять пряди на лету", default=True)
    live: BoolProperty(name="Живое обновление по направляющим", default=True,
                       description="Правка точек или перемещение кривой сразу перестраивает прядь")
    show_guides: BoolProperty(name="Показывать направляющие", default=True, update=_upd_guides)


# ---------------------------------------------------------------- живое обновление
@persistent
def _on_depsgraph(scene, depsgraph):
    if S.busy:
        return
    hs = getattr(scene, "hair_strands", None)
    if hs is None:
        return
    head = hs.head
    changed = set()
    for u in depsgraph.updates:
        idb = u.id
        if isinstance(idb, bpy.types.Object):
            o = idb.original
            if not (u.is_updated_geometry or u.is_updated_transform):
                continue
            if head is not None and o.name == head.name:
                S.bvh_dirty = True
            if o.type == 'CURVE':
                changed.add(o.name)
        elif isinstance(idb, bpy.types.Curve):
            data = idb.original
            for o in scene.objects:
                if o.type == 'CURVE' and o.data == data:
                    changed.add(o.name)
    if not hs.live or not changed:
        return
    strands = []
    for o in scene.objects:
        if is_strand(o):
            g = guide_of(o)
            if g is not None and g.name in changed:
                strands.append(o)
    if not strands:
        return
    S.busy = True
    try:
        for s in strands:
            rebuild_strand(s, scene, depsgraph)
    finally:
        S.busy = False


@persistent
def _on_load(_dummy):
    S.bvh = None
    S.bvh_head = None
    S.bvh_dirty = True
    S.drawing = False
    S.stop = False


# ---------------------------------------------------------------- рисование
def _line_shader():
    for name in ('UNIFORM_COLOR', '3D_UNIFORM_COLOR'):
        try:
            return gpu.shader.from_builtin(name)
        except (ValueError, TypeError):
            continue
    return None


def _draw_preview(op):
    if len(op.pts) < 2:
        return
    sh = _line_shader()
    if sh is None:
        return
    batch = batch_for_shader(sh, 'LINE_STRIP', {"pos": [tuple(p) for p in op.pts]})
    gpu.state.line_width_set(2.0)
    gpu.state.depth_test_set('NONE')
    sh.bind()
    sh.uniform_float("color", (1.0, 0.8, 0.0, 1.0))
    batch.draw(sh)
    gpu.state.line_width_set(1.0)


class HAIRSTRANDS_OT_draw(bpy.types.Operator):
    """ЛКМ — вести по модели. ПКМ / Esc — выход"""
    bl_idname = "hair_strands.draw"
    bl_label = "Рисовать пряди"
    bl_options = {'REGISTER', 'UNDO'}

    def _region_at(self, context, mx, my):
        for area in context.window.screen.areas:
            if area.type != 'VIEW_3D':
                continue
            for r in area.regions:
                if r.type == 'WINDOW' and r.x <= mx < r.x + r.width and r.y <= my < r.y + r.height:
                    return area, r
        return None, None

    def _screen_point(self, coord):
        region, rv3d = self.region, self.region.data
        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
        direc = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
        hit = S.bvh.ray_cast(origin, direc) if S.bvh is not None else (None, None, None, None)
        vdir = rv3d.view_rotation @ Vector((0, 0, -1))
        ref = self.last_pos if self.last_pos is not None else self.center
        d = direc.dot(vdir)
        pp = origin + direc * ((ref - origin).dot(vdir) / d) if abs(d) > 1e-9 else ref.copy()
        res = pp
        if hit[0] is not None and ((not self.off_surface) or (hit[0] - pp).length <= self.stick):
            res = hit[0]
            self.had_hit = True
            self.off_surface = False
        elif self.had_hit:
            self.off_surface = True
        self.last_pos = res.copy()
        return res

    def invoke(self, context, event):
        hs = context.scene.hair_strands
        if hs.head is None:
            self.report({'WARNING'}, "Сначала выберите модель, по которой рисовать")
            return {'CANCELLED'}
        if S.drawing:
            return {'CANCELLED'}
        build_cache(hs.head, context.evaluated_depsgraph_get())
        self.center = bbox_center(hs.head)
        self.stick = hs.stick
        self.pts = []
        self.px = []
        self.stroking = False
        self.region = None
        self.last_pos = None
        self.had_hit = False
        self.off_surface = False
        self._handle = bpy.types.SpaceView3D.draw_handler_add(_draw_preview, (self,), 'WINDOW', 'POST_VIEW')
        context.window_manager.modal_handler_add(self)
        S.drawing = True
        S.stop = False
        context.workspace.status_text_set("Hair Strands: ЛКМ — рисовать прядь, ПКМ / Esc — выход")
        self._redraw(context)
        return {'RUNNING_MODAL'}

    def _redraw(self, context):
        for area in context.window.screen.areas:
            area.tag_redraw()

    def _finish(self, context):
        bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
        context.workspace.status_text_set(None)
        S.drawing = False
        S.stop = False
        self._redraw(context)

    def _end_stroke(self, context):
        self.stroking = False
        pts = self.pts
        self.pts = []
        if len(pts) < 3:
            return
        kn = min(max(len(pts) // 5, 3), 10)
        kp = resample_poly(pts, kn)
        hs = context.scene.hair_strands
        S.busy = True
        try:
            g = create_guide(context, kp)
            p = params_from_props(hs)
            p["seed"] = new_seed()
            create_strand(context, g, 0, p)
        finally:
            S.busy = False
        bpy.ed.undo_push(message="Hair strand")

    def modal(self, context, event):
        if S.stop or (event.type in {'RIGHTMOUSE', 'ESC'} and event.value == 'PRESS'):
            self.pts = []
            self._finish(context)
            return {'FINISHED'}

        area, region = self._region_at(context, event.mouse_x, event.mouse_y)

        if event.type == 'LEFTMOUSE':
            if event.value == 'PRESS' and region is not None:
                self.region = region
                self.last_pos = None
                self.had_hit = False
                self.off_surface = False
                coord = (event.mouse_x - region.x, event.mouse_y - region.y)
                self.px = [coord]
                self.pts = [self._screen_point(coord)]
                self.stroking = True
                return {'RUNNING_MODAL'}
            if event.value == 'RELEASE' and self.stroking:
                self._end_stroke(context)
                self._redraw(context)
                return {'RUNNING_MODAL'}

        if event.type == 'MOUSEMOVE' and self.stroking:
            r = self.region
            coord = (event.mouse_x - r.x, event.mouse_y - r.y)
            lx, ly = self.px[-1]
            if ((coord[0] - lx) ** 2 + (coord[1] - ly) ** 2) ** 0.5 >= 4:
                self.px.append(coord)
                self.pts.append(self._screen_point(coord))
                self._redraw(context)
            return {'RUNNING_MODAL'}

        if region is None or not self.stroking:
            return {'PASS_THROUGH'}    # панели, навигация
        return {'RUNNING_MODAL'}


class HAIRSTRANDS_OT_draw_stop(bpy.types.Operator):
    """Выйти из режима рисования"""
    bl_idname = "hair_strands.draw_stop"
    bl_label = "Остановить рисование"

    def execute(self, context):
        S.stop = True
        return {'FINISHED'}


# ---------------------------------------------------------------- остальные команды
class HAIRSTRANDS_OT_from_curves(bpy.types.Operator):
    """Создать пряди по выделенным кривым (каждый сплайн — прядь)"""
    bl_idname = "hair_strands.from_curves"
    bl_label = "Из выделенных кривых"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        hs = context.scene.hair_strands
        S.bvh_dirty = True
        made = 0
        S.busy = True
        try:
            for g in [o for o in context.selected_objects if o.type == 'CURVE']:
                existing = {int(o.get("hc_spline", 0)) for o in context.scene.objects
                            if is_strand(o) and guide_of(o) == g}
                for si in range(len(g.data.splines)):
                    if si in existing:
                        continue
                    p = params_from_props(hs)
                    p["seed"] = new_seed()
                    if create_strand(context, g, si, p) is not None:
                        made += 1
        finally:
            S.busy = False
        self.report({'INFO'}, f"Создано прядей: {made}")
        return {'FINISHED'}


class HAIRSTRANDS_OT_reseed(bpy.types.Operator):
    """Другой случайный разброс ширины и слоёв"""
    bl_idname = "hair_strands.reseed"
    bl_label = "Другой случайный вариант"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        hs = context.scene.hair_strands
        update_strands(context, all_=(hs.scope == 'ALL'), apply_params=False, reseed=True)
        return {'FINISHED'}


class HAIRSTRANDS_OT_update(bpy.types.Operator):
    """Пересобрать пряди (с параметрами каждой пряди)"""
    bl_idname = "hair_strands.update"
    bl_label = "Обновить"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        hs = context.scene.hair_strands
        S.bvh_dirty = True
        n = update_strands(context, all_=(hs.scope == 'ALL'), apply_params=False)
        self.report({'INFO'}, f"Обновлено прядей: {n}")
        return {'FINISHED'}


class HAIRSTRANDS_OT_read_params(bpy.types.Operator):
    """Считать параметры выделенной пряди в панель"""
    bl_idname = "hair_strands.read_params"
    bl_label = "Считать параметры"

    def execute(self, context):
        s = strands_from_selection(context)
        if not s:
            self.report({'WARNING'}, "Выделите прядь или направляющую")
            return {'CANCELLED'}
        p = parse_params(s[0].get("hc_params"))
        hs = context.scene.hair_strands
        S.loading = True
        try:
            for k in PARAM_KEYS:
                setattr(hs, k, bool(p[k]) if k == "flip_v" else p[k])
        finally:
            S.loading = False
        self.report({'INFO'}, f"Параметры: {s[0].name}")
        return {'FINISHED'}


class HAIRSTRANDS_OT_assign_material(bpy.types.Operator):
    """Собрать материал HairStrands_Mat и назначить всем прядям"""
    bl_idname = "hair_strands.assign_material"
    bl_label = "Назначить материал всем прядям"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        mat = build_material(context.scene.hair_strands)
        n = assign_material(context, mat)
        self.report({'INFO'}, f"Материал назначен: {n}")
        return {'FINISHED'}


class HAIRSTRANDS_OT_merge(bpy.types.Operator):
    """Объединить все пряди в один меш (копия, исходники остаются)"""
    bl_idname = "hair_strands.merge"
    bl_label = "Объединить всё в один меш"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        strands = [o for o in context.scene.objects if is_strand(o)]
        if not strands:
            self.report({'WARNING'}, "Прядей в сцене нет")
            return {'CANCELLED'}
        bm = bmesh.new()
        for s in strands:
            tmp = s.data.copy()
            tmp.transform(s.matrix_world)
            bm.from_mesh(tmp)
            bpy.data.meshes.remove(tmp)
        me = bpy.data.meshes.new("Hair_Merged")
        bm.to_mesh(me)
        bm.free()
        me.materials.clear()
        mat = bpy.data.materials.get(MAT_NAME)
        if mat is not None:
            me.materials.append(mat)
        ob = bpy.data.objects.new("Hair_Merged", me)
        context.scene.collection.objects.link(ob)
        for o in context.selected_objects:
            o.select_set(False)
        ob.select_set(True)
        context.view_layer.objects.active = ob
        self.report({'INFO'}, f"Создан {ob.name}")
        return {'FINISHED'}


# ---------------------------------------------------------------- панель
class HAIRSTRANDS_PT_panel(bpy.types.Panel):
    bl_label = "Hair Strands"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Hair Strands"

    def draw(self, context):
        hs = context.scene.hair_strands
        L = self.layout

        b = L.box()
        b.label(text="Модель", icon='MESH_DATA')
        b.prop(hs, "head", text="")

        b = L.box()
        b.label(text="Создание", icon='GREASEPENCIL')
        if S.drawing:
            b.operator("hair_strands.draw_stop", icon='CANCEL', depress=True)
        else:
            r = b.row()
            r.scale_y = 1.5
            r.operator("hair_strands.draw", icon='BRUSH_DATA')
        b.operator("hair_strands.from_curves", icon='CURVE_DATA')

        b = L.box()
        b.label(text="Форма пряди", icon='MOD_CURVE')
        c = b.column(align=True)
        for k in ("width", "root", "tip", "bulge", "offset", "thick", "segs", "wsegs", "smooth"):
            c.prop(hs, k)

        b = L.box()
        b.label(text="Слои и разброс", icon='OUTLINER_OB_GROUP_INSTANCE')
        c = b.column(align=True)
        for k in ("layers", "lstep", "rand", "stick"):
            c.prop(hs, k)
        b.operator("hair_strands.reseed", icon='FILE_REFRESH')

        b = L.box()
        b.label(text="Текстуры (HairStrands_Mat)", icon='TEXTURE')
        b.prop(hs, "albedo")
        b.prop(hs, "use_alpha")
        b.prop(hs, "normal")
        b.prop(hs, "flip_g")
        b.prop(hs, "normal_strength")
        b.prop(hs, "flip_v")
        b.operator("hair_strands.assign_material", icon='MATERIAL')

        b = L.box()
        b.label(text="Применение", icon='MODIFIER')
        b.row().prop(hs, "scope", expand=True)
        b.prop(hs, "auto_apply")
        b.prop(hs, "live")
        r = b.row(align=True)
        r.operator("hair_strands.update")
        r.operator("hair_strands.read_params")
        b.prop(hs, "show_guides")
        b.operator("hair_strands.merge", icon='AUTOMERGE_ON')


# ---------------------------------------------------------------- регистрация
_classes = (
    HairStrandsSettings,
    HAIRSTRANDS_OT_draw,
    HAIRSTRANDS_OT_draw_stop,
    HAIRSTRANDS_OT_from_curves,
    HAIRSTRANDS_OT_reseed,
    HAIRSTRANDS_OT_update,
    HAIRSTRANDS_OT_read_params,
    HAIRSTRANDS_OT_assign_material,
    HAIRSTRANDS_OT_merge,
    HAIRSTRANDS_PT_panel,
)


def register():
    for c in _classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.hair_strands = PointerProperty(type=HairStrandsSettings)
    if _on_depsgraph not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_on_depsgraph)
    if _on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load)


def unregister():
    if _on_depsgraph in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_on_depsgraph)
    if _on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load)
    del bpy.types.Scene.hair_strands
    for c in reversed(_classes):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
