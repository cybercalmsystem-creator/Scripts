/*
    HairStrands.ms — рисование прядей волос (hair cards) по поверхности модели.

    Запуск:  Scripting > Run Script... > HairStrands.ms
             (после первого запуска появляется команда Customize > Hair Tools > "Hair Strands")

    Работа:
      1. "Выбрать модель" — указать голову/тело.
      2. "Рисовать пряди" — зажать ЛКМ и вести по модели; отпустить — прядь создана.
         Правая кнопка / Esc — выход из режима рисования.
      3. Либо нарисовать линии штатными Line / Freehand и нажать "Из выделенных сплайнов".
      4. Параметры меняют пряди на лету: выделенные или все (переключатель "Применять к").
      5. Живое обновление: двигайте вершины / весь сплайн направляющей — прядь
         перестраивается сразу, без кнопки "Обновить".
      6. Текстуры: Albedo (прозрачность берётся из альфа-канала) и Normal (tangent space).
         Общий материал "HairStrands_Mat" назначается всем прядям.
         Прозрачность во вьюпорте видна в режиме High Quality / Standard.
*/

global HCT
global HCT_Rollout
global HCT_fromMacro

try (destroyDialog HCT_Rollout) catch ()
if HCT != undefined do (try (HCT.freeCache()) catch ())
try (deleteAllChangeHandlers id:#hcLive) catch ()
try (callbacks.removeScripts id:#hcLive) catch ()

struct HairCardsToolStruct
(
    -- 1 ширина, 2 сегм. по длине, 3 сегм. по ширине, 4 корень %, 5 кончик %, 6 выпуклость,
    -- 7 отступ, 8 толщина, 9 слои, 10 шаг слоёв, 11 разброс %, 12 сглаживание, 13 прилипание, 14 seed,
    -- 15 перевернуть V (0/1)
    params = #(1.2, 12, 3, 60.0, 25.0, 0.25, 0.1, 0.0, 1, 0.15, 15.0, 3, 1.0, 1, 0),

    head = undefined,
    mpi = undefined,
    headMesh = undefined,
    cacheDirty = false,

    strokePts = #(),
    lastVP = undefined,
    lastPos = undefined,
    hadHit = false,
    offSurface = false,
    preview = undefined,
    drawing = false,
    strokeDone = false,
    stopDraw = false,

    -- живое обновление
    liveOn = true,

    -- текстуры
    matName = "HairStrands_Mat",
    mat = undefined,
    albedoFile = undefined,
    normalFile = undefined,
    useAlpha = true,
    flipG = false,
    normalStr = 1.0,

    ------------------------------------------------------------------ кэш поверхности
    fn freeCache =
    (
        if mpi != undefined do (try (mpi.free()) catch ())
        mpi = undefined
        if headMesh != undefined do (try (delete headMesh) catch ())
        headMesh = undefined
    ),

    fn buildCache =
    (
        freeCache()
        cacheDirty = false
        if isValidNode head do
        (
            mpi = MeshProjIntersect()
            mpi.setNode head
            mpi.build()
            headMesh = snapshotAsMesh head
        )
    ),

    -- ближайшая точка поверхности и её нормаль: #(pos, normal) либо undefined
    fn surfaceInfo p =
    (
        if (cacheDirty or mpi == undefined) and isValidNode head do buildCache()
        local res = undefined
        if mpi != undefined and headMesh != undefined do
        (
            if (mpi.closestFace p doubleSided:true) do
            (
                local f = (mpi.getHitFace()) + 1
                if f >= 1 and f <= headMesh.numfaces do res = #(mpi.getHitPos(), getFaceNormal headMesh f)
            )
        )
        res
    ),

    ------------------------------------------------------------------ параметры
    fn paramsToString p =
    (
        local s = ""
        for i = 1 to p.count do (s += (p[i] as string); if i < p.count do s += ";")
        s
    ),

    fn parseParams str =
    (
        local p = deepCopy params
        p[15] = 0  -- у старых прядей этого параметра нет
        if str != undefined do
        (
            local parts = filterString (str as string) ";"
            for i = 1 to parts.count where i <= p.count do
            (
                local v = parts[i] as float
                if v != undefined do p[i] = v
            )
        )
        p
    ),

    ------------------------------------------------------------------ геометрия
    fn resamplePoly pts count =
    (
        local res = #()
        local lens = #(0.0)
        for i = 2 to pts.count do append lens (lens[i-1] + distance pts[i-1] pts[i])
        local total = lens[lens.count]
        if total < 1e-6 then res = #(pts[1], pts[pts.count])
        else
        (
            local k = 1
            for i = 0 to count do
            (
                local d = total * i / (count as float)
                while k < pts.count - 1 and lens[k+1] < d do k += 1
                local segLen = lens[k+1] - lens[k]
                local f = if segLen > 1e-9 then (d - lens[k]) / segLen else 0.0
                append res (pts[k] + (pts[k+1] - pts[k]) * f)
            )
        )
        res
    ),

    fn samplePath guide si p =
    (
        local count = p[2] as integer
        if count < 2 do count = 2
        local pts = for i = 0 to count collect (lengthInterp guide si (i / (count as float)))
        for it = 1 to (p[12] as integer) do
        (
            local np = deepCopy pts
            for i = 2 to pts.count - 1 do np[i] = (pts[i-1] + pts[i] * 2 + pts[i+1]) / 4.0
            pts = np
        )
        pts
    ),

    fn ease x =
    (
        if x < 0 do x = 0.0
        if x > 1 do x = 1.0
        x * x * (3 - 2 * x)
    ),

    fn makeStrandMesh pts p =
    (
        local n = pts.count
        local W = p[3] as integer
        if W < 1 do W = 1
        local flipV = p[15] > 0.5
        local center = [0,0,0]
        if isValidNode head then center = head.center
        else (for q in pts do center += q; center /= n)

        -- локальные оси вдоль пряди
        local tans = #(), nrms = #(), bins = #()
        local prevN = undefined
        for i = 1 to n do
        (
            local a = if i > 1 then i - 1 else 1
            local b = if i < n then i + 1 else n
            local t = pts[b] - pts[a]
            t = if length t > 1e-6 then normalize t else [0,0,1]
            local nn = undefined
            local si = surfaceInfo pts[i]
            if si != undefined and (prevN == undefined or distance si[1] pts[i] <= p[13]) do nn = si[2]
            if nn == undefined do nn = if prevN != undefined then prevN else (pts[i] - center)
            nn = nn - t * (dot nn t)
            if length nn < 1e-5 do nn = cross t [0,0,1]
            if length nn < 1e-5 do nn = cross t [1,0,0]
            nn = normalize nn
            tans[i] = t
            nrms[i] = nn
            prevN = nn
        )
        for it = 1 to 2 do
        (
            local n2 = deepCopy nrms
            for i = 2 to n - 1 do n2[i] = normalize (nrms[i-1] + nrms[i] * 2 + nrms[i+1])
            nrms = n2
        )
        for i = 1 to n do
        (
            local nn = nrms[i] - tans[i] * (dot nrms[i] tans[i])
            if length nn < 1e-5 do nn = cross tans[i] [0,0,1]
            if length nn < 1e-5 do nn = cross tans[i] [1,0,0]
            nrms[i] = normalize nn
            local bb = cross tans[i] nrms[i]
            if i > 1 and length bb < 1e-5 then bins[i] = bins[i-1] else bins[i] = normalize bb
        )

        -- UV: у каждой пряди своя ровная развёртка 0–1.
        -- V — по реальной длине вдоль пряди (без растяжений после сглаживания),
        -- U — по реальной ширине каждого ряда (с учётом выпуклости).
        local arc = #(0.0)
        for i = 2 to n do append arc (arc[i-1] + distance pts[i-1] pts[i])
        local arcT = arc[n]
        local vs = #()
        for i = 1 to n do
        (
            local v = if arcT > 1e-6 then arc[i] / arcT else (i - 1) / ((n - 1) as float)
            vs[i] = if flipV then 1.0 - v else v
        )
        local vDir = if flipV then -1.0 else 1.0

        local verts = #(), faces = #(), sgs = #(), tvs = #(), tvf = #()
        local rootS = p[4] / 100.0, tipS = p[5] / 100.0
        local thick = p[8]
        local layers = p[9] as integer
        if layers < 1 do layers = 1
        local row = W + 1
        seed (p[14] as integer)

        for layer = 1 to layers do
        (
            local lw = p[1] * (1.0 + (p[11] / 100.0) * (random -1.0 1.0))
            local shift = if layer == 1 then 0.0 else (random -0.3 0.3) * p[1]
            local lift = p[7] + (layer - 1) * p[10]
            local base = verts.count
            local tb = tvs.count
            local tvOff = tb - base   -- вершина верха k -> UV-вершина k + tvOff

            for i = 1 to n do
            (
                local t = (i - 1) / ((n - 1) as float)
                local wd = lw * (rootS + (1 - rootS) * (ease (t / 0.25))) * (tipS + (1 - tipS) * (ease ((1 - t) / 0.4)))
                local rowPts = #()
                for j = 0 to W do
                (
                    local u = j / (W as float) - 0.5
                    local arch = p[6] * wd * (1 - 4 * u * u)
                    append rowPts (pts[i] + bins[i] * (u * wd + shift) + nrms[i] * (lift + arch))
                )
                local acc = #(0.0)
                for j = 2 to row do append acc (acc[j-1] + distance rowPts[j-1] rowPts[j])
                local tot = acc[row]
                for j = 1 to row do
                (
                    append verts rowPts[j]
                    local uu = if tot > 1e-6 then acc[j] / tot else (j - 1) / (W as float)
                    append tvs [uu, vs[i], 0]
                )
            )
            -- верх
            for i = 1 to n - 1 do for j = 1 to W do
            (
                local a = base + (i - 1) * row + j
                local b = a + 1, d = a + row
                local c = d + 1
                append faces [a, b, c]; append faces [a, c, d]
                append tvf [a + tvOff, b + tvOff, c + tvOff]; append tvf [a + tvOff, c + tvOff, d + tvOff]
                append sgs 1; append sgs 1
            )
            if thick > 0 do
            (
                local bbase = verts.count
                local cnt = n * row
                local tbb = tvs.count
                for k = 1 to cnt do
                (
                    local i = ((k - 1) / row) + 1
                    append verts (verts[base + k] - nrms[i] * thick)
                    append tvs tvs[tb + k]          -- низ: та же развёртка, что и верх
                )
                local dOff = bbase - base
                local offB = tbb - bbase
                -- низ
                for i = 1 to n - 1 do for j = 1 to W do
                (
                    local a = base + (i - 1) * row + j + dOff
                    local b = a + 1, d = a + row
                    local c = d + 1
                    append faces [a, d, c]; append faces [a, c, b]
                    append tvf [a + offB, d + offB, c + offB]; append tvf [a + offB, c + offB, b + offB]
                    append sgs 2; append sgs 2
                )

                -- боковины и торцы: свои узкие полоски по краям 0–1 (без вырожденных UV)
                local su = thick / (amax lw 1e-6)
                if su < 0.005 do su = 0.005
                if su > 0.25 do su = 0.25
                local sv = if arcT > 1e-6 then thick / arcT else 0.01
                if sv < 0.005 do sv = 0.005
                if sv > 0.25 do sv = 0.25

                local lT = #(), lB = #(), rT = #(), rB = #()
                for i = 1 to n do
                (
                    append tvs [0, vs[i], 0];        append lT tvs.count
                    append tvs [su, vs[i], 0];       append lB tvs.count
                    append tvs [1, vs[i], 0];        append rT tvs.count
                    append tvs [1 - su, vs[i], 0];   append rB tvs.count
                )
                local cT = #(), cB = #(), eT = #(), eB = #()
                for j = 1 to row do
                (
                    local u0 = tvs[tb + j].x
                    local u1 = tvs[tb + (n - 1) * row + j].x
                    append tvs [u0, vs[1], 0];                   append cT tvs.count
                    append tvs [u0, vs[1] + vDir * sv, 0];       append cB tvs.count
                    append tvs [u1, vs[n], 0];                   append eT tvs.count
                    append tvs [u1, vs[n] - vDir * sv, 0];       append eB tvs.count
                )

                -- боковины
                for i = 1 to n - 1 do
                (
                    local t1 = base + (i - 1) * row + 1
                    local t2 = t1 + row
                    append faces [t1, t2, t2 + dOff]; append faces [t1, t2 + dOff, t1 + dOff]
                    append tvf [lT[i], lT[i+1], lB[i+1]]; append tvf [lT[i], lB[i+1], lB[i]]
                    local r1 = t1 + W, r2 = t2 + W
                    append faces [r1, r1 + dOff, r2 + dOff]; append faces [r1, r2 + dOff, r2]
                    append tvf [rT[i], rB[i], rB[i+1]]; append tvf [rT[i], rB[i+1], rT[i+1]]
                    for q = 1 to 4 do append sgs 4
                )
                -- торцы
                for j = 1 to W do
                (
                    local s1 = base + j
                    local s2 = s1 + 1
                    append faces [s1, s1 + dOff, s2 + dOff]; append faces [s1, s2 + dOff, s2]
                    append tvf [cT[j], cB[j], cB[j+1]]; append tvf [cT[j], cB[j+1], cT[j+1]]
                    local e1 = base + (n - 1) * row + j
                    local e2 = e1 + 1
                    append faces [e1, e2, e2 + dOff]; append faces [e1, e2 + dOff, e1 + dOff]
                    append tvf [eT[j], eT[j+1], eB[j+1]]; append tvf [eT[j], eB[j+1], eB[j]]
                    for q = 1 to 4 do append sgs 8
                )
            )
        )

        local m = TriMesh()
        setMesh m vertices:verts faces:faces
        setNumTVerts m tvs.count
        buildTVFaces m
        for i = 1 to tvs.count do setTVert m i tvs[i]
        for f = 1 to faces.count do
        (
            setTVFace m f tvf[f]
            setFaceSmoothGroup m f sgs[f]
            setFaceMatID m f 1
            -- диагональ квада скрыта: в Editable Poly получаются четырёхугольники
            if (mod f 2) == 1 then setEdgeVis m f 3 false else setEdgeVis m f 1 false
        )
        update m
        m
    ),

    ------------------------------------------------------------------ материал и текстуры
    fn isPhysMat m = (m != undefined and (classOf m) == PhysicalMaterial),

    fn ensureMaterial =
    (
        if mat == undefined do
        (
            mat = try (PhysicalMaterial name:matName) catch (undefined)
            if mat != undefined then (try (mat.thin_walled = true) catch ())
            else (mat = Standardmaterial name:matName; mat.twoSided = true)
        )
        mat
    ),

    fn makeBitmap f normal:false =
    (
        local t = Bitmaptexture filename:f name:(getFilenameFile f)
        -- карта нормалей должна читаться линейно (gamma 1.0)
        if normal do (try (t.bitmap = openBitmap f gamma:1.0) catch ())
        t
    ),

    -- пересобрать карты материала по текущим файлам и настройкам
    fn applyTextures =
    (
        local m = ensureMaterial()
        local a = undefined, op = undefined, nb = undefined
        if albedoFile != undefined and doesFileExist albedoFile do
        (
            a = makeBitmap albedoFile
            a.alphaSource = if useAlpha then 0 else 2       -- 0 альфа из файла, 2 без альфы
            a.preMultAlpha = false
            if useAlpha do
            (
                op = makeBitmap albedoFile
                op.name = "Opacity (alpha)"
                op.alphaSource = 0
                op.preMultAlpha = false
                op.monoOutput = 1                           -- моно-выход = альфа
                op.rgbOutput = 1                            -- RGB-выход = альфа
            )
        )
        if normalFile != undefined and doesFileExist normalFile do
        (
            nb = Normal_Bump name:"Normal"
            nb.normal_map = makeBitmap normalFile normal:true
            nb.method = 0                                   -- tangent space
            nb.mult_spin = normalStr
            nb.flipgreen = flipG
        )
        if isPhysMat m then
        (
            m.base_color_map = a
            m.cutout_map = op
            m.bump_map = nb
            m.bump_map_amt = 1.0
        )
        else
        (
            m.diffuseMap = a
            m.opacityMap = op
            m.bumpMap = nb
            m.bumpMapAmount = 100
        )
        if a != undefined do (try (showTextureMap m a true) catch ())
        try (showTextureMap m true) catch ()
        m
    ),

    -- только сила / инверсия G — без пересоздания карт
    fn updateNormalSettings =
    (
        if mat != undefined do
        (
            local nb = if isPhysMat mat then mat.bump_map else mat.bumpMap
            if nb != undefined and (classOf nb) == Normal_Bump do
            (
                nb.mult_spin = normalStr
                nb.flipgreen = flipG
            )
        )
    ),

    fn isStrand o = (isValidNode o and (getUserProp o "hcGuide") != undefined),

    -- назначить материал прядям без материала или со старой копией HairStrands_Mat
    fn assignMaterial =
    (
        local cnt = 0
        if mat != undefined do
            for o in objects where isStrand o do
                if o.material == undefined or o.material == mat or o.material.name == matName do
                (
                    o.material = mat
                    cnt += 1
                )
        cnt
    ),

    -- найти материал в сцене и прочитать из него файлы и настройки
    fn restoreMaterial =
    (
        mat = undefined
        albedoFile = undefined
        normalFile = undefined
        for o in objects where isStrand o and o.material != undefined and o.material.name == matName while mat == undefined do mat = o.material
        if mat != undefined do
        (
            local phys = isPhysMat mat
            local a  = if phys then mat.base_color_map else mat.diffuseMap
            local op = if phys then mat.cutout_map else mat.opacityMap
            local nb = if phys then mat.bump_map else mat.bumpMap
            if a != undefined and isProperty a #filename do albedoFile = a.filename
            useAlpha = op != undefined
            if nb != undefined and (classOf nb) == Normal_Bump do
            (
                local nt = nb.normal_map
                if nt != undefined and isProperty nt #filename do normalFile = nt.filename
                flipG = nb.flipgreen
                normalStr = nb.mult_spin
            )
        )
        mat
    ),

    ------------------------------------------------------------------ узлы сцены
    fn getLayer nm =
    (
        local l = LayerManager.getLayerFromName nm
        if l == undefined do l = LayerManager.newLayerFromName nm
        l
    ),

    fn guideOf node =
    (
        local h = getUserProp node "hcGuide"
        local g = undefined
        if h != undefined do g = try (maxOps.getNodeByHandle (h as integer)) catch (undefined)
        if isValidNode g and superClassOf g == shape then g else undefined
    ),

    -- после перезапуска скрипта / открытия файла: вернуть модель из свойств прядей
    fn restoreHead =
    (
        if not (isValidNode head) do
        (
            head = undefined
            for o in objects where isStrand o while head == undefined do
            (
                local h = getUserProp o "hcHead"
                if h != undefined do
                (
                    local n = try (maxOps.getNodeByHandle (h as integer)) catch (undefined)
                    if isValidNode n and superClassOf n == GeometryClass do head = n
                )
            )
            cacheDirty = true
        )
        head
    ),

    fn rebuildStrand node =
    (
        local ok = false
        local g = guideOf node
        if g != undefined do
        (
            local si = getUserProp node "hcSpline"
            si = if si == undefined then 1 else (si as integer)
            local ns = try (numSplines g) catch (0)
            if si >= 1 and si <= ns do
            (
                local p = parseParams (getUserProp node "hcParams")
                local pts = samplePath g si p
                local m = makeStrandMesh pts p
                node.transform = matrix3 1
                node.mesh = m
                update node
                ok = true
            )
        )
        ok
    ),

    fn createStrand guide si p =
    (
        local node = Editable_Mesh name:(uniqueName "HairStrand_")
        node.wirecolor = color 150 150 150
        setUserProp node "hcGuide" guide.inode.handle
        setUserProp node "hcSpline" si
        setUserProp node "hcParams" (paramsToString p)
        if isValidNode head do setUserProp node "hcHead" head.inode.handle
        if mat != undefined do node.material = mat
        (getLayer "HairStrands").addNode node
        if not (rebuildStrand node) do (delete node; node = undefined)
        node
    ),

    fn newSeed = (random 1 999999),

    fn strandsFromSelection =
    (
        local res = #()
        local sel = selection as array
        local handles = for o in sel where superClassOf o == shape collect o.inode.handle
        for o in sel where isStrand o do appendIfUnique res o
        if handles.count > 0 do
            for o in objects where isStrand o do
                if (findItem handles ((getUserProp o "hcGuide") as integer)) > 0 do appendIfUnique res o
        res
    ),

    fn collectStrands all:false =
    (
        if all then (for o in objects where isStrand o collect o) else strandsFromSelection()
    ),

    fn updateStrands all:false applyParams:true reseed:false =
    (
        local strands = collectStrands all:all
        for s in strands do
        (
            if applyParams or reseed do
            (
                local old = parseParams (getUserProp s "hcParams")
                local p = if applyParams then deepCopy params else old
                p[14] = if reseed then newSeed() else old[14]
                setUserProp s "hcParams" (paramsToString p)
            )
            rebuildStrand s
        )
        redrawViews()
        strands.count
    ),

    ------------------------------------------------------------------ живое обновление
    -- направляющая изменилась (вершины, касательные, перемещение) — перестроить её пряди
    fn onGuideChanged g =
    (
        if not drawing and isValidNode g do
        (
            local h = g.inode.handle
            local list = for o in objects where isStrand o and ((getUserProp o "hcGuide") as integer) == h collect o
            if list.count > 0 do undo off (for s in list do rebuildStrand s)
        )
    ),

    fn registerLive =
    (
        deleteAllChangeHandlers id:#hcLive
        if liveOn do
        (
            local guides = #()
            for o in objects where isStrand o do
            (
                local g = guideOf o
                if g != undefined do appendIfUnique guides g
            )
            if guides.count > 0 do
            (
                when geometry guides changes id:#hcLive handleAt:#redrawViews g do (if HCT != undefined do HCT.onGuideChanged g)
                when topology guides changes id:#hcLive handleAt:#redrawViews g do (if HCT != undefined do HCT.onGuideChanged g)
                when transform guides changes id:#hcLive handleAt:#redrawViews g do (if HCT != undefined do HCT.onGuideChanged g)
            )
        )
        -- модель сдвинули / изменили — кэш поверхности пересобрать при следующем обращении
        if isValidNode head do
        (
            when geometry head changes id:#hcLive do (if HCT != undefined do HCT.cacheDirty = true)
            when transform head changes id:#hcLive do (if HCT != undefined do HCT.cacheDirty = true)
        )
        OK
    ),

    fn convertSelectedSplines =
    (
        local made = 0
        local shapes = for o in selection where superClassOf o == shape collect o
        buildCache()
        undo "Hair strands from splines" on
        (
            for g in shapes do
            (
                if classOf g != SplineShape and classOf g != line do (try (convertToSplineShape g) catch ())
                local ns = try (numSplines g) catch (0)
                local existing = for o in objects where isStrand o and ((getUserProp o "hcGuide") as integer) == g.inode.handle collect ((getUserProp o "hcSpline") as integer)
                for si = 1 to ns where (findItem existing si) == 0 do
                (
                    local p = deepCopy params
                    p[14] = newSeed()
                    if (createStrand g si p) != undefined do made += 1
                )
            )
        )
        registerLive()
        redrawViews()
        made
    ),

    fn mergeAll =
    (
        local strands = for o in objects where isStrand o collect o
        local res = undefined
        if strands.count > 0 do
        (
            undo "Merge hair strands" on
            (
                res = Editable_Mesh name:(uniqueName "Hair_Merged_")
                res.wirecolor = color 150 150 150
                for s in strands do meshop.attach res s deleteSourceNode:false
                update res
                convertTo res Editable_Poly
                if mat != undefined do res.material = mat
            )
        )
        res
    ),

    ------------------------------------------------------------------ рисование
    fn screenPoint vp =
    (
        local r = mapScreenToWorldRay vp
        local hit = if isValidNode head then (intersectRay head r) else undefined
        local vdir = -((inverse (getViewTM())).row3)
        local ref = if lastPos != undefined then lastPos else (if isValidNode head then head.center else [0,0,0])
        local d = dot r.dir vdir
        local pp = if (abs d) > 1e-6 then (r.pos + r.dir * ((dot (ref - r.pos) vdir) / d)) else ref
        local res = pp
        if hit != undefined and ((not offSurface) or (distance hit.pos pp) <= params[13]) then
        (
            res = hit.pos
            hadHit = true
            offSurface = false
        )
        else (if hadHit do offSurface = true)
        lastPos = res
        res
    ),

    fn killPreview =
    (
        if isValidNode preview do (undo off (delete preview))
        preview = undefined
    ),

    fn strokeBegin vp =
    (
        killPreview()
        strokePts = #()
        lastPos = undefined
        hadHit = false
        offSurface = false
        drawing = true
        lastVP = vp
        append strokePts (screenPoint vp)
        undo off
        (
            preview = SplineShape name:"__HairStrokePreview" wirecolor:(color 255 200 0)
            addNewSpline preview
            addKnot preview 1 #corner #line strokePts[1]
        )
        OK
    ),

    fn strokeAdd vp =
    (
        if drawing and (distance vp lastVP) >= 4 do
        (
            lastVP = vp
            local p = screenPoint vp
            append strokePts p
            if isValidNode preview do
            (
                undo off (addKnot preview 1 #corner #line p; updateShape preview)
                redrawViews()
            )
        )
        OK
    ),

    fn strokeEnd =
    (
        local wasDrawing = drawing
        drawing = false
        killPreview()
        if wasDrawing and strokePts.count >= 3 do
        (
            undo "Hair strand" on
            (
                local kn = strokePts.count / 5
                if kn < 3 do kn = 3
                if kn > 10 do kn = 10
                local kp = resamplePoly strokePts kn
                local g = SplineShape name:(uniqueName "HairGuide_") wirecolor:(color 255 140 0)
                addNewSpline g
                for q in kp do addKnot g 1 #smooth #curve q
                updateShape g
                (getLayer "HairGuides").addNode g
                local p = deepCopy params
                p[14] = newSeed()
                createStrand g 1 p
            )
            registerLive()
            redrawViews()
        )
        strokePts = #()
        strokeDone = true
        OK
    ),

    fn strokeCancel =
    (
        drawing = false
        killPreview()
        strokePts = #()
        OK
    )
)

HCT = HairCardsToolStruct()

tool HCT_DrawTool
(
    on mousePoint clickNo do
    (
        if clickNo == 1 then HCT.strokeBegin viewPoint
        else (HCT.strokeEnd(); #stop)
    )
    on mouseMove clickNo do (HCT.strokeAdd viewPoint)
    on mouseAbort clickNo do (HCT.strokeCancel())
)

rollout HCT_Rollout "Hair Strands" width:240
(
    local imgTypes = "Images|*.png;*.tga;*.tif;*.tiff;*.dds;*.exr;*.psd;*.jpg;*.jpeg;*.bmp|All files|*.*|"

    group "Модель"
    (
        pickbutton pbHead "Выбрать модель (голова)" width:210 autoDisplay:false
    )
    group "Создание"
    (
        checkbutton cbDraw "Рисовать пряди" width:210 height:30 tooltip:"ЛКМ — вести по модели. ПКМ / Esc — выход"
        button btConvert "Из выделенных сплайнов" width:210 tooltip:"Line / Freehand, нарисованные вручную"
    )
    group "Форма пряди"
    (
        spinner spWidth    "Ширина:"            range:[0.001, 100000, 1.2]  type:#worldunits fieldwidth:60
        spinner spRoot     "Корень, %:"         range:[1, 200, 60]          type:#float      fieldwidth:60
        spinner spTip      "Кончик, %:"         range:[1, 200, 25]          type:#float      fieldwidth:60
        spinner spBulge    "Выпуклость:"        range:[-2, 2, 0.25]         type:#float      fieldwidth:60 scale:0.01
        spinner spOffset   "Отступ от модели:"  range:[-10000, 10000, 0.1]  type:#worldunits fieldwidth:60
        spinner spThick    "Толщина:"           range:[0, 10000, 0]         type:#worldunits fieldwidth:60
        spinner spSegs     "Сегм. по длине:"    range:[2, 200, 12]          type:#integer    fieldwidth:60
        spinner spWSegs    "Сегм. по ширине:"   range:[1, 32, 3]            type:#integer    fieldwidth:60
        spinner spSmooth   "Сглаживание:"       range:[0, 50, 3]            type:#integer    fieldwidth:60
    )
    group "Слои и разброс"
    (
        spinner spLayers   "Слоёв:"             range:[1, 12, 1]            type:#integer    fieldwidth:60
        spinner spLStep    "Шаг слоёв:"         range:[0, 10000, 0.15]      type:#worldunits fieldwidth:60
        spinner spRand     "Разброс ширины, %:" range:[0, 90, 15]           type:#float      fieldwidth:60
        spinner spStick    "Прилипание:"        range:[0, 100000, 1.0]      type:#worldunits fieldwidth:60
        button btSeed "Другой случайный вариант" width:210
    )
    group "Текстуры (HairStrands_Mat)"
    (
        button btAlbedo  "Albedo: нет" width:175 align:#left across:2 tooltip:"Цвет волос. Прозрачность — из альфа-канала (PNG / TGA / TIF / DDS / EXR)"
        button btAlbedoX "X" width:25 align:#right tooltip:"Убрать albedo"
        checkbox chkAlpha "Прозрачность из альфа-канала" checked:true
        button btNormal  "Normal: нет" width:175 align:#left across:2 tooltip:"Карта нормалей, tangent space"
        button btNormalX "X" width:25 align:#right tooltip:"Убрать normal"
        checkbox chkFlipG "Инвертировать G (DirectX)" checked:false
        spinner spNStr   "Сила нормали:"      range:[0, 10, 1.0]          type:#float      fieldwidth:60 scale:0.05
        checkbox chkFlipV "Перевернуть V (корень вверху текстуры)" checked:false
        button btAssign  "Назначить материал всем прядям" width:210
    )
    group "Применение"
    (
        radiobuttons rbScope "Применять к:" labels:#("Выделенным", "Всем прядям") default:1 columns:2 align:#left
        checkbox chkAuto "Менять пряди на лету" checked:true
        checkbox chkLive "Живое обновление по направляющим" checked:true tooltip:"Правка вершин или перемещение сплайна сразу перестраивает прядь"
        button btUpdate "Обновить" width:103 across:2 tooltip:"Пересобрать пряди (с параметрами каждой пряди)"
        button btRead "Считать параметры" width:103
        checkbox chkGuides "Показывать направляющие" checked:true
        button btMerge "Объединить всё в Editable Poly" width:210
    )
    label lbInfo "" align:#left

    fn info s = (lbInfo.text = s)
    fn scopeAll = (rbScope.state == 2)

    fn pushUI =
    (
        local p = HCT.params
        spWidth.value = p[1]; spSegs.value = p[2]; spWSegs.value = p[3]; spRoot.value = p[4]
        spTip.value = p[5]; spBulge.value = p[6]; spOffset.value = p[7]; spThick.value = p[8]
        spLayers.value = p[9]; spLStep.value = p[10]; spRand.value = p[11]; spSmooth.value = p[12]
        spStick.value = p[13]; chkFlipV.checked = p[15] > 0.5
    )

    fn texUI =
    (
        btAlbedo.text = "Albedo: " + (if HCT.albedoFile != undefined then filenameFromPath HCT.albedoFile else "нет")
        btNormal.text = "Normal: " + (if HCT.normalFile != undefined then filenameFromPath HCT.normalFile else "нет")
        chkAlpha.checked = HCT.useAlpha
        chkFlipG.checked = HCT.flipG
        spNStr.value = HCT.normalStr
    )

    -- состояние из сцены: модель, материал, обработчики
    fn syncScene =
    (
        HCT.head = undefined
        HCT.freeCache()
        HCT.restoreHead()
        pbHead.text = if isValidNode HCT.head then ("Модель: " + HCT.head.name) else "Выбрать модель (голова)"
        HCT.restoreMaterial()
        texUI()
        HCT.registerLive()
    )

    fn setParam i v =
    (
        HCT.params[i] = v
        if chkAuto.checked do (undo off (HCT.updateStrands all:(scopeAll()) applyParams:true))
    )

    fn texChanged =
    (
        undo off
        (
            HCT.applyTextures()
            HCT.assignMaterial()
        )
        texUI()
        redrawViews()
    )

    on spWidth  changed v do setParam 1 v
    on spSegs   changed v do setParam 2 v
    on spWSegs  changed v do setParam 3 v
    on spRoot   changed v do setParam 4 v
    on spTip    changed v do setParam 5 v
    on spBulge  changed v do setParam 6 v
    on spOffset changed v do setParam 7 v
    on spThick  changed v do setParam 8 v
    on spLayers changed v do setParam 9 v
    on spLStep  changed v do setParam 10 v
    on spRand   changed v do setParam 11 v
    on spSmooth changed v do setParam 12 v
    on spStick  changed v do setParam 13 v
    on chkFlipV changed s do setParam 15 (if s then 1 else 0)

    on pbHead picked obj do
    (
        if superClassOf obj == GeometryClass then
        (
            HCT.head = obj
            HCT.buildCache()
            HCT.registerLive()
            pbHead.text = "Модель: " + obj.name
            info ""
        )
        else info "Нужен геометрический объект"
    )

    on cbDraw changed state do
    (
        if state then
        (
            if not (isValidNode HCT.head) then
            (
                messageBox "Сначала выберите модель, по которой рисовать." title:"Hair Strands"
                cbDraw.checked = false
            )
            else
            (
                HCT.buildCache()
                HCT.stopDraw = false
                info "ЛКМ — рисовать, ПКМ/Esc — выход"
                while not HCT.stopDraw do
                (
                    HCT.strokeDone = false
                    startTool HCT_DrawTool
                    if not HCT.strokeDone do HCT.stopDraw = true
                )
                HCT.strokeCancel()
                cbDraw.checked = false
                info ""
            )
        )
        else
        (
            HCT.stopDraw = true
            try (stopTool HCT_DrawTool) catch ()
        )
    )

    on btConvert pressed do
    (
        local n = HCT.convertSelectedSplines()
        info ("Создано прядей: " + n as string)
    )

    on btSeed pressed do
    (
        undo "Hair strands reseed" on (HCT.updateStrands all:(scopeAll()) applyParams:false reseed:true)
    )

    ------------------------------------------------ текстуры
    on btAlbedo pressed do
    (
        local f = getOpenFileName caption:"Albedo (с альфа-каналом)" types:imgTypes
        if f != undefined do (HCT.albedoFile = f; texChanged(); info "Albedo назначен")
    )
    on btAlbedoX pressed do (HCT.albedoFile = undefined; texChanged())

    on btNormal pressed do
    (
        local f = getOpenFileName caption:"Normal map (tangent space)" types:imgTypes
        if f != undefined do (HCT.normalFile = f; texChanged(); info "Normal назначен")
    )
    on btNormalX pressed do (HCT.normalFile = undefined; texChanged())

    on chkAlpha changed s do (HCT.useAlpha = s; texChanged())
    on chkFlipG changed s do (HCT.flipG = s; HCT.updateNormalSettings(); redrawViews())
    on spNStr changed v do (HCT.normalStr = v; HCT.updateNormalSettings(); redrawViews())

    on btAssign pressed do
    (
        local n = 0
        undo "Hair strands material" on
        (
            HCT.applyTextures()
            n = HCT.assignMaterial()
        )
        redrawViews()
        info ("Материал назначен: " + n as string)
    )

    ------------------------------------------------ применение
    on chkLive changed s do (HCT.liveOn = s; HCT.registerLive())

    on btUpdate pressed do
    (
        HCT.buildCache()
        local n = 0
        undo "Hair strands update" on (n = HCT.updateStrands all:(scopeAll()) applyParams:false)
        HCT.registerLive()
        info ("Обновлено прядей: " + n as string)
    )

    on btRead pressed do
    (
        local s = HCT.strandsFromSelection()
        if s.count > 0 then
        (
            local seedKeep = HCT.params[14]
            HCT.params = HCT.parseParams (getUserProp s[1] "hcParams")
            HCT.params[14] = seedKeep
            pushUI()
            info ("Параметры: " + s[1].name)
        )
        else info "Выделите прядь или направляющую"
    )

    on chkGuides changed state do
    (
        local l = LayerManager.getLayerFromName "HairGuides"
        if l != undefined do l.on = state
        redrawViews()
    )

    on btMerge pressed do
    (
        local r = HCT.mergeAll()
        if r != undefined then (select r; info ("Создан " + r.name)) else info "Прядей в сцене нет"
    )

    on HCT_Rollout open do
    (
        pushUI()
        syncScene()
        local cb = "if HCT_Rollout != undefined and HCT_Rollout.open do HCT_Rollout.syncScene()"
        callbacks.addScript #filePostOpen cb id:#hcLive
        callbacks.addScript #systemPostNew cb id:#hcLive
        callbacks.addScript #systemPostReset cb id:#hcLive
    )

    on HCT_Rollout close do
    (
        HCT.stopDraw = true
        try (stopTool HCT_DrawTool) catch ()
        HCT.strokeCancel()
        deleteAllChangeHandlers id:#hcLive
        callbacks.removeScripts id:#hcLive
        HCT.freeCache()
    )
)

createDialog HCT_Rollout

-- команда для панели инструментов / меню / горячей клавиши
if HCT_fromMacro != true do
(
    local src = getSourceFileName()
    if src != undefined and doesFileExist src do
    (
        try
        (
            execute ("macroScript HairStrands category:\"Hair Tools\" buttonText:\"Hair Strands\" tooltip:\"Hair Strands - draw hair cards\" (on execute do (global HCT_fromMacro = true; try (fileIn @\"" + src + "\") catch (print (getCurrentException())); HCT_fromMacro = false))")
        )
        catch ()
    )
)
