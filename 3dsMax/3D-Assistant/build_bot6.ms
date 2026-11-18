/*
	build_bot6.ms -- разовая подготовка Source/Bot6/Bot6.max под ASSET_RULES (2026-11-18).
	Запуск (фоном, открытая у художника сцена не трогается):
		3dsmaxbatch.exe Tools/build_bot6.ms -sceneFile Source/Bot6/Bot6.max
	Исходная сцена художника: E:/.../npc/bot#6/bot#6.max (модель Meshy AI «Twin Ring Battle Drone», 1 132 358 тр.).

	1. bot#6_HP -> HP_Bot6 (исходник для запечки, слой Source_HP, скрыт, в экспорт не идёт).
	   Текстуры HP перепривязаны к копии в Source/Bot6/Meshy/ (репозиторий самодостаточен).
	2. Прежний low-poly 36 698 тр. удалён: проп-лимит правил -- 15 000 тр. на меш (G02).
	   Он остаётся в исходной сцене художника.
	3. SM_Bot6: ProOptimizer с HP до ~14 500 тр., сварка, обломки < 20 граней, закрытие дыр, сглаживание 45°
	   (Tools/lowpoly_bake.ms). UV и запечка -- отдельно в интерактивном Max (Unwrap не работает без UI).
	4. Слой Export = SM_Bot6.
*/
fileIn ((pathConfig.removePathLeaf (getFilenamePath (getThisScriptFilename()))) + "/Tools/lowpoly_bake.ms")
(
	local repo = pathConfig.removePathLeaf (getFilenamePath (getThisScriptFilename()))
	local meshyDir = repo + "/Source/Bot6/Meshy/"
	local logDir = repo + "/Export/Bot6/_reports"
	makeDir logDir all:true
	local out = createFile (logDir + "/build_bot6_" + ((dotNetClass "System.DateTime").Now.ToString "yyyy-MM-dd_HHmmss") + ".txt") encoding:#utf8
	format "build_bot6.ms  %\nscene: %\n\n" ((dotNetClass "System.DateTime").Now.ToString "yyyy-MM-dd HH:mm:ss") (maxFilePath + maxFileName) to:out

	-- 1. исходник
	local hp = getNodeByName "bot#6_HP"
	if hp == undefined do hp = getNodeByName "HP_Bot6"
	hp.name = "HP_Bot6"
	for b in (getClassInstances BitmapTexture target:hp.material) do
	(
		local f = filenameFromPath b.filename
		local cands = #(meshyDir + f, meshyDir + "Meshy_AI_Twin_Ring_Battle_Drone_20261118_texture.fbm/" + f)
		for c in cands where doesFileExist c do (b.filename = c)
		format "HP texture: %  exists: %\n" b.filename (doesFileExist b.filename) to:out
	)
	format "HP_Bot6: % tris\n" (LB_tris hp) to:out

	-- 2. прежний low-poly
	for n in #("bot#6", "bot6_LP37k_backup") do
	(
		local o = getNodeByName n
		if o != undefined do (format "deleted old low-poly '%': % tris\n" n (LB_tris o) to:out; delete o)
	)
	local old = getNodeByName "SM_Bot6"
	if old != undefined do delete old

	-- 3. новый low-poly
	local t0 = timestamp()
	local lp = LB_makeLowPoly hp "SM_Bot6" 14500
	local cl = LB_clean lp
	format "SM_Bot6: % tris, open edges %, non-manifold %, % s\n" (LB_tris lp) cl[1] cl[2] ((timestamp() - t0) / 1000) to:out

	-- 4. слои
	local le = LayerManager.getLayerFromName "Export"
	if le == undefined do le = LayerManager.newLayerFromName "Export"
	le.addNode lp
	local ls = LayerManager.getLayerFromName "Source_HP"
	if ls == undefined do ls = LayerManager.newLayerFromName "Source_HP"
	ls.addNode hp
	hide hp

	saveMaxFile (maxFilePath + maxFileName) quiet:true
	format "\nsaved: %\nobjects: %\n" (maxFilePath + maxFileName) (for o in objects collect o.name) to:out
	close out
)
