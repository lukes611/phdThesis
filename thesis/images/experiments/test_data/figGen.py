

files = [
	"Apartment.Texture.rotate",
        "Apartment.Texture.rotateXAxis",
        "Boxes.Texture.arbitrarycamera",
        "Boxes.Texture.rotate",
        "Boxes.Texture.zoomOut",
        "Desk.Texture.Translation",
        "IndoorSpace.tc.translation",
        "Kitchen.littleTexture.pan",
        "Kitchen.littleTexture.zoom",
        "OfficeDesk.Texture.rotationLift",
        "Office.Texture.blindSpotRotation",
        "Office.TexturedItems.Translation",
        "Office.Texture.rotation",
        "Office.Texture.rotationXAxis",
        "Office.Texture.Translation",
        "Outside.NoTexture.rotation",
        "Outside.NoTexture.translation",
        "Outside.TextureConfusion.rotation",
        "Outside.TextureConfusion.Translation",
        "PlantsOutdoors.tc.rotation"
]

for j,f in enumerate(files):
	print '\\begin{figure*}[t]'
	print '\\centering'
	for i,x in enumerate([1,10,15,20]):
		print '\\begin{subfigure}[b]{1.5in}'
		print '\\includegraphics[width=1.5in]{{images/experiments/test_data/'+f+'.'+str(i)+'}.png}'
		print '\\caption{Frame '+str(x)+'}'
		print '\\end{subfigure}%'
	print '\\caption{'+f+' Scene.}'
	print '\\label{fig:'+f.replace('.','_')+'}'
	print '\\end{figure*}'
	print '\n\n'
	if j > 0 and j % 3 == 0:
		print '\\clearpage\n\n'