
#get list of xlsl files (not tests)
def getFileNames():
	from os import listdir
	files = listdir('./')
	return [f for f in files if f.endswith('.csv') and 'v1.0' in f]
#get data
def getData(fileName):
	D = {}
	D['algorithms'] = {}
	f = open(fileName, 'r')
	lines = f.readlines()
	f.close()
	lines = [l.rstrip().split(',') for l in lines]
	#data-name,algorithm-name,description,frame1,frame2,error-added,seconds,mse,%match,hausdorff distance
	if len(lines) > 1:
		D["Data-Name"] = lines[1][0]
	for line in lines[1:]:
		alg = line[1]
		frame = line[3]
		hd = line[9]
		if alg not in D['algorithms']: D['algorithms'][alg] = []
		D['algorithms'][alg].append(float(hd))
	return D
	
#format data
def formatData(data):
	s = data['Data-Name'] + '\n'
	s += 'none, FM, FM-3D, ICP, PCA, PC, PC2, PC3\n'
	for y in range(len(data['algorithms']['none'])):
		line = []
		for x in 'none fm fm3d icp pca pc pc2 pc3'.split(' '):
			line.append(str(data['algorithms'][x][y]))
		s += ', '.join(line) + '\n'
	if s[-1] != '\n': s += '\n'
	return s + '\n'
#write to file
def writeOut(fileName, data):
	file = open(fileName, 'w')
	file.write(data)
	file.close()

def main():
	print '***************************'
	print '  converting application'
	print '***************************'
	output = ''
	for file in getFileNames():
		print file
		data = getData(file)
		data = formatData(data)
		output += data
		
	writeOut('pose-estimation Hausdorff distances.csv', output)

	
	
main()