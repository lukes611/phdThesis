import os, sys

'''
use cases:

ls
ls [directoryName]
ls -f [directorName] # only include files
ls -d [directorName] # only include directorys
ls -t type [directoryName]

'''

def parse(argv, flag):
	if flag in argv:
		ind = argv.index(flag)
		return True, argv[:ind] + argv[ind+1:]
	return False, argv

def parseArg(argv, flag):
	if flag in argv:
		ind = argv.index(flag)
		nind = ind + 1
		if nind >= len(argv):
			return False, None, argv[:ind] + argv[ind+1:]
		return True, argv[nind], argv[:ind] + argv[nind+1:]
	return False, None, argv

	
argv = sys.argv[1:]

hasF, argv = parse(argv, '-f')
hasD, argv = parse(argv, '-d')
hasT, Type, argv = parseArg(argv, '-t')
_dir = '.'
if len(argv) >= 1:
	_dir = argv[0]

if _dir.endswith('/'):
	_dir = _dir[:-1]
	
output = []
for i in os.listdir(_dir):
	if hasF:
		if not os.path.isfile(_dir + '/' + i):
			continue
	if hasD:
		if not os.path.isdir(_dir + '/' + i):
			continue
	if hasT:
		if not i.endswith(Type):
			continue
	output.append(i)
	
	
print ','.join(output),