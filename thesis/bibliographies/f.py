
import os

while True:
	x = raw_input('find: ')
	if x == 'clear' or x == 'cls':
		os.system('clear')
		continue
	if x in 'quit q exit'.split(' '): break
	cmd = 'grep %s -n -i slam.bib' % (x)
	os.system(cmd)
print 'bye'


