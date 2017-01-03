
des = [
	'Registration Error for the Apartment Y-Axis Rotation Data Set',
	'Registration Error for the Apartment X/Y-Axis Rotation Data Set',
	'Registration Error for the Boxes Y-Axis Rotation Data Set',
	'Registration Error for the Boxes Zoom Data Set',
	'Registration Error for the Desk Translation Data Set',
	'Registration Error for the Texture Confusion Indoor-Space Translation Data Set',
	'Registration Error for the Low-Texture Kitchen Translation Data Set',
	'Registration Error for the Low-Texture Kitchen Zoom Data Set',
	'Registration Error for the Office Translation Data Set',
	'Registration Error for the Office Centered Object Rotation Data Set',
	'Registration Error for the Office X/Y-Axis Rotation Data Set',
	'Registration Error for the Office Y-Axis Rotation Data Set',
	'Registration Error for the Office Translation Data Set',
	'Registration Error for the Little Texture Outdoors Rotation Data Set',
	'Registration Error for the Little Texture Outdoors Translation Data Set',
	'Registration Error for the Outdoors Texture Confusion Rotation Data Set',
	'Registration Error for the Outdoors Texture Confusion Translation Data Set',
	'Registration Error for the Outdoor Plants Texture Confusion Rotation Data Set',
	
	
]
c = 0

def toString(f):
	global c
	s =  '\\begin{figure*}[t]\n'
	s += '\\centering\n'
	s += '\\includegraphics[width=6.0in]{images/results/'+f+'}\n'
	s += '\\caption{'+des[c]+'}\n'
	s += '\\label{fig:PET'+str(c)+'}\n'
	s += '\\end{figure*}\n'
	c += 1
	return s
	
from os import listdir
for f in [i for i in listdir('.') if i.endswith('.pdf')]:
	print toString(f.replace('.pdf', ''))