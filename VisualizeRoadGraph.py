import cv2 
import pickle
import sys  
import RoadGraph
import numpy as np 
import math 

roadgraph = pickle.load(open(sys.argv[1],"rb"))
outputfile = sys.argv[2]

resolution = 1.0 # 1 pixel per meter 
margin = 0.1 # 10%

minlat = 100000
minlon = 100000
maxlat = -100000
maxlon = -100000

for loc in roadgraph.nodes.values():
	minlat = min(minlat, loc[0])
	maxlat = max(maxlat, loc[0])
	minlon = min(minlon, loc[1])
	maxlon = max(maxlon, loc[1])


m = (maxlat-minlat) * margin 
maxlat += m 
minlat -= m 

m = (maxlon-minlon) * margin 
maxlon += m 
minlon -= m 


width = int((maxlon-minlon) * 111111.0 * math.cos(math.radians(minlat)) / resolution)
height = int((maxlat-minlat) * 111111.0 / resolution)

print("width, height = ", width, height)

img = np.ones((height, width, 3), dtype=np.uint8) * 255

for edge in roadgraph.edges.values():
	n1, n2 = edge[0], edge[1]

	x1 = int((roadgraph.nodes[n1][1] - minlon) / (maxlon-minlon) * width)
	x2 = int((roadgraph.nodes[n2][1] - minlon) / (maxlon-minlon) * width)
	
	y1 = int((1.0 - (roadgraph.nodes[n1][0] - minlat) / (maxlat-minlat)) * height)
	y2 = int((1.0 - (roadgraph.nodes[n2][0] - minlat) / (maxlat-minlat)) * height)

	#print(x1,x2,y1,y2)
	cv2.line(img, (x1,y1), (x2,y2), (0,0,0), 3)

for loc in roadgraph.nodes.values():
	x = int((loc[1] - minlon) / (maxlon-minlon) * width)
	y = int((1.0 - (loc[0] - minlat) / (maxlat-minlat)) * height)

	cv2.circle(img, (x,y), 3, (0,0,255), -1)

cv2.imwrite(outputfile, img)



	