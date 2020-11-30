import cv2 
import pickle
import sys  
import RoadGraph
import numpy as np 
import math 
import json 

roadgraph = pickle.load(open(sys.argv[1],"rb"))
outputfile = sys.argv[2]

output = {}
output["vertices"] = {}
output["edges"] = []

for nid, loc in roadgraph.nodes.items():
    output["vertices"][str(nid)] = [loc[0], loc[1]]

for edge in roadgraph.edges.values():
    n1, n2 = edge[0], edge[1]
    if n1 != n2:
        output["edges"].append((str(n1),str(n2)))

json.dump(output, open(outputfile, "w"), indent=2)




	
