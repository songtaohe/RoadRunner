
import numpy as np
import math
import sys
import scipy.ndimage
import scipy.misc
#import cv
#import cv2
#from PIL import Image
import pickle
import RoadGraph as splfy
import matplotlib.pyplot as plt
import socket
import code
import random

from rtree import index

#import RoadExploreTOPO as topo
#import TOPORender


def pointToLineDistance(p1,p2,p3):
	# p1 --> p2 is the line
	# p1 is (0,0)

	dist = np.sqrt(p2[0] * p2[0] + p2[1] * p2[1])

	proj_length = (p2[0] * p3[0] + p2[1] * p3[1]) / dist

	if proj_length > dist :
		a = p3[0] - p2[0]
		b = p3[1] - p2[1]

		return np.sqrt(a*a + b*b)

	if proj_length < 0 :
		a = p3[0] - p1[0]
		b = p3[1] - p1[1]

		return np.sqrt(a*a + b*b)


	alpha = proj_length / dist

	p4 = [0,0]

	p4[0] = alpha * p2[0]
	p4[1] = alpha * p2[1]

	a = p3[0] - p4[0]
	b = p3[1] - p4[1]

	return np.sqrt(a*a + b*b)

def pointToLineDistanceLatLon(p1,p2,p3):
	pp2 = [0,0]
	pp3 = [0,0]

	pp2[0] = p2[0] - p1[0]
	pp2[1] = (p2[1] - p1[1]) * math.cos(math.radians(p1[0]))

	pp3[0] = p3[0] - p1[0]
	pp3[1] = (p3[1] - p1[1]) * math.cos(math.radians(p1[0]))

	return pointToLineDistance((0,0), pp2, pp3)




dumpDat = pickle.load(open(sys.argv[1], "rb"))

RoadGraph = splfy.RoadGraph()
RoadGraph.region = dumpDat[0]


trees = dumpDat[1]
tree_id= 0
for treenodes in trees:
	for node in treenodes:
		#print(tree_id, node['id'])
		if node['similarWith'][0] != -1 and node['removed'] == False:
			tid = node['similarWith'][0]
			p = node['similarWith'][1]

			try:
				while trees[tid][p]['similarWith'][0] != -1:
					print(tid,p)
					if trees[tid][p]['similarWith'][1] == p and trees[tid][p]['similarWith'][0] == tid:
						break
					tid,p = trees[tid][p]['similarWith'][0],trees[tid][p]['similarWith'][1]
			except:
				print(tid,p)
				print(trees[tid][p]['similarWith'])

				exit()

			node['similarWith'][0] = tid
			node['similarWith'][1] = p

	tree_id += 1

def closeMergePoint(treenodes, trees, threshold = 0.00006):
	counter = 0

	node_candidate = []

	for node in treenodes:
		if node['similarWith'][0] != -1 and node['removed'] == False:
			node_candidate.append(node)

	for node in node_candidate:
		target_tree = node['similarWith'][0]
		target_id = node['similarWith'][1]

		path = []

		nodes = trees[target_tree]

		p = target_id
		if nodes[p]['removed'] == True:
			continue
		for i in range(8):
			lat = nodes[p]['lat']
			lon = nodes[p]['lon']
			path.append((lat, lon, p))


			if p == nodes[p]['parent']:
				break
			p = nodes[p]['parent']

		if len(path)!=8:
			continue


		nodes = treenodes

		this_id = node['id']
		old_p = this_id
		p = nodes[this_id]['parent']
		path2 = [this_id]

		for i in range(8):
			lat = nodes[p]['lat']
			lon = nodes[p]['lon']

			min_d = 10000
			pair = -1

			for j in range(7):
				d = pointToLineDistanceLatLon(path[j], path[j+1], (lat, lon))

				if d < min_d :
					min_d = d
					pair = path[j][2]


			if min_d < threshold:
				counter += 1
				nodes[p]['similarWith'] = [target_tree, pair]
				if nodes[old_p]['referenceCounter'] <= 1:
					nodes[old_p]['removed'] = True

			else:
				break

			old_p = p
			p = nodes[p]['parent']
			path2.append(p)




	print("Merged ", counter )




	for node in treenodes:
		if node['similarWith'][0] != -1 and node['removed'] == False:
			tid = node['similarWith'][0]
			p = node['similarWith'][1]


			loop_item = []

			while trees[tid][p]['similarWith'][0] != -1:

				print(trees[tid][p]['similarWith'][0],trees[tid][p]['similarWith'][1])
				if trees[tid][p]['similarWith'][1] == p and trees[tid][p]['similarWith'][0] == tid:
					break
				tid,p = trees[tid][p]['similarWith'][0],trees[tid][p]['similarWith'][1]

				if (tid,p) in loop_item:
					break
				else:
					loop_item.append((tid,p))
				node['similarWith'][0] = tid
				node['similarWith'][1] = p


			node['similarWith'][0] = tid
			node['similarWith'][1] = p




for nodetree in dumpDat[1]:
	closeMergePoint(nodetree, dumpDat[1], threshold = 0.00010)




for i in range(len(dumpDat[1])):
	for node in dumpDat[1][i]:
		if node['removed'] == True:
			continue

		id1 = node['id']
		lat1 = node['lat']
		lon1 = node['lon']

		oid1 = id1
		olat1 = lat1
		olon1 = lon1
		otid1 = i


		id2 = node['parent']
		lat2 = dumpDat[1][i][id2]['lat']
		lon2 = dumpDat[1][i][id2]['lon']

		# if i == 111 and id1 == 39:
		# 	print(id1, lat1,lon1, node['similarWith'])
		# 	print(id2, lat2,lon2, dumpDat[1][i][id2]['similarWith'])

		tid1 = i
		tid2 = i

		if node['similarWith'][0] != -1:
			tid1 = node['similarWith'][0]
			id1_ = node['similarWith'][1]
			lat1 = dumpDat[1][node['similarWith'][0]][id1_]['lat']
			lon1 = dumpDat[1][node['similarWith'][0]][id1_]['lon']
			id1 = id1_


		if dumpDat[1][i][id2]['similarWith'][0] != -1:
			tid2 = dumpDat[1][i][id2]['similarWith'][0]
			id2_ = dumpDat[1][i][id2]['similarWith'][1]
			lat2 = dumpDat[1][dumpDat[1][i][id2]['similarWith'][0]][id2_]['lat']
			lon2 = dumpDat[1][dumpDat[1][i][id2]['similarWith'][0]][id2_]['lon']
			id2 = id2_


		RoadGraph.addEdge(str(tid2)+"_"+str(id2), lat2, lon2, str(otid1)+"_"+str(oid1), olat1, olon1)

		if not (otid1 == tid1 and oid1 == id1):
			RoadGraph.addEdge(str(otid1)+"_"+str(oid1), olat1, olon1, str(tid1)+"_"+str(id1), lat1, lon1)




RoadGraph.ReverseDirectionLink()
#RoadGraph.region = region
#OSMRoadGraph.region = region

pickle.dump(RoadGraph, open(sys.argv[2],"wb"))

#TOPORender.RenderGraphSVG(OSMRoadGraph, RoadGraph, sys.argv[3])
