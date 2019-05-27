
import numpy as np
import math
import sys
import scipy.ndimage
import scipy.misc
#import cv
import cv2
from PIL import Image
import pickle
import bintrees 
from PathSimilarity import PathSimilarity
from rtree import index
from scipy import interpolate
import socket
from multiprocessing import Process
from multiprocessing.pool import Pool
import zlib
from geojson import LineString, Feature, FeatureCollection
import geojson 


# This is not needed
# import os
# sys.path.append(os.path.dirname(sys.path[0])+"/MLInputGen/")
# import mapdriver as md 



def GetClassifierResult(img, host="localhost", port=8001):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))

    data_string = pickle.dumps(img)

    data_string = zlib.compress(data_string, 6)

    print(len(data_string))
    #print("Compress", len(zlib.compress(data_string, 6)))


    data_length = pickle.dumps(len(data_string))
    s.send(data_length)

    ack = s.recv(4096)

    s.send(data_string)

    result_string = s.recv(4096)
    result = pickle.loads(result_string)

    return result



def Coord2Pixels(lat, lon, min_lat, min_lon, max_lat, max_lon, sizex, sizey):
	#print(max_lat, min_lat, sizex)
	ilat = sizex - int((lat-min_lat) / ((max_lat - min_lat)/sizex))
	#ilat = int((lat-min_lat) / ((max_lat - min_lat)/sizex))
	ilon = int((lon-min_lon) / ((max_lon - min_lon)/sizey))

	return ilat, ilon



def TraceQueryBatch(data, host="localhost", port=8006):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    data_string = ""
    for i in range(len(data)/5):
    	data_string = data_string + str(data[i*5+0])+","+str(data[i*5+1])+","+str(data[i*5+2])+","+str(data[i*5+3])+","+str(data[i*5+4])+","


    #print("DataSentLen", len(data_string))

    s.send(data_string)
    result = s.recv(16384)
    items = result.split()
    result = [int(item) for item in items]
    #print(result)

    return result

def TraceQueryBatch3P(data, host="localhost", port=8002):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    data_string = ""
    for i in range(len(data)/9):
    	data_string = data_string + str(data[i*9+0])+","+str(data[i*9+1])+","+str(data[i*9+2])+","+str(data[i*9+3])+","+str(data[i*9+4])+","+str(data[i*9+5])+","+str(data[i*9+6])+","+str(data[i*9+7])+","+str(data[i*9+8])+",-1,-1,"


    #print("DataSentLen", len(data_string))

    s.send(data_string)
    result = s.recv(16384)
    items = result.split()
    result = [int(item) for item in items]
    #print(result)

    return result

def SpeedQueryBatch(data, host="localhost", port=8002):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    data_string = "S,"
    for i in range(len(data)/4):
    	data_string = data_string + str(data[i*4+0])+","+str(data[i*4+1])+","+str(data[i*4+2])+","+str(data[i*4+3])+","

    #print("DataSentLen", len(data_string))

    s.send(data_string)
    result = s.recv(16384)
    items = result.split()
    result = [float(item) for item in items]
    #print(result)

    return result


def distance(p1, p2):
	a = p1[0] - p2[0]
	b = (p1[1] - p2[1])*math.cos(math.radians(p1[0]))
	return np.sqrt(a*a + b*b)


def pointDistance(p1, p2):
	a = p1[0] - p2[0]
	b = p1[1] - p2[1]
	return np.sqrt(a*a + b*b)



def latlonNorm(p1, lat = 40):

	p11 = p1[1] * math.cos(math.radians(lat))

	l = np.sqrt(p11 * p11 + p1[0] * p1[0])

	return p1[0]/l, p11/l



def simplfyWithShortestPathBatchStatic(self, sourceIDs, state = 1, idx = 0):

		print(idx, sourceIDs)
		results = []
		c = 0
		for sourceID in sourceIDs:
			print(state, idx, c, len(sourceIDs))
			results.append(simplfyWithShortestPathStatic(self, sourceID, state = state))
			c = c + 1

		return results 


def simplfyWithShortestPathStatic(self, sourceID, state=1, deferred_apply = False):
		self.nodeDistance = {}
		self.nodeFrom = {}
		self.nodeSettle = {}


		result = {}

		result['edgePairs'] = {}

		result['locations'] = {} 


		tree = bintrees.BinaryTree()

		for nid in self.nodes.keys():
			self.nodeDistance[nid] = 10000000000
			self.nodeFrom[nid] = -1
			self.nodeSettle[nid] = 0


		tree[sourceID] = 0
		self.nodeDistance[sourceID] = 0






		#for i in range(len(self.nodes.keys())):
		while True:
			dmin = 10000000
			dminid = -1

			# for j in self.nodes.keys():
			# 	if self.nodeDistance[j] < dmin and self.nodeSettle[j] == 0:
			# 		dmin = self.nodeDistance[j]
			# 		dminid = j

			# if dminid == -1 :
			# 	break
			if len(tree) == 0:
				break

			dminid = tree.min_key()
			dmin = tree[dminid]

			tree.remove_items([dminid])
			#print(dminid, dmin)
			#self.nodeSettle[dminid] = 1
			for nextNode in self.nodeLink[dminid]:
				lat1 = self.nodes[dminid][0]
				lon1 = self.nodes[dminid][1]

				lat2 = self.nodes[nextNode][0]
				lon2 = self.nodes[nextNode][1]


				a = lat1 - lat2
				b = (lon1 - lon2) /  math.cos(math.radians(lat1))

				dist = np.sqrt(a*a + b*b)

				if self.nodeDistance[nextNode] > dmin + dist:
					self.nodeDistance[nextNode] = dmin + dist
					self.nodeFrom[nextNode] = dminid
					tree[nextNode] = dmin + dist



		print("Tree Size", len(tree))

		if state == 1:
			#for i in range(len(self.nodes.keys())):
			for idthis in self.nodeTerminate.keys():
				i = self.nodeHash[idthis]
				if self.nodeFrom[i] != -1:
					cur = i
					path = []
					while True:
						path.append(cur)
						p = self.nodeFrom[cur]

						if (p, cur) not in result['edgePairs'].keys():
							result['edgePairs'][(p,cur)] = 1
						else:
							result['edgePairs'][(p,cur)] += 1


						if deferred_apply  == False:
							self.edgeScore[self.edgeHash[p*10000000+cur]] += 1

							self.nodeScore[p] = max(self.nodeScore[p], self.edgeScore[self.edgeHash[p*10000000+cur]])
							self.nodeScore[cur] = max(self.nodeScore[cur], self.edgeScore[self.edgeHash[p*10000000+cur]])


						cur = p

						if cur == sourceID:
							break

					path.reverse() # From root...

					# for j in range(2, len(path)-1, 20):
					# 	self.nodeLocations[path[j]] = 1

					for j in range(len(path)):
						if path[j] % 20 == 0:
							if deferred_apply  == False:
								self.nodeLocations[path[j]] = 1
							result['locations'][path[j]] = 1

					if deferred_apply  == False:
						self.nodeLocations[self.nodeHash[idthis]] = 1
					result['locations'][self.nodeHash[idthis]] = 1

			if deferred_apply  == False:
				self.nodeLocations[sourceID] = 1
			result['locations'][sourceID] = 1



		if state == 2:
			for idthis in self.nodeLocations.keys():
				i = idthis
				if self.nodeFrom[i] != -1:
					cur = i
					path = []
					while True:
						path.append(cur)
						p = self.nodeFrom[cur]

						if (p, cur) not in result['edgePairs'].keys():
							result['edgePairs'][(p,cur)] = 1
						else:
							result['edgePairs'][(p,cur)] += 1


						if deferred_apply  == False:
							self.edgeScore[self.edgeHash[p*10000000+cur]] += 1

							self.nodeScore[p] = max(self.nodeScore[p], self.edgeScore[self.edgeHash[p*10000000+cur]])
							self.nodeScore[cur] = max(self.nodeScore[cur], self.edgeScore[self.edgeHash[p*10000000+cur]])


						cur = p

						if cur == sourceID:
							break

		return result









class RoadGraph:
	def __init__(self, filename=None, skip_threshold = 0.0):
		self.nodeHash = {} # [tree_idx*10000000 + local_id] ->  id 
		self.nodeHashReverse = {} 
		self.nodes = {}	# id -> [lat,lon]
		self.edges = {} # id -> [n1, n2]
		self.nodeLink = {}   # id -> list of next node
		self.nodeID = 0 
		self.edgeID = 0
		self.edgeHash = {} # [nid1 * 10000000 + nid2] -> edge id 
		self.edgeScore = {}
		self.nodeTerminate = {}
		self.nodeScore = {}
		self.nodeLocations = {}
		self.edgeInt = {}
		self.deletedNodes = {}
		

		if filename is not None:

			dumpDat = pickle.load(open(filename, "rb"))

			forest = dumpDat[1]
			region = dumpDat[0]
			self.region = region

			#self.forest = forest

			self.NumOfTrees = len(forest)
			self.forest = [1 for _ in range(self.NumOfTrees)] 

			tid = 0
			for t in forest:
				for n in t:
					idthis = tid*10000000 + n['id']

					thislat = n['lat']
					thislon = n['lon']

					if region is not None:
						if thislat < region[0] or thislon < region[1] or thislat > region[2] or thislon > region[3]:
							continue

					if n['edgeScore'] < skip_threshold : # skip those low confidential edges
					
						continue

					if n['similarWith'][0] != -1:
						idthis = n['similarWith'][0]*10000000 + n['similarWith'][1]

						thislat = forest[n['similarWith'][0]][n['similarWith'][1]]['lat']
						thislon = forest[n['similarWith'][0]][n['similarWith'][1]]['lon']

						

					if n['OutRegion'] == 1:
						self.nodeTerminate[tid*10000000+n['parent']] = 1

					if n['isExited'] == True:
						self.nodeTerminate[tid*10000000+n['parent']] = 1



					idparent = tid*10000000 + n['parent']
					parentlat = t[n['parent']]['lat']
					parentlon = t[n['parent']]['lon']

					if n['parent'] == 0:
						print(tid, n['id'])


					self.addEdge(idparent, parentlat, parentlon, idthis, thislat, thislon)



				tid += 1


		self.ReverseDirectionLink()
		



	def addEdge(self, nid1,lat1,lon1,nid2,lat2,lon2, reverse=False, nodeScore1 = 0, nodeScore2 = 0, edgeScore = 0):  #n1d1->n1d2
		

		if nid1 not in self.nodeHash.keys():
			self.nodeHash[nid1] = self.nodeID
			self.nodeHashReverse[self.nodeID] = nid1
			self.nodes[self.nodeID] = [lat1, lon1]
			self.nodeLink[self.nodeID] = []
			#self.nodeLinkReverse[self.nodeID] = []
			self.nodeScore[self.nodeID] = nodeScore1
			self.nodeID += 1

		if nid2 not in self.nodeHash.keys():
			self.nodeHash[nid2] = self.nodeID
			self.nodeHashReverse[self.nodeID] = nid2
			self.nodes[self.nodeID] = [lat2, lon2]
			self.nodeLink[self.nodeID] = []
			#self.nodeLinkReverse[self.nodeID] = []
			self.nodeScore[self.nodeID] = nodeScore2
			self.nodeID += 1

		localid1 = self.nodeHash[nid1]
		localid2 = self.nodeHash[nid2]

		if localid1 * 10000000 + localid2 in self.edgeHash.keys():
			print("Duplicated Edge !!!", nid1, nid2)

			return self.edgeHash[localid1 * 10000000 + localid2]

		self.edges[self.edgeID] = [localid1, localid2]
		self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
		self.edgeScore[self.edgeID] = edgeScore
		self.edgeID += 1

		if localid2 not in self.nodeLink[localid1]:
			self.nodeLink[localid1].append(localid2)

		if reverse == True:
			if localid2 not in self.nodeLinkReverse.keys():
				self.nodeLinkReverse[localid2] = []

			if localid1 not in self.nodeLinkReverse[localid2]:
				self.nodeLinkReverse[localid2].append(localid1)

		return self.edgeID - 1


	def addEdgeToOneExistedNode(self, nid1,lat1,lon1,nid2, reverse=False, nodeScore1 = 0, edgeScore = 0):  #n1d1->n1d2
		

		if nid1 not in self.nodeHash.keys():
			self.nodeHash[nid1] = self.nodeID
			self.nodeHashReverse[self.nodeID] = nid1
			self.nodes[self.nodeID] = [lat1, lon1]
			self.nodeLink[self.nodeID] = []
			self.nodeLinkReverse[self.nodeID] = []
			self.nodeScore[self.nodeID] = nodeScore1
			self.nodeID += 1

		localid1 = self.nodeHash[nid1]
		localid2 = nid2

		self.edges[self.edgeID] = [localid1, localid2]
		self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
		self.edgeScore[self.edgeID] = edgeScore
		self.edgeID += 1

		if localid2 not in self.nodeLink[localid1]:
			self.nodeLink[localid1].append(localid2)

		if localid1 not in self.nodeLinkReverse[localid2]:
			self.nodeLinkReverse[localid2].append(localid1)


	def connectTwoNodes(self, n1, n2):
		lat1 = self.nodes[n1][0]
		lon1 = self.nodes[n1][1]

		lat2 = self.nodes[n2][0]
		lon2 = self.nodes[n2][1]

		nn1 = self.nodeHashReverse[n1]
		nn2 = self.nodeHashReverse[n2]


		return self.addEdge(nn1, lat1, lon1, nn2, lat2, lon2)




	def applyShortestPathResult(self, edgePairs, location):
		for ep in edgePairs.keys():
			p = ep[0]
			cur = ep[1]

			self.edgeScore[self.edgeHash[p*10000000+cur]] += 1

			self.nodeScore[p] = max(self.nodeScore[p], self.edgeScore[self.edgeHash[p*10000000+cur]])
			self.nodeScore[cur] = max(self.nodeScore[cur], self.edgeScore[self.edgeHash[p*10000000+cur]])


		for loc in location.keys():
			self.nodeLocations[loc] = 1



	def simplfyWithShortestPathParallel(self, ProcessNum = 12):
		
		pool = Pool(ProcessNum)

		sourceIDlist = [[] for i in range(ProcessNum)]

		c = 0
		print(len(self.forest))
		for i in range(len(self.forest)):
			#print(i,"/", len(roadMap.forest))
			if i*10000000 in self.nodeHash.keys():
				#print(i)
				sourceIDlist[c].append(self.nodeHash[i*10000000])

				c = c + 1
				if c >= ProcessNum:
					c = 0




		args = []
		for i in range(ProcessNum):
			args.append((self, sourceIDlist[i], 1, i))

		print(args)

		result = []


		for i in range(ProcessNum):
			result.append(pool.apply_async(simplfyWithShortestPathBatchStatic, args=args[i]))


		for i in range(ProcessNum):
			r = result[i].get()

			for rr in r:
				self.applyShortestPathResult(rr['edgePairs'], rr['locations'])






		sourceIDlist = [[] for i in range(ProcessNum)]

		cc = 0
		for i in self.nodeLocations:
			sourceIDlist[cc].append(i)

			cc = cc + 1
			if cc >= ProcessNum:
				cc = 0


		args = []
		for i in range(ProcessNum):
			args.append((self, sourceIDlist[i], 2, i))


		result = []


		for i in range(ProcessNum):
			result.append(pool.apply_async(simplfyWithShortestPathBatchStatic, args=args[i]))


		for i in range(ProcessNum):
			r = result[i].get()

			for rr in r:
				self.applyShortestPathResult(rr['edgePairs'], rr['locations'])







			


	#@staticmethod
	def simplfyWithShortestPathBatch(self, sourceIDs, state = 1, idx = 0):

		results = []
		c = 0
		for sourceID in sourceIDs:
			print(state, idx, c, len(sourceIDs))
			results.append(self.simplfyWithShortestPath(sourceID, state = state))
			c = c + 1

		return results 





	#@staticmethod
	def simplfyWithShortestPath(self, sourceID, state=1, deferred_apply = False):
		self.nodeDistance = {}
		self.nodeFrom = {}
		self.nodeSettle = {}


		result = {}

		result['edgePairs'] = {}

		result['locations'] = {} 


		tree = bintrees.BinaryTree()

		for nid in self.nodes.keys():
			self.nodeDistance[nid] = 10000000000
			self.nodeFrom[nid] = -1
			self.nodeSettle[nid] = 0


		tree[sourceID] = 0
		self.nodeDistance[sourceID] = 0






		#for i in range(len(self.nodes.keys())):
		while True:
			dmin = 10000000
			dminid = -1

			# for j in self.nodes.keys():
			# 	if self.nodeDistance[j] < dmin and self.nodeSettle[j] == 0:
			# 		dmin = self.nodeDistance[j]
			# 		dminid = j

			# if dminid == -1 :
			# 	break
			if len(tree) == 0:
				break

			dminid = tree.min_key()
			dmin = tree[dminid]

			tree.remove_items([dminid])



			#print(dminid, dmin)


			#self.nodeSettle[dminid] = 1


			for nextNode in self.nodeLink[dminid]:
				lat1 = self.nodes[dminid][0]
				lon1 = self.nodes[dminid][1]

				lat2 = self.nodes[nextNode][0]
				lon2 = self.nodes[nextNode][1]


				a = lat1 - lat2
				b = (lon1 - lon2) /  math.cos(math.radians(lat1))

				dist = np.sqrt(a*a + b*b)


				if self.nodeDistance[nextNode] > dmin + dist:
					self.nodeDistance[nextNode] = dmin + dist
					self.nodeFrom[nextNode] = dminid
					tree[nextNode] = dmin + dist

		print("Tree Size", len(tree))

		if state == 1:
			#for i in range(len(self.nodes.keys())):
			for idthis in self.nodeTerminate.keys():
				i = self.nodeHash[idthis]
				if self.nodeFrom[i] != -1:
					cur = i
					path = []
					while True:
						path.append(cur)
						p = self.nodeFrom[cur]

						if (p, cur) not in result['edgePairs'].keys():
							result['edgePairs'][(p,cur)] = 1
						else:
							result['edgePairs'][(p,cur)] += 1


						if deferred_apply  == False:
							self.edgeScore[self.edgeHash[p*10000000+cur]] += 1

							self.nodeScore[p] = max(self.nodeScore[p], self.edgeScore[self.edgeHash[p*10000000+cur]])
							self.nodeScore[cur] = max(self.nodeScore[cur], self.edgeScore[self.edgeHash[p*10000000+cur]])


						cur = p

						if cur == sourceID:
							break

					path.reverse() # From root...

					# for j in range(2, len(path)-1, 20):
					# 	self.nodeLocations[path[j]] = 1

					for j in range(len(path)):
						if path[j] % 20 == 0:
							if deferred_apply  == False:
								self.nodeLocations[path[j]] = 1
							result['locations'][path[j]] = 1

					if deferred_apply  == False:
						self.nodeLocations[self.nodeHash[idthis]] = 1
					result['locations'][self.nodeHash[idthis]] = 1

			if deferred_apply  == False:
				self.nodeLocations[sourceID] = 1
			result['locations'][sourceID] = 1



		if state == 2:
			for idthis in self.nodeLocations.keys():
				i = idthis
				if self.nodeFrom[i] != -1:
					cur = i
					path = []
					while True:
						path.append(cur)
						p = self.nodeFrom[cur]

						if (p, cur) not in result['edgePairs'].keys():
							result['edgePairs'][(p,cur)] = 1
						else:
							result['edgePairs'][(p,cur)] += 1


						if deferred_apply  == False:
							self.edgeScore[self.edgeHash[p*10000000+cur]] += 1

							self.nodeScore[p] = max(self.nodeScore[p], self.edgeScore[self.edgeHash[p*10000000+cur]])
							self.nodeScore[cur] = max(self.nodeScore[cur], self.edgeScore[self.edgeHash[p*10000000+cur]])


						cur = p

						if cur == sourceID:
							break

		return result



	def mergeTwoNodesBiDirection(self, id1, id2): #remove id2
		if id1 == id2:
			return 

		print("id1, id2", id1, id2)

		print(self.nodeLink[id2])
		for next_node in self.nodeLink[id2]:

			# remove links
			print(next_node, id2, self.nodeLink[next_node], self.nodeLink[id1])
			self.nodeLink[next_node].remove(id2)

			edgeId = self.edgeHash[next_node * 10000000 + id2]
			del self.edges[edgeId]

			edgeId = self.edgeHash[id2 * 10000000 + next_node]
			del self.edges[edgeId]

			#print("Deleted ", self.deletedNodes.keys())
			# Add to id1's nodeLink list
			if (next_node not in self.nodeLink[id1]) and (next_node not in self.deletedNodes.keys()) and (next_node != id1):
				self.nodeLink[id1].append(next_node)
				print(id1,"+",next_node)

				localid1 = id1
				localid2 = next_node

				self.edges[self.edgeID] = [localid1, localid2]
				self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
				self.edgeScore[self.edgeID] = 10
				self.edgeID += 1

			# Add id1 to next node's nodeLink
			if id1 not in self.nodeLink[next_node] and next_node != id1:
				self.nodeLink[next_node].append(id1)

				localid1 = next_node
				localid2 = id1

				self.edges[self.edgeID] = [localid1, localid2]
				self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
				self.edgeScore[self.edgeID] = 10
				self.edgeID += 1




		pass

	def mergeTwoNodes(self, id1, id2): #remove id2
		if id1 == id2:
			return 

		self.nodes[id1][0] = (self.nodes[id1][0] + self.nodes[id2][0]) / 2
		self.nodes[id1][1] = (self.nodes[id1][1] + self.nodes[id2][1]) / 2


		print("id1, id2", id1, id2)

		print(self.nodeLink[id2])


		for next_node in self.nodeLink[id2]:
			self.nodeLinkReverse[next_node].remove(id2)

			edgeId = self.edgeHash[id2 * 10000000 + next_node]
			del self.edges[edgeId]

			if (next_node not in self.nodeLink[id1]) and (next_node not in self.deletedNodes.keys()) and (next_node != id1):
				self.nodeLink[id1].append(next_node)

				localid1 = id1
				localid2 = next_node

				self.edges[self.edgeID] = [localid1, localid2] 
				self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
				self.edgeScore[self.edgeID] = 10
				self.edgeID += 1


			if id1 not in self.nodeLinkReverse[next_node] and next_node != id1:
				self.nodeLinkReverse[next_node].append(id1)


		for next_node in self.nodeLinkReverse[id2]:
			self.nodeLink[next_node].remove(id2)

			edgeId = self.edgeHash[next_node * 10000000 + id2]
			del self.edges[edgeId]

			if (next_node not in self.nodeLinkReverse[id1]) and (next_node not in self.deletedNodes.keys()) and (next_node != id1):
				self.nodeLinkReverse[id1].append(next_node)

				localid1 = next_node
				localid2 = id1

				self.edges[self.edgeID] = [localid1, localid2]
				self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
				self.edgeScore[self.edgeID] = 10
				self.edgeID += 1

			if id1 not in self.nodeLink[next_node] and next_node != id1:
				self.nodeLink[next_node].append(id1)

		pass

	def mergeTwoNodesReserveEdge(self, id1, id2): #remove id2
		if id1 == id2:
			return 

		self.nodes[id1][0] = (self.nodes[id1][0] + self.nodes[id2][0]) / 2
		self.nodes[id1][1] = (self.nodes[id1][1] + self.nodes[id2][1]) / 2


		print("id1, id2", id1, id2)

		print(self.nodeLink[id2])


		for next_node in self.nodeLink[id2]:
			self.nodeLinkReverse[next_node].remove(id2)

			edgeId = self.edgeHash[id2 * 10000000 + next_node]
			self.edges[edgeId][0] = id1
			self.edgeHash[id1 * 10000000 + next_node] = edgeId
			del self.edgeHash[id2 * 10000000 + next_node]


			if (next_node not in self.nodeLink[id1]) and (next_node not in self.deletedNodes.keys()) and (next_node != id1):
				self.nodeLink[id1].append(next_node)

			if id1 not in self.nodeLinkReverse[next_node] and next_node != id1:
				self.nodeLinkReverse[next_node].append(id1)


		for next_node in self.nodeLinkReverse[id2]:
			self.nodeLink[next_node].remove(id2)

			edgeId = self.edgeHash[next_node * 10000000 + id2]
			self.edges[edgeId][1] = id1
			self.edgeHash[next_node * 10000000 + id1] = edgeId
			del self.edgeHash[next_node * 10000000 + id2]


			if (next_node not in self.nodeLinkReverse[id1]) and (next_node not in self.deletedNodes.keys()) and (next_node != id1):
				self.nodeLinkReverse[id1].append(next_node)

			if id1 not in self.nodeLink[next_node] and next_node != id1:
				self.nodeLink[next_node].append(id1)

		pass

	def removeDuplicateEdges(self):
		edges = {}
		c = 0
		for edgeID in self.edges.keys():
			if (self.edges[edgeID][0],self.edges[edgeID][1]) in edges.keys():
				del self.edges[edgeID]
				c += 1
			else:
				edges[(self.edges[edgeID][0],self.edges[edgeID][1])] = edgeID

		for edge, edgeid in edges.iteritems():
			self.edgeHash[edge[0] * 10000000 + edge[1]] = edgeid 

		print("Remove", c, "Duplicated Edges")



	def getPath(self, id1, length = 6, limit = 40):

		paths = []
		path_tmp = list(range(length))

		def searchPath(idthis, depth, pid = -1):
			#global paths 
			#global path_tmp 
			#global length

			if len(paths) > limit:
				return 

			path_tmp[length - depth] = idthis

			if depth == 1:
				p = map(lambda x:[self.nodes[x][0], self.nodes[x][1], x], path_tmp)
				paths.append(p)
				return 

			for next_node in self.nodeLink[idthis] :
				ok = True


				if next_node == pid :
					continue

				if next_node not in self.nodeScore.keys():
					continue

				if self.nodeScore[next_node] < 1 :
					continue

				if self.edgeScore[self.edgeHash[idthis * 10000000 + next_node]] < 1:
					continue

				searchPath(next_node, depth - 1, pid = idthis)

			pass



		searchPath(id1, length)

		return paths



	def BiDirection(self):
		edgeList = list(self.edges.values())

		for edge in edgeList:
			localid1 = edge[1]
			localid2 = edge[0]

			self.edges[self.edgeID] = [localid1, localid2]
			self.edgeHash[localid1 * 10000000 + localid2] = self.edgeID
			self.edgeScore[self.edgeID] = self.edgeScore[self.edgeHash[localid2 * 10000000 + localid1]]
			self.edgeID += 1

			if localid2 not in self.nodeLink[localid1]:
				self.nodeLink[localid1].append(localid2)

	def ReverseDirectionLink(self):
		edgeList = list(self.edges.values())

		self.nodeLinkReverse = {}

		for edge in edgeList:
			localid1 = edge[1]
			localid2 = edge[0]

			if localid1 not in self.nodeLinkReverse :
				self.nodeLinkReverse[localid1] = [localid2]
			else:
				if localid2 not in self.nodeLinkReverse[localid1]:
					self.nodeLinkReverse[localid1].append(localid2)

		for nodeId in self.nodes.keys():
			if nodeId not in self.nodeLinkReverse.keys():
				self.nodeLinkReverse[nodeId] = []



	def CombineSimilarSnippets(self, region = None, oneround = False, score_threshold = 0):
		idx = index.Index()
		for idthis in self.nodes.keys():
			if idthis in self.nodeScore.keys():
				if self.nodeScore[idthis] > score_threshold:
					if region == None:
						idx.insert(idthis, (self.nodes[idthis][0], self.nodes[idthis][1],self.nodes[idthis][0]+0.000001, self.nodes[idthis][1]+0.000001))
					elif self.nodes[idthis][0] > region[0] and self.nodes[idthis][1] > region[1] and self.nodes[idthis][0] < region[2] and self.nodes[idthis][1] < region[3]:
						idx.insert(idthis, (self.nodes[idthis][0], self.nodes[idthis][1],self.nodes[idthis][0]+0.000001, self.nodes[idthis][1]+0.000001))

		#self.deletedNodes = {}

		while True:

			update_counter = 0

			for idthis in self.nodes.keys():
				if idthis in self.deletedNodes.keys():
					continue

				if idthis in self.nodeScore.keys():
					if self.nodeScore[idthis] > score_threshold:
						

						lat = self.nodes[idthis][0]
						lon = self.nodes[idthis][1]

						if region is not None:
							if lat < region[0] or lon < region[1] or lat > region[2] or lon > region[3]:
								continue



						r = 0.00015
						possible_nodes = list(idx.intersection((lat-r,lon-r, lat+r, lon+r)))

						paths_this = self.getPath(idthis)

						best_p1 = []
						best_p2 = []
						best_d = 10000
						best_err = []

						threshold = 0.00048


						print(idthis, self.nodeScore[idthis], " PossibleNodes ", len(possible_nodes))

						def getDistance(item):
							lat_ = self.nodes[item][0]
							lon_ = self.nodes[item][1]

							a = lat_ - lat
							b = lon_ - lon 

							return np.sqrt(a*a + b*b)

						possible_nodes = sorted(possible_nodes, key = getDistance)

						c = 0
						for possible_node in possible_nodes:
							if possible_node == idthis:
								continue 

							if possible_node in self.deletedNodes.keys():
								continue

							c = c + 1

							if c > 3:
								break

							paths_target = self.getPath(possible_node)

							print("Check "+str(len(paths_this)*len(paths_target)) + " path pairs")

							cc = 0
							for p1 in paths_this:

								if cc > 200:
									break
								for p2 in paths_target:
									flag = True

									cc = cc + 1

									d,err = PathSimilarity(p1,p2)

									if d < best_d and d < threshold:
										best_d = d
										best_err = err
										best_p1 = p1
										best_p2 = p2


						
						if best_d < threshold:
							print(idthis, best_d, best_err,update_counter)
							print(best_p1)
							print(best_p2)
							print(" ")
							update_counter = update_counter + 1
							for i in range(len(p1)):

								# if distance(self.nodes[best_p1[i]], self.nodes[best_p2[i]]) > 0.00020:
								# 	continue

								if best_p1[i][2] in self.deletedNodes.keys() and best_p2[i][2] in self.deletedNodes.keys():
									continue

								if best_p1[i][2] in self.deletedNodes.keys() and best_p2[i][2] not in self.deletedNodes.keys():
									self.mergeTwoNodes(self.deletedNodes[best_p1[i][2]], best_p2[i][2])
									if self.deletedNodes[best_p1[i][2]] != best_p2[i][2]:
										self.deletedNodes[best_p2[i][2]] = self.deletedNodes[best_p1[i][2]]
								else:
									self.mergeTwoNodes(best_p1[i][2], best_p2[i][2])
									if best_p2[i][2] != best_p1[i][2]:
										self.deletedNodes[best_p2[i][2]] = best_p1[i][2]
								
								#print("Deleted ", self.deletedNodes)
								

			print(update_counter," Update(s)")
			#break

			if update_counter == 0 or oneround == True:
				break

		return update_counter
			

	def Smoothen(self, n = 3):

		self.edgeInt = {}

		 

		for edgeId, edge in self.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

			#self.edgeInt[edgeId] = None
			#continue

			if n1 not in self.nodeLinkReverse:
				self.edgeInt[edgeId] = None
				continue


			n0 = -1

			for nn in self.nodeLinkReverse[n1]:
				if nn not in self.deletedNodes.keys() and self.nodeScore[nn] > 0 and self.edgeScore[self.edgeHash[nn * 10000000 + n1]] > 0:
					n0 = nn 


			if n0 == -1 :
				self.edgeInt[edgeId] = None
				continue



			paths = self.getPath(n0, length = 4)


			if len(paths) == 0:
				self.edgeInt[edgeId] = None
				continue

			path = None

			for p in paths :
				if p[1][2] == n1 and p[2][2] == n2:
					path = p


			if path is None:
				self.edgeInt[edgeId] = None
				continue




			X = map(lambda x: x[0], path)
			Y = map(lambda x: x[1], path)

			tck1, u = interpolate.splprep([X, Y], s=0, k = 3)


			interval = (u[2] - u[1])/(n+1)
			TT = np.arange(u[1]+interval,u[2]-0.000000001,interval)

			pathInt_ = interpolate.splev(TT, tck1)

			pathInt = map(lambda x: [pathInt_[0][x], pathInt_[1][x]], range(len(pathInt_[0])))

			self.edgeInt[edgeId] = pathInt


		pass


	def TOPOWalk(self, nodeid, step = 0.00005, r = 0.00300, direction = False):

		localNodeList = {}
		localNodeDistance = {}

		mables = []

		localEdges = {}


		#localNodeList[nodeid] = 1
		#localNodeDistance[nodeid] = 0

		def explore(node_cur, node_prev, dist):

			if node_cur in localNodeList.keys():
				if localNodeDistance[node_cur] <= dist:
					return

			if dist > r :
				return 


			lat1 = self.nodes[node_cur][0]
			lon1 = self.nodes[node_cur][1]

			localNodeList[node_cur] = 1
			localNodeDistance[node_cur] = dist
			
			#mables.append((lat1, lon1))

			if node_cur not in self.nodeLinkReverse.keys():
				self.nodeLinkReverse[node_cur] = []

			reverseList = []

			if direction == False:
				reverseList = self.nodeLinkReverse[node_cur]

			for next_node in self.nodeLink[node_cur] + reverseList:

				edgeS = 0

				if node_cur * 10000000 + next_node in self.edgeHash.keys():
					edgeS = self.edgeScore[self.edgeHash[node_cur * 10000000 + next_node]]
				
				if next_node * 10000000 + node_cur in self.edgeHash.keys():
					edgeS = max(edgeS, self.edgeScore[self.edgeHash[next_node * 10000000 + node_cur]])


				if self.nodeScore[next_node] > 0 and edgeS > 0:
					pass
				else:
					continue

				if next_node == node_prev :
					continue

				lat0 = 0
				lon0 = 0

				lat1 = self.nodes[node_cur][0]
				lon1 = self.nodes[node_cur][1]

				lat2 = self.nodes[next_node][0]
				lon2 = self.nodes[next_node][1]

				#TODO check angle of next_node


				localEdgeId = node_cur * 10000000 + next_node

				# if localEdgeId not in localEdges.keys():
				# 	localEdges[localEdgeId] = 1

				l = distance((lat2,lon2), (lat1,lon1))
				num = int(math.ceil(l / step))


				bias = step * math.ceil(dist / step) - dist
				cur = bias

				while cur < l:
					alpha = cur / l 
				#for a in range(1,num):
				#	alpha = float(a)/num 
					if dist + l * alpha > r :
						break

					latI = lat2 * alpha + lat1 * (1-alpha)
					lonI = lon2 * alpha + lon1 * (1-alpha)

					if (latI, lonI) not in mables:
						mables.append((latI, lonI))

					cur += step

				l = distance((lat2,lon2), (lat1,lon1))

				explore(next_node, node_cur, dist + l)



		explore(nodeid, -1, 0)


		return mables


	def removeNode(self, nodeid):
		for next_node in self.nodeLink[nodeid]:
			edgeid = self.edgeHash[nodeid * 10000000 + next_node]

			del self.edges[edgeid]
			del self.edgeScore[edgeid]
			del self.edgeHash[nodeid * 10000000 + next_node]

			if nodeid in self.nodeLinkReverse[next_node]:
				self.nodeLinkReverse[next_node].remove(nodeid)


		for prev_node in self.nodeLinkReverse[nodeid]:
			edgeid = self.edgeHash[prev_node * 10000000 + nodeid]

			del self.edges[edgeid]
			del self.edgeScore[edgeid]
			del self.edgeHash[prev_node * 10000000 + nodeid]

			if nodeid in self.nodeLink[prev_node]:
				self.nodeLink[prev_node].remove(nodeid)


		del self.nodes[nodeid]
		del self.nodeScore[nodeid]
		del self.nodeLink[nodeid]
		del self.nodeLinkReverse[nodeid]



	def removeDeadEnds(self, oneround = False):
		deleted = 0
		for nodeid in self.nodes.keys():
			if self.nodeHashReverse[nodeid] in self.nodeTerminate.keys():
				continue

			if self.nodeHashReverse[nodeid] % 10000000 == 0:
				continue

			d = self.NumOfNeighbors(nodeid)
			if d == 1 or len(self.nodeLink[nodeid]) == 0 or len(self.nodeLinkReverse[nodeid]) == 0:
				self.removeNode(nodeid)
				deleted += 1

		return deleted 




	def setScoreThreshold(self, threshold = 1):
		self.scoreThreshold = threshold


	def NumOfNeighbors(self, nodeid):
		neighbor = {}

		for next_node in self.nodeLink[nodeid] + self.nodeLinkReverse[nodeid]:
			if next_node != nodeid:
				neighbor[next_node] = 1

		return len(neighbor.keys())

	def getNeighbors(self,nodeid):
		neighbor = {}

		for next_node in self.nodeLink[nodeid] + self.nodeLinkReverse[nodeid]:
			if next_node != nodeid:
				neighbor[next_node] = 1

		return neighbor.keys()


	def removeEdgesByThreshold(self, score_threshold = 0.0):
		for edgeid in self.edges.keys():
			if edgeid in self.edgeScore.keys():
				if self.edgeScore[edgeid] > score_threshold:
					continue

				node1 = self.edges[edgeid][0]
				node2 = self.edges[edgeid][1]


				if node1 not in self.deletedNodes.keys() and node2 not in self.deletedNodes.keys():
					continue

				if self.nodeScore[node1] > score_threshold:
					continue

				if self.nodeScore[node2] > score_threshold:
					continue


				del self.edges[edgeid]
				del self.edgeHash[node1 * 10000000 + node2]





	def findNearbySegments(self, score_threshold = 1.0, limitation = 100000000, region = None, dist_threshold = 0.00035):
		parallelSegs = []

		idx = index.Index()


		print(len(self.edges.keys()))
		for edgeid in self.edges.keys():
			if region is not None:
				if self.nodes[self.edges[edgeid][0]][0] < region[0] or self.nodes[self.edges[edgeid][0]][0] > region[2] :
					continue

				if self.nodes[self.edges[edgeid][0]][1] < region[1] or self.nodes[self.edges[edgeid][0]][1] > region[3] :
					continue



			if edgeid in self.edgeScore.keys():
				if self.edgeScore[edgeid] < score_threshold:
					continue

				node1 = self.edges[edgeid][0]
				node2 = self.edges[edgeid][1]

				if node1 in self.deletedNodes.keys() or node2 in self.deletedNodes.keys():
					continue

				if self.nodeScore[node1] < score_threshold:
					continue

				if self.nodeScore[node2] < score_threshold:
					continue


				print(edgeid, len(self.edges.keys()))

				lat1 = self.nodes[node1][0]
				lon1 = self.nodes[node1][1]

				lat2 = self.nodes[node2][0]
				lon2 = self.nodes[node2][1]




				idx.insert(edgeid, (min(lat1, lat2), min(lon1, lon2), max(lat1, lat2), max(lon1, lon2)))



		for edgeid in self.edges.keys():
			if region is not None:
				if self.nodes[self.edges[edgeid][0]][0] < region[0] or self.nodes[self.edges[edgeid][0]][0] > region[2] :
					continue

				if self.nodes[self.edges[edgeid][0]][1] < region[1] or self.nodes[self.edges[edgeid][0]][1] > region[3] :
					continue


			if edgeid in self.edgeScore.keys():
				if self.edgeScore[edgeid] < score_threshold:
					continue

				node1 = self.edges[edgeid][0]
				node2 = self.edges[edgeid][1]

				if node1 in self.deletedNodes.keys() or node2 in self.deletedNodes.keys():
					continue

				if self.nodeScore[node1] < score_threshold:
					continue

				if self.nodeScore[node2] < score_threshold:
					continue


				#find the closest edge pair


				lat = (self.nodes[node1][0] + self.nodes[node2][0]) / 2
				lon = (self.nodes[node1][1] + self.nodes[node2][1]) / 2
				r = dist_threshold

				possibleEdges = list(idx.intersection((lat-r,lon-r, lat+r, lon+r)))

				parallelRoads = []
				candidate_edgeid = -1
				min_dist = dist_threshold

				dir_lat, dir_lon = latlonNorm((self.nodes[node2][0] - self.nodes[node1][0], self.nodes[node2][1] - self.nodes[node1][1]), lat = self.nodes[node2][0])
				norm_lat = -dir_lon
				norm_lon = dir_lat

				center_lat = lat
				center_lon = lon

				meter_lat = 1.0 / (111111.0) 
				meter_lon = 1.0 / (111111.0 * math.cos(lat / 360.0 * (math.pi * 2)))

				p1 = (center_lat - 12.5 * meter_lat * dir_lat, center_lon - 12.5 * meter_lon * dir_lon)
				p2 = (center_lat + 12.5 * meter_lat * dir_lat, center_lon + 12.5 * meter_lon * dir_lon)


				for id_ in possibleEdges :
					if id_ == edgeid :
						continue


					can_node1 = self.edges[id_][0]
					can_node2 = self.edges[id_][1]

					can_dir_lat, can_dir_lon = latlonNorm((self.nodes[can_node2][0] - self.nodes[can_node1][0], self.nodes[can_node2][1] - self.nodes[can_node1][1]), lat = self.nodes[node2][0])


					angle_dist = pointDistance((can_dir_lat, can_dir_lon), (dir_lat, dir_lon)) 

					if angle_dist < 1.9 and angle_dist > 0.1 :
						continue

					ilat, ilon, extend_length, ok = edgeIntersection(center_lat, center_lon, norm_lat, norm_lon, self.nodes[can_node1][0], self.nodes[can_node1][1], self.nodes[can_node2][0], self.nodes[can_node2][1])
						#print(ilat, ilon, extend_length, ok)

					if ok == 0:
						ilat, ilon, extend_length, ok =  edgeIntersection(center_lat, center_lon, -norm_lat, -norm_lon, self.nodes[can_node1][0], self.nodes[can_node1][1], self.nodes[can_node2][0], self.nodes[can_node2][1])
						#print(ilat, ilon, extend_length, ok)

					if ok == 0:
						continue


					if extend_length < min_dist:
						
						can_p1 = (ilat - 12.5 * meter_lat * can_dir_lat, ilon - 12.5 * meter_lon * can_dir_lon)
						can_p2 = (ilat + 12.5 * meter_lat * can_dir_lat, ilon + 12.5 * meter_lon * can_dir_lon)




						parallelRoads = (edgeid, id_, p1, p2, can_p1, can_p2)
						min_dist = extend_length
						candidate_edgeid  = id_


				if candidate_edgeid != -1:
					print("Add new pairs", len(parallelSegs), parallelRoads)
					parallelSegs.append(parallelRoads)

					if len(parallelSegs) > limitation :
						return parallelSegs

		return parallelSegs



	def getConnectedComponent(self, nodeID):
		node_list = []

		queue = [nodeID]

		while len(queue) > 0:
			n0 = queue.pop(0)
			node_list.append(n0)
			print(n0)
			for n in self.nodeLink[n0] + self.nodeLinkReverse[n0]:
				if n in queue:
					continue
				if n in node_list:
					continue

				queue.append(n)


		return node_list


	def removeSmallConnectedComponent(self, threshold = 20):
		node_list = []


		for n in self.nodes.keys():
			if n not in node_list:
				cclist = self.getConnectedComponent(n)

				if len(cclist) < threshold:
					for nn in cclist:
						self.removeNode(nn)

				node_list = node_list + cclist


			
	def extendDeadEndP2P(self, lowbound = 0):
		idx = index.Index()

		DeadEnds = {}

		newNodeID = 0

		for nid in self.nodes.keys():
			if nid > lowbound:
				if self.NumOfNeighbors(nid) == 1:
					DeadEnds[nid] = 1
					lat = self.nodes[nid][0]
					lon = self.nodes[nid][1]

					r = 0.00001

					idx.insert(nid, (lat-r, lon-r, lat+r, lon+r))


		print("Dead Ends", len(DeadEnds.keys()))
		for nid in DeadEnds.keys():
			lat1 = self.nodes[nid][0]
			lon1 = self.nodes[nid][1]

			rlat = 0.00050
			rlon = 0.00050 / math.cos(math.radians(lat1))

			possibleNodes = list(idx.intersection((lat1-rlat,lon1-rlon, lat1+rlat, lon1+rlon)))

			min_d = 10000
			target = -1
			for pid in possibleNodes:
				id0 = 0
				id3 = 0

				print(nid, self.nodeLink[nid] + self.nodeLinkReverse[nid])
				for id_ in self.nodeLink[nid] + self.nodeLinkReverse[nid]:
					if id_ != nid:
						id0 = id_

				print(pid, self.nodeLink[pid] + self.nodeLinkReverse[pid])
				for id_ in self.nodeLink[pid] + self.nodeLinkReverse[pid]:
					if id_ != pid:
						id3 = id_


				lat2 = self.nodes[pid][0]
				lon2 = self.nodes[pid][1]

				lat0 = self.nodes[id0][0]
				lon0 = self.nodes[id0][1]

				lat3 = self.nodes[id3][0]
				lon3 = self.nodes[id3][1]


				np1x, np1y = latlonNorm((lat1 - lat0, lon1 - lon0))
				np2x, np2y = latlonNorm((lat2 - lat1, lon2 - lon1))
				np3x, np3y = latlonNorm((lat3 - lat2, lon3 - lon2))

				if pointDistance((np1x, np1y), (np2x, np2y)) < 1.4 and pointDistance((np2x, np2y), (np3x, np3y)) < 1.4:
					d = distance((lat1,lon1), (lat2, lon2))
					if d < min_d:
						min_d = d
						target = pid



			if target != -1:
				lat2 = self.nodes[target][0]
				lon2 = self.nodes[target][1]
				self.addEdge("p2p"+str(newNodeID), lat1, lon1, "p2p"+str(newNodeID +1) , lat2, lon2)
				newNodeID += 2

		print("New Nodes", newNodeID)

	def findIntersections(self):
		idx = index.Index()
		for edgeId, edge in self.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

			lat1 = self.nodes[n1][0]
			lon1 = self.nodes[n1][1]

			lat2 = self.nodes[n2][0]
			lon2 = self.nodes[n2][1]

			idx.insert(edgeId, (min(lat1, lat2), min(lon1, lon2), max(lat1, lat2), max(lon1, lon2)))


		edgePairs = []
		edgePairsMask = []

		def crossProduct(p1,p2):
			return p1[0] * p2[1] - p1[1] * p2[0]


		for edgeId, edge in self.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

			lat1 = self.nodes[n1][0]
			lon1 = self.nodes[n1][1]

			lat2 = self.nodes[n2][0]
			lon2 = self.nodes[n2][1]

			possibleEdges = list(idx.intersection((min(lat1, lat2), min(lon1, lon2), max(lat1, lat2), max(lon1, lon2))))


			for p_edgeId in possibleEdges:
				if p_edgeId == edgeId:
					continue

				n3 = self.edges[p_edgeId][0]
				n4 = self.edges[p_edgeId][1]

				lat3 = self.nodes[n3][0]
				lon3 = self.nodes[n3][1]

				lat4 = self.nodes[n4][0]
				lon4 = self.nodes[n4][1]


				d1 = crossProduct((lat3 - lat1, lon3 - lon1), (lat2 - lat3, lon2 - lon3))
				d2 = crossProduct((lat2 - lat3, lon2 - lon3), (lat4 - lat2, lon4 - lon2))
				d3 = crossProduct((lat4 - lat2, lon4 - lon2), (lat1 - lat4, lon1 - lon4))
				d4 = crossProduct((lat1 - lat4, lon1 - lon4), (lat3 - lat1, lon3 - lon1))

				if (d1 > 0 and d2 > 0 and d3 > 0 and d4 >0) or (d1<0 and d2<0 and d3<0 and d4<0):
					if (edgeId, p_edgeId) not in edgePairsMask and (p_edgeId, edgeId) not in edgePairsMask:

						dlat1, dlon1 = latlonNorm((lat2 - lat1, lon2 - lon1))
						dlat2, dlon2 = latlonNorm((lat4 - lat3, lon4 - lon3))

						d = dlat1 * dlat2 + dlon1 * dlon2 

						angle = abs(d)


						check_distance = 0.0005

						check_lat1 = lat1 - dlat1 * check_distance
						check_lon1 = lon1 - dlon1 * check_distance / math.cos(math.radians(lat1))

						check_lat2 = lat2 + dlat1 * check_distance
						check_lon2 = lon2 + dlon1 * check_distance / math.cos(math.radians(lat1))

						check_lat3 = lat3 - dlat2 * check_distance
						check_lon3 = lon3 - dlon2 * check_distance / math.cos(math.radians(lat1))

						check_lat4 = lat4 + dlat2 * check_distance
						check_lon4 = lon4 + dlon2 * check_distance / math.cos(math.radians(lat1))



						center_lat1 , center_lon1, _,_ = edgeIntersection(lat1, lon1, dlat1, dlon1/math.cos(math.radians(lat1)), lat3, lon3, lat4, lon4)

						#center_lat1 = (lat1 + lat2 + lat3 + lat4) / 4
						#center_lon1 = (lon1 + lon2 + lon3 + lon4) / 4


						r1 = 0.00005
						rc = 0.00008

						data = [check_lat1, check_lon1, center_lat1, center_lon1, check_lat3, check_lon3, r1, rc, r1]
						data += [check_lat3, check_lon3, center_lat1, center_lon1, check_lat1, check_lon1, r1, rc, r1]
						data += [check_lat3, check_lon3, center_lat1, center_lon1, check_lat2, check_lon2, r1, rc, r1]
						data += [check_lat2, check_lon2, center_lat1, center_lon1, check_lat3, check_lon3, r1, rc, r1]
						data += [check_lat4, check_lon4, center_lat1, center_lon1, check_lat2, check_lon2, r1, rc, r1]
						data += [check_lat2, check_lon2, center_lat1, center_lon1, check_lat4, check_lon4, r1, rc, r1]
						data += [check_lat4, check_lon4, center_lat1, center_lon1, check_lat1, check_lon1, r1, rc, r1]
						data += [check_lat1, check_lon1, center_lat1, center_lon1, check_lat4, check_lon4, r1, rc, r1]

						result = TraceQueryBatch3P(data)


						print(edgeId, p_edgeId, angle, result, center_lat1 ,center_lon1)

						edgePairs.append((edgeId, p_edgeId, angle, result)) 
						edgePairsMask.append((edgeId, p_edgeId))

						#if len(edgePairs) > 100:
						#	return edgePairs


		return edgePairs


	def Dump2GeoJson(self, filename, region = [-200,-200,200,200]):

		print("Dump geojson to "+filename)
		res = ""

		myfeature = []


		for edgeId, edge in self.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

			lat1 = self.nodes[n1][0]
			lon1 = self.nodes[n1][1]

			lat2 = self.nodes[n2][0]
			lon2 = self.nodes[n2][1]

			lat_min = min(lat1, lat2)
			lat_max = max(lat1, lat2)

			lon_min = min(lon1, lon2)
			lon_max = max(lon1, lon2)


			inside = True
			if lon_min > region[3] or lon_max < region[1] or lat_max < region[0] or lat_min > region[2]:
				inside = False

			if inside == True:
				myfeature.append(Feature(properties = {"id": edgeId, "type": "residential"}, geometry=LineString([(lon1,lat1), (lon2, lat2)])))

		feature_collection = FeatureCollection(myfeature)


		with open(filename, "w") as fout:
			geojson.dump(feature_collection, fout, indent=2)







def ConnectivityChecker2Points(lat1, lon1, lat2, lon2, radius = 0.00005):

	dist = distance((lat1, lon1), (lat2, lon2))

	if dist < radius * 2 :
		return True
	else:
		radius = min(max(dist * 0.1, 0.00005), 0.00012)

	data = [lat1, lon1, lat2, lon2, radius]

	result = TraceQueryBatch(data)

	gpsAt1 = result[0]
	gpsAt2 = result[1]

	validateTrips = result[3] + result[4]

	print(result)

	#validateTrips = 0

	if gpsAt1 < 5 or gpsAt2 < 5 :
		return True

	if validateTrips < 3:
		return False

	return True












def edgeIntersection(baseX, baseY, dX, dY, n1X, n1Y, n2X, n2Y):
	t = dX * n1Y + dY * n2X - dX * n2Y - dY * n1X

	c = n2X * n1Y - n1X * n2Y + baseX * (n2Y - n1Y) + baseY * (n1X -n2X)

	if t == 0 :
		return 0,0,0,0

	alpha = c / t

	if alpha < 0 : 
		return 0,0,0,0

	iX = baseX + alpha * dX
	iY = baseY + alpha * dY

	d = (iX - n1X)*(n2X - iX) + (iY - n1Y) * (n2Y - iY)

	if d < 0 :
		return 0,0,0,0

	extend_length = np.sqrt(alpha * dX * alpha * dX + alpha * dY * alpha * dY)

	return iX, iY, extend_length, 1


class RoadGraphCombiner:
	def __init__(self, RoadGraphA, RoadGraphB, ConnectivityChecker = None, CNNOutput = None, region = None):

		if CNNOutput is not None:
			cnn_dat = scipy.ndimage.imread(CNNOutput) * 255
			


		# Add to index
		idx = index.Index()
		for edgeId, edge in RoadGraphA.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

		
			if n1 in RoadGraphA.deletedNodes.keys() or n2 in RoadGraphA.deletedNodes.keys():
				continue

			if RoadGraphA.nodeScore[n1] < 1 or RoadGraphA.nodeScore[n2] < 1 :
				continue

			if n1 in RoadGraphA.nodeTerminate.keys() or n2 in RoadGraphA.nodeTerminate.keys():
				continue

			score = RoadGraphA.edgeScore[RoadGraphA.edgeHash[n1*10000000 + n2]]
			if score <1:
				continue


			lat1 = RoadGraphA.nodes[n1][0]
			lon1 = RoadGraphA.nodes[n1][1]
			lat2 = RoadGraphA.nodes[n2][0]
			lon2 = RoadGraphA.nodes[n2][1]


			idx.insert(edgeId, (min(lat1, lat2), min(lon1, lon2), max(lat1, lat2), max(lon1, lon2)))


		# Find Candidate Edge
		DeadEnds = {}

		for edgeId, edge in RoadGraphB.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

			nn1 = RoadGraphB.NumOfNeighbors(n1)
			nn2 = RoadGraphB.NumOfNeighbors(n2)

			if nn1 == 1 or nn2 == 1 :
				DeadEnds[edgeId] = 1

		# Extend Edge 

		newNodeId = 0

		print("DeadEnds ", len(DeadEnds.keys()))

		for edgeId in DeadEnds.keys():
			edge = RoadGraphB.edges[edgeId]
			n1 = edge[0]
			n2 = edge[1]

			nn1 = RoadGraphB.NumOfNeighbors(n1)
			nn2 = RoadGraphB.NumOfNeighbors(n2)

			lat1 = RoadGraphB.nodes[n1][0]
			lon1 = RoadGraphB.nodes[n1][1]

			lat2 = RoadGraphB.nodes[n2][0]
			lon2 = RoadGraphB.nodes[n2][1]


			# if lat1 < 42.3519 or lat1 > 42.3631 or lon1 < -71.1277 or lon1 > -71.1127:
			# 	continue




			avglat = (lat1+lat2)/2
			avglon = (lon1+lon2)/2




			if nn1 == 1 :
				dlat = lat1 - lat2
				dlon = lon1 - lon2

				lat = lat1
				lon = lon1
				r = 0.00100 # 100 meters

				possibleEdges = list(idx.intersection((lat-r,lon-r, lat+r, lon+r)))

				# if len(possibleEdges) > 0:
				# 	print(len(possibleEdges))

				min_dist = 0.00050
				newlat = 0
				newlon = 0
				old_n1 = 0
				old_n2 = 0


				for possibleEdge in possibleEdges:
					pn1 = RoadGraphA.edges[possibleEdge][0]
					pn2 = RoadGraphA.edges[possibleEdge][1]

					pn1lat = RoadGraphA.nodes[pn1][0]
					pn1lon = RoadGraphA.nodes[pn1][1]

					pn2lat = RoadGraphA.nodes[pn2][0]
					pn2lon = RoadGraphA.nodes[pn2][1]


					pn3 = -1 
					pn0 = -1

					for next_node in RoadGraphA.nodeLink[pn2]:
						if next_node != pn1 :
							pn3 = next_node

					for prev_node in RoadGraphA.nodeLinkReverse[pn1]:
						if prev_node != pn2 :
							pn0 = prev_node

					if pn0 == -1 or pn3 == -1 :
						continue

					pn0lat = RoadGraphA.nodes[pn0][0]
					pn0lon = RoadGraphA.nodes[pn0][1]

					pn3lat = RoadGraphA.nodes[pn3][0]
					pn3lon = RoadGraphA.nodes[pn3][1]




					newlat_, newlon_, dist, ok = edgeIntersection(lat, lon, dlat, dlon, pn1lat, pn1lon, pn2lat, pn2lon)

					if dist < min_dist and ok == 1:
						min_dist = dist

						newlat = newlat_
						newlon = newlon_

						old_n1 = pn1
						old_n2 = pn2

						if self.CNNConnectivityChecker(lat, lon, newlat, newlon, cnn_dat, region) == True:
							if self.ConnectivityChecker(avglat, avglon, pn1lat, pn1lon, pn0lat, pn0lon)==True or self.ConnectivityChecker(avglat, avglon, pn2lat, pn2lon, pn3lat, pn3lon)==True:

								RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n1, reverse=True, nodeScore1 = 100, edgeScore = 100)
								RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n2, reverse=True, nodeScore1 = 100, edgeScore = 100)

								RoadGraphA.addEdge("Tmp"+str(newNodeId), newlat, newlon, "CNN"+str(n1), lat, lon, reverse=True, nodeScore1 = 100, nodeScore2 = 100, edgeScore = 100)

								newNodeId += 1
								print(newNodeId)



				# TODO
				# print(min_dist)

				# if min_dist < 0.00050: # 50 meters
				# 	print(newlat, newlon)
				# 	RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n1, reverse=True, nodeScore1 = 100, edgeScore = 100)
				# 	RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n2, reverse=True, nodeScore1 = 100, edgeScore = 100)

				# 	RoadGraphA.addEdge("Tmp"+str(newNodeId), newlat, newlon, "CNN"+str(n1), lat, lon, reverse=True, nodeScore1 = 100, nodeScore2 = 100, edgeScore = 100)

				# 	newNodeId += 1


			if nn2 == 1 :
				dlat = lat2 - lat1
				dlon = lon2 - lon1

				lat = lat2
				lon = lon2
				r = 0.00100 # 100 meters

				possibleEdges = list(idx.intersection((lat-r,lon-r, lat+r, lon+r)))

				# if len(possibleEdges) > 0:
				# 	print(len(possibleEdges))


				min_dist = 0.00050
				newlat = 0
				newlon = 0
				old_n1 = 0
				old_n2 = 0


				for possibleEdge in possibleEdges:
					pn1 = RoadGraphA.edges[possibleEdge][0]
					pn2 = RoadGraphA.edges[possibleEdge][1]

					pn1lat = RoadGraphA.nodes[pn1][0]
					pn1lon = RoadGraphA.nodes[pn1][1]

					pn2lat = RoadGraphA.nodes[pn2][0]
					pn2lon = RoadGraphA.nodes[pn2][1]

					pn3 = -1 
					pn0 = -1

					for next_node in RoadGraphA.nodeLink[pn2]:
						if next_node != pn1 :
							pn3 = next_node

					for prev_node in RoadGraphA.nodeLinkReverse[pn1]:
						if prev_node != pn2 :
							pn0 = prev_node

					if pn0 == -1 or pn3 == -1 :
						continue

					pn0lat = RoadGraphA.nodes[pn0][0]
					pn0lon = RoadGraphA.nodes[pn0][1]

					pn3lat = RoadGraphA.nodes[pn3][0]
					pn3lon = RoadGraphA.nodes[pn3][1]




					newlat_, newlon_, dist, ok = edgeIntersection(lat, lon, dlat, dlon, pn1lat, pn1lon, pn2lat, pn2lon)

					if dist < min_dist and ok == 1:
						min_dist = dist

						newlat = newlat_
						newlon = newlon_

						old_n1 = pn1
						old_n2 = pn2

						if self.CNNConnectivityChecker(lat, lon, newlat, newlon, cnn_dat, region) == True:
							if self.ConnectivityChecker(avglat, avglon, pn1lat, pn1lon, pn0lat, pn0lon)==True or self.ConnectivityChecker(avglat, avglon, pn2lat, pn2lon, pn3lat, pn3lon)==True:
								RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n1, reverse=True, nodeScore1 = 100, edgeScore = 100)
								RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n2, reverse=True, nodeScore1 = 100, edgeScore = 100)

								RoadGraphA.addEdge("Tmp"+str(newNodeId), newlat, newlon, "CNN"+str(n2), lat, lon, reverse=True, nodeScore1 = 100, nodeScore2 = 100, edgeScore = 100)

								newNodeId += 1
								print("NodeID",newNodeId)


				# TODO

				#print(min_dist)

				# if min_dist < 0.00050: # 50 meters
				# 	print(newlat, newlon)
				# 	RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n1, reverse=True, nodeScore1 = 100, edgeScore = 100)
				# 	RoadGraphA.addEdgeToOneExistedNode("Tmp"+str(newNodeId), newlat, newlon, old_n2, reverse=True, nodeScore1 = 100, edgeScore = 100)

				# 	RoadGraphA.addEdge("Tmp"+str(newNodeId), newlat, newlon, "CNN"+str(n2), lat, lon, reverse=True, nodeScore1 = 100, nodeScore2 = 100, edgeScore = 100)

				# 	newNodeId += 1

		# Add GraphB to GraphA 
		for edgeId, edge in RoadGraphB.edges.iteritems():
			n1 = edge[0]
			n2 = edge[1]

			lat1 = RoadGraphB.nodes[n1][0]
			lon1 = RoadGraphB.nodes[n1][1]

			lat2 = RoadGraphB.nodes[n2][0]
			lon2 = RoadGraphB.nodes[n2][1]

			RoadGraphA.addEdge("CNN"+str(n1), lat1, lon1, "CNN"+str(n2), lat2, lon2, reverse=True,  nodeScore1 = 100, nodeScore2 = 100, edgeScore = 100)

		pass

	def ConnectivityChecker(self, lat1, lon1, lat2, lon2, lat3, lon3,  radius = 0.00005):
		#TODO check speed?  

		data = [lat1, lon1, 0.00030, 0.2, lat2, lon2, 0.00030, 0.2, lat3, lon3, 0.00030, 0.2]

		result = SpeedQueryBatch(data, port = 8020)

		result = [(5 / x * 3.6 / 1.6) for x in result]

		if result[0] < 40 and result[1] < 40 and result[2] < 40:
			return True

		# else:
		# 	return False




		data = [lat1, lon1, lat2, lon2, radius]

		result = TraceQueryBatch(data)

		gpsAt1 = result[0]
		gpsAt2 = result[1]

		validateTrips = result[3] + result[4]

		print(result)

		#validateTrips = 0

		if gpsAt1 < 5 :
			return True

		if validateTrips < 3:
			return False

		data = [lat1, lon1, lat2, lon2, lat3, lon3, radius, radius, radius]
		validateTrips = max(validateTrips, TraceQueryBatch3P(data))


		data = [lat3, lon3, lat2, lon2, lat1, lon1, radius, radius, radius]
		validateTrips = max(validateTrips, TraceQueryBatch3P(data))


		print("validateTrips ", validateTrips)

		if validateTrips > 2 :
			return True

		return False



	def CNNConnectivityChecker(self, lat1, lon1, lat2, lon2, img, region, l = 80, check_range = 12):
		min_lat = region[0]
		min_lon = region[1]
		max_lat = region[2]
		max_lon = region[3]

		sizex = np.shape(img)[0]
		sizey = np.shape(img)[1]


		ilat, ilon = Coord2Pixels(lat1, lon1, min_lat, min_lon, max_lat, max_lon, sizex, sizey)
		ilatT, ilonT = Coord2Pixels(lat2, lon2, min_lat, min_lon, max_lat, max_lon, sizex, sizey)

		n = []

		n.append((ilat, ilon))
		n.append((ilat+1, ilon))
		n.append((ilat-1, ilon))
		n.append((ilat, ilon+1))
		n.append((ilat, ilon-1))
		n.append((ilat+1, ilon+1))
		n.append((ilat+1, ilon-1))
		n.append((ilat-1, ilon+1))
		n.append((ilat-1, ilon-1))


		visited = []
		mark = []

		flag = False

		while True:
			if len(n) == 0:
				break

			loc = n.pop(0)

			if loc in visited:
				continue

			if (loc[0] - ilat) * (loc[0] - ilat) + (loc[1] - ilon) * (loc[1] - ilon) > l * l :
				continue 

			if img[loc[0], loc[1]] == 0:
				continue

			visited.append(loc)

			n.append((loc[0]+1, loc[1]))
			n.append((loc[0]-1, loc[1]))
			n.append((loc[0], loc[1]+1))
			n.append((loc[0], loc[1]-1))
			n.append((loc[0]+1, loc[1]+1))
			n.append((loc[0]-1, loc[1]+1))
			n.append((loc[0]+1, loc[1]-1))
			n.append((loc[0]-1, loc[1]-1))


			if (loc[0] - ilatT) * (loc[0] - ilatT) + (loc[1] - ilonT) * (loc[1] - ilonT) <= check_range * check_range :
				flag = True
				break


		return flag

def CheckParallelSegments(s1, s2, lat_res, lon_res, image_size=512, folder = "", start_lat = 0, start_lon = 0, resolution = 1024, padding = 128, zoom = 18, scale = 1, output_id = 0):
	center_lat = (s1[0][0] + s1[1][0]) / 2.0
	center_lon = (s1[0][1] + s1[1][1]) / 2.0

	range_minlat = center_lat - lat_res * image_size / 2
	range_maxlat = center_lat + lat_res * image_size / 2

	range_minlon = center_lon - lon_res * image_size / 2
	range_maxlon = center_lon + lon_res * image_size / 2

	m,ok= md.GetMapInRect(range_minlat, range_minlon, range_maxlat, range_maxlon , folder = folder, start_lat = start_lat, start_lon = start_lon, resolution = resolution, padding = padding, zoom = zoom, scale = scale)
	m1 = scipy.misc.imresize(m.astype(np.uint8),(image_size, image_size), mode="RGB")
	
	seg_img = np.zeros((image_size, image_size),np.uint8)
	seg_cv = cv.fromarray(seg_img)

	s1_lat_i = int((s1[0][0] - range_minlat) / lat_res)
	s1_lon_i = int((s1[0][1] - range_minlon) / lon_res)
	s2_lat_i = int((s1[1][0] - range_minlat) / lat_res)
	s2_lon_i = int((s1[1][1] - range_minlon) / lon_res)

	s3_lat_i = int((s2[0][0] - range_minlat) / lat_res)
	s3_lon_i = int((s2[0][1] - range_minlon) / lon_res)
	s4_lat_i = int((s2[1][0] - range_minlat) / lat_res)
	s4_lon_i = int((s2[1][1] - range_minlon) / lon_res)
	
	cv.Line(seg_cv, (s1_lon_i, s1_lat_i), (s2_lon_i, s2_lat_i), (255), 1, cv.CV_AA)
	cv.Line(seg_cv, (s3_lon_i, s3_lat_i), (s4_lon_i, s4_lat_i), (255), 1, cv.CV_AA)

	cv.Flip(seg_cv)

	seg_np = np.asarray(seg_cv, dtype='uint8')

	inputs = np.zeros((image_size, image_size, 4))

	inputs[:,:,0:3] = m1
	inputs[:,:,3] = seg_np

	Image.fromarray(m1).save("test111.png")
	Image.fromarray(inputs[:,:,3].reshape((image_size, image_size)).astype(np.uint8)).save("test112.png")


	Image.fromarray(m1).save("test111.png")

	result = md.GetClassifierResult(inputs)
	#result = 0
	print(result)

	inputs[:,:,0] = np.clip(inputs[:,:,3]+inputs[:,:,0], 0, 255)
	Image.fromarray(inputs[:,:,0:3].reshape((image_size, image_size,3)).astype(np.uint8)).save("test/test"+str(output_id)+".png")



	return result










if __name__ == "__main__":
	data = [42.355538, -71.083505, 0.00030, float(sys.argv[1]), 42.352176, -71.081198, 0.00030, float(sys.argv[1]), 42.352176, -71.081198, 0.00030, float(sys.argv[1])]
	
	s = SpeedQueryBatch(data, port = 8020)
	print(s)
	speed = s[0] + 0.0000001
	print(5 / speed * 3.6 / 1.6, "miles/h")
	exit()
	dumpDat = pickle.load(open(sys.argv[1], "rb"))
