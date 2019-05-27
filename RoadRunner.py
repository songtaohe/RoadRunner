import pickle
import socket
import numpy as np
import math
import sys
import scipy.ndimage
import scipy.ndimage as nd
import scipy.misc
from PIL import Image
#from RoadExploreCNN import CNNOutput
from time import time
import json
import PathSimilarity
from rtree import index
from sklearn.mixture import GMM
import RoadGraph as splfy # splfy is some old name ... 
import MergeDetector
import inspect
import random

from subprocess import Popen

base_port = 8001

isDebugOn = False

def debug_print(*args):
	isDebugOn
	if isDebugOn:
		#print(*args)

		print( "DEBUG Line: "+str(inspect.currentframe().f_back.f_lineno)+" "+" ".join(map(str,args)))



def DirectionClustering(direction_list, tolerance=4, m = 72):
	direction_list = sorted(direction_list)

	label = 0
	assignment = [-1] * len(direction_list)

	for i in range(len(direction_list)):
		if assignment[i] == -1:
			assignment[i] = label

		for j in range(i+1,len(direction_list)):
			if abs(direction_list[j],direction_list[j-1]) <= tolerance:
				assignment[j] = label
			else:
				break

		j = i-1 

		if i == 0:
			while j != i :
				if j < 0:
					j = j + m 

				jj = (jj + 1) % m 

				if abs(direction_list[j],direction_list[jj]) <= tolerance:
					assignment[j] = label
				else:
					break
				j = j - 1

		label = label + 1


	clusters = [[]] * label 

	for ind in xrange(len(assignment)):
		clusters[assignment[ind]].append(direction_list[ind])


	return clusters



def TraceQueryBatch(data, host="localhost", port=8002):
	global base_port
	port = base_port + 1

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
	global base_port
	port = base_port + 1

	ts1 = time()
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host, port))
	data_string = ""
	for i in range(len(data)/11):
		data_string = data_string + str(data[i*11+0])+","+str(data[i*11+1])+","+str(data[i*11+2])+","+str(data[i*11+3])+","+str(data[i*11+4])+","+str(data[i*11+5])+","+str(data[i*11+6])+","+str(data[i*11+7])+","+str(data[i*11+8])+","+str(data[i*11+9])+","+str(data[i*11+10])+","
		#data_string = data_string + str(data[i*11+0])+","+str(data[i*11+1])+","+str(data[i*11+2])+","+str(data[i*11+3])+","+str(data[i*11+4])+","+str(data[i*11+5])+","+str(data[i*11+6])+","+str(data[i*11+7])+","+str(data[i*11+8])+",-1,-1,"


	#print("DataSentLen", len(data_string))
	ts2 = time()
	s.send(data_string)
	result = s.recv(16384)
	ts3 = time()
	items = result.split()
	result = [int(item) for item in items]
	#print(result)

	t = time() - ts1

	print(t, ts2-ts1, ts3-ts2) 

	return result

def TraceQueryNodeScores(data, host="localhost", port=8002):
	global base_port
	port = base_port + 1

	ts1 = time()
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host, port))
	data_string = "N,%d,"% len(data)
	for i in range(len(data)):
		data_string = data_string + "%f,%f,%f,%f," % (data[i][0],data[i][1],data[i][2],data[i][3])


	#print("DataSentLen", len(data_string))
	ts2 = time()
	s.send(data_string)
	result = s.recv(16384)
	ts3 = time()
	items = result.split()
	result = [int(item) for item in items]
	#print(result)

	t = time() - ts1

	print(t, ts2-ts1, ts3-ts2) 

	return result



def TraceQuery3PInterpolation(data, host="localhost", port=8002):
	global base_port
	port = base_port + 1

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host, port))
	data_string = "C,"
	for i in range(len(data)/11):
		data_string = data_string + str(data[i*11+0])+","+str(data[i*11+1])+","+str(data[i*11+2])+","+str(data[i*11+3])+","+str(data[i*11+4])+","+str(data[i*11+5])+","+str(data[i*11+6])+","+str(data[i*11+7])+","+str(data[i*11+8])+","+str(data[i*11+9])+","+str(data[i*11+10])+","
		#data_string = data_string + str(data[i*11+0])+","+str(data[i*11+1])+","+str(data[i*11+2])+","+str(data[i*11+3])+","+str(data[i*11+4])+","+str(data[i*11+5])+","+str(data[i*11+6])+","+str(data[i*11+7])+","+str(data[i*11+8])+",-1,-1,"


	#print("DataSentLen", len(data_string))
	s.send(data_string)

	num_str = s.recv(16384)
	num = int(num_str.split(' ')[1])

	s.send("ok")

	result = ""
	for i in range(num):
		result += s.recv(16384)

	items = result.split()
	result = [float(item) for item in items[2:]]
	print("Interpolation ",result)

	return result

# Minimum Square Distance
def RawSimilarity(traceA, traceB):
	biasX = 0
	biasY = 0

	n = len(traceA)

	for i in xrange(n) :
		biasX = biasX + traceB[i][0] - traceA[i][0]
		biasY = biasY + traceB[i][1] - traceA[i][1]

	biasX = biasX / n
	biasY = biasY / n

	#biasX = 0
	#biasY = 0

	s = 0

	for i in xrange(n) :
		s = s + np.sqrt((traceA[i][0] - traceB[i][0] + biasX) * (traceA[i][0] - traceB[i][0] + biasX) + (traceA[i][1] - traceB[i][1] + biasY) * (traceA[i][1] - traceB[i][1] + biasY))


	return s/n



DegreeResolution = 5
#StepSize = 0.00025 # 15 meters
StepSize = 0.00015 # 15 meters
#StepSize = 0.00025 # 15 meters
QueryRange = 0.00003 # 2 meters

DBDCheckRange = 35 # default is 20 
DBDExclusionRange = 10 # 10 is good


def latlondistance(p1, p2):
	a = p1[0] - p2[0]
	b = (p1[1] - p2[1]) * math.cos(math.radians(p1[0]))
	return np.sqrt(a*a + b*b)

def AngleDifference(a,b, round=72):
	if abs(a-b) > round/2:
		return 72 - abs(a-b)
	else:
		return abs(a-b)

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


def latlonAngle(p1, p2):
	x = p2[0] - p1[0]
	y = (p2[1] - p1[1]) * math.cos(math.radians(p1[0]))



	angle = np.arctan2(x,y) * 180.0 / 3.1415926
	if angle < 0:
		angle += 360

	print(p1, p2, x, y, angle)

	return angle
# Check the merge!

def CommonRootCheck(traceA, traceB):
	commonRoot = False

	for i in range(len(traceA)):
		for j in range(len(traceB)):
			if traceA[i][2] == traceB[j][2]:
				commonRoot = True
				break

	return commonRoot



# Old Config
#def MergeChecker(traceA, traceB, r1 = 0.00002, r2 = 0.00002, r3 = 0.00002):

def MergeChecker(traceA, traceB, r1 = 0.00005, r2 = 0.00005, r3 = 0.00005, d1 = -1, d2 = -1):
	data = []

	
	n1 = 0
	n2 = len(traceB) / 2 + 1
	n3 = len(traceB) - 1

	data.append(traceA[n3][0])
	data.append(traceA[n3][1])
	data.append(traceA[n2][0])
	data.append(traceA[n2][1])
	data.append(traceB[n1][0])
	data.append(traceB[n1][1])

	data.append(r1)
	data.append(r2)
	data.append(r3)

	data.append(d1)
	data.append(d2)



	data.append(traceB[n3][0])
	data.append(traceB[n3][1])
	data.append(traceB[n2][0])
	data.append(traceB[n2][1])
	data.append(traceA[n1][0])
	data.append(traceA[n1][1])

	data.append(r1)
	data.append(r2)
	data.append(r3)

	data.append(d1)
	data.append(d2)


	result = TraceQueryBatch3P(data)

	print(result)

	if result[0] > 2 and result[1] > 2:
		return True
	else:
		return False




# class OSMMapmatching():
# 	def __init__(self, region):




def InitNode(lat, lon, id, parent, children, parentDIR):
	d = {}
	d['lat'] = lat
	d['lon'] = lon
	d['parent'] = parent
	d['children'] = children
	d['score'] = [0]*(360/DegreeResolution)
	d['rawScore'] = [0]*(360/DegreeResolution)
	d['rawScoreStepsize1'] = [0]*(360/DegreeResolution)
	d['rawScoreStepsize2'] = [0]*(360/DegreeResolution)
	d['rawScoreLongRange'] = [0]*(360/DegreeResolution)
	d['rawScoreHorizontal'] = [0]*(DBDCheckRange*2+1)
	d['rawScoreHorizontalLoc'] = []
	d['rawScoreHorizontalGMM'] = []
	d['deferredLinksInfo'] = [0,0]
	d['id'] = id
	d['parentDIR'] = parentDIR
	d['similarWith'] = [-1,-1]
	d['edgeScore'] = 0
	d['rawScoreMax'] = 0
	d['OutRegion'] = 0
	d['CNNCheckResult'] = []
	d['exitID'] = 0
	d['isExited'] = False
	d['nextNode'] = []
	d['nextNodeNumber'] = 0
	d['initDir'] = -1
	d['hyperJump'] = 0
	d['isDead'] = 0
	d['stepSize'] = 1.0
	d['radius'] = 0.00005
	d['noiseradius'] = 0.00005
	d['terminateType'] = -1


	d['BFSDepth'] = 0
	d['Aux'] = 0
	d['heavynoise'] = 0
	d['referenceCounter'] = 0
	d['removed'] = False
	d['refDeferredLinks'] = []

	d['AllBackPaths'] = [] #  All the paths (with certain length) that ends at this node.
	d['OutNodes'] = [] # Pointed to all of those outbound nodes. (treeid, nodeid, dir)
	d['InNodes'] = [] # Pointed to all of those inbound nodes, including parent node and reference node. (treeid, nodeid)


	d['needRevisit'] = False
	d['exploredDirections'] = []  # Should be removed, this is similar to OutNodes. 




 	return d



class RoadTree:
	def __init__(self, config, mCNNOutput=None):
		self.nodes = []
		self.nodes_cur = 0

		self.deferredLinks = []
		self.auxiliaryLinks = []

		self.index_segment = index.Index()
		self.index_deferredLinks = index.Index()

		self.deferredLinks_depth = 0
		self.maxBFSDepth = 0

		self.config = config 

		self.mCNNOutput = mCNNOutput
		self.forestIdx = 0
		self.forest = []

	def addInitialRoads(self, nodes):
		for n in nodes:
			self.nodes.append(n)


	def nodeScoreActivationFunc(self, score, threshold = 5):
		for ind in xrange(len(score)):
			if score[ind] >= threshold:
				score[ind] = math.log(score[ind]+0.00000001)
			else:
				score[ind] = -1000
		return score 

	def node2nodeScore(self, nodeA, nodeB): # A->B
		score = [1]*(360/DegreeResolution)
		batch_size = 36

		for i in range(360/DegreeResolution/batch_size):
			data = []
			for j in range(batch_size):
				degree = (i*batch_size+j) * DegreeResolution
				data.append(nodeA['lat'])
				data.append(nodeA['lon'])
				data.append(nodeB['lat']+math.sin(math.radians(degree))*StepSize)
				data.append(nodeB['lon']+math.cos(math.radians(degree))*StepSize / math.cos(math.radians(nodeB['lat'])))
				data.append(QueryRange)

			
			result = TraceQueryBatch(data)

		
		# This function is discarded
		# move the activation function out of this function

			for j in range(batch_size):
				#score[i*batch_size+j] = math.log(result[j*5+3]+result[j*5+4]+1)
				if result[j*5+3] > 1 :
					score[i*batch_size+j] = math.log(result[j*5+3]+0.00000001)
				else:
					score[i*batch_size+j] = -1000





		#return result 

		return score

	def nodeScoreWithPath(self, path):
		# path[i] =  (lat,lon,radius,distance)
		# path[0]'s distance is the radius of the circle. 



		result = TraceQueryNodeScores(path)

		for j in range(72):
			if result[j] >= self.config['minimal_number_of_trips'] :
				result[j] = math.log(result[j]+0.00000001)
			else:
				result[j] = -1000

		return result



	def node3Score(self, nodeA, nodeB, nodeC, r1=QueryRange,r2=QueryRange,r3=QueryRange, stepSize=StepSize, d1 = -1, d2 = -1): # A->B->C
		score = [1]*(360/DegreeResolution)
		batch_size = 72 

		for i in range(360/DegreeResolution/batch_size):
			data = []
			for j in range(batch_size):
				degree = (i*batch_size+j) * DegreeResolution
				data.append(nodeA['lat'])
				data.append(nodeA['lon'])
				data.append(nodeB['lat'])
				data.append(nodeB['lon'])
				data.append(nodeC['lat']+math.sin(math.radians(degree))*stepSize)
				data.append(nodeC['lon']+math.cos(math.radians(degree))*stepSize / math.cos(math.radians(nodeB['lat'])))
				data.append(r1)
				data.append(r2)
				data.append(r3)

				data.append(d1)
				data.append(d2)
			
			result = TraceQueryBatch3P(data)
		

		#return result 

		# move the activation function out of this function
		

			for j in range(batch_size):
				#score[i*batch_size+j] = math.log(result[j*5+3]+result[j*5+4]+1)
				# 4 gps traces
				# Modified from 3 to 2
				if result[j] >= self.config['minimal_number_of_trips'] :
					score[i*batch_size+j] = math.log(result[j]+0.00000001)
				else:
					score[i*batch_size+j] = -1000



		return score

	def node3ScoreWithRange(self, nodeA, nodeB, nodeC, r1=QueryRange,r2=QueryRange,r3=QueryRange, d1 = -1, d2 = -1, stepSize=StepSize, DegreeResolution = DegreeResolution, center = 0, size = 16): # A->B->C
		score = [1]*int(360/DegreeResolution)
		batch_size = 72 

		n = int(360/DegreeResolution)

		data = []
		for i in range(-size, size):
			degree = (center + i ) * DegreeResolution

			data.append(nodeA['lat'])
			data.append(nodeA['lon'])
			data.append(nodeB['lat'])
			data.append(nodeB['lon'])
			data.append(nodeC['lat']+math.sin(math.radians(degree))*stepSize)
			data.append(nodeC['lon']+math.cos(math.radians(degree))*stepSize / math.cos(math.radians(nodeB['lat'])))
			data.append(r1)
			data.append(r2)
			data.append(r3)

			data.append(d1)
			data.append(d2)

		result = TraceQueryBatch3P(data)

		return result 

		# move the activation function out of this function
		



		for j in range(size*2):
			if result[j] >= self.config['minimal_number_of_trips'] :
				score[j] = math.log(result[j]+0.00000001)
			else:
				score[j] = -1000

		return score


	def node3ScoreHorizontal(self, nodeA, nodeB, nodeC, r1=QueryRange,r2=QueryRange,r3=QueryRange, d1 = -1, d2 = -1, stepSize=0.00003, DegreeResolution = DegreeResolution, degree = 0, size = 16): # A->B->C
		score = [1]*(size*2+1)
		data = []

		for i in range(-size, size+1):
			data.append(nodeA['lat'])
			data.append(nodeA['lon'])
			data.append(nodeB['lat'])
			data.append(nodeB['lon'])
			data.append(nodeC['lat']+math.sin(math.radians(degree*DegreeResolution))*stepSize*i)
			data.append(nodeC['lon']+math.cos(math.radians(degree*DegreeResolution))*stepSize*i / math.cos(math.radians(nodeB['lat'])))
			data.append(r1)
			data.append(r2)
			data.append(r3)

			data.append(d1)
			data.append(d2)


			nodeC['rawScoreHorizontalLoc'].append((nodeC['lat']+math.sin(math.radians(degree*DegreeResolution))*stepSize*i, nodeC['lon']+math.cos(math.radians(degree*DegreeResolution))*stepSize*i / math.cos(math.radians(nodeB['lat']))))

		result = TraceQueryBatch3P(data)

		return result 

		# move the activation function out of this function
		

		for j in range(size*2+1):
			if result[j] >= self.config['minimal_number_of_trips']:
				score[j] = result[j] #math.log(result[j]+0.00000001)
			else:
				score[j] = -10000

		return score

	def node3ScoreHorizontalVStyle(self, nodeA, nodeB, nodeC, r1=QueryRange,r2=QueryRange,r3=QueryRange, d1 = -1, d2 = -1, stepSize=0.00003, DegreeResolution = DegreeResolution, degree1 = 0, degree2 = 0,  size = 16): # A->B->C
		score = [1]*(size*2+1)
		data = []

		#for i in range(-size, size+1):

		for i in range(size, 0, -1):
			data.append(nodeA['lat'])
			data.append(nodeA['lon'])
			data.append(nodeB['lat'])
			data.append(nodeB['lon'])
			data.append(nodeC['lat']+math.sin(math.radians(degree1*DegreeResolution))*stepSize*i)
			data.append(nodeC['lon']+math.cos(math.radians(degree1*DegreeResolution))*stepSize*i / math.cos(math.radians(nodeB['lat'])))
			data.append(r1)
			data.append(r2)
			data.append(r3)

			data.append(d1)
			data.append(d2)

			nodeC['rawScoreHorizontalLoc'].append((nodeC['lat']+math.sin(math.radians(degree1*DegreeResolution))*stepSize*i, nodeC['lon']+math.cos(math.radians(degree1*DegreeResolution))*stepSize*i / math.cos(math.radians(nodeB['lat']))))

		for i in range(size + 1):
			data.append(nodeA['lat'])
			data.append(nodeA['lon'])
			data.append(nodeB['lat'])
			data.append(nodeB['lon'])
			data.append(nodeC['lat']+math.sin(math.radians(degree2*DegreeResolution))*stepSize*i)
			data.append(nodeC['lon']+math.cos(math.radians(degree2*DegreeResolution))*stepSize*i / math.cos(math.radians(nodeB['lat'])))
			data.append(r1)
			data.append(r2)
			data.append(r3)

			data.append(d1)
			data.append(d2)

			nodeC['rawScoreHorizontalLoc'].append((nodeC['lat']+math.sin(math.radians(degree2*DegreeResolution))*stepSize*i, nodeC['lon']+math.cos(math.radians(degree2*DegreeResolution))*stepSize*i / math.cos(math.radians(nodeB['lat']))))

		


		

		# move the activation function out of this function
		



		noise = self.checkNoise(nodeC['lat'], nodeC['lon'])

		threshold = self.config['minimal_number_of_trips_deferred_branch']


		if noise == True:
			threshold = 5 # Very high threshold to reduce noise!

		result = TraceQueryBatch3P(data)

		#return result 



		for j in range(size*2+1):
			if result[j] >= threshold :
				score[j] = result[j] #math.log(result[j]+0.00000001)
			else:
				score[j] = -10000

		return score


	def isNodeOnPath(self, nodeA, nodeB):
		p = nodeA
		while p != 0:
			if p == nodeB:
				return True
			p = self.nodes[p]['parent']
			

		if p == nodeB:
				return True


		return False

	def updateScore(self, nodeID):
		parentId = nodeID
		#self.nodes[nodeID]['score'] = self.node2nodeScore(self.nodes[nodeID],self.nodes[nodeID])
		gamma = 1.0
		discount = 1.0
		node_list = []
		for i in range(13):
			node_list.append(parentId)
			parentId = self.nodes[parentId]['parent']


		n1 = 0
		n2 = 0
		n3 = 1

		new_score = self.node3Score(self.nodes[node_list[n3]], self.nodes[node_list[n2]],self.nodes[node_list[n1]], r1=0.00005, r2=0.00005,r3=0.00003)

		self.nodes[nodeID]['directConnectScore'] = new_score


		for i in range(4):
			if i == 0 :
				n1 = 0
				n2 = 0
				n3 = 3

			else:
				n1 = 0
				n3 = i*2
				n2 = i

			r = QueryRange
			r = 0.00005

			r = r + i * 0.00001

			r2 = r
			# Modified the r3 from 0.00003 to 0.00005 
			new_score = self.node3Score(self.nodes[node_list[n3]], self.nodes[node_list[n2]],self.nodes[node_list[n1]], r1=r, r2=r,r3=0.00005)

			for j in range(360/DegreeResolution):
		 		#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
		 		self.nodes[nodeID]['rawScore'][j] = self.nodes[nodeID]['rawScore'][j] + new_score[j]


		for j in range(360/DegreeResolution):
		 	self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['rawScore'][j]

		 	self.nodes[nodeID]['rawScoreMax'] = max(self.nodes[nodeID]['rawScoreMax'], self.nodes[nodeID]['rawScore'][j])

		if self.mCNNOutput != None :
			_,cnnSoft = self.mCNNOutput.checkPixel(self.nodes[nodeID]['lat'],self.nodes[nodeID]['lon'])
			self.nodes[nodeID]['CNNCheckResult'] = cnnSoft
		else:
			self.nodes[nodeID]['CNNCheckResult'] = None


		# Add Gaussian Blur
		tmp_score = [0] * (360/DegreeResolution)
		gaussian_kernel = [0.071303, 0.131514,0.189879,0.214607,0.189879,0.131514,0.071303]
		kernel_size = len(gaussian_kernel)

		for j in range(360/DegreeResolution):
		 	#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
		 	if self.nodes[nodeID]['rawScore'][j] < 0:
		 		 self.nodes[nodeID]['rawScore'][j] = 0

		 	tmp_score[j] = self.nodes[nodeID]['rawScore'][j]




		for j in range(360/DegreeResolution):
		 	if self.nodes[nodeID]['rawScore'][j] > 0 :
		 		s = 0
		 		for i in range(kernel_size/2):
		 			if tmp_score[(j + i + 1 + 360/DegreeResolution) % (360/DegreeResolution)] > 0:
		 				s = s + tmp_score[(j + i + 1+ 360/DegreeResolution) % (360/DegreeResolution)] * gaussian_kernel[i + kernel_size/2 + 1]
		 			else:
		 				break

		 		for i in range(kernel_size/2):
		 			if tmp_score[(j - i - 1 + 360/DegreeResolution) % (360/DegreeResolution)] > 0:
		 				s = s + tmp_score[(j - i - 1 + 360/DegreeResolution) % (360/DegreeResolution)] * gaussian_kernel[kernel_size/2 - i -1]
		 			else:
		 				break

		 		s = s + tmp_score[j] * gaussian_kernel[kernel_size/2]



		 		self.nodes[nodeID]['rawScore'][j] = s







		pass


	def _addGaussianBlurCircular(self, nodeID, itemname = 'rawScore'):
		tmp_score = [0] * (360/DegreeResolution)
		gaussian_kernel = [0.071303, 0.131514,0.189879,0.214607,0.189879,0.131514,0.071303]
		kernel_size = len(gaussian_kernel)

		for j in range(360/DegreeResolution):
		 	#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
		 	if self.nodes[nodeID][itemname][j] < 0:
		 		 self.nodes[nodeID][itemname][j] = 0

		 	tmp_score[j] = self.nodes[nodeID][itemname][j]


		for j in range(360/DegreeResolution):
		 	if self.nodes[nodeID][itemname][j] > 0 :
		 		s = 0
		 		for i in range(kernel_size/2):
		 			if tmp_score[(j + i + 1 + 360/DegreeResolution) % (360/DegreeResolution)] > 0:
		 				s = s + tmp_score[(j + i + 1+ 360/DegreeResolution) % (360/DegreeResolution)] * gaussian_kernel[i + kernel_size/2 + 1]
		 			else:
		 				break

		 		for i in range(kernel_size/2):
		 			if tmp_score[(j - i - 1 + 360/DegreeResolution) % (360/DegreeResolution)] > 0:
		 				s = s + tmp_score[(j - i - 1 + 360/DegreeResolution) % (360/DegreeResolution)] * gaussian_kernel[kernel_size/2 - i -1]
		 			else:
		 				break

		 		s = s + tmp_score[j] * gaussian_kernel[kernel_size/2]



		 		self.nodes[nodeID][itemname][j] = s




	def addGaussianBlurCircular(self, nodeID, itemname = 'rawScore'):
		self._addGaussianBlurCircular(nodeID, itemname = itemname)
		pass  # this is wrong... This won't return ...


		counter = 0
		while True:
			self._addGaussianBlurCircular(nodeID, itemname = itemname)

			peaks = self.getPeaksCircular(nodeID, itemname, center = self.nodes[nodeID]['parentDIR'], r = 24)

			d_peak = 1000
			if len(peaks) > 1:
				for i in range(len(peaks)):
					p1 = peaks[i]
					p2 = peaks[i-1]

					if abs(p1 - p2) > 36:
						d = 72 - abs(p1 - p2)
					else:
						d = abs(p1-p2)

					if d < d_peak :
						d_peak = d 


			if d_peak > 4:
				break

			counter += 1

			if counter > 10:
				break

		print("--- Gaussian Blur Iteration ", counter+1, d_peak, peaks)


			



	def addGaussianBlurNonCircular(self, nodeID, itemname = 'rawScore'):
		n = len(self.nodes[nodeID][itemname])

		tmp_score = [0] * n
		gaussian_kernel = [0.071303, 0.131514,0.189879,0.214607,0.189879,0.131514,0.071303]
		kernel_size = len(gaussian_kernel)

		
		for j in range(n):
		 	#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
		 	if self.nodes[nodeID][itemname][j] < 0:
		 		 self.nodes[nodeID][itemname][j] = 0

		 	tmp_score[j] = self.nodes[nodeID][itemname][j]


		for j in range(n):
		 	if self.nodes[nodeID][itemname][j] > 0 :
		 		s = 0
		 		for i in range(kernel_size/2):
		 			if tmp_score[(j + i + 1 + n) % n] > 0 and j + i + 1 < n:
		 				s = s + tmp_score[(j + i + 1 + n) % n] * gaussian_kernel[i + kernel_size/2 + 1]
		 			else:
		 				break

		 		for i in range(kernel_size/2):
		 			if tmp_score[(j - i - 1 + n) % n] > 0 and j - i - 1 >= 0:
		 				s = s + tmp_score[(j - i - 1 + n) % n] * gaussian_kernel[kernel_size/2 - i -1]
		 			else:
		 				break

		 		s = s + tmp_score[j] * gaussian_kernel[kernel_size/2]



		 		self.nodes[nodeID][itemname][j] = s

	def numberOfPeaks(self, nodeID, itemname = 'rawScore', center = 0, r = 24):
		n = 0
		value = 0
		for i in range(-r+1,r-1):
			ind = (i + center + 360/DegreeResolution) % (360/DegreeResolution)
			indp = (i + center + 1 + 360/DegreeResolution) % (360/DegreeResolution)
			indn = (i + center - 1 + 360/DegreeResolution) % (360/DegreeResolution)

			if self.nodes[nodeID][itemname][ind] > self.nodes[nodeID][itemname][indp] and self.nodes[nodeID][itemname][indp] > 0 and self.nodes[nodeID][itemname][ind] > self.nodes[nodeID][itemname][indn] and self.nodes[nodeID][itemname][indn] > 0 :
				n = n + 1
				value = value + self.nodes[nodeID][itemname][ind]

		return n, value

	def getPeaksCircular(self, nodeID, itemname = 'rawScore', center = 0, r = 24):
		n = 0
		value = 0
		peaks = []
		for i in range(-r+1,r-1):
			ind = (i + center + 360/DegreeResolution) % (360/DegreeResolution)
			indp = (i + center + 1 + 360/DegreeResolution) % (360/DegreeResolution)
			indn = (i + center - 1 + 360/DegreeResolution) % (360/DegreeResolution)

			if self.nodes[nodeID][itemname][ind] > self.nodes[nodeID][itemname][indp] and self.nodes[nodeID][itemname][indp] > 0 and self.nodes[nodeID][itemname][ind] > self.nodes[nodeID][itemname][indn] and self.nodes[nodeID][itemname][indn] > 0 :
				n = n + 1
				value = value + self.nodes[nodeID][itemname][ind]
				peaks.append(ind)

		return peaks

	def getPeaks(self, nodeID, itemname = 'rawScore'):
		peaks = []

		n = len(self.nodes[nodeID][itemname])

		for i in range(1, n-1):
			if self.nodes[nodeID][itemname][i] > self.nodes[nodeID][itemname][i+1] and self.nodes[nodeID][itemname][i+1] > 0 and self.nodes[nodeID][itemname][i] > self.nodes[nodeID][itemname][i-1] and self.nodes[nodeID][itemname][i-1] > 0 :
				peaks.append(i)

		return peaks

	def getPeaksGMM(self, nodeID, itemname = 'rawScore'):


		X = []
		for i in range(len(self.nodes[nodeID][itemname])):
			X = X + [i] * int(self.nodes[nodeID][itemname][i])

		if len(X) < 10 :
			return [],[],[]


		print("GMM size of X", len(X))

		X = np.asarray(X).reshape(-1,1)

		m = [GMM(i+1).fit(X) for i in range(3)]
		bic = [m[i].bic(X) for i in range(3)]

		nPeak = bic.index(min(bic))

		peaks = [int(m[nPeak].means_[i]) for i in range(nPeak+1)]
		covars = m[nPeak].covars_
		weight = m[nPeak].weights_

		print(peaks)
		print(covars)
		print(weight)

		return peaks, covars, weight

	# Need more test 
	def getAllBackPaths(self, nodeID, depth, forest):
		curID = nodeID 
		result = []

		
		stack = [[self.forestIdx,nodeID,0]]

		counter = 0

		while len(stack) != 0:
			tree_id = stack[-1][0]
			cur_id = stack[-1][1]
			cur_ind = stack[-1][2]

			tree = forest[tree_id]
			
			if cur_ind < len(tree.nodes[cur_id]['InNodes']):
				if counter == depth:
					result.append([(item[0],item[1]) for item in stack])
					stack.pop(len(stack)-1)
					counter = counter - 1 
				else:
					stack[-1][2] += 1
					stack.append([tree.nodes[cur_id]['InNodes'][cur_ind][0], tree.nodes[cur_id]['InNodes'][cur_ind][1], 0])
					counter = counter + 1 

			else:
				stack.pop(len(stack)-1)
				counter = counter - 1
		return result 






	# 
	def updateScoreMultipleStepSize(self, nodeID, multiPath = False):
		# d['AllBackPaths'] 

		parentId = nodeID
		#self.nodes[nodeID]['score'] = self.node2nodeScore(self.nodes[nodeID],self.nodes[nodeID])
		gamma = 1.0
		discount = 1.0
		node_list = []

		direction = self.nodes[parentId]['parentDIR']


		# Change 20 to 30 (225m)
		if multiPath == True:
			paths = self.getAllBackPaths(self, nodeID, 30, self.forest)
		else:
			#paths = []
			path = []

			for i in range(30):
				path.append((self.forestIdx,parentId))
				parentId = self.nodes[parentId]['parent']

			paths = [path]




		for j in range(360/DegreeResolution):
	 		#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
	 		self.nodes[nodeID]['rawScore'][j] = 0


		for path in paths:

			# TODO work on these code   convert path to node_list and dist_list
			# Also, the default 'InNodes' should be (self.forestIdx, 0)  (itself)
			# 20180119
			dist_list = [0]
			node_list = path 

			isLocalPath  =  True

			for i in range(29):
				#node_list.append(parentId)
				#parentId = self.nodes[parentId]['parent']
				if node_list[i][0] != self.forestIdx:
					isLocalPath = False

				#print(len(self.forest), node_list[i], i, len(node_list))
				lat1 = self.forest[node_list[i][0]].nodes[node_list[i][1]]['lat']
				lon1 = self.forest[node_list[i][0]].nodes[node_list[i][1]]['lon']

				lat2 = self.forest[node_list[i+1][0]].nodes[node_list[i+1][1]]['lat']
				lon2 = self.forest[node_list[i+1][0]].nodes[node_list[i+1][1]]['lon']


				d = latlondistance((lat1,lon1),(lat2,lon2))
				dist_list.append(dist_list[-1] + d)



			if isLocalPath  == True:
				#for i in range(8,9):
				for i in range(3):
					
					n1 = 0
					n2 = 12 # 90 meter
					n3 = 18 # 135 meter
					n4 = 24 # 180 meter
					n5 = 29 # 225 meter 

					r = 0.00010

					
					if i == 0 :
						new_score1 = self.node3ScoreHorizontalVStyle(self.nodes[node_list[n3][1]], self.nodes[node_list[n2][1]],self.nodes[node_list[n1][1]], r1=r, r2=r,r3=0.00002, d1 = dist_list[n3] - dist_list[n2], d2 = dist_list[n2] - dist_list[n1],  stepSize = 0.000015, DegreeResolution = 5, degree1 = (direction+24) % 72, degree2 = (direction-24 + 72) % 72, size=DBDCheckRange)
					
					elif i == 1:
						new_score1 = self.node3ScoreHorizontalVStyle(self.nodes[node_list[n4][1]], self.nodes[node_list[n3][1]],self.nodes[node_list[n1][1]], r1=r, r2=r,r3=0.00002, d1 = dist_list[n4] - dist_list[n3], d2 = dist_list[n3] - dist_list[n1],  stepSize = 0.000015, DegreeResolution = 5, degree1 = (direction+24) % 72, degree2 = (direction-24 + 72) % 72, size=DBDCheckRange)
						
					elif i == 2:
						new_score1 = self.node3ScoreHorizontalVStyle(self.nodes[node_list[n5][1]], self.nodes[node_list[n3][1]],self.nodes[node_list[n1][1]], r1=r, r2=r,r3=0.00002, d1 = dist_list[n5] - dist_list[n3], d2 = dist_list[n3] - dist_list[n1],  stepSize = 0.000015, DegreeResolution = 5, degree1 = (direction+24) % 72, degree2 = (direction-24 + 72) % 72, size=DBDCheckRange)


					for j in range(DBDCheckRange*2+1):
						self.nodes[nodeID]['rawScoreHorizontal'][j] += new_score1[j]
					


					self.nodes[nodeID]['deferredLinksInfo'] = [node_list[n2][1], node_list[n3][1], r, r, node_list[n4][1], dist_list[n3] - dist_list[n2], dist_list[n2] - dist_list[n1]]
					



				for j in range(DBDCheckRange*2+1):
						self.nodes[nodeID]['rawScoreHorizontal'][j] /= 3

					

				self.addGaussianBlurNonCircular(nodeID, 'rawScoreHorizontal')

				# Estimate the width of the road
				peaks = self.getPeaks(nodeID, "rawScoreHorizontal")


				maxpeak = 0.0
				for peak in peaks :
					if self.nodes[nodeID]["rawScoreHorizontal"][peak] > maxpeak:
						maxpeak = self.nodes[nodeID]["rawScoreHorizontal"][peak]



				widthOfRoad = 0

				for p in range(DBDCheckRange, DBDCheckRange*2+1):
					if self.nodes[nodeID]["rawScoreHorizontal"][p] > maxpeak / 2:
						widthOfRoad  += 1
					else:
						break

				for p in range(DBDCheckRange-1, 0, -1):
					if self.nodes[nodeID]["rawScoreHorizontal"][p] > maxpeak / 2:
						widthOfRoad  += 1
					else:
						break
						
				widthOfRoad = (widthOfRoad - 1) * 0.000015

				self.nodes[nodeID]['radius'] = min(0.000125, max(0.00005, widthOfRoad/2))



			n1 = 0
			n2 = 0
			n3 = 1

			
			query_data = []

			def getNode(k):
				return self.forest[k[0]].nodes[k[1]]


			##e.g., self.config['history_length'] = 5  i = 0..4
			last_node_ind = 0


			

			for noise_radius in [0.00005, 0.00007, 0.00009, 0.00011, 0.00013, 0.00015]:
				query_data = []
				last_node_ind = 0
				for i in range(self.config['history_length']*2+1):
					if i == 0:
						d = StepSize

					else:
						d = dist_list[i] - dist_list[last_node_ind]

					
					r = 0.00005+i*0.00001 
					# Change this to a smaller value, focus on the precision 
					r = noise_radius

					#getNode(node_list[i])['noiseradius'] = max(r, getNode(node_list[i])['noiseradius'])

					r2 = max(getNode(node_list[i])['radius'],r)
					#r3 = max(getNode(node_list[(i+1)*2])['radius'],r)

					# make sure the circle don't overlap 

					print(i,r2,d,last_node_ind)

					if i == 0:
						r2 = 0.00005

						# Try to remove this... Although 5 meter radius might be a bottomneck
						# r2 = noise_radius



					else:
						if query_data[-1][2] + r2 > d and d != 0:
							if i == self.config['history_length']*2 :
								# always keep the last node!
								if len(query_data) > 1:
									query_data.pop(len(query_data)-1)
							else:
								continue

					#query_data.append([getNode(node_list[i*2])['lat'],getNode(node_list[i*2])['lon'], 0.00005+i*0.00001, d])
					query_data.append([getNode(node_list[i])['lat'],getNode(node_list[i])['lon'], r2, d])
					last_node_ind = i 

				print(query_data)

				new_score = self.nodeScoreWithPath(query_data)

				isAllZero = True 
				for ind in xrange(len(new_score)):
					if new_score[ind] > 0 and new_score[(ind+1)%len(new_score)]>0 and new_score[(ind+2)%len(new_score)]>0:
						isAllZero = False
						break
				

				if isAllZero == False:
					for i in range(self.config['history_length']*2+1):
						r = noise_radius
						getNode(node_list[i])['noiseradius'] = max(r, getNode(node_list[i])['noiseradius'])


					break

			for j in range(360/DegreeResolution):
		 		#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
		 		self.nodes[nodeID]['rawScore'][j] += new_score[j]


		for j in range(360/DegreeResolution):
			self.nodes[nodeID]['rawScore'][j] += random.random()*0.00001

		for j in range(360/DegreeResolution):
		 	self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['rawScore'][j]
		 	self.nodes[nodeID]['rawScoreMax'] = max(self.nodes[nodeID]['rawScoreMax'], self.nodes[nodeID]['rawScore'][j])

		self.nodes[nodeID]['CNNCheckResult'] = None


		# Add Gaussian Blur
		tmp_score = [0] * (360/DegreeResolution)
		gaussian_kernel = [0.071303, 0.131514,0.189879,0.214607,0.189879,0.131514,0.071303]
		kernel_size = len(gaussian_kernel)

		for j in range(360/DegreeResolution):
		 	#self.nodes[nodeID]['score'][j] = self.nodes[nodeID]['score'][j] + new_score[j]
		 	if self.nodes[nodeID]['rawScore'][j] < 0:
		 		 self.nodes[nodeID]['rawScore'][j] = 0

		 	tmp_score[j] = self.nodes[nodeID]['rawScore'][j]


		for j in range(360/DegreeResolution):
		 	if self.nodes[nodeID]['rawScore'][j] > 0 :
		 		s = 0
		 		for i in range(kernel_size/2):
		 			if tmp_score[(j + i + 1 + 360/DegreeResolution) % (360/DegreeResolution)] > 0:
		 				s = s + tmp_score[(j + i + 1+ 360/DegreeResolution) % (360/DegreeResolution)] * gaussian_kernel[i + kernel_size/2 + 1]
		 			else:
		 				break

		 		for i in range(kernel_size/2):
		 			if tmp_score[(j - i - 1 + 360/DegreeResolution) % (360/DegreeResolution)] > 0:
		 				s = s + tmp_score[(j - i - 1 + 360/DegreeResolution) % (360/DegreeResolution)] * gaussian_kernel[kernel_size/2 - i -1]
		 			else:
		 				break

		 		s = s + tmp_score[j] * gaussian_kernel[kernel_size/2]



		 		self.nodes[nodeID]['rawScore'][j] = s


		pass

	def findCommonRoot(self, nodeID1, nodeID2, depth = 100):
		parent1 = {}

		cur_p = nodeID1 
		counter  = 0 

		while counter < depth :
			if cur_p == 0 :
				parent1[0] = 1
				break

			parent1[cur_p] = 1

			cur_p = self.nodes[cur_p]['parent']
			counter += 1


		cur_p = nodeID2
		counter  = 0 

		while counter < depth :
			if cur_p == 0 :
				if 0 in parent1.keys():
					return 0
				else:
					return -1

			if cur_p in parent1.keys():
				return cur_p

			cur_p = self.nodes[cur_p]['parent']
			counter += 1

		return -1


	def getPathBetweenTwoNodes(self, nodeID1, nodeID2, depth = 200):
		result = []

		cur_p = nodeID1 
		counter = 0
		while True:

			result.append((self.nodes[cur_p]['lat'], self.nodes[cur_p]['lon']))
			counter += 1
			if cur_p == nodeID2:
				break
			else:
				cur_p = self.nodes[cur_p]['parent']

			if counter > depth:
				break

		return result


	def getPathLength(self, path):
		d = 0
		for i in range(len(path)-1):
			d += latlondistance(path[i], path[i+1])

		return d


	def getPath(self, nodeID, n):
		result = []
		parentId = nodeID
		for i in range(n):
			result.append([self.nodes[parentId]['lat'],self.nodes[parentId]['lon'],parentId])

			parentId = self.nodes[parentId]['parent']
		return result


	def checkNoise(self, lat, lon):
		return False

		sizex = np.shape(config['noise_image'])[0]
		sizey = np.shape(config['noise_image'])[1]

		#print(sizex, sizey)
		#exit()

		nlat = int((1.0 - (lat - self.Region[0])/(self.Region[2] - self.Region[0])) * sizex)
		nlon = int(((lon - self.Region[1])/(self.Region[3] - self.Region[1])) * sizey)


		if nlat >= 0 and nlat < sizex and nlon >= 0 and nlon < sizey:
			# print(sizex, sizey)
			# print(nlat, nlon)
			# print(lat, lon)
			
			# print(config['noise_image'][nlat][nlon])

			# exit()

			#if config['noise_image'][nlat][nlon][0] > 0 and config['noise_image'][nlat][nlon][3] > 0:
			if config['noise_image'][nlat][nlon][0] > 10:	
				return True

		return False


	def findNearbySegments(self, new_id, r = 0.00030):

		result = []
		parent_list = []
		cur_id = new_id

		for i in xrange(10):
			parent_list.append(cur_id)
			cur_id = self.nodes[cur_id]['parent']


		new_lat = self.nodes[new_id]['lat']
		new_lon = self.nodes[new_id]['lon']

		for tree in self.forest:
			r_lat = r/2
			r_lon = r/2 / math.cos(math.radians(self.nodes[new_id]['lat']))

			possible_segment = list(tree.index_segment.intersection((new_lat-r_lat, new_lon-r_lon, new_lat+r_lat, new_lon+r_lon)))

			if tree == self:
				for p in possible_segment:
					if p not in parent_list:
						result.append((tree.forestIdx, p))
			else:
				for p in possible_segment:
					result.append((tree.forestIdx, p))

		return result


	def findSimilarNode(self, new_id, otherTree=None):
		new_lat, new_lon = self.nodes[new_id]['lat'], self.nodes[new_id]['lon']

		other = self

		if otherTree != None:
			other = otherTree

		k = -1
		min_dist = 10000


		r_lat = 0.00015
		r_lon = 0.00025

		r_lat = 0.00015*2
		r_lon = 0.00025*2

		possible_segment = list(other.index_segment.intersection((new_lat-r_lat, new_lon-r_lon, new_lat+r_lat, new_lon+r_lon)))

		#for i in range(len(other.nodes)-1):

		allks = []
		allds = []

		for i in possible_segment:

			if other.nodes[i]['similarWith'][0] != -1:
				continue
			if other.nodes[i]['removed'] == True:
				continue
			if other.nodes[i]['referenceCounter'] == 0:
				continue

			# Don't merge to a aux node! 
			if other.nodes[i]['Aux'] == 1:
				continue


			a = other.nodes[i]['lat'] - new_lat
			b = (other.nodes[i]['lon'] - new_lon) * math.cos(math.radians(new_lat))
			d = np.sqrt(a*a + b*b)

			if d < min_dist:
				if not (self == other and i == new_id):
					min_dist = d
					k = i

			if d < self.config['merge_detector_search_threshold']:
				if not (self == other and i == new_id):
					allks.append(i)
					allds.append(d)


		#print(k, min_dist)

		result = -1


	

		heavy_noise = self.checkNoise(new_lat, new_lon)

		if heavy_noise == True:
			self.nodes[new_id]['heavynoise'] = 1


		#heavy_noise = False

		if heavy_noise == True:  
			for k in allks: # do it super aggressively  (Evaluate ALL !!!)
				if result != -1:
					break


				path1 = other.getPath(k,6)
				path2 = self.getPath(new_id,6)


				#s = RawSimilarity(path1, path2)

				#s, _ = PathSimilarity.PathSimilarity(path1, path2, threshold = 0.00050)
				s, s_max = PathSimilarity.PathSimilarityLatLon(path1, path2, threshold = 0.00050)
				s = s / 32.0
				ss = s

				print("Terminate Check", s)


			
				if self == other and k == new_id:
					s = 100

				print("Terminate Check", s)
				if self == other and CommonRootCheck(path1, path2) == True:
					s = 100

				print("Terminate Check", s)


				detectionType = 0

				
				if s < 0.00015 : # old is 0.00006 
					valid = MergeChecker(path1, path2)
					result = k #### Test
					pass



					if valid :
						print(new_id, k, s)
				#if s < 0.00005:
						result = k
					elif self.config['merge_detector'] == 'kde_based':
						detectionType = 1
						similarity = MergeDetector.mergeDetector(path1, path2)
						if similarity > 0.4 : # Old is 0.7
							print(new_id, k, s)
							result = k

				elif s < self.config['merge_detector_search_threshold'] and self.config['merge_detector'] == 'kde_based': # old is this
				#elif s < 0.00100 and self.config['merge_detector'] == 'kde_based': # merge aggressively
					detectionType = 2
					similarity = MergeDetector.mergeDetector(path1, path2)
					if similarity > 0.4 : # Old is 0.7
						print(new_id, k, s)
						result = k




		if self.config['merge_detector_evaluate_all'] == False:
			allks = []
			allds = []
			if min_dist < self.config['merge_detector_search_threshold']:
				allks = [k]
				allds = [min_dist]




		terminate_type = 0

		#if k!=-1 and min_dist < 0.00025 : # 25 meters  # old one 
		for k_ind in range(len(allks)): # do it aggressively
			k = allks[k_ind] 
			dist = allds[k_ind]

		

			if result != -1:
				continue 

			if dist > 0.00015:
				continue


			path1 = other.getPath(k,self.config['merge_detector_path_length'])
			path2 = self.getPath(new_id,self.config['merge_detector_path_length'])

			

			#s = RawSimilarity(path1, path2)

			#s, _ = PathSimilarity.PathSimilarity(path1, path2, threshold = 0.00050)
			s, s_max = PathSimilarity.PathSimilarityLatLon(path1, path2, threshold = 0.00050)
			s = s / 32.0
			ss = s

			print("Terminate Check", s)


			

			#print("Terminate Check", s)
			if self == other and k == new_id:
				s = 100

			print("Terminate Check", s)
			if self == other and CommonRootCheck(path1, path2) == True:
				s = 100

			print("Terminate Check", s)


			detectionType = 0

			global base_port
			
			if s < 0.00006 and s_max < 0.00015: # old is 0.00006 
				valid = MergeChecker(path1, path2)

				if valid :
					print(new_id, k, s)
			#if s < 0.00005:
					result = k
					terminate_type = 1
				elif self.config['merge_detector'] == 'kde_based':
					detectionType = 1
					similarity = MergeDetector.mergeDetector(path1, path2,port=base_port+1)
					if similarity > 0.7 : # Old is 0.7
						print(new_id, k, s)
						result = k
						terminate_type = 2

			#elif s < self.config['merge_detector_search_threshold'] and self.config['merge_detector'] == 'kde_based': # old is this
			elif s < 0.00025 and s_max < 0.00030 and self.config['merge_detector'] == 'kde_based': # old is this
			#elif s < 0.00100 and self.config['merge_detector'] == 'kde_based': # merge aggressively
				detectionType = 2
				similarity = MergeDetector.mergeDetector(path1, path2,port=base_port+1)
				if similarity > 0.7 : # Old is 0.7
					print(new_id, k, s)
					result = k
					terminate_type = 3



		return result,terminate_type





	def updateDeferredLink(self, Forest = None):
		flag = False
		link = []
		for i in range(len(self.deferredLinks)):
			if self.deferredLinks[i]['triggerDepth'] < self.maxBFSDepth and self.deferredLinks[i]['state'] == 'active':
				link = self.deferredLinks[i]
				flag = True

		if flag == False:
			return flag

		parent_id = link['node2']
		

		for i in range(len(link['shape'])/2 - 1):
			lat1 = link['shape'][i*2]
			lon1 = link['shape'][i*2+1]
			lat2 = link['shape'][i*2+2]
			lon2 = link['shape'][i*2+3]

			new_id = len(self.nodes)
			node = InitNode(lat2, lon2, new_id, parent_id, [],0) 

			node['InNodes'].append((-1, parent_id))
			self.nodes[parent_id]['OutNodes'].append((-1, new_id, -1))


			node['isDead'] = 1
			node['Aux'] = 1
			self.nodes.append(node)

			#self.updateScoreMultipleStepSize(new_id)

			self.nodes[new_id]['BFSDepth'] = self.nodes[parent_id]['BFSDepth'] + 1
			self.nodes[parent_id]['referenceCounter'] += 1

			parent_id = new_id


		node['isDead'] = 0
		node['Aux'] = 0
		self.updateScoreMultipleStepSize(new_id)
		_direction = int(latlonAngle((lat1,lon1),(node['lat'],node['lon'])) / DegreeResolution)
		node['parentDIR'] = _direction
		self.updateNewNode(new_id, Forest, new_id - 1, lat1, lon1, _direction,20)



		link['state'] = 'recoveried'

		return flag




	def updateNewNode(self,  new_id, Forest,_parent, parent_lat, parent_lon, _direction, max_score):
		self.nodes[new_id]['BFSDepth'] = self.nodes[_parent]['BFSDepth'] + 1
		self.maxBFSDepth = max(self.maxBFSDepth, self.nodes[new_id]['BFSDepth'])


		### TODO?  Might be wrong for the first node?

		n1,v1 = self.numberOfPeaks(new_id, "rawScore", _direction, 24)
		# n2,v2 = self.numberOfPeaks(new_id, "rawScoreStepsize1", _direction, 24)
		# n3,v3 = self.numberOfPeaks(new_id, "rawScoreStepsize2", _direction, 24)

		debug_print(new_id, n1,v1,_direction)
		debug_print(new_id, self.nodes[new_id]["rawScore"])

		print(n1, v1)

		v2 = 0
		v3 = 0


		s = 1.0

				

		# !!! Config of NDE No Dead End
		if self.config['keep_deadend'] == True:
			if v1 == 0 and v2 == 0 and v3 == 0:
				self.nodes[new_id]['OutRegion'] = 1
		else:
			# Check whether there is a node nearby (30 meters maybe)
			if v1 == 0 and v2 == 0 and v3 == 0:
				ns = self.findNearbySegments(new_id, r = 0.00040)
				if len(ns) == 0:

					# If it is a Aux node, remove it for no reason
					if self.nodes[new_id]['Aux'] == 1 or self.nodes[self.nodes[new_id]['parent']]['Aux'] == 1 or self.nodes[self.nodes[self.nodes[new_id]['parent']]['parent']]['Aux'] == 1:
						pass
					else:
						self.nodes[new_id]['OutRegion'] = 1




		if v1 == 0 and v2 == 0 and v3 == 0:
			# If the node is close to boundary, keep the node
			# TODO

			new_lat = self.nodes[new_id]['lat']
			new_lon = self.nodes[new_id]['lon']


			if abs(new_lat-self.Region[0]) < 0.00025 or abs(new_lat-self.Region[2]) < 0.00025 or abs(new_lon-self.Region[1]) < 0.00040 or abs(new_lon-self.Region[3]) < 0.00040: 
				self.nodes[new_id]['OutRegion'] = 1

		# if n2 > n1 :
		# 	s = 0.8

		# if n3 > n2 and n2 >= n1 :
		# 	s = 0.6


		# We won't change the stepsize if there is a turning (Removed by setting the threshold to a large value , e.g., 150 ...)
		if AngleDifference(_direction, self.nodes[_parent]['parentDIR']) < 150:
			node_old_lat = self.nodes[new_id]['lat']
			node_old_lon = self.nodes[new_id]['lon']

			self.nodes[new_id]['lat'] = self.nodes[new_id]['lat'] * s + parent_lat * (1-s)
			self.nodes[new_id]['lon'] = self.nodes[new_id]['lon'] * s + parent_lon * (1-s)

			self.nodes[new_id]['stepSize'] = s

			# TODO Assign value to 'score'

			for j in range(360/DegreeResolution):
				if s == 1.0:
					self.nodes[new_id]['score'][j] = self.nodes[new_id]['rawScore'][j]
				if s == 0.8:
					self.nodes[new_id]['score'][j] = self.nodes[new_id]['rawScoreStepsize1'][j]
				if s == 0.6:
					self.nodes[new_id]['score'][j] = self.nodes[new_id]['rawScoreStepsize2'][j]



		self.index_segment.insert(new_id, (min(self.nodes[new_id]['lat'], parent_lat), min(self.nodes[new_id]['lon'], parent_lon), max(self.nodes[new_id]['lat'], parent_lat), max(self.nodes[new_id]['lon'], parent_lon)))

		# Check Deferred Links 
		possible_links = list(self.index_deferredLinks.intersection((min(self.nodes[new_id]['lat'], parent_lat), min(self.nodes[new_id]['lon'], parent_lon), max(self.nodes[new_id]['lat'], parent_lat), max(self.nodes[new_id]['lon'], parent_lon))))

		for linkid in possible_links :
			link = self.deferredLinks[linkid]

			# TODO check distance!

			p1 = (0,0)
			p2 = (self.nodes[new_id]['lat'] - parent_lat, (self.nodes[new_id]['lon'] - parent_lon) * math.cos(math.radians(parent_lat)))
			p3 = (link['location'][0] - parent_lat, (link['location'][1] - parent_lon) * math.cos(math.radians(parent_lat)))

			d = pointToLineDistance(p1,p2,p3)

			#if d > 0.00010:
			if d > 0.00015: # This was 0.00010 when we got the best result 92.1% 21.3%
				continue

			if self.isNodeOnPath(new_id,link['node4']) == True:
				link['state'] = 'covered'



		# Normal Direction Check
		peaks = self.getPeaks(new_id, "rawScoreHorizontal")
		self.nodes[new_id]["rawScoreHorizontalGMM"] = self.getPeaksGMM(new_id, "rawScoreHorizontal")
		peaks = self.nodes[new_id]["rawScoreHorizontalGMM"][0]

		maxpeak = 0.0
		for peak in peaks :
			if self.nodes[new_id]["rawScoreHorizontal"][peak] > maxpeak:
				maxpeak = self.nodes[new_id]["rawScoreHorizontal"][peak]
		for peak in peaks :
			#if peak < 17 or peak > 23:
			if peak < DBDCheckRange - DBDExclusionRange or peak > DBDCheckRange + DBDExclusionRange:
				if self.nodes[new_id]["rawScoreHorizontal"][peak] < maxpeak * 0.05:
					continue
				# Add deferred links
				item = {}
				item['node4'] = self.nodes[new_id]['deferredLinksInfo'][4]
				item['node3'] = self.nodes[new_id]['deferredLinksInfo'][1]
				item['node2'] = self.nodes[new_id]['deferredLinksInfo'][0]
				item['node1'] = new_id
				item['location'] = self.nodes[new_id]['rawScoreHorizontalLoc'][peak]
				item['state'] = 'active'


				data = []

				data.append(self.nodes[item['node3']]['lat'])
				data.append(self.nodes[item['node3']]['lon'])
				data.append(self.nodes[item['node2']]['lat'])
				data.append(self.nodes[item['node2']]['lon'])
				data.append(item['location'][0])
				data.append(item['location'][1])
				data.append(self.nodes[new_id]['deferredLinksInfo'][3])
				data.append(self.nodes[new_id]['deferredLinksInfo'][2])
				data.append(0.00005)
				data.append(self.nodes[new_id]['deferredLinksInfo'][5])
				data.append(self.nodes[new_id]['deferredLinksInfo'][6])

				item['shape'] = [self.nodes[item['node2']]['lat'], self.nodes[item['node2']]['lon']]
				item['shape'] += TraceQuery3PInterpolation(data)
				item['shape'].append(item['location'][0])
				item['shape'].append(item['location'][1])

				# Check shape

				flag = False
				for k in range(0,len(item['shape'])-2, 2):
					if latlondistance((item['shape'][k], item['shape'][k+1]), (item['shape'][k+2], item['shape'][k+3])) > 0.00050:
						flag = True

				if flag == True:
					continue


				peakID = len(self.deferredLinks)
				peaklat =  self.nodes[new_id]['rawScoreHorizontalLoc'][peak][0]
				peaklon =  self.nodes[new_id]['rawScoreHorizontalLoc'][peak][1]

				# Check whether there has already been a segment nearby


				r_lat = 0.00015
				r_lon = r_lat / math.cos(math.radians(peaklat))


				#possible_segment = list(self.index_segment.intersection((peaklat-0.00010, peaklon-0.00015, peaklat+0.00010, peaklon+0.00015)))
				possible_segment = list(self.index_segment.intersection((peaklat-r_lat, peaklon-r_lon, peaklat+r_lat, peaklon+r_lon)))


				flag = False
				for segment in possible_segment:

					lat2 = self.nodes[segment]['lat']
					lon2 = self.nodes[segment]['lon']

					lat1 = self.nodes[self.nodes[segment]['parent']]['lat']
					lon1 = self.nodes[self.nodes[segment]['parent']]['lon']


					p1 = (0,0)
					p2 = (lat2 - lat1, (lon2 - lon1) * math.cos(math.radians(lat1)))
					p3 = (item['location'][0] - lat1, (item['location'][1] - lon1) * math.cos(math.radians(lat1)))

					d = pointToLineDistance(p1,p2,p3)

					#if d > 0.00010:
					if d > r_lat:
						continue


					if self.isNodeOnPath(segment,item['node4']):
						item['state'] = 'covered'
						flag = True
						break


				last_depth = 0
				last_links = self.nodes[self.nodes[new_id]['parent']]['refDeferredLinks'] 

				if len(last_links) != 0:
					last_depth = self.deferredLinks[last_links[0]]['triggerDepth'] + 2

				item['triggerDepth'] = max(self.maxBFSDepth+5, last_depth)

				with open("Log_deferred_depth", "a") as fout:
					fout.write(str(len(self.deferredLinks))+" "+str(item['triggerDepth']) + " " + str(self.maxBFSDepth) + "\n")



				if flag == False:
					#item['triggerDepth'] = max(self.maxBFSDepth+5, self.deferredLinks_depth)
					self.deferredLinks_depth = item['triggerDepth'] + 1 # total order ?



				if flag == False:
					self.nodes[new_id]['refDeferredLinks'].append(len(self.deferredLinks))

				self.deferredLinks.append(item)

				
				if flag == False:
					self.index_deferredLinks.insert(peakID, (peaklat-0.00010, peaklon-0.00015, peaklat+0.00010, peaklon+0.00015))

		
		reverseDIR = (360/DegreeResolution/2 + _direction) % (360/DegreeResolution)

		for i in range(360/DegreeResolution/3):
			self.nodes[new_id]['score'][(reverseDIR+i-360/DegreeResolution/6) % (360/DegreeResolution)] = 0


		for bias in range(-1,2):
			self.nodes[_parent]['score'][(_direction+bias) % (360/DegreeResolution)] = -1

		
		forestIdx = 0

		if Forest is None:
			return 
		for t in Forest:
			terminate,terminate_type=self.findSimilarNode(new_id, t)
			if terminate != -1:
				for i in range(360/DegreeResolution):
					if self.nodes[new_id]['score'][i] > t.nodes[terminate]['score'][i] and t.nodes[terminate]['score'][i] > -0.5:
						t.nodes[terminate]['score'][i] = self.nodes[new_id]['score'][i]
		 			self.nodes[new_id]['score'][i] = 0

		 		self.nodes[new_id]['similarWith'] = [forestIdx, terminate]
		 		self.nodes[new_id]['referenceCounter'] = 1
		 		self.nodes[new_id]['terminateType'] = terminate_type

		 		self.nodes[new_id]['OutNodes'].append((forestIdx, terminate))
		 		t.nodes[terminate]['InNodes'].append((self.forestIdx, new_id))


		 		# Remove similar loop  Decide to remove this part, too complex
		 		# Decide to add this part back
		 		# The deffered branch detection can introduce a lot of loops...
		 		# Maybe add something to focus only on deffered branch detection
		 		# 

		 		if t == self :
			 		commonRoot = self.findCommonRoot(new_id, terminate, depth = 128)

			 		if commonRoot != -1:

				 		print(new_id, terminate, commonRoot)

				 		path1 = self.getPathBetweenTwoNodes(new_id, commonRoot)
				 		path2 = self.getPathBetweenTwoNodes(terminate, commonRoot)

				 		ddd = 10000
				 		if len(path1) > 128 or len(path2) > 128:
				 			ddd = 100000
				 		else:
				 			ddd,_ = PathSimilarity.PathSimilarityLatLon(path1, path2, threshold = 0.00030)

				 		print("Remove Merged Road!!!", ddd/32.0)

				 		lenp1 = self.getPathLength(path1)
				 		lenp2 = self.getPathLength(path2)


				 		if ddd/32.0 < 0.00020:
				 			# Only remove the new node
				 			self.nodes[new_id]['referenceCounter'] = 0 # Will be deleted soon

				 			# delete it now to avoid racing

				 			cur_node = self.nodes[new_id]
				 			while True:
								if cur_node['referenceCounter'] == 0 and cur_node['OutRegion'] != 1 and cur_node['removed'] == False:
									new_lat = cur_node['lat']
									new_lon = cur_node['lon']
									if abs(new_lat-self.Region[0]) < 0.00025*2 or abs(new_lat-self.Region[2]) < 0.00025*2 or abs(new_lon-self.Region[1]) < 0.00040*2 or abs(new_lon-self.Region[3]) < 0.00040*2: 
										break


									done = False
									cur_node['removed'] = True
									self.nodes[cur_node['parent']]['referenceCounter'] -= 1

									for d in cur_node['refDeferredLinks']:
										self.deferredLinks[d]['state'] = "removed"

									cur_node = self.nodes[cur_node['parent']]
								else:
									break


		 		print("Terminate Node "+str(new_id))
		 		break

		 	forestIdx += 1



	def explore(self, Reigon = None, Forest = None, RegionImage = None, mCNNOutput = None, exitMask = None, exitPoint = None, BFS = True):

		done = True

		def NextNodes(raw_score, cnn_score, mask):
			n = len(raw_score)
			score_mask = [0]*n

			result = [0] * n

			for i in range(n):
				#if raw_score[i] > 0 and raw_score[i] >= raw_score[i-1] and raw_score[i] >= raw_score[(i+1) % n]:
				if raw_score[i] > 0 and raw_score[(i+1) % n] > 0 and raw_score[(i-1+n) % n] > 0 and raw_score[i] >= raw_score[(i-1)] and raw_score[i] >= raw_score[(i+1) % n]:
					result[i] = raw_score[i] 
					for j in range(-1,2):
						score_mask[(i+j)%n] = 1


			return result



		# Find the node with max score
		_parent = -1
		_direction = -1
		max_score = 0
		min_index = 0

		# BFS
		#for node in self.nodes[self.nodes_cur:]:
		for node in self.nodes:

			if node['isDead'] == 1:
				cur_node = node 

				# Remove them 1 layer by 1 layer (depth)
				while True:
					if cur_node['referenceCounter'] == 0 and cur_node['OutRegion'] != 1 and cur_node['removed'] == False:
						new_lat = cur_node['lat']
						new_lon = cur_node['lon']
						if abs(new_lat-self.Region[0]) < 0.00025*2 or abs(new_lat-self.Region[2]) < 0.00025*2 or abs(new_lon-self.Region[1]) < 0.00040*2 or abs(new_lon-self.Region[3]) < 0.00040*2: 
							break


						done = False
						cur_node['removed'] = True
						self.nodes[cur_node['parent']]['referenceCounter'] -= 1

						for d in cur_node['refDeferredLinks']:
							self.deferredLinks[d]['state'] = "removed"

						cur_node = self.nodes[cur_node['parent']]
					else:
						break

				continue

			if node['isExited'] == True:
				continue

			if node['OutRegion'] == 1:
				continue



			cnnSoft = node['CNNCheckResult']

			if len(node['nextNode']) == 0:

				nodeScore = list(node['rawScore'])
				#if node['stepSize'] == 1.0:
				#	nodeScore = list(node['rawScore'])

				if node['stepSize'] == 0.8:
					nodeScore = list(node['rawScoreStepsize1'])

				if node['stepSize'] == 0.6:
					nodeScore = list(node['rawScoreStepsize2'])






				low_threshold = 5.0  # Was 5.0
				low_threshold = 0.1
				#next_nodes = NextNodes(node['rawScore'],cnnSoft, cnnSoft)
				next_nodes = NextNodes(nodeScore,None, None)

				# Need more test = = not stable yet
				# Enable this, should work
				# Angle Limitation
				if node['parent'] != node['id'] and self.nodes[node['parent']]['Aux'] != 1:

					mask = [0] * (360/DegreeResolution)

					p_dir = node['parentDIR']

					p_next_node = self.nodes[node['parent']]['nextNode']


					p_cur = p_dir + 2

					for i in range(360/DegreeResolution/3 - 2):
						if p_next_node[(p_cur+360/DegreeResolution) % (360/DegreeResolution)] > low_threshold:
							break

						mask[(p_cur+360/DegreeResolution) % (360/DegreeResolution)] = 1
						p_cur = p_cur + 1



					p_cur = p_dir - 2

					for i in range(360/DegreeResolution/3 - 2):
						if p_next_node[(p_cur+360/DegreeResolution) % (360/DegreeResolution)] > low_threshold:
							break

						mask[(p_cur+360/DegreeResolution) % (360/DegreeResolution)] = 1
						p_cur = p_cur - 1


					for p_cur in range(p_dir - 2, p_dir + 3):
						mask[(p_cur+360/DegreeResolution) % (360/DegreeResolution)] = 1


					for i in range(360/DegreeResolution):
						if mask[i] == 0:
							next_nodes[i] = 0


				node['nextNode'] = next_nodes


				# Forks

				if node['id'] == 0 and node['initDir'] != -1:
					
					p_dir = int(node['initDir'] / DegreeResolution)
					print("Limited Angle Entrance", p_dir - 2, p_dir + 2)

					for p_cur in range(p_dir - 2, p_dir + 3):
						node['nextNode'][(p_cur+360/DegreeResolution) % (360/DegreeResolution)] *= -1

					for i in range(360/DegreeResolution):
						if node['nextNode'][i] < 0:
							node['nextNode'][i] *= -1
						else:
							node['nextNode'][i] = 0



				### 

				for nd in node['nextNode']:
					if nd > low_threshold:
						node['nextNodeNumber'] += 1 



			next_nodes = node['nextNode']




			inRange = True
			if Reigon is not None:
				if node['lat']<Reigon[0] or node['lat']>Reigon[2]:
					inRange = False
				if node['lon']<Reigon[1] or node['lon']>Reigon[3]:
					inRange = False

			debug_print(node['lat'],node['lon'], Reigon, inRange)

			if inRange == True:
				if RegionImage is not None:
					ix = int((1.0-(node['lat'] - Reigon[0])/(Reigon[2] - Reigon[0])) * np.shape(RegionImage)[0])
					iy = int(((node['lon'] - Reigon[1])/(Reigon[3] - Reigon[1])) * np.shape(RegionImage)[1])

					if RegionImage[ix,iy] == 0 :
						inRange = False

					debug_print(ix, iy, np.shape(RegionImage),  RegionImage[ix,iy], inRange)

			if inRange == False:
				node['OutRegion'] = 1
				#print(node['id'],"Out of Region")
				continue


			if exitMask is not None:
				ix = int((1.0-(node['lat'] - Reigon[0])/(Reigon[2] - Reigon[0])) * np.shape(exitMask)[0])
				iy = int(((node['lon'] - Reigon[1])/(Reigon[3] - Reigon[1])) * np.shape(exitMask)[1])

				exitID = exitMask[ix,iy]

				node['exitID'] = exitID
				#if node['exitID'] == self.nodes[node['parent']]['exitID'] and node['exitID'] == self.nodes[self.nodes[node['parent']]['parent']]['exitID'] and node['exitID'] != 0:
				
				if node['exitID'] == self.nodes[node['parent']]['exitID'] and node['exitID'] != 0:
					node['isExited'] = True
					#print(node['id'],"Exit the Region", node['exitID'])
					continue


			node['isDead'] = 1 
			for i in range(360/DegreeResolution):
				
				debug_print(i, node['score'][i], max_score, next_nodes[i], inRange, node['children'])

				if node['score'][i] > max_score and next_nodes[i] > 0 and inRange and i not in node['children']:
					_parent = node['id']
					_direction = i
					max_score = node['score'][i]

					
					

				if  node['score'][i] > 0 and next_nodes[i] > 0 and inRange and i not in node['children']:
					node['isDead'] = 0


			if max_score > 0:
				if BFS == True:
					break


		debug_print(_parent, max_score)
		if _parent > -1 and max_score > 0:
			self.nodes_cur = _parent

			self.nodes[_parent]['referenceCounter'] += 1

			# Add new node
			parent_lat = self.nodes[_parent]['lat']
			parent_lon = self.nodes[_parent]['lon']
			new_id = len(self.nodes)


			new_parent = _parent

			hyperJump = 0 # Removed?

			# Half step size
			node = InitNode(parent_lat + math.sin(math.radians(_direction * DegreeResolution)) * StepSize/2, parent_lon + math.cos(math.radians(_direction * DegreeResolution)) / math.cos(math.radians(parent_lat)) * StepSize/2, new_id, new_parent, [], _direction) 
			self.nodes.append(node)

			node['InNodes'].append((-1, new_parent))
			self.nodes[new_parent]['OutNodes'].append((-1, new_id, -1))

			self.updateScoreMultipleStepSize(new_id)
			self.updateNewNode(new_id, Forest, _parent, parent_lat, parent_lon, _direction, max_score)


			return True, max_score

		else:
			return (not done), max_score



class RoadForest:
	def __init__(self, config):
		self.config = config

		self.Region = config["Region"]

		startingPoints = []

		for item in config["Entrances"]:

			startingPoints.append([item['Lat'], item['Lon'], -1])

			if 'InitDirection' in item.keys():
				startingPoints[-1][2] = item['InitDirection']


		if config["CNNInput"] is not None:
			self.CNNOutput = CNNOutput(config["CNNInput"], self.Region)
		else:
			self.CNNOutput = None


		# mask = scipy.ndimage.imread(config["RegionMask"])


		# RegionImage = np.zeros((np.shape(mask)[0], np.shape(mask)[1]), dtype = np.uint8)

		# Red = mask[:,:,0]
		# Blue = mask[:,:,2]
		# Alpha = mask[:,:,3]

		# RegionImage[np.where((Blue > 10) & (Alpha) != 0)] = 255
		# self.RegionImage = RegionImage

		# disabled
		self.RegionImage = None 




		#ExitImage = np.zeros((np.shape(mask)[0], np.shape(mask)[1]), dtype = np.uint8)
		#ExitImage = Red

		#ExitImage[np.where((Alpha == 0))] = 0 

		#self.ExitMask = ExitImage
		self.ExitMask = None
		self.ExitPoints = {}

		for item in config["Exits"]:
			self.ExitPoints[item['ID']] = ((item['Lat'], item['Lon']))


		self.RoadTrees = []
		self.oks = []

		self.score = []




		
		ind = 0
		for startingPoint in startingPoints:
			newTree = RoadTree(config, self.CNNOutput)
			newTree.Region = self.Region
			node = InitNode(startingPoint[0],startingPoint[1],0,0,[],0)
			node['initDir'] = startingPoint[2]
			print(startingPoint[0],startingPoint[1])
			newTree.addInitialRoads([node])

			self.oks.append(True)
			self.score.append((ind, 1))
			newTree.forestIdx = ind 

			ind += 1

			self.RoadTrees.append(newTree)

			

		for tree in self.RoadTrees:
			tree.forest = self.RoadTrees  # reference 
			tree.updateScoreMultipleStepSize(0)





	def explore(self, idx = 0):
		tcounter = 0

		def getKey(item):
			return item[1]



		max_score = 0


		for tree in self.RoadTrees:

			if self.oks[tcounter] == True:
				tree.updateDeferredLink()

				self.oks[tcounter], score = tree.explore(self.Region, self.RoadTrees, self.RegionImage, self.CNNOutput, self.ExitMask, self.ExitPoints)

				max_score = max(max_score, score)


			tcounter += 1

		#print("State:",idx, oks)

		okcounter = 0

		oklist = []
		counter = 0
		for s in self.oks:
			if s==False:
				okcounter += 1
			else:
				oklist.append(counter)
			counter += 1

		print("State:", idx, "maxscore", max_score, oklist ,okcounter,"/",len(self.oks))

		if okcounter == len(self.oks):
			return False
		else:
			return True

	def exploreOneByOne(self, idx = 0):
		tcounter = 0

		#if idx == 0:
		self.current_tid = 0

		def getKey(item):
			return item[1]

		#sorted(self.score, reverse=True, key=getKey)


		#for ind in range(len(self.score)):
		#	tree = self.RoadTrees[self.score[ind][0]]


		max_score = 0


		for tree in self.RoadTrees:
			if self.oks[tcounter] == True:
				tree.updateDeferredLink()

				self.oks[tcounter], score = tree.explore(self.Region, self.RoadTrees, self.RegionImage, self.CNNOutput, self.ExitMask, self.ExitPoints)

				max_score = max(max_score, score)

				break


			tcounter += 1


		if tcounter != self.current_tid or idx % 2000 == 0:
			with open("LogExplorOneByOne","a") as fout:
				fout.write(str(tcounter)+" "+str(idx)+"\n")

			self.current_tid = tcounter



		#print("State:",idx, oks)

		okcounter = 0

		oklist = []
		counter = 0
		for s in self.oks:
			if s==False:
				okcounter += 1
			else:
				oklist.append(counter)
			counter += 1

		print("State:", idx, "maxscore", max_score, oklist ,okcounter,"/",len(self.oks))

		if okcounter == len(self.oks):
			return False
		else:
			return True




	def exploreSingle(self, tid=0, idx=0):
		ok = self.RoadTrees[tid].explore(self.Region, self.RoadTrees, self.RegionImage, self.CNNOutput)
		print("State:",tid, idx, ok)

		return ok



	def dump(self, filename):
		data = []
		for tree in self.RoadTrees:
			data.append(tree.nodes)

		pickle.dump([self.Region, data, self.config], open(filename, "wb"))



	def load(self, filename):
		dumpDat = pickle.load(open(filename, "rb"))
		tcounter = 0
		for tree in self.RoadTrees:
			tree.nodes = dumpDat[1][tcounter]
			tcounter += 1

		self.config = dumpDat[2]
		self.Region = dumpDat[0]





if __name__ == "__main__":
	global StepSize


	config_file = sys.argv[1]
	config = {}
	dumpname = sys.argv[2]

	
	with open(config_file, "r") as fin:
		config = json.load(fin)


	# Some Default Internal Configs (overrided by the config file)
	if 'history_length' not in config.keys():
		# You may choose one of the following choices. 
		# The larger the number is, the 'higher' the threshold is.
		# A 'higher' threshold will lead to high precision but low recall.

		config['history_length'] = 3
		#config['history_length'] = 4
		#config['history_length'] = 5
		#config['history_length'] = 6



	if 'minimal_number_of_trips' not in config.keys():
		config['minimal_number_of_trips'] = 2

	if 'minimal_number_of_trips_deferred_branch' not in config.keys():
		config['minimal_number_of_trips_deferred_branch'] = 2

	if 'merge_detector' not in config.keys():
		# The 'distance_only' approach is the basic one. It will only
		# check the distance of two road segments. If they are close 
		# enough, they will be merged.

		# The 'kde_based' approach add some heuristic criterions
		# for those regions with heavy GPS noise.

		#config['merge_detector'] = 'distance_only'
		config['merge_detector'] = 'kde_based'

	if 'merge_detector_search_threshold' not in config.keys():
		config['merge_detector_search_threshold'] = 0.00025

	if 'merge_detector_evaluate_all' not in config.keys():
		config['merge_detector_evaluate_all'] = True

	if 'merge_detector_path_length' not in config.keys():
		config['merge_detector_path_length'] = 12 # 

	if 'keep_deadend' not in config.keys():
		# This option determines wheterh we should keep those dead ends.
		config['keep_deadend'] = False # (default)
		#config['keep_deadend'] = True

	print(config)

	#config['history_length'] = 5
	#config['history_length'] = int(sys.argv[5])

	#config['minimal_number_of_trips'] = int(sys.argv[3])
	#config['minimal_number_of_trips_deferred_branch'] = int(sys.argv[4])

	# global base_port
	# base_port = int(sys.argv[6])

	# subname = sys.argv[7]

	subname = ""

	
	dumpname += "_"+str(config['minimal_number_of_trips'])+"_"+str(config['minimal_number_of_trips_deferred_branch'])+"_"+str(config['history_length'])+"_"+str(config['keep_deadend'])
	dumpname += "_"+str(StepSize)+"_"+str(DBDCheckRange)+"_"+str(config['merge_detector_path_length'])+"_"
	dumpname += "20180731_"+str(base_port)+"_"

	date = "20180731_auto_radius_stable_"+subname

	
	Popen("mkdir -p %s"% config['OutputFolder']+"/"+date, shell=True).wait()

	config['OutputFolder'] = config['OutputFolder'] + "/"+date 


	
	roadForest = RoadForest(config)

	start_ts = time()

	for i in xrange(500000):
		print("Time Elpsed ", time()-start_ts, " s")
		if roadForest.exploreOneByOne(idx = i) == False:
			roadForest.dump(config['OutputFolder']+"/"+dumpname+str(i)+"_last")
			print("Exploration finished at", i)
			exit()

		# save the map every 1000 iterations
		if (i % 1000 == 0):
			roadForest.dump(config['OutputFolder']+"/"+dumpname+str(i))


	
