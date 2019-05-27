import math
import numpy as np
from scipy import interpolate




def distance(p1, p2):
	a = p1[0] - p2[0]
	b = p1[1] - p2[1]
	return np.sqrt(a*a + b*b)

def latlondistance(p1, p2):
	a = p1[0] - p2[0]
	b = (p1[1] - p2[1]) * math.cos(math.radians(p1[0]))
	return np.sqrt(a*a + b*b)

def rawSimilarity(path1, path2):
	vx,vy = 0, 0
	s = 0 

	for i in range(len(path1)):
		s = s + distance(path1[i], path2[i])

		vx = vx + path1[i][0] - path2[i][0]
		vy = vy + path1[i][1] - path2[i][1]

	ErrVector = (vx, vy)

	return s, ErrVector


def rawSimilarityLatLon(path1, path2, threshold = 1000):
	vx,vy = 0, 0
	s = 0 

	max_d = 0

	for i in range(len(path1)):
		d = latlondistance(path1[i], path2[i])
		if d > threshold:
			d = 1000
		s = s + d

		if d > max_d:
			max_d = d 

	return s, d


def PathSimilarity(path1, path2, interpolation=40, subWindowSize = 32, kind = 'quadratic', threshold = 0.00020): #, kind = 'cubic'):
	
	XX = np.arange(1.0/interpolation,1.0,1.0/interpolation)
	#XX = list(np.arange(0,1,0.1))

	path1Lat = map(lambda x:x[0], path1)
	path1Lon = map(lambda x:x[1], path1)

	path2Lat = map(lambda x:x[0], path2)
	path2Lon = map(lambda x:x[1], path2)


	for i in range(len(path1Lat)):
		d = distance([path1Lat[i], path1Lon[i]], [path2Lat[i], path2Lon[i]])
		if d > threshold:
			return 1000, (0,0)

	duplicate_loc = {}
	flag = False
	for i in range(len(path1Lat)-1):
		if (path1Lat[i], path1Lon[i]) in duplicate_loc.keys():
			flag = True
		else:
			duplicate_loc[(path1Lat[i], path1Lon[i])] = 1

	if flag == True:
		return 1000, (0,0)

	duplicate_loc = {}
	flag = False
	for i in range(len(path2Lat)-1):
		if (path2Lat[i], path2Lon[i]) in duplicate_loc.keys():
			flag = True
		else:
			duplicate_loc[(path2Lat[i], path2Lon[i])] = 1


	if flag == True:
		return 1000, (0,0)
		


	tck1, _ = interpolate.splprep([path1Lat, path1Lon], s=0, k = 3)
	tck2, _ = interpolate.splprep([path2Lat, path2Lon], s=0, k = 3)

	path1Int_ = interpolate.splev(XX, tck1)
	path2Int_ = interpolate.splev(XX, tck2)

	path1Int = map(lambda x: [path1Int_[0][x], path1Int_[1][x]], range(len(path1Int_[0])))
	path2Int = map(lambda x: [path2Int_[0][x], path2Int_[1][x]], range(len(path2Int_[0])))


	n = len(path1Int)



	min_dist = 10000
	err_vec = (0,0)


	for s1 in range(0, n - subWindowSize):
		for s2 in range(0, n - subWindowSize):
			dist, vec = rawSimilarity(path1Int[s1:s1+subWindowSize], path2Int[s2:s2+subWindowSize])

			if dist < min_dist:
				min_dist = dist
				err_vec = vec


	return min_dist, err_vec




def PathSimilarityLatLon(path1, path2, interpolation=40, subWindowSize = 32, threshold = 0.00050):

	dist1 = [0] * len(path1)
	dist2 = [0] * len(path2)

	for i in range(1, len(path1)):
		dist1[i] = dist1[i-1] + latlondistance(path1[i-1], path1[i])

	for i in range(1, len(path2)):
		dist2[i] = dist2[i-1] + latlondistance(path2[i-1], path2[i])

	interval1 = dist1[-1] / (interpolation+1)
	interval2 = dist2[-1] / (interpolation+1)

	P1 = []
	P2 = []

	p = 0

	interval = interval1
	path = path1
	dist = dist1 

	cur_dist = interval

	while cur_dist < dist[-1] and p < len(path)-1:
		if cur_dist >= dist[p] and cur_dist < dist[p+1]:
			alpha = (cur_dist - dist[p]) / (dist[p+1] - dist[p])
			lat = path[p][0] * (1-alpha) + path[p+1][0] * alpha
			lon = path[p][1] * (1-alpha) + path[p+1][1] * alpha

			P1.append((lat, lon))
			cur_dist += interval 
		elif cur_dist >= dist[p+1]:
			p = p + 1

	interval = interval2
	path = path2
	dist = dist2 

	cur_dist = interval
	p = 0
	while cur_dist < dist[-1] and p < len(path)-1:
		if cur_dist >= dist[p] and cur_dist < dist[p+1]:
			alpha = (cur_dist - dist[p]) / (dist[p+1] - dist[p])
			lat = path[p][0] * (1-alpha) + path[p+1][0] * alpha
			lon = path[p][1] * (1-alpha) + path[p+1][1] * alpha

			P2.append((lat, lon))
			cur_dist += interval 
		elif cur_dist >= dist[p+1]:
			p = p + 1



	n = min(len(P1), len(P2))

	P1 = P1[0:n]
	P2 = P2[0:n]

	min_dist = 1000000
	min_dist_max = 0
	for s1 in range(0, n - subWindowSize):
		for s2 in range(0, n - subWindowSize):
			d, d_max = rawSimilarityLatLon(P1[s1:s1+subWindowSize], P2[s2:s2+subWindowSize], threshold = threshold)

			if d < min_dist:
				min_dist = d
				min_dist_max = d_max
				

	# return min_dist, min_dist_max
	return min_dist, 0 



if __name__ == "__main__":
	path1 = [[1,1],[2,2],[3,2.5],[4,3]]
	path2 = [[0.5,0.5], [1.5,1.5], [2.5,2.25], [3.5,2.75]]
	#path2 = [[0.5,2.5], [1.5,3.5], [2.5,4.25], [3.5,4.75]]
	#path2 = [[0,0],[1,0.5],[2,1.1],[3,2],[4,3],[5,3]]
	test = PathSimilarityLatLon(path1, path2, threshold = 100000, subWindowSize= 32) / 32
	print(test)

	exit()

	dist,err_vec = PathSimilarity(path1, path2, interpolation=20)

	print(dist)
	print(err_vec)