import numpy as np
import math
import scipy.ndimage
from scipy.ndimage.filters import gaussian_filter


import socket
from PIL import Image
import sys,os

import StringIO


def getGPSHistogram(lat2, lon2, lat1, lon1, lat0, lon0, radius, img_size, region, host="localhost", port=8002, center_lat = None, center_lon = None, filename = "tmp.png"):

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host, port))

	if center_lat is None:
		center_lat = lat0
		center_lon = lon0


	minlat = region[0]
	maxlat = region[2]

	minlon = region[1]
	maxlon = region[3]


	data_string = "G,"+str(lat2)+","+str(lon2)+","+str(lat1)+","+str(lon1)+","+str(lat0)+","+str(lon0)+","+str(radius)+","+str(minlat)+","+str(minlon)+","+str(maxlat)+","+str(maxlon)+","+str(img_size)+", "
	#print(data_string)
	s.send(data_string)
	num_str = s.recv(16384)
 	num = int(num_str.split(' ')[1])

 	#print(num_str)
	s.send("ok")

	result = ""
	for i in range(num):
		result += s.recv(16384)

	imgfile = StringIO.StringIO(result)
	# #print(len(result))

	# f = open("tmp"+filename, 'wb')

	# f.write(result)
	# f.close()

	img = scipy.ndimage.imread(imgfile)

	img = img.astype(np.float)

	#GPSHistogramCache[(lat2, lon2, lat1, lon1, lat0, lon0, radius, img_size)] = gps_img/255.0

	return img


def mergeDetector(path1, path2, port=8002):
	lat = path1[0][0]
	lon = path1[0][1]

	size = 0.00400 # 200 meters

	sub_region = [lat-size, lon-size/math.cos(math.radians(lat)), lat+size, lon+size/math.cos(math.radians(lat))]

	img1 = getGPSHistogram(path1[5][0], path1[5][1], path1[3][0], path1[3][1],path1[0][0], path1[0][1], 0.00005, 512, sub_region, filename = "path1.png", port=port)
	img2 = getGPSHistogram(path2[5][0], path2[5][1], path2[3][0], path2[3][1],path2[0][0], path2[0][1], 0.00005, 512, sub_region, filename = "path2.png", port=port)


	img1[192:320,192:320] = 0
	img2[192:320,192:320] = 0


	blurred1 = gaussian_filter(img1, sigma=4)
	blurred2 = gaussian_filter(img2, sigma=4)

	m1 = np.amax(blurred1)
	m2 = np.amax(blurred2)

	if m1 != 0:
		blurred1 /= m1 

	if m2 != 0:
		blurred2 /= m2

	diff = blurred2 - blurred1
	diff[np.where(diff<0)] = 0

	sum_diff = np.sum(diff)
	sum_2 = np.sum(blurred2)

	if sum_2 == 0:
		return 0.0


	#print(sum_diff, sum_2, 1.0 - sum_diff/sum_2)

	return 1.0 - sum_diff/sum_2












