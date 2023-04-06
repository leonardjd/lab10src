import numpy as np
from PIL import Image
import cv2
import os
import math
import sys

FRAME_DIMENSION = (240, 320) #Height and width

def filter_img(on_imgs, off_imgs):
	"""
	This method will return numpy array of led on and led off image
	"""
	on_img_avg = np.average(on_imgs, axis=0)
	off_img_avg = np.average(off_imgs, axis=0)
	difference = on_img_avg - off_img_avg
	binary_difference = ((difference >= 50) * 1).astype(np.float32)

	return on_img_avg, off_img_avg, binary_difference

def find_centroid(area):
	moment = cv2.moments(area)
	centroid_x = moment["m10"]/moment["m00"]
	centroid_y = moment["m01"]/moment["m00"]

	return (centroid_x, centroid_y)

def find_centroids(cnts, bin_img):
	centroids = []
	for i in range(len(cnts)):
		mask = np.zeros((bin_img.shape[0], bin_img.shape[1], 3), dtype = np.uint8)

		#Get the mask of inner area of each contour
		mask = cv2.drawContours(mask, cnts[i], contourIdx = -1, color=(255,255,255), thickness = cv2.FILLED)

		#Convert the mask to binary format
		mask = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
		#print(mask.shape, np.unique(mask))
		mask = (mask > 200) * 1

		centroids.append(find_centroid(mask.astype(np.uint16)))
	return centroids

def sort_points(points):
	"""
	Return a list where elements at idx0 and idx1 are two front leds, idx2 is back led
	Idea: the front leds will have approxiamtely the same y-coordinates, and their y-coordinates are smaller than the back led's.
	The front led on the left will have smaller x-coordinates than the right front led.
	"""
	sorted_points = sorted(sorted(points, key=lambda point : point[1])[:2]) +  [sorted(points, key=lambda point : point[1])[2]]
	return sorted_points

def calc_dist_between_2_points(point1, point2):
	return np.linalg.norm(generate_vector(point1, point2))

def generate_vector(point1, point2):
	return np.array([point2[0] - point1[0], point2[1] - point1[1]])

def connect_points(img, ori_points):
	"""
	Draw the line connecting three points in the given list on the input image
	"""
	points = []
	for i in range(len(ori_points)):
		points.append((round(ori_points[i][0]), round(ori_points[i][1])))
	cv2.line(img, points[0], points[1],(0,0,255), 3)
	cv2.line(img, points[0], points[2],(0,0,255), 3)
	cv2.line(img, points[1], points[2],(0,0,255), 3)
