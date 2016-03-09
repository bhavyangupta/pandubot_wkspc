#!/usr/bin/env python
import numpy as np
import cv2
import faceDetect as fd
#Create Video capture variable
camera_id = 1 # 0 - default webcam, 1 - usb webcam
cap = cv2.VideoCapture(camera_id)
# cv2.namedWindow('image');
FPS = 30;

while (True):
	#Capture frames
	retvar,img = cap.read()
	print img.shape # (480,640,3)
	print type(img)
	#Compute gist
	#gistfeat = computeGist(img)
	#Classify the image from the above gist
	#sceneClass = classifyMulticlass(svmModel,gistfeat)
	bbox,flag = fd.faceDetect(img)
	# print type(bbox)
	# print(bbox)
	print(flag)
	#Display the frame'
	# if flag:
		# cv2.rectangle(img,(bbox[0,1],bbox[0,1]),(bbox[0,2],bbox[0,3]),1)
	# cv2.imshow('image',img)
	cv2.waitKey(1000/FPS)
cap.release()
cv2.destroyAllWindows()

