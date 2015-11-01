import cv2 
import cv2.cv as cv
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import os
from subprocess import PIPE, Popen
import math
import random
import pickle
from sklearn import svm
from time import gmtime, strftime
import InitAll


def SceneTrain(NumSub, ImagPerClassToRead, DataPath, TrainImgs, ResizeAmt):
	# Read Images and Generate Image Features XNow
	for i in range(0,NumSub):
		for j in range(0, ImagPerClassToRead):
			InitAll.tic()
			#print DataPath+str(i+1)+'frame'+str(TrainImgs[i][j])+'.png'
			XNow = cv2.imread(DataPath+str(i+1)+'frame'+str(TrainImgs[i][j])+'.png',0)
			XNow = cv2.resize(XNow, ResizeAmt, interpolation = cv2.INTER_CUBIC)
			XNow = InitAll.ComputeGIST(XNow)
			#print("Sub " + str(i+1) + " Image " + str(j+1))
			if(i==0 and j==0):
				X = np.reshape(XNow, (1,np.product(XNow.shape)))
			else:
				X = np.vstack((X,np.reshape(XNow, (1,np.product(XNow.shape)))))
			InitAll.toc()
		print "Subject " + str(i+1) + " done...."

	# Now Generate Class Labels Y
	# Class labels start from 1 and not 0
	Y = [i for i in range(1,NumSub+1)]*ImagPerClassToRead
	Y = list(np.sort(Y))

	SVMModel = svm.SVC()
	SVMModel.fit(X, Y)
	# Saving the objects:
	with open('SceneTrainedSVMModel'+strftime("%Y-%m-%d %H:%M:%S", gmtime())+'.pickle', 'w') as f:
		pickle.dump([X, Y, SVMModel], f)
	return SVMModel