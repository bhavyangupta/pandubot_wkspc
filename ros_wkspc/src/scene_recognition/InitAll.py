#! /usr/bin/env python
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

def Pause():
	wait = input("PRESS ENTER TO CONTINUE.")

def ComputeGIST(ImageIn):
	cv2.imwrite('ar1.ppm', ImageIn); # Save File as ar1.ppm for GIST To Read
	GISTFeat = cmdline("./compute_gist ar1.ppm") # Get string from command line
	GISTFeat = np.array([[float(x) for x in GISTFeat.split()]]) # Convert them to actual floats
	return GISTFeat

def cmdline(command):
    process = Popen(
        args=command,
        stdout=PIPE,
        shell=True
    )
    return process.communicate()[0]

def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print "Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds."
    else:
        print "Toc: start time not set"



def InitAll():
	DataPath = '/home/nitin/Nitin/CIS700/SceneDataset/Frames/' # Frames are saved Here

	AllDirs = os.listdir(DataPath)
	AllDirs.sort()
	ClassNo = [int(x[0]) for x in AllDirs]
	ImagPerClass = np.bincount(ClassNo) # It assumes that Names start from Class 0

	ResizeAmt = (256,256)

	NumSub = max(ClassNo)
	print("Read "+str(NumSub)+" Classes....")

	ImagPerClassToRead = 500

	print("Using "+str(ImagPerClassToRead)+" Images for Training per class....")

	# Generate all training and testing image labels (mutually exclusive)
	TrainImgs = np.sort([random.sample(range(1,x), ImagPerClassToRead) for x in ImagPerClass[1:len(ImagPerClass)]]) # This has all the image Numbers to be read


	TestImgs = []
	for i in range(0, NumSub):
		ListNow = np.arange(1, ImagPerClass[1:len(ImagPerClass)][i]+1)
		ListNow = [x for x in ListNow if x not in TrainImgs[i]]
		TestImgs[len(TestImgs):] = [ListNow] # This has all the Testing Image Numbers

	return (DataPath, ImagPerClass, NumSub, ImagPerClassToRead, TrainImgs, TestImgs, ResizeAmt)