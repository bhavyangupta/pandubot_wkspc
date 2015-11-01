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


def SceneTest(TestImg, SVMModel):
	# Use SVM to predict class label and return it
	return SVMModel.predict(InitAll.ComputeGIST(TestImg))[0] 