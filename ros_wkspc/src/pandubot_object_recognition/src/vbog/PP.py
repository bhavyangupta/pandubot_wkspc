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
import AutoWB
import AD

def PP(I, ADParams):
	# B,G,R is the default order 
	I = AutoWB.AutoWB(I)
	NIter = ADParams[0]
	Kappa = ADParams[1]
	Gamma = ADParams[2]
	Option = ADParams[3]
	Step = ADParams[4]

	I = AD.anisodiff3(I, NIter, Kappa, Gamma, Step ,Option, False)

	return I