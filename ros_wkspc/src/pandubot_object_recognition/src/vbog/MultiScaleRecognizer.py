import cv2
import os
import math
import random
import pickle
import sys

import cv2.cv as cv
import numpy as np
import matplotlib.cm as cm

from matplotlib import pyplot as plt
from subprocess import PIPE, Popen
from sklearn import svm
from sklearn.metrics import confusion_matrix

## Custom files:
import AD as anisotropic_diffuser
# from PP import PP as anisotropic_diffuser
import extractSIFT as SIFTExtractor

class MultiScaleRecognizerObject:
  def __init__(self):
    self.ADParams = (4, 50, 0.1, 1, (1.,1.,1.,)) #(NIter, Kappa, Gamma, Option, Step)
    self.ResizeAmt = (1920, 1080)
    self.Stride = (50, 50)
    self.PatchSize = (50,50)
    self.NumScales = 9
    self.ScaleFactor = 1.25
    self.model_file = open('/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_object_recognition/src/vbog/TrainedSVMModel.pickle','rb')
    self.SVMModel, self.TextLabels, self.km = pickle.load(self.model_file)

  def preprocess_bgr_frame(self,bgr_frame):
    bgr_frame = cv2.resize(bgr_frame, self.ResizeAmt, interpolation = cv2.INTER_CUBIC)
    bgr_frame = self.Colorbalance(bgr_frame)
    NIter  = self.ADParams[0]
    Kappa  = self.ADParams[1]
    Gamma  = self.ADParams[2]
    Option = self.ADParams[3]
    Step   = self.ADParams[4]
    bgr_frame = anisotropic_diffuser.anisodiff3(bgr_frame,NIter,Kappa,Gamma,Step,Option,False)
    gray_frame = cv2.cvtColor(bgr_frame,cv2.COLOR_BGR2GRAY)
    return gray_frame

## Input Frame should be bgr opncv
## The SIFT extractor and the anisotropic diffuser use grayscale internally.
  def predictLabel(self,bgr_frame):
    gray_frame = self.preprocess_bgr_frame(bgr_frame)
    label,prob = self.SIFTPredictor(gray_frame,self.SVMModel,self.km)
    return label

  def SIFTPredictor(self,gray_img,SVMModel,km):
    gray_img = np.float32(gray_img)
    f,d = SIFTExtractor.extractSIFT(gray_img)
    if np.shape(f)[1]==0:
     print("skipped")
     return (100,0)
    # # plt.imshow(gray_img)
    # Xc = f[0,:]
    # Yc = f[1,:]
    # plt.scatter(x=Xc,y=Yc,c='r',s=40)
    # plt.show()  
    tempvar = km.predict(np.transpose(d))
    hist = (np.histogram(tempvar,range(1,km.n_clusters+1)))
    testData = hist[0]
    predictedLabel = SVMModel.predict(testData)
    predictedProbabilities = SVMModel.predict_proba(testData)
    return (predictedLabel,predictedProbabilities)

  def Colorbalance(self, bgr_img):
    B, G, R = cv2.split(bgr_img)
    BAvg = np.mean(B)
    GAvg = np.mean(G)
    RAvg = np.mean(R)
    K    = np.mean((RAvg, GAvg, BAvg))
    BNew = np.multiply(B, K/BAvg)
    GNew = np.multiply(G, K/GAvg)
    RNew = np.multiply(R, K/RAvg)
    bgr_img = cv2.merge((BNew,GNew,RNew))
    np.clip(bgr_img, 0, 255, out=bgr_img)
    bgr_img = bgr_img.astype('uint8')
    return bgr_img
