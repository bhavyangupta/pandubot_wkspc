import vlfeat
import sklearn
import cv2 
import numpy as np

def extractSIFT(gray_img):
  # No need for conversion here, since the input frame should be grayscale
	# gray= np.float32(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY))
	f,d = vlfeat.vl_sift(gray_img,peak_thresh=0.1)
	return (f,d)