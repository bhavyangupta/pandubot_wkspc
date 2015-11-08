import cv2 
import numpy as np


def AutoWB(I):
  B, G, R = cv2.split(I)
  BAvg = np.mean(B)
  GAvg = np.mean(G)
  RAvg = np.mean(R)

  K = np.mean((RAvg, GAvg, BAvg))

  BNew = np.multiply(B, K/BAvg)
  GNew = np.multiply(G, K/GAvg)
  RNew = np.multiply(R, K/RAvg)

  I = cv2.merge((BNew,GNew,RNew))

  # MOST IMPORTANT PART IS TO CLIP OUTPUT!
  np.clip(I, 0, 255, out=I)
  I = I.astype('uint8')
  return I
