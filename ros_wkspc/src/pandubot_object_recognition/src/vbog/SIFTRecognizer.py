import numpy as np
import vlfeat
from sklearn import cross_validation
from sklearn import cluster
import pickle
from sklearn.neighbors import KNeighborsClassifier
import extractSIFT as ex
import matplotlib.pyplot as plt

def testSIFT(gray_img, SVMModel, km):
  f,d = ex.extractSIFT(gray_img)
  if np.shape(f)[1]==0:
    print("skipped")
    return (100,0)


  plt.imshow(gray_img)
  Xc = f[0,:]
  Yc = f[1,:]
  plt.scatter(x=Xc,y=Yc,c='r',s=40)
  plt.show()  

  tempvar = km.predict(np.transpose(d))
  hist = (np.histogram(tempvar,range(1,km.n_clusters+1)))
  testData = hist[0]
  predictedLabel = SVMModel.predict(testData)
  predictedProbabilities = SVMModel.predict_proba(testData)
  return (predictedLabel,predictedProbabilities)
