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
import InitAll
from SceneTrain import SceneTrain
from SceneTest import SceneTest
from sklearn.metrics import confusion_matrix


# Initialize everything
DataPath, ImagPerClass, NumSub, ImagPerClassToRead, TrainImgs, TestImgs, ResizeAmt = InitAll.InitAll()

# Perform Training
SVMModel = SceneTrain(NumSub, ImagPerClassToRead, DataPath, TrainImgs, ResizeAmt)

# Perform Testing
k = 0
PredictedLabels = np.zeros(sum(ImagPerClass)-NumSub*ImagPerClassToRead, int)
TrueLabels = np.zeros(sum(ImagPerClass)-NumSub*ImagPerClassToRead, int)
for i in range(0,NumSub):
	for j in TestImgs[i]:
		InitAll.tic()
		#print "Tested " + "Subject " + str(i+1) + " Image " + str(j) 
		# Read Images and Generate Image Features XNow 
		XNow = cv2.imread(DataPath+str(i+1)+'frame'+str(j)+'.png',0)
		XNow = cv2.resize(XNow, ResizeAmt, interpolation = cv2.INTER_CUBIC)
		PredictedLabels[k] = SceneTest(XNow, SVMModel)
		TrueLabels[k] = i+1
		k += 1
		#print "Predicted " + str(PredictedLabels[k-1]) + " Actual " + str(i+1)
		InitAll.toc()
	print "Testing Subject "+str(i+1) + " done...."

# Compute confusion matrix
cm = confusion_matrix(TrueLabels, PredictedLabels)

print(cm)

# Show confusion matrix in a separate window
plt.matshow(cm)
plt.title('Confusion matrix')
plt.colorbar()
plt.ylabel('True label')
plt.xlabel('Predicted label')
plt.show()

#If you need to save multiple objects, you can simply put them in a single list, or tuple, for instance:
# obj0, obj1, obj2 are created here...

# Saving the objects:
#with open('objs.pickle', 'w') as f:
#    pickle.dump([X,Y, SVMModel], f)

# Getting back the objects:
#with open('objs.pickle') as f:
#    obj0, obj1, obj2 = pickle.load(f)