import numpy as np
import vlfeat
from sklearn import cluster

def computeKMeans(K,trainDataStack):
	print('Clustering...')
	km = cluster.KMeans(n_clusters=K,init='k-means++')
	km.fit(trainDataStack)
	return km