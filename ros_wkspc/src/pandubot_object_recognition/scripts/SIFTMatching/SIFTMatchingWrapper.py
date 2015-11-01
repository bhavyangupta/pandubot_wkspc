#!/usr/bin/env python

import numpy as np
import cv2
from matplotlib import pyplot as plt
import SIFTMatching


img2 = cv2.imread('Cropped.png',0) # trainImage

Label = SIFTMatching.IsMatch(img2)
print Label


