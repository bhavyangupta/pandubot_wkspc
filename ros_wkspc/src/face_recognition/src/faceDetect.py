#!/usr/bin/env python

import sys, getopt
import numpy as np
import cv2
import cv2.cv as cv
from common import clock, draw_str

def faceDetect(img,classifier_xml_dir):
    cascade = cv2.CascadeClassifier(classifier_xml_dir)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    t = clock()
    rects = detect(gray, cascade)
    if len(rects)==0:
        facesFound = 0
    else:
        facesFound = 1    
    vis = img.copy()
    draw_rects(vis, rects, (0, 255, 0))
    for x1, y1, x2, y2 in rects:
        roi = gray[y1:y2, x1:x2]
        vis_roi = vis[y1:y2, x1:x2]
    dt = clock() - t
    draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
    return (rects,facesFound)

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)


