#!/usr/bin/python

import cv2
import numpy as np

cap = cv2.VideoCapture(1)
while True:
    ret,frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ymn = np.array([20,100,100])
    ymx = np.array([30,255,255])
    

    mask = cv2.inRange(hsv, ymn, ymx)
    res = cv2.bitwise_and(frame, frame, mask = mask)
    


    cv2.imshow('res', res)
    cv2.imshow('mask', mask)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray2 = np.float32(gray)

    edge = cv2.Canny(mask, 70, 150)

    
    def filter_region(image, vertices):
        mask = np.zeros_like(image)
        if len(mask.shape) == 2:
            cv2.fillPoly(mask, vertices, 255)
            
        else:
            cv2.fillPoly(mask, vertices, (255,)*mask.shape[2])
        return cv2.bitwise_and(image, mask)


    rows, cols = edge.shape[:2]
    bottom_left = [cols*.01, rows*0.95]
    top_left = [cols*0.4, rows*0.6]
    bottom_right = [cols*0.9, rows*0.95]
    top_right = [cols*0.6, rows*0.6]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype = np.int32)
    filter_region(edge, vertices)

    cv2.imshow('edges', edge)

    # def hough(image):
    #     return cv2.HoughLinesP(image, rho =1, theta=np.pi/180, threshold=20, minLineLenght=20, maxLineGap=300)
    # l_l = list(map(hough, edge))

   
    k = cv2.waitKey(5) 
    if k == 30:
        break
cv2.destroyAllWindows()
cap.release()