#!/usr/bin/python

import cv2
import numpy as np

kernel = np.ones((5,5),np.uint8)
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

    cv2.imshow('edges', edge)
    def filter_region(image, vertices):
        mask = np.zeros_like(image)
        if len(mask.shape) == 2:
            x = cv2.fillPoly(mask, vertices, 255)
            cv2.imshow('x', x)
        else:
            y = cv2.fillPoly(mask, vertices, (255,)*mask.shape[2])
        return cv2.bitwise_and(image, mask)


    rows, cols = edge.shape[:2]
    bottom_left = [cols*.01, rows*0.95]
    top_left = [cols*0.4, rows*0.6]
    bottom_right = [cols*0.9, rows*0.95]
    top_right = [cols*0.6, rows*0.6]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype = np.int32)
    filter_region(edge, vertices)
    
    # def Hough(image):
    #     lines = cv2.HoughLinesP(image,1,np.pi/180,100,1,300)

    #     if lines is not None:
    #         for line in lines:
    #             pt1 = (line[0], line[1])
    #             pt2 = (line[2], line[3])
    #             line = cv2.line(image, pt1, pt2, (0, 0, 255), 5) 
    #             cv2.imshow('line', line)
    #         return image
    # Hough(edge)
   
    a = cv2.waitKey(5) 
    if a == 20:
        break
cv2.destroyAllWindows()
cap.release()
