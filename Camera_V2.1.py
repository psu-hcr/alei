import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import math
import pandas as pd
import time
N=1 #number of targets that are desired to be watched
location= [[0]*3]*N #used to store the location of the objects to measure the distance between them
locationOrigin=[0,0,0]
DistanceData=[] #stores the data of the distance between the two objects
TimeData=[] #stores the time of each sample 

#this section just sets up and takes in the camera
def nothing(x):
    pass
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
profile = pipe.start(config)

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2
cv2.namedWindow("Peg",cv2.WINDOW_NORMAL) #creates the window that is used to adjust the colors

cv2.createTrackbar("LH","Peg", 103, 255, nothing)#creates the color tracking bars
cv2.createTrackbar("LS","Peg", 117, 255, nothing)
cv2.createTrackbar("LV","Peg", 62, 255, nothing)
cv2.createTrackbar("UH","Peg", 132, 255, nothing)
cv2.createTrackbar("US","Peg", 255, 255, nothing)
cv2.createTrackbar("UV","Peg", 114, 255, nothing)

cv2.namedWindow("Base",cv2.WINDOW_NORMAL) #creates the window that is used to adjust the colors for origin object

cv2.createTrackbar("LH","Base", 147, 255, nothing)#creates the color tracking bars
cv2.createTrackbar("LS","Base", 237, 255, nothing)
cv2.createTrackbar("LV","Base", 117, 255, nothing)
cv2.createTrackbar("UH","Base", 196, 255, nothing)
cv2.createTrackbar("US","Base", 255, 255, nothing)
cv2.createTrackbar("UV","Base", 144, 255, nothing)

starttime=time.time()
try:
  while True:
    
    L_HOb=cv2.getTrackbarPos("LH","Peg") #gets data from the tracking bars objects
    L_SOb=cv2.getTrackbarPos("LS","Peg")
    L_VOb=cv2.getTrackbarPos("LV","Peg")
    U_HOb=cv2.getTrackbarPos("UH","Peg")
    U_SOb=cv2.getTrackbarPos("US","Peg")
    U_VOb=cv2.getTrackbarPos("UV","Peg")

    L_HOg=cv2.getTrackbarPos("LH","Base") #gets data from the tracking bars origin
    L_SOg=cv2.getTrackbarPos("LS","Base")
    L_VOg=cv2.getTrackbarPos("LV","Base")
    U_HOg=cv2.getTrackbarPos("UH","Base")
    U_SOg=cv2.getTrackbarPos("US","Base")
    U_VOg=cv2.getTrackbarPos("UV","Base")

    # Store next frameset for later processing:
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    color = np.asanyarray(color_frame.get_data())#gets color fram data
    res = color.copy()
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

    l_bob = np.array([L_HOb, L_SOb, L_VOb])#sets lower and upper bounds of the colors
    u_bob = np.array([U_HOb, U_SOb, U_VOb])

    l_bog = np.array([L_HOg, L_SOg, L_VOg])#sets lower and upper bounds of the colors Origin
    u_bog = np.array([U_HOg, U_SOg, U_VOg])

    maskob = cv2.inRange(hsv, l_bob, u_bob)#creates a mask with range that takes in the hsv image and gets rid of any colors outside the range
    colorob = cv2.bitwise_and(color, color, mask=maskob)#bitwise and between the mask and the color image
    
    maskog=cv2.inRange(hsv,l_bog,u_bog)
    colorog=cv2.bitwise_and(color,color,mask=maskog)

    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())#mmakes array from colorized depth frame

    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

   
    (cob, _) = cv2.findContours(maskob, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#finds a list of contours using the mask image 
    (cog, _) = cv2.findContours(maskog, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#finds a list of contours using the mask image 
    
    color_init = color
   # c=np.array(c)
    
    depth = np.asanyarray(aligned_depth_frame.get_data())#turns the depth frame data into an array

#This is the start of the origin tracker

    largestArea=0
    index=0
    for i in range(len(cog)):#looks through the contour array to find the largest contour
        if(cv2.contourArea(cog[i])>largestArea):
           
                LargestContour=cog[i]
                index=i
                largestArea=cv2.contourArea(LargestContour)
                
    
       
    if (largestArea>0):#if there is a contour
            
            
        (x, y, w, h) = cv2.boundingRect(LargestContour)   #finds coords of rectangle around the contour       

        cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3)#draws rectangle on the color image around contour
        #creates the text for the position of the object.
        cv2.rectangle(colorized_depth, (x, y), (x + w, y + h), (255, 0, 0), 3)#draws recangle of contour on depth image
        
#This is the start of the object tracker
    for n in range(N):#for N objects, outlines and finds the position of the Nth largest objects
        largestArea=0
        index=0
        for i in range(len(cob)):#looks through the contour array to find the largest contour
            if(cv2.contourArea(cob[i])>largestArea):
           
                LargestContour=cob[i]
                index=i
                largestArea=cv2.contourArea(LargestContour)

        if(cob):        
            cob.pop(index)#removes the selected contour so that the next iteration finds the next largest contour
       
        if (largestArea>0):#if there is a contour
            
            
            (x, y, w, h) = cv2.boundingRect(LargestContour)   #finds coords of rectangle around the contour       

            dist=aligned_depth_frame.get_distance(x+int(w/2),y+int(h/2))#finds the distance to the center of the contour in meter
           
            cv2.rectangle(colorized_depth, (x, y), (x + w, y + h), (255, 0, 0), 3)#draws recangle of contour on depth image
            cv2.rectangle(colorized_depth, (x, y), (x + w, y + h), (255, 0, 0), 3)#draws recangle of contour on depth image

    
        

    #updates/shows windows
    cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RBG', res)
    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Depth', colorized_depth)
    cv2.namedWindow('maskog', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('maskog', maskog)
    cv2.namedWindow('maskob', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('maskob', maskob)
    

    cv2.waitKey(1)#waits 0.01 seconds
except KeyboardInterrupt:
    pass
finally:
  pipe.stop()