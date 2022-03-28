import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import math
import pandas as pd
import time

# storing array for peg and hole
data = np.zeros(7)

# Create a pipeline
pipe = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipe.start(config)

# Getting the depth sensor's depth scale 
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 2.5
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


# Format for text
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2

#parameter for tracking bars objects


# blue peg
L_HOb=100 
L_SOb=150
L_VOb=10
U_HOb=132
U_SOb=255
U_VOb=140

# red peg
L_HOg=170
L_SOg=10
L_VOg=10
U_HOg=180
U_SOg=255
U_VOg=255

# yellow base
L_Hy=20
L_Sy=98
L_Vy=38
U_Hy=45
U_Sy=255
U_Vy=255

#sets lower and upper bounds of the colorsS
blueLower = np.array([L_HOb, L_SOb, L_VOb])
blueUpper = np.array([U_HOb, U_SOb, U_VOb])

#sets lower and upper bounds of the colors Origin
redLower = np.array([L_HOg, L_SOg, L_VOg])
redUpper = np.array([U_HOg, U_SOg, U_VOg])

#sets lower and upper bounds of the colors Origin
yellowLower = np.array([L_Hy, L_Sy, L_Vy])
yellowUpper = np.array([U_Hy, U_Sy, U_Vy])

# Radius for peg and hole
r_peg = 0.01    #meter
r_base = 0.021  #meter

PegPosition = [[0], [0], [0]]
BasePosition = [[0], [0], [0]]

starttime=time.time()
try:
  while True:
    

    # Get frameset of color and depth
    frameset = pipe.wait_for_frames()
    
    # Align the depth frame to color frame
    aligned_frames = align.process(frameset)
    
    # Get aligned frames
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    
    # Validate that both frames are valid
    if not depth_frame or not color_frame:
        continue

    #gets color and depth from data
    color_image = np.asanyarray(color_frame.get_data())
    depth = np.asanyarray(depth_frame.get_data())
    
    # Remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    depth_image_3d = np.dstack((depth,depth,depth)) #depth image is 1 channel, color is 3 channels
    color = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
    
    res = color.copy()
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    
    #creates a mask with range that takes in the hsv image and gets rid of any colors outside the range
    maskPegRed = cv2.inRange(hsv, redLower, redUpper)
    maskPegBlue = cv2.inRange(hsv, blueLower, blueUpper)
    maskBase = cv2.inRange(hsv,yellowLower,yellowUpper)

    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    
    # define depth camera intrinsics
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

   	#finds a list of contours using the mask image 
    (cPegRed, _) = cv2.findContours(maskPegRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    (cPegBlue, _) = cv2.findContours(maskPegBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	#finds a list of contours using the mask image 
    (cBase, _) = cv2.findContours(maskBase, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    color_init = color
    
    depth = np.asanyarray(aligned_depth_frame.get_data())#turns the depth frame data into an array

	#Middle peg tracker(red)

    largestArea=0
    index=0
    for i in range(len(cPegRed)):#looks through the contour array to find the largest contour
        if(cv2.contourArea(cPegRed[i])>largestArea):
           
                LargestContour=cPegRed[i]
                index=i
                largestArea=cv2.contourArea(LargestContour)
         
    if (largestArea>0):#if there is a contour
            
        (x, y, w, h) = cv2.boundingRect(LargestContour)   #finds coords of rectangle around the contour
        
        pegCenter = [x+int(w/2), y+int(h/2)]

        bottomLeftCornerOfText = (x, y)#finds coords for text
        
        #finds the distance to the center of the contour in meter
        dist=aligned_depth_frame.get_distance(pegCenter[0],pegCenter[1])
        
        #convert center of the contour into world coordinates
        # Need to change coordinates for rviz. Refer to https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
        PegPosition = rs.rs2_deproject_pixel_to_point(depth_intrin, pegCenter, dist)
        
        # compensate for raduis
        PegPosition[0] = PegPosition[0]+r_peg
        
        #draws rectangle on the color image around contour
        cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3)
        
        #creates the text for the position of the object.
        textpose = "Peg X: " + str("{:.3f}".format(PegPosition[0]))+" Y: " + str("{:.3f}".format(PegPosition[1]))+" Z: " + str("{:.3f}".format(PegPosition[2]))

        #puts the distance and the position info for the contour on the colored screen
        cv2.putText(res,
                textpose,
                bottomLeftCornerOfText,
                font,
                fontScale,
                fontColor,
                lineType)
		
	#Bottom part peg tracker(blue)

    largestArea2=0
    index2=0
    for i in range(len(cPegBlue)):#looks through the contour array to find the largest contour
        if(cv2.contourArea(cPegBlue[i])>largestArea2):
           
                LargestContour2=cPegBlue[i]
                index2=i
                largestArea2=cv2.contourArea(LargestContour2)
         
    if (largestArea2>0):#if there is a contour
            
        (x2, y2, w2, h2) = cv2.boundingRect(LargestContour2)   #finds coords of rectangle around the contour
        
        pegCenterB = [x2+int(w2/2), y2+int(h2/2)]

        bottomLeftCornerOfText = (x2, y2)#finds coords for text
        
        #finds the distance to the center of the contour in meter
        distB=aligned_depth_frame.get_distance(pegCenterB[0],pegCenterB[1])
        
        #convert center of the contour into world coordinates
        # Need to change coordinates for rviz. Refer to https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
        PegPositionB = rs.rs2_deproject_pixel_to_point(depth_intrin, pegCenterB, distB)
        
        # compensate for raduis
        PegPositionB[0] = PegPositionB[0]+r_peg
        
        # calculate orientation
        Orientation = np.array(PegPositionB) - np.array(PegPosition)
        
        #draws rectangle on the color image around contour
        cv2.rectangle(res, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 3)
        
        #creates the text for the position of the object.
        textpose = "Orientation X: " + str("{:.3f}".format(Orientation[0]))+" Y: " + str("{:.3f}".format(Orientation[1]))+" Z: " + str("{:.3f}".format(Orientation[2]))

        #puts the distance and the position info for the contour on the colored screen
        cv2.putText(res,
                textpose,
                bottomLeftCornerOfText,
                font,
                fontScale,
                fontColor,
                lineType)

#This is the start of the base tracker
    largestAreaB=0
    indexB=0
    for i in range(len(cBase)):#looks through the contour array to find the largest contour
        if(cv2.contourArea(cBase[i])>largestAreaB):
           
                LargestContourB=cBase[i]
                indexB=i
                largestAreaB=cv2.contourArea(LargestContourB)
         
    if (largestAreaB>0):#if there is a contour
            
        (xb, yb, wb, hb) = cv2.boundingRect(LargestContourB)   #finds coords of rectangle around the contour
        
        baseCenter = [xb+int(wb/2), yb+int(hb/2)]

        bottomLeftCornerOfTextB = (xb, yb)#finds coords for text
        
        #finds the distance to the center of the contour in meter
        distb=aligned_depth_frame.get_distance(baseCenter[0],baseCenter[1])
        
        #convert center of the contour into world coordinates
        # Need to change coordinates for rviz. Refer to https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
        BasePosition = rs.rs2_deproject_pixel_to_point(depth_intrin, baseCenter, distb) 
        
        # compensate for raduis
        BasePosition[0] = BasePosition[0]+r_peg
        
        #draws rectangle on the color image around contour
        cv2.rectangle(res, (xb, yb), (xb + wb, yb + hb), (0, 255, 0), 3)
        
        #creates the text for the position of the object.
        textbase = "Base X: " + str("{:.3f}".format(BasePosition[0]))+" Y: " + str("{:.3f}".format(BasePosition[1]))+" Z: " + str("{:.3f}".format(BasePosition[2]))

        #puts the distance and the position info for the contour on the colored screen
        cv2.putText(res,
                textbase,
                bottomLeftCornerOfTextB,
                font,
                fontScale,
                fontColor,
                lineType)    
        

    #updates/shows windows
    cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RBG', res)
    
    # current time
    current_time = time.time()-starttime
    
    # update storing
    data = np.vstack((data, np.concatenate((current_time, PegPosition, BasePosition),axis=None)))
    
    cv2.waitKey(1)#waits 0.01 seconds
    
except KeyboardInterrupt:
    pass
finally:
    pipe.stop()
    df = pd.DataFrame(data)
    path = '/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw3_twoturns.csv'
    df.to_csv(path, header=True, index=False)
