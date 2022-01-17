 #!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  4 12:04:25 2019

@author: ryan harvey
"""
import cv2
import numpy as np
import numpy
import math
import os

#import time
# creates an object for the video capture class

# capture  one from (image) of video, save the 
greenHueMin = 45
greenHueMax = 70

blueHueMin = 135
blueHueMax = 165

yellowHueMin = 20
yellowHueMax = 30

redHueMin = 0
redHueMax = 3

green = (0,255,0)
blue = (255,0,0)
red = (0,0,255)
yellow = (0,255,255)

fps = 60

lineColor = (255,255,255)

voice1 = True
voice2 = True

def readFile():
    text_file = open("videoisolation.txt",'r')
    text_lines = text_file.readlines()
    text_words = []
    
    for line in text_lines:
        templine = line.split(":")
        templine[-1] = templine[-1].strip()
        templine[-1] = templine[-1].strip('\n')
        text_words.append(templine)
    return text_words
    text_file.close()

text_words = readFile()


while True:
    
    text_words = readFile()
    
    LowerAngleRange = int(text_words[1][1])
    UpperAngleRange = int(text_words[2][1])
    
    vid = cv2.VideoCapture(0)
    ret, frame = vid.read()
    cv2.namedWindow('IsolateGreen')
    cv2.namedWindow('IsolateBlue')
    cv2.namedWindow('IsolateYellow')
    cv2.namedWindow('isolateRed')
    cv2.namedWindow('joint')
    
    height = frame.shape[0]
    width = frame.shape[1]
    channels = frame.shape[2]
    black = numpy.zeros((height,width,channels), numpy.uint8)
    black[0:height,0:width, 0:channels] = [0,0,0]
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # converting original image to new image wth HSV values
    
    lowerGreen = np.array([greenHueMin,100,0])     # creating an array for the bottom hsv cutoff
    upperGreen = np.array([greenHueMax,255,255]) # creating an array for the top hsv cutoff
    
    #creating bottom and top values for the blue color
    lowerBlue = np.array([blueHueMin,100,50])     
    upperBlue = np.array([blueHueMax,255,255]) 
    
    #creating bottom and top values for the yellow color
    lowerYellow = np.array([yellowHueMin,100,0])     
    upperYellow = np.array([yellowHueMax,255,255]) 
    
    #the red color requirs two ranges because it is represented on the back and front of the hsv spectrum
    lowerRedOne = np.array([redHueMin,135,50])
    upperRedOne = np.array([redHueMax,255,255])
    
    lowerRedTwo = np.array([177,135,50])
    upperRedTwo = np.array([180,255,255])
    
    # this converts all pixels that are not in the lower to upper ranges to null for each of the differnt filters
    greenColor_filter = cv2.inRange(hsv,lowerGreen,upperGreen) 
    
    blueColor_filter = cv2.inRange(hsv,lowerBlue,upperBlue) 
    
    yellowColor_filter = cv2.inRange(hsv,lowerYellow,upperYellow) 
    
    redColor_filter1 = cv2.inRange(hsv,lowerRedOne,upperRedOne)
    redColor_filter2 = cv2.inRange(hsv,lowerRedTwo,upperRedTwo)

    #this assigns the filters to new variables called masks which are used to create new videos of each color
    greenColor_mask = greenColor_filter
    
    blueColor_mask = blueColor_filter
    
    yellowColor_mask = yellowColor_filter
    
    redColor_mask = redColor_filter1 + redColor_filter2
     
    # applies the mask to the original image and puts it into a new variable bois 
    greenNewVid = cv2.bitwise_or(frame, frame, mask = greenColor_mask)
    
    blueNewVid = cv2.bitwise_or(frame, frame, mask = blueColor_mask)
    
    yellowNewVid = cv2.bitwise_or(frame, frame, mask = yellowColor_mask)
    
    redNewVid = cv2.bitwise_or(frame, frame, mask = redColor_mask)
    #creates bois which is an image with only the chosen color with everything else null
    
    #these meathods, erode and dialate refine the image so only large clumps of each color appear
    #this makes the color detection more accurate
    kernel = np.ones((5,5),np.uint8)
    greenNewVid = cv2.erode(greenNewVid,kernel,iterations = 2)
    greenNewVid = cv2.dilate(greenNewVid,kernel,iterations = 1)
    
    kernel = np.ones((5,5),np.uint8)
    yellowNewVid = cv2.erode(yellowNewVid,kernel,iterations = 2)
    yellowNewVid = cv2.dilate(yellowNewVid,kernel,iterations = 1)
    
    kernel = np.ones((5,5),np.uint8)
    redNewVid = cv2.erode(redNewVid,kernel,iterations = 2)
    redNewVid = cv2.dilate(redNewVid,kernel,iterations = 1)
    
    kernel = np.ones((5,5),np.uint8)
    blueNewVid = cv2.erode(blueNewVid,kernel,iterations = 2)
    blueNewVid = cv2.dilate(blueNewVid,kernel,iterations = 1)
    
    #this is creating variable of "coordinates" of each color spots
    #it measures the center of mass for each color and loads that data into a variable
    Gmoment_of_inertia = cv2.moments(greenColor_filter)
    GmHorizontal = Gmoment_of_inertia["m10"]
    GmVertical = Gmoment_of_inertia["m01"]
    GmTotal = Gmoment_of_inertia["m00"]
    if GmTotal > 0:
        Gcentroid = (int(GmHorizontal/GmTotal), int(GmVertical/GmTotal))
        cv2.circle(greenNewVid,Gcentroid, 10, (0,255,255), -1)
    
    Ymoment_of_inertia = cv2.moments(yellowColor_filter)
    YmHorizontal = Ymoment_of_inertia["m10"]
    YmVertical = Ymoment_of_inertia["m01"]
    YmTotal = Ymoment_of_inertia["m00"]
    if YmTotal > 0:
        Ycentroid = (int(YmHorizontal/YmTotal), int(YmVertical/YmTotal))
        cv2.circle(yellowNewVid,Ycentroid, 10, (0,255,255), -1)
        
    
    Rmoment_of_inertia = cv2.moments(redColor_mask)
    RmHorizontal = Rmoment_of_inertia["m10"]
    RmVertical = Rmoment_of_inertia["m01"]
    RmTotal = Rmoment_of_inertia["m00"]
    if RmTotal > 0:
        Rcentroid = (int(RmHorizontal/RmTotal), int(RmVertical/RmTotal))
        cv2.circle(redNewVid,Rcentroid, 10, (255,255,255), -1)
        
    Bmoment_of_inertia = cv2.moments(blueColor_filter)
    BmHorizontal = Bmoment_of_inertia["m10"]
    BmVertical = Bmoment_of_inertia["m01"]
    BmTotal = Bmoment_of_inertia["m00"]
    if BmTotal > 0:
        Bcentroid = (int(BmHorizontal/BmTotal), int(BmVertical/BmTotal))
        cv2.circle(blueNewVid,Bcentroid, 10, (0,255,255), -1)
    
    #creating the 2D model using the centroinds
    cv2.circle(black,Gcentroid, 10, green, -1)
    cv2.circle(black,Ycentroid, 10, yellow, -1)
    joint_one = cv2.line(black,(Gcentroid),(Ycentroid),lineColor,1)
    
    cv2.circle(black,Rcentroid, 10, red, -1)
    cv2.circle(black,Bcentroid, 10, blue, -1)
    joint_two = cv2.line(black,(Rcentroid),(Bcentroid),lineColor,1)
    
    
    """Calculating the angle of the joint"""
    
    vectorWx = (Rcentroid[0] - Bcentroid[0])
    vectorWy = (Rcentroid[1] -  Bcentroid[1])
    
    vectorLx = (Gcentroid[0] - Ycentroid[0])
    vectorLy = (Gcentroid[1] -  Ycentroid[1])
    
    numerator = vectorWx*vectorLx + vectorLy*vectorWy # dot product of the two vectors
    
    magnitudeW = math.sqrt(pow(vectorWx,2) + pow(vectorWy,2)) # magnitude of first vector
    magnitudeL = math.sqrt(pow(vectorLx,2) + pow(vectorLy,2)) # magnitude of the second vector
    
    denominator = magnitudeW * magnitudeL
    
    # final step to calculating the angle of the two vectors
    # finding the inverse cosine of the math above
    if denominator != 0:
        angle = math.acos(numerator/denominator)
        angle = math.degrees(angle)
        print(angle)
        
        font = cv2.FONT_HERSHEY_SIMPLEX # font for letter on display
        
        # logic added so the voice will not repete itself
        if angle >= LowerAngleRange and angle <= UpperAngleRange:
            cv2.putText(black,'In range',(100,100), font, 2,green,2,cv2.LINE_AA)
            lineColor = green
            if voice1 == True:
                os.system("say 'good'")
            voice1 = False
            voice2 = True
            
        else:
            cv2.putText(black,'Out of range',(100,100), font, 2,red,2,cv2.LINE_AA)
            lineColor = red
            if voice2 == True:
                os.system("say 'stop'")
            voice1 = True
            voice2 = False
            
            
    # showing each of the windows
    cv2.imshow('isolateGreen', greenNewVid)
    cv2.imshow('isolateBlue', blueNewVid)
    cv2.imshow('isolateYellow', yellowNewVid)
    cv2.imshow('isolateRed', redNewVid)
    cv2.imshow('joint', black)
    
    key = cv2.waitKey(1000/fps)
    if key == 27:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        vid.release()
        break
