#!/usr/bin/env python

import face_recognition
import cv2
import dlib
import re

def cudainfo():
    # Print information about cv e gpu"
    cv_info = [re.sub('\s+', ' ', ci.strip()) for ci in cv2.getBuildInformation().strip().split('\n') 
                if len(ci) > 0 and re.search(r'(nvidia*:?)|(cuda*:)|(cudnn*:)', ci.lower()) is not None]
    print 'OpenCV version:', cv2.__version__, cv_info
    print 'dlib use GPU:', dlib.DLIB_USE_CUDA

def openwebcam():
    # open webcam
    image = cv2.VideoCapture(0)
    if not image.isOpened():
        print("Could not open webcam")
        exit()
    return image


cudainfo()
image = openwebcam()

# loop through frames
while image.isOpened():

    # read frame from webcam 
    status, frame = image.read()

    if not status:
        print("Could not read frame")
        exit()

    # display output
    cv2.imshow("Webcam", frame)

    # press "Q" to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
# release resources
image.release()
cv2.destroyAllWindows()