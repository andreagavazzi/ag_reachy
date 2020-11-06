#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
import re
import face_recognition

rospy.init_node('ag_faces')
rospy.loginfo('Node started.')

# Visualizza informazioni su opencv
cv_info = [re.sub(' +', ' ', ci.strip()) for ci in cv2.getBuildInformation().strip().split('\n') 
            if len(ci) > 0 and re.search(r'(cuda*:)', ci.lower()) is not None]

rospy.loginfo('OpenCV version: ' + cv2.__version__)
rospy.loginfo(cv_info[0])

if cv_info[0] != 'Use Cuda: NO':
    iamodel = 'cnn'
else: iamodel = ''

# crea oggetto bridge e il publisher per le coordinate
bridge = CvBridge()
faces_pub = rospy.Publisher('faces', Int8, queue_size=1)


# funzione callback
def callback(data):

    global iamodel
    try:
        """ Convert the raw image to OpenCV format """
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv_image

        face_locations = face_recognition.face_locations(frame, model=iamodel)

        for (row1, col1, row2, col2) in face_locations:
            cv2.rectangle(frame, (col1, row1), (col2, row2), (0,255,0), 1)

        # Display the resulting image
        cv2.imshow('Video', frame)
        cv2.waitKey(3)

        faces = Int8()
        found_faces = len(face_locations)
        
        faces.data = found_faces
        faces_pub.publish(faces)

    except CvBridgeError as e:
        rospy.loginfo(e)


def main():
    # Sottoscreive il raw camera image topic e pubblica il RoI
    while not rospy.is_shutdown():
        camera_sub = rospy.Subscriber("/usb_cam/image_raw", Image, callback)
        rospy.spin()
    rospy.on_shutdown(shutdown)


def shutdown():
    cv2.destroyAllWindows()
    rospy.loginfo('Node terminated.')


# Main routine
if __name__ == '__main__':
    main()