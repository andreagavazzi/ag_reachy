#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import re
import face_recognition

rospy.init_node('openCV')
rospy.loginfo('Node started.')

# Visualizza informazioni su opencv
cv_info = [re.sub(' +', ' ', ci.strip()) for ci in cv2.getBuildInformation().strip().split('\n') 
            if len(ci) > 0 and re.search(r'(nvidia*:?)|(cuda*:)|(cudnn*:)', ci.lower()) is not None]
rospy.loginfo('OpenCV version: ' + cv2.__version__+ ' ' + str(cv_info))


# crea oggetto bridge e il publisher per le coordinate
bridge = CvBridge()
roi_pub = rospy.Publisher('roi', RegionOfInterest, queue_size=1)



# funzione callback
def callback(data):

    try:
        """ Convert the raw image to OpenCV format """
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv_image

        face_locations = face_recognition.face_locations(frame)
        #face_locations = face_recognition.face_locations(frame, model='cnn')

        for (row1, col1, row2, col2) in face_locations:
            cv2.rectangle(frame, (col1, row1), (col2, row2), (0,255,0), 1)


        # Display the resulting image
        cv2.imshow('Video', frame)
        cv2.waitKey(3)

        roi = RegionOfInterest()

        roi.x_offset = 0
        roi.y_offset = 0
        roi.width = 0
        roi.height = 0

        roi_pub.publish(roi)


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