#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
import re
import face_recognition
import numpy as np

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

# Load a sample picture and learn how to recognize it.
obama_image = face_recognition.load_image_file("/home/andrea/catkin_ws/src/ag_reachy/ag_faces/images/obama.jpg")
obama_face_encoding = face_recognition.face_encodings(obama_image)[0]

# Load my sample picture and learn how to recognize it.
andrea_image = face_recognition.load_image_file("/home/andrea/catkin_ws/src/ag_reachy/ag_faces/images/andrea.jpg")
andrea_face_encoding = face_recognition.face_encodings(andrea_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [obama_face_encoding, andrea_face_encoding]
known_face_names = ["Obama", "Andrea"]

# Initialize some variables
face_encodings = []
face_names = []
process_this_frame = True


# funzione callback
def callback(data):

    #global face_names
    global face_encodings
    global face_locations
    global process_this_frame
    global face_names

    global iamodel

    try:
        """ Convert the raw image to OpenCV format """
        cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
        frame1 = bridge.imgmsg_to_cv2(data, "bgr8")

        frame = cv_image

        if process_this_frame == True:

            face_locations = face_recognition.face_locations(frame, model=iamodel)
            face_encodings = face_recognition.face_encodings(frame, face_locations)

            face_names = [] 
            
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                # # If a match was found in known_face_encodings, just use the first one.
                # if True in matches:
                #     first_match_index = matches.index(True)
                #     name = known_face_names[first_match_index]

                # Or instead, use the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)

                if matches[best_match_index]:
                    name = known_face_names[best_match_index]

                face_names.append(name)

        process_this_frame = not process_this_frame


        
        for (top, right, bottom, left), name in zip(face_locations, face_names):

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 1)

            # Draw a label with a name below the face
            #cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
            #font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)



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