#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('joint_control')
rospy.loginfo('Node started.')

# crea il publisher

pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

jt = JointTrajectory()

#jt.header.stamp = rospy.Time.now()
#jt.header.frame_id = "pan"

jt.joint_names=["pan"]

p = JointTrajectoryPoint()

p.positions = [1.5]

jt.points.append(p)

pub.publish(jt) 





