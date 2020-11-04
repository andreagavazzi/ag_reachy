#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket

pub_l1 = rospy.Publisher('lcd_line1', String, queue_size=10)
pub_l2 = rospy.Publisher('lcd_line2', String, queue_size=10)
rospy.init_node('sense', anonymous=True)
rate = rospy.Rate(1)


def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except:
        return 'Offline         '
	rospy.loginfo('Offline! Check network connection')

def main():

    ip = get_ip_address()
    pub_l1.publish(ip)
    if ip == 'Offline         ':    
        pub_l2.publish('Status: Error   ')
    else:
        pub_l2.publish('Status: OK      ')
    
    

if __name__ == '__main__':
    try:
        for i in range(5):
            main()
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass
