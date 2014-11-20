#!/usr/bin/env python

import time
import os
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    
    try:
        pub = rospy.Publisher('scale', String, queue_size=10)
        f = open("scaleData.txt")
        rospy.init_node('scaleReader', anonymous=True)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            where = f.tell()
            line = f.readline()
            if not line:
                time.sleep(1)
                f.seek(where)
            else:
                data = line
                pub.publish(data)
            r.sleep()
    except rospy.ROSInterruptException: pass

##############
"""
print os.system("echo "" > scaleData.txt")
print os.system("./usbscale.pl >> scaleData.txt")
"""
