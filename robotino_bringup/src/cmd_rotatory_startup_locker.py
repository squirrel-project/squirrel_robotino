#!/usr/bin/env python
#
# cmd_rotatory_startup_locker.py --- 
# 
# Filename: cmd_rotatory_startup_locker.py
# Description: Lock rotatory control on start up
# Author: Federico Boniardi
# Maintainer: boniardi@informatik.uni-freiburg.de
# Created: Tue Apr 12 11:29:31 2016 (+0200)
# Version: 0.0.1
# Last-Updated: 
#           By: 
#     Update #: 0
# URL: 
# Keywords: 
# Compatibility: 
# 
# 

# Code:

import rospy
import std_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('cmd_rotatory_startup_locker', anonymous=False)
    
    cmd_topic = rospy.get_param("~cmd_topic",'/cmd_rotatory') 
    lock_topic = rospy.get_param("~lock_topic",'/twist_mux/locks/pause_rotatory') 
    timeout_sec = rospy.get_param("~timeout", 10.0);
    
    lock_pub = rospy.Publisher(lock_topic, std_msgs.msg.Bool, queue_size=1)

    try:
        rospy.wait_for_message(cmd_topic, geometry_msgs.msg.Twist, timeout=timeout_sec)
        lock_pub.publish(True);
    except rospy.exceptions.ROSException as ex:
        rospy.logwarn("%s: %s", rospy.get_name(), ex)
        
    exit(0)
# 
# cmd_rotatory_startup_locker.py ends here
