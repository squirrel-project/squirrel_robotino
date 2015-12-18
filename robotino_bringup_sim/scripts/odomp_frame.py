#!/usr/bin/python
# \note
# Project name: Squirrel
# \note
# ROS stack name: robotino_simulation
# \note
# ROS package name: robotino_bringup_sim
#
# \author
# Author: Nadia Hammoudeh, email:nhg@ipa.fhg.de
# \author
# Supervised by: Nadia Hammoudeh, email:nhg@ipa.fhg.de
#
# \date Date of creation: Dic 2015
#
# \brief
# Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
import roslib

import rospy
import math
import tf
import geometry_msgs.msg
import nav_msgs.msg


def odom_cb(odom):

    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
      x_trans = odom.pose.pose.position.x
      y_trans = odom.pose.pose.position.y
      
      x_rot = odom.pose.pose.orientation.x
      y_rot = odom.pose.pose.orientation.y
      z_rot = odom.pose.pose.orientation.z
      w_rot = odom.pose.pose.orientation.w

      broadcaster.sendTransform((x_trans, y_trans, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "odomp", "odom")

      #broadcaster.sendTransform((0.0, 0.0, 0.0), (x_rot, y_rot, z_rot, w_rot), rospy.Time.now(), "base_link", "odomp")

    rate.sleep()
        
        
if __name__ == '__main__':
    rospy.init_node('tf_odomp')
    rospy.Subscriber("odom",nav_msgs.msg.Odometry,odom_cb)
    rospy.spin()
