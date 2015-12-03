#!/usr/bin/env python
#
# This node just takes the joint state as published by the dynamixel controller,
# e.g. /tilt_controller/state, and publishes it as standard joint state, so
# that it can be used for the TF tree.
#
# Takes two parameters:
# joint_name: e.g. tilt (default: tilt)
# frame_id: base frame for that joint, e.g. base_link, or pan_link (for a pan/tilt setup)
#           (default: base_link)
#
# date: Dec. 2015
# author: Nadia Hammoudeh Garcia <Nadia.HammoudehGarcia@ipa.fraunhofer.de>
#         Michael Zillich <michael.zillich@tuwien.ac.at>

import rospy
import sensor_msgs.msg
import dynamixel_msgs.msg

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)

	self.joint_name = rospy.get_param('~joint_name', 'tilt')
	self.frame_id = rospy.get_param('~frame_id', 'base_link')

        rospy.Subscriber('/' + self.joint_name + '_controller/state', dynamixel_msgs.msg.JointState, self.controller_state_handler)
        self.joint_states_pub = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=10)
	rospy.spin()

    def controller_state_handler(self, msg):
        # Construct and fill normal ROS joint message and publish joint states
        out = sensor_msgs.msg.JointState()
        out.name = []
        out.position = []
        out.velocity = []
        out.effort = []

        out.name.append(msg.name)
        out.position.append(msg.current_pos)
        out.velocity.append(msg.velocity)
        out.effort.append(msg.load)

        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        self.joint_states_pub.publish(out)

if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

