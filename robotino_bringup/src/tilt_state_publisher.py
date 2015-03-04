#!/usr/bin/env python
  
  
import roslib; roslib.load_manifest('robotino_bringup')
import rospy
  
from sensor_msgs.msg import JointState as JointStatePR2
from dynamixel_msgs.msg import JointState as JointStateDynamixel
  
class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort
  
class JointStatePublisher():
    def __init__(self):
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)
  
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
  
        # The namespace and joints parameter needs to be set by the servo controller
        # (The namespace is usually null.)
	namespace = 'tilt_controller'
        #namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/joints', '')
  
        self.servos = list()
        self.controllers = list()
        self.joint_states = dict({})
  
        for controller in sorted(self.joints):
            self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0)
            self.controllers.append(controller)
  
        # Start controller state subscribers
        rospy.Subscriber('/tilt_controller/state', JointStateDynamixel, self.controller_state_handler)
  
        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStatePR2)
  
        rospy.loginfo("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
  
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()
  
    def controller_state_handler(self, msg):
        js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)
        self.joint_states[msg.name] = js
  
    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStatePR2()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
  
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
  
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        self.joint_states_pub.publish(msg)
  
if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass
