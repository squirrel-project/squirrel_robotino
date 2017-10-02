#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from squirrel_interaction.srv import DoorController

def move_door(req):
  pub = rospy.Publisher('door_controller/command', Float64, queue_size=10)
  if req.message == 'open':
    pub.publish(1.7)
    return True
  elif req.message == 'close':
    pub.publish(0)
    return True
  return False

def door_contoller():
    rospy.init_node('door_contoller_server')
    s = rospy.Service('door_controller/command', DoorController, move_door)
    rospy.spin()

if __name__ == "__main__":
    door_contoller()
