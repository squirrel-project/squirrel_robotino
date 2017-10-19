#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from squirrel_interaction.srv import *

class door_controller():

  def __init__(self):
      rospy.init_node('door_contoller_server')
      s = rospy.Service('door_controller/command', DoorController, self.move_door)
      self.pub = rospy.Publisher('door_controller/command', Float64, queue_size=10)
      rospy.spin()

  def move_door(self,req):
    if req.message == 'open':
      self.pub.publish(1.7)
      return True
    elif req.message == 'close':
      self.pub.publish(0)
      return True

if __name__ == "__main__":
    door_controller()
