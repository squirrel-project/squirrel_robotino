#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import math as m
import numpy as np
from kclhand_control import kclhand_forward_kinematics as kclhand
from sensor_msgs.msg import JointState
from tf import TransformBroadcaster




class kclhand_fingertip_state(kclhand_forward_kinematics.HandFK):

    active_joint_name_list = ['palm_A','palm_E','left_finger_lower', 'middle_finger_lower', 'right_finger_lower']

    joint_name_list = ['palm_A', 'palm_B', 'palm_D', 'palm_E',
                       'left_finger_lower', 'left_finger_upper', 'middle_finger_lower', 'middle_finger_upper',
                       'right_finger_lower','right_finger_upper']

    fingertip_name_list = ['left_finger','middle_finger','right_finger']

    def fingertip_data_collection(self):
        fingertip_data_collection = [self.rightFinger(), self.middleFinger(),self.leftFinger()]
        return fingertip_data_collection

    def get_fingertip_data(self):
        self.fingertip_data = []
        for i in [0,1,2]:
            for j in [0,1,2]:
                self.fingertip_data.append(self.fingertip_data_collection()[i][j])
        return self.fingertip_data



    kclhand_joint_name = ['hand_left_crank_base_joint', 'hand_right_crank_base_joint',
                      "hand_left_coupler_crank_joint", "hand_right_coupler_crank_joint",
                      "hand_left_finger_lower_joint", "hand_left_finger_upper_joint" ,
                      "hand_middle_finger_lower_joint", "hand_middle_finger_upper_joint",
                      "hand_right_finger_lower_joint", "hand_right_finger_upper_joint" ]



class send_joint_state(object):


    def __init__(self):

        self.hand_joint_value = [10,10,0,0,0]

        rospy.Subscriber("hand_joint_state_pub", Float64MultiArray, self.sensor_call_back)
        fingertip_data = rospy.Publisher('fingertip_state', Float64MultiArray, queue_size=10)
        joint_displacement_array = rospy.Publisher('joint_group_position_controller/command', Float64MultiArray, queue_size=10)

        data_send = Float64MultiArray()
        displacement_send = Float64MultiArray()

        rospy.init_node('hand_joint_state_pub_sim', anonymous=True)
        rate = rospy.Rate(30) # 10hz


        while not rospy.is_shutdown():



                new_fingertip_state = kclhand_fingertip_state(palmJointA = -self.hand_joint_value[0], palmJointE = -self.hand_joint_value[1],
                                                              leftFingerLower = -self.hand_joint_value[2], middleFingerLower = -self.hand_joint_value[3],
                                                              rightFingerLower = self.hand_joint_value[4])
                fingertip = new_fingertip_state.get_fingertip_data()
                #displacement_send.name = new_fingertip_state.kclhand_joint_name
                displacement_send.data = new_fingertip_state.get_joint_displacement()
                #displacement_send.position = [0,0,0,0,0,0,0,0,0,0]


                dim = MultiArrayDimension()
                dim.label = "fingertip_position"
                dim.size = 9
                dim.stride = 9
                data_send.layout.data_offset = 0
                #data_send.layout.dim.append(dim)

                data_send.data = []
                for i in range(0,8):
                    data_send.data.append(fingertip[i])
                fingertip_data.publish(data_send)


                joint_displacement_array.publish(displacement_send)

                rate.sleep()





    def sensor_call_back(self,msg):

        joint_value_from_sensor = Float64MultiArray()
        self.hand_joint_value = msg.data


        self.sensor_lower_boundary = [0, 0, -80.0, -80.0, -80.0]
        self.sensor_upper_boundary = [40.0, 40.0, 90.0, 90.0, 90.0]
        for i in [0,1,2,3,4]:
            if ((self.hand_joint_value[i] >= self.sensor_lower_boundary[i]) & (self.hand_joint_value[i] <= self.sensor_upper_boundary[i])):
                self.sensor_check_boolean = True
                #print self.hand_angle_data[i]
            else:
                self.sensor_check_boolean = False
                #rospy.is_shutdown()
                break
            rospy.loginfo(self.sensor_check_boolean)




   
if __name__ == '__main__':
    try:
        send_joint_state()
    except rospy.ROSInterruptException:
        pass

