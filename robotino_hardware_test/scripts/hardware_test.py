#!/usr/bin/env python
import roslib
import sys
import rospy
import time
from dialog_client import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

def run():
	test = HardwareTest('robotino_hardware_test')
	if dialog_client(0, 'START the test?'):
		test.log_file.write('\n  ROBOTINO TEST: ')
	else:
		return
	
	#### MOVE TEST  ###
	test.log_file.write('\n\n[MOVE_TEST] [%s]' %(time.strftime('%H:%M:%S')))

	for component in test.actuators:
		if dialog_client(1, 'Move %s?' %(component['name'])):
			test.log_file.write('\n %s: ' %(component['name']))

			if component['name'] == 'base' :
					if test.move_base() and test.dialog(component['name']):
						test.log_file.write('\t<<OK>>')
					else:
						test.log_file.write('\t<<FAIL>>')					
			if component['name'] == 'pan' :
					if test.move_pan() and test.dialog(component['name']):
						test.log_file.write('\t<<OK>>')
					else:
						test.log_file.write('\t<<FAIL>>')					
			if component['name'] == 'tilt' :
					if test.move_tilt() and test.dialog(component['name']):
						test.log_file.write('\t<<OK>>')
					else:
						test.log_file.write('\t<<FAIL>>')
			if component['name'] == 'arm' :
					if test.move_arm() and test.dialog(component['name']):
						test.log_file.write('\t<<OK>>')
					else:
						test.log_file.write('\t<<FAIL>>')	
		else:
			test.log_file.write('\n  %s: not tested' %(component['name']))

	# Sensor test
	test.log_file.write('\n\n[SENSOR_TEST] [%s]' %(time.strftime('%H:%M:%S')))
	
	for sensor in test.sensors:
		test.log_file.write('\n  %s: ' %(sensor['name']))
		if test.check_sensor(sensor['topic'], sensor['msg_type']):
			test.log_file.write('<<OK>>')
		else:
			test.log_file.write('<<NO_MSG>>')	
	
	
	test.log_file.close()
	dialog_client(2, 'The test finished, please see the result on \n %s' %(test.complete_name))


class HardwareTest:
	def __init__(self, test_type):
		
		### GET PARAMETERS ###
		self.base_params = []
		self.actuators = []
		self.sensors = []
		self.init_hardware_test()

	def init_hardware_test(self):
		
		rospy.init_node('hardware_test')
		
		### INTERNAL PARAMETERS
		self.wait_time = 5		# waiting time (in seconds) before trying initialization again
		self.wait_time_recover = 1
		self.wait_time_diag = 3
		###

		### GET PARAMETERS ###
		# Get actuator parameters
		try:
			params_base = rospy.get_param('/hardware_test/components/base')
			for k in params_bases.keys():
				self.base_params.append(params_base[k])
		except:
			pass

		# Get actuator parameters
		try:
			params_actuator = rospy.get_param('/hardware_test/components/actuators')
			for k in params_actuator.keys():
				self.actuators.append(params_actuator[k])
		except:
			pass
		
		# Get sensor parameters	
		try:
			params = rospy.get_param('/hardware_test/components/sensors')
			for k in params.keys():
				self.sensors.append(params[k])
		except:
			pass
			
		if not self.base_params and not self.actuators and not self.sensors:
			raise NameError('Couldn\'t find any components to test under /component_test namespace. '
							'You need to define at least one component in order to run the program (base, actuator, sensor). '
							'View the documentation to see how to define test-components.')

		# Get log-file directory
		try:
			log_dir = rospy.get_param('/hardware_test/result_dir')
		except:
			log_dir = '/tmp'
			
		# Create and prepare logfile
		self.complete_name = '%s/hardware_test_%s.txt' %(log_dir, time.strftime("%Y%m%d_%H-%M"))
		self.log_file = open(self.complete_name,'w')
		
		
		self.log_file.write('Robotino Hardware test')
		self.log_file.write('\n%s \n%s' %(time.strftime('%d.%m.%Y'), time.strftime('%H:%M:%S')))
		self.print_topic('TESTED COMPONENTS')
		if self.actuators:
			for actuator in self.actuators:
				self.log_file.write('\n  ' + actuator['name'])

		self.print_topic('TEST PARAMETERS')
		params = rospy.get_param('/hardware_test/components')
		for key in params:
			self.log_file.write('\n%s:' %(key))
			params_2 = rospy.get_param('/hardware_test/components/%s' %(key))
			for key_2 in params_2:
				self.log_file.write('\n  %s:' %(key_2))
				params_3 = rospy.get_param('/hardware_test/components/%s/%s' %(key, key_2))
				for key_3, value in params_3.iteritems():
					self.log_file.write('\n    %s: %s' %(key_3, value))
					
		self.print_topic('!!!!!!!!!!!!!!!!!!!!TEST LOG!!!!!!!!!!!!!!!!!!!!')

	def move_pan(self):
		try:
			pub = rospy.Publisher('/pan_controller/command', Float64, queue_size=10)
			rospy.sleep(2.0)
			pos = Float64()
			pos.data = 1.0
			pub.publish(pos)
			rospy.sleep(2.0)
			pos.data = 0.0
			pub.publish(pos)
		except rospy.ROSException, e:
			return(False,0)
		return(True, 0)

	def move_tilt(self):
		try:
			pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
			rospy.sleep(2.0)
			pos = Float64()
			pos.data = 1.0
			pub.publish(pos)
			rospy.sleep(2.0)
			pos.data = 0.0
			pub.publish(pos)
		except rospy.ROSException, e:
			return(False,0)
		return(True, 0)

	def move_base(self):
		try:
			pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
			twist = Twist()
			twist.linear.x = 0
			twist.linear.y = 0
			twist.angular.z = 0.1
			end_time = rospy.Time.now() + rospy.Duration(3)
			r = rospy.Rate(10) # send velocity commands at 10 Hz
			while not rospy.is_shutdown() and rospy.Time.now() < end_time:
				pub.publish(twist)
				r.sleep()
		except rospy.ROSException, e:
			return(False,0)
		return(True, 0)

	def move_arm(self):
		try:
			mode_pub = rospy.Publisher('/real/robotino/settings/switch_mode', Int32, queue_size=10, latch=True)
			pub = rospy.Publisher('/real/robotino/joint_control/ptp', Float64MultiArray, queue_size=10)
			rospy.sleep(2)

			mode_pub.publish(data=10)
			ptp_command = Float64MultiArray()
			ptp_command.data = [float('nan'),float('nan'),float('nan')+0.1,float('nan'),float('nan'),float('nan')+0.02,float('nan'),float('nan')-0.02]
			pub.publish(ptp_command)

		except rospy.ROSException, e:
			return(False,0)
		return(True, 0)
	
	def check_sensor(self, topic, msg_type):
		self.msg_received = False
		if msg_type == "LaserScan": cb_func = self.cb_scanner
		elif msg_type == "PointCloud2": cb_func = self.cb_point_cloud
		elif msg_type == "Image": cb_func = self.cb_camera
		elif msg_type == "Bool": cb_func = self.cb_bumper
		else: raise NameError('Unknown message! No callback function defined for message type <<%s>>' %(msg_type))
		
		sub_topic = rospy.Subscriber(topic, eval(msg_type), cb_func)
		
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.msg_received and rospy.get_rostime() < abort_time:
			rospy.sleep(1)
			
		sub_topic.unregister()
		
		if self.msg_received:
			return True
		return False


	def dialog(self, component):
		if dialog_client(1, 'Did component <<%s>> move ?' %(component)):
			return True
		return False
	
	def print_topic(self, text):
		topic = '\n\n\n'
		for i in enumerate(text):
			topic += '='
		topic += '\n%s\n' %(text)
		for i in enumerate(text):
			topic += '='
		topic += '\n'
		self.log_file.write(topic)

	def handle_test_trigger(self, req):
		return self.test_on

	def test_trigger_server(self):
		s = rospy.Service('test_trigger', TestTrigger, self.handle_test_trigger)
	
	def time_limit_handler(self, signum, frame):
		raise Exception("Time limit exceeded")
	
	
	### CALLBACKS ###
	def cb_actuator(self, msg):
		self.actuator_position = msg.actual.positions
		self.msg_received = True

	def cb_scanner(self, msg):
		self.scanner_msg = msg.ranges
		self.msg_received = True

	def cb_point_cloud(self, msg):
		self.point_cloud_msg = msg.fields
		self.msg_received = True

	def cb_camera(self, msg):
		self.camera_msg = msg.data
		self.msg_received = True
		
	def cb_bumper(self, msg):
		self.bumper_msg = msg.data
		self.msg_received = True


if __name__ == '__main__':
	try:
		run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
