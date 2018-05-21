#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import *
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

PI = 3.14159265359

class TiltMotorController:

	def __init__(self):

		self.leftMotor_publisher = rospy.Publisher('/left_motor_tilt/command', Float64, queue_size = 2)
		self.rightMotor_publisher = rospy.Publisher('/right_motor_tilt/command', Float64, queue_size = 2)

		self.leftMotorState_publisher = rospy.Publisher('/left/tilt/angle', Float64, queue_size = 2)
		self.rightMotorState_publisher = rospy.Publisher('/right/tilt/angle', Float64, queue_size = 2)


		# Alternative command topics
		self.right_motor_subscriper = rospy.Subscriber('/right/tilt/move', Float64, self.right_motor_callback)
		self.left_motor_subscriper = rospy.Subscriber("/left/tilt/move", Float64, self.left_motor_callback)

		self.joint_command = rospy.Subscriber('/joint_states', JointState, self.jointCommandCb)


	def rad2Deg(self, val):
		return val * 180 / PI

	def deg2Rad(self, val):
		return val * PI / 180

	def jointCommandCb(self, msg):
		leftMotorState = Float64
		rightMotorState = Float64

		left = msg.position[0]
		right = msg.position[1]
		# print (left, right)
		leftMotorState.data = self.rad2Deg(left)
		rightMotorState.data = self.rad2Deg(right-10)
		self.leftMotorState_publisher.publish(leftMotorState)
		self.rightMotorState_publisher.publish(rightMotorState)


	def right_motor_callback(self, msg):
		rad = Float64
		rad.date = self.deg2Rad(msg.data)
		self.rightMotor_publisher.publish(rad)

	def left_motor_callback(self, msg):
		rad = Float64
		rad.date = self.deg2Rad(msg.data)
		print(rad)
		self.leftMotor_publisher.publish(rad)


	def controlLoop(self):
		"""
			Runs the control loop
		"""
		rate = rospy.Rate(15) # 10hz
		while not rospy.is_shutdown():
			rate.sleep()


	def start(self):
		"""
			Starts the control loop and runs spin
		"""
		self.controlLoop()


def main():

	rospy.init_node('TiltMotorController')

	tiltMotorController = TiltMotorController()

	tiltMotorController.start()



if __name__=='__main__':
	main()
	exit()
