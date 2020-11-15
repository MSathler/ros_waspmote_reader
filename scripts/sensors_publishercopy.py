#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float32MultiArray,MultiArrayDimension,MultiArrayLayout



def parse(ser):

	data = ser.readline().split(":")
	g_data = data[1].split(",")
	return [int(g_data[i]) for i in range(len(g_data))]


def gas_pub():

	pub = rospy.Publisher('espeleo_gas_pub', Float32MultiArray, queue_size = 10)
	rospy.init_node('wasp_node')
	rate = rospy.Rate(1)
	gas_data = Float32MultiArray()
	ser = serial.Serial('/dev/ttyUSB0', 115200)
	rospy.loginfo("Publisher Iniciated")

	while not rospy.is_shutdown():

		gas_data.data = parse(ser)
		pub.publish(gas_data)
		rate.sleep()


if __name__ == '__main__':

	try:
		gas_pub()

	except rospy.ROSInterruptException:
		pass






