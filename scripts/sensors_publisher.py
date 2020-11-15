import rospy
import serial
from std_msgs.msg import Float32MultiArray,MultiArrayDimension,MultiArrayLayout
from ros_waspmote_reader.msg import wasp

### $ sudo usermod -a -G dialout $USER

class wasp_reader():

	def __init__(self, 
				 frame_id = 'gas_sensor', 
				 serial_port = '/dev/ttyUSB0', 
				 serial_baudrate = 115200
				 ):

		self.pub = rospy.Publisher('espeleo_gas_pub', wasp, queue_size = 10)
		rospy.init_node('wasp_node')
		self.rate = rospy.Rate(1)

		self.gas_data = wasp()
		self.gas_data.header.stamp = rospy.Time.now()
		self.gas_data.header.frame_id = frame_id
		self.gas_data.sensor_name = ['CO2','H2S','O2','CO','NO2','NH3']
		self.ser = serial.Serial(serial_port, serial_baudrate)

		rospy.loginfo("Publisher Created")

	def parse(self):

		self.data = self.ser.readline().split(":")
		self.g_data = self.data[1].split(",")
		return [int(self.g_data[i]) for i in range(len(self.g_data))]

	def initiate(self):
		rospy.loginfo("Publisher Initiated")

		while not rospy.is_shutdown():
			self.gas_data.reads = self.parse()
			self.pub.publish(self.gas_data)
			self.rate.sleep()


