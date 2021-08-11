import rospy
import serial
from ros_waspmote_reader.msg import wasp

### $ sudo usermod -a -G dialout $USER

class co2_reader():

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
		self.gas_data.sensor_name = ['CO2','TEMPERATURE','HUMIDITY','PRESSURE']
		self.ser = serial.Serial(serial_port, serial_baudrate)

		rospy.loginfo("Publisher Created")

	def parse(self):

		self.data = self.ser.readline().split(":")
		rospy.loginfo(self.data)
		if self.data[0] == "Gas concentration":
			
			self.g_data = self.data[1].split(",")
			#rospy.loginfo(self.g_data)
			return [float(self.g_data[i]) for i in range(len(self.g_data)-1)]
		else:
			rospy.loginfo("Waiting for the sensor to warm up! 2 Minutes")

	def initiate(self):
		rospy.loginfo("Publisher Initiated")

		while not rospy.is_shutdown():
			#self.aaa = self.ser.readline()
			self.gas_data.reads = self.parse()
			if (self.gas_data is not None):
				self.pub.publish(self.gas_data)
				self.rate.sleep()


