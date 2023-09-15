import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from ahrs.ahrs.filters import Mahony
import numpy as np

class DynamicBroadcaster(Node):
    
    def __init__(self, given_name):
        super().__init__('dynamic_broadcaster')
        self.name_ = given_name
        self.lines_to_skip = 11  # Number of lines to skip before processing data
        self.current_line = 0
        self.orientation = Mahony()
        self.time = np.tile([0.], (2 , 1))
        self.quaternions = np.tile([0., 0., -1., 0.], (1, 1))
        self.get_logger().info("Broadcasting pose of: {}".format(self.name_))
        self.tfb_ = TransformBroadcaster(self)
        self.pub_transform = self.create_publisher(TransformStamped, given_name, 10)

    def handle_data(self, data):
        if self.current_line <= self.lines_to_skip:
            self.current_line += 1
            return
        try:
            data = data.strip().split("; ")
            self.time[1] = self.time[0] 
            self.time[0] = float(data[0])
            #roll = float(data[1])
            #pitch = float(data[2])
            #yaw = float(data[3])
            accel_x = float(data[1])
            accel_y = float(data[2])
            accel_z = float(data[3])
            gyro_x = float(data[4])
            gyro_y = float(data[5])
            gyro_z = float(data[6])
            mag_x = float(data[7])
            mag_y = float(data[8])
            mag_z = float(data[9])
            gyro_data = np.array([gyro_x, gyro_y, gyro_z])
            accel_data = np.array([accel_x, accel_y, accel_z])
            #quaternion = Mahony(gyr=gyro_data, acc=accel_data, frequency = 500).Q\
            self.quaternions[0] = self.orientation.updateIMU(self.quaternions[0], gyr=gyro_data, acc=accel_data, dt = self.time[0] - self.time[1])
            tfs = TransformStamped()
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id = "world"
            tfs.child_frame_id = "IMU"
            tfs.transform.translation.x = 0.0
            tfs.transform.translation.y = 0.0
            tfs.transform.translation.z = 0.0
            tfs.transform.rotation.x = self.quaternions[0][0]
            tfs.transform.rotation.y = self.quaternions[0][1]
            tfs.transform.rotation.z = self.quaternions[0][2]
            tfs.transform.rotation.w = self.quaternions[0][3]

            self.pub_transform.publish(tfs)

        except ValueError:
            print("Invalid input! Please provide data in the correct format.")

def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)
    node = DynamicBroadcaster("IMU")

    try:
        while True:
            data = sys.stdin.readline()
            node.handle_data(data)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
