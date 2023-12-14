import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from ahrs.ahrs.filters import Mahony
import numpy as np
import time

class DynamicBroadcaster(Node):
    
    def __init__(self, given_name):
        super().__init__('dynamic_broadcaster')
        self.name_ = given_name
        self.lines_to_skip = 15  # Number of lines to skip before processing data
        self.current_line = 0
        self.timeDelta = 0.0
        self.orientation = Mahony()
        self.ErrorInRow = 0
        self.time = np.tile([0.], (2 , 1)) #[ms]
        self.quaternions = np.tile([1., 0., 0., 0.], (2, 1))
        self.get_logger().info("Broadcasting pose of: {}".format(self.name_))
        self.tfb_ = TransformBroadcaster(self)
        self.pub_transform = self.create_publisher(TransformStamped, given_name, 10)

        self.CalibrationQuatDiffValueThreshold = 0.00002
        self.SensorDataFrequency = 500
        self.CalibrationTimeThresholdSeconds = 0.5
        self.CalibrationTimeThresholdInCycles = self.CalibrationTimeThresholdSeconds * self.SensorDataFrequency
        self.CalibrationCyclesCounter = self.CalibrationTimeThresholdInCycles

        self.CalibrationValueThreshold = 0.02 #decrease if movement is less than 1deg/s 
        self.alpha = 0.01
        self.diff = 0

    def handle_data(self, data):
        if self.current_line <= self.lines_to_skip:
            self.current_line += 1
            return
       
        try:
            data = data.strip().split("; ")
            if(self.time[0] < 0.000001):
                self.time[0] = float(data[0])
                return
            if(float(data[0]) > self.time[0]):
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
                #mag_x = float(data[7])
                #mag_y = float(data[8])
                #mag_z = float(data[9])
                # q_1 = float(data[10])
                # q_2 = float(data[11])
                # q_3 = float(data[12])
                # q_4 = float(data[13])
                timeDeltaInSeconds = (self.time[0] - self.time[1]) / 1000.0 # from miliseconds to seconds

                accel_data = np.array([accel_x, accel_y, accel_z]).reshape(3,1)
                gyro_data = np.array([gyro_x, gyro_y, gyro_z]).reshape(3,1)
                #mag_data = np.array([mag_x, mag_y, mag_z])

                #Acc Calibration correction
                a = np.array([[ 0.98200578, 0.03031919, -0.00180864], [-0.02499362, 0.99994621, 0.00318627], [0.00802986,  0.00428184, 1.00266212]])
                b = np.array([ 0.01507779, -0.011765  , -0.00597272]).reshape(3,1)
                accel_data = (np.matmul(a, accel_data) + b).ravel()
                #Gyro Calibration correction
                a = np.array([[1.01284611, 0.0202367, 0.00203724], [-0.02300336, 0.98376085, -0.01937286], [-0.00339458, 0.01617729, 0.99807192]])
                b = np.array([-0.00249166, 0.00553142, -0.00051446]).reshape(3,1)
                gyro_data = (np.matmul(a, gyro_data) + b).ravel()
                
                #Acc Calibration correction
                # a = np.array([[ 0.9813075, 0.04244318, -0.01216444], [-0.03717048, 0.99962564, 0.03350584], [0.01926684,  0.02555855, 1.00015883]])
                # b = np.array([ 0.02206801, -0.02715399  , -0.00464662]).reshape(3,1)
                # accel_data = (np.matmul(a, accel_data) + b).ravel()
                # # #Gyro Calibration correction
                # a = np.array([[1.01058705, 0.0379745, -0.01416165], [-0.04473507, 0.98536418, 0.02325682], [0.01174236, -0.02686885, 0.99883812]])
                # b = np.array([-0.00308075, 0.00979748, -0.00001606]).reshape(3,1)
                # gyro_data = (np.matmul(a, gyro_data) + b).ravel()
                
                
                print(gyro_data[0], gyro_data[1], gyro_data[2])
                print(sum(abs(self.quaternions[1] - self.quaternions[0])))
                self.diff = sum(abs(self.quaternions[1] - self.quaternions[0])) * self.alpha + self.diff * (1.0 - self.alpha)
                if(abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2]) < self.CalibrationValueThreshold and self.diff < self.CalibrationQuatDiffValueThreshold and self.CalibrationCyclesCounter > 0):
                    self.CalibrationCyclesCounter = self.CalibrationCyclesCounter - 1
                elif(abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2]) < self.CalibrationValueThreshold and self.CalibrationCyclesCounter == 0):
                    gyro_data[0] = 0
                    gyro_data[1] = 0
                    gyro_data[2] = 0 
                else:
                    self.CalibrationCyclesCounter = self.CalibrationTimeThresholdInCycles
                #
                self.quaternions[1] = self.quaternions[0]
                self.quaternions[0] = self.orientation.updateIMU(self.quaternions[0], gyr=gyro_data, acc=accel_data, dt = timeDeltaInSeconds)
                #self.quaternions[0] = self.orientation.updateMARG(self.quaternions[0], gyr=gyro_data, acc=accel_data, mag = mag_data, dt = timeDeltaInSeconds)
                #self.quaternions[0] = (q_1, q_2, q_3, q_4) #directly from IMU
                print(self.quaternions[0])



                tfs = TransformStamped()
                tfs.header.stamp.sec = int(self.time[0] / 1000.0)  
                tfs.header.stamp.nanosec = int(self.time[0] % 1000.0 * 1000000.0) 
                #tfs.header.stamp = time(self.time[0] / 1000.0).to_msg()  # self.get_clock().now().to_msg()  
                tfs.header.frame_id = "world"
                tfs.child_frame_id = "IMU"
                tfs.transform.translation.x = 0.0
                tfs.transform.translation.y = 0.0
                tfs.transform.translation.z = 0.0
                tfs.transform.rotation.w = self.quaternions[0][0]
                tfs.transform.rotation.x = self.quaternions[0][1]
                tfs.transform.rotation.y = self.quaternions[0][2]
                tfs.transform.rotation.z = self.quaternions[0][3]

                self.pub_transform.publish(tfs)
                self.ErrorInRow = 0

        except ValueError:
            if(self.ErrorInRow > 20):
                print("Access to device denied, closing connection")
                sys.exit(3)    
            print("Invalid input! Please provide data in the correct format.")
            self.ErrorInRow = self.ErrorInRow + 1
                  
    
def main(argv=sys.argv[1:]):
    np.set_printoptions(suppress=True)

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
