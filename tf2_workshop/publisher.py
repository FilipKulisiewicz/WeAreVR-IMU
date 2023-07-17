#!/usr/bin/env python3

import sys
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

class TfListener(Node):

    def __init__(self, first_joint, second_joint):
        super().__init__('tf_listener')
        self.first_name_ = first_joint
        self.second_name_ = second_joint
        self.get_logger().info("Transforming from {} to {}".format(self.first_name_,self.second_name_))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.cmd_ = TransformStamped()
        self.publisher_ = self.create_publisher(TransformStamped, "{}".format(self.first_name_),10)
        self.timer = self.create_timer(0.002, self.timer_callback) #500 Hz = 0.002s

    def timer_callback(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.first_name_, self.second_name_, rclpy.time.Time())
            self.cmd_ = trans
            self.publisher_.publish(self.cmd_) 

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = TfListener(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()