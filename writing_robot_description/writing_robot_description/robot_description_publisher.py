#!/usr/bin/env python3
"""
Publishes the robot_description as a latched topic for RViz
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import os
from ament_index_python.packages import get_package_share_directory


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        
        # Get URDF file path
        urdf_file = os.path.join(
            get_package_share_directory('writing_robot_description'),
            'urdf',
            'robot_arm.urdf'
        )
        
        # Read URDF
        with open(urdf_file, 'r') as file:
            robot_description = file.read()
        
        # Create QoS profile with transient local durability (latched)
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            'robot_description',
            qos_profile
        )
        
        # Publish the description
        msg = String()
        msg.data = robot_description
        self.publisher.publish(msg)
        
        self.get_logger().info('Published robot_description topic')
        self.get_logger().info(f'URDF has {len(robot_description)} characters')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    
    # Keep node alive to maintain the latched topic
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
