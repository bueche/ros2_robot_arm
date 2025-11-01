#!/usr/bin/env python3
"""
Test Individual Motor Movement
Tests one motor at a time with small movements
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys

# Safe home position within measured hardware limits
HOME_POSITION = {
    'shoulder_pan': 3.0630,    # +175.50°
    'shoulder_lift': 1.9010,   # +108.92°
    'elbow_flex': 1.4095,      #  +80.76°
    'wrist_flex': 1.4695,      #  +84.20°
    'wrist_roll': 0.0000,      #   +0.00°
    'pen_holder': 1.7055,      #  +97.72°
}

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'pen_holder'
]


class SingleMotorTester(Node):
    """Test a single motor with small movements"""
    
    def __init__(self):
        super().__init__('single_motor_tester')
        
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/koch_v11_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('✓ Connected to action server')
    
    def send_position(self, positions, duration=2.0):
        """Send a single position command"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        return result.status == 4  # SUCCEEDED
    
    def test_motor(self, motor_index, movement_range=0.1):
        """
        Test a single motor by moving it slightly
        
        Args:
            motor_index: 0-5 (which motor to test)
            movement_range: How far to move (radians)
        """
        joint_name = JOINT_NAMES[motor_index]
        home_angle = HOME_POSITION[joint_name]
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Testing Motor {motor_index + 1}: {joint_name}')
        self.get_logger().info('=' * 60)
        
        # Create position list with all at home except the test motor
        test_positions = [HOME_POSITION[name] for name in JOINT_NAMES]
        
        # Move to home first
        self.get_logger().info(f'Moving {joint_name} to home position ({home_angle:.3f} rad)...')
        if not self.send_position(test_positions, duration=3.0):
            self.get_logger().error('Failed to reach home!')
            return False
        
        self.get_logger().info('✓ At home position')
        
        # Test positive movement
        test_positions[motor_index] = home_angle + movement_range
        self.get_logger().info(f'Testing +{movement_range:.3f} rad movement...')
        if not self.send_position(test_positions, duration=2.0):
            self.get_logger().error('Positive movement failed!')
            return False
        self.get_logger().info('✓ Positive movement OK')
        
        # Test negative movement
        test_positions[motor_index] = home_angle - movement_range
        self.get_logger().info(f'Testing -{movement_range:.3f} rad movement...')
        if not self.send_position(test_positions, duration=2.0):
            self.get_logger().error('Negative movement failed!')
            return False
        self.get_logger().info('✓ Negative movement OK')
        
        # Return to home
        test_positions[motor_index] = home_angle
        self.get_logger().info('Returning to home...')
        if not self.send_position(test_positions, duration=2.0):
            self.get_logger().error('Return to home failed!')
            return False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✓ Motor {motor_index + 1} ({joint_name}) test PASSED')
        self.get_logger().info('=' * 60)
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = SingleMotorTester()
    
    # Check command line arguments
    if len(sys.argv) < 2:
        node.get_logger().error('Usage: ros2 run writing_robot_control test_single_motor <motor_id>')
        node.get_logger().info('Motor IDs: 1-6')
        node.get_logger().info('  1: shoulder_pan')
        node.get_logger().info('  2: shoulder_lift')
        node.get_logger().info('  3: elbow_flex')
        node.get_logger().info('  4: wrist_flex')
        node.get_logger().info('  5: wrist_roll')
        node.get_logger().info('  6: pen_holder')
        return
    
    try:
        motor_id = int(sys.argv[1])
        if motor_id < 1 or motor_id > 6:
            raise ValueError()
        motor_index = motor_id - 1
    except (ValueError, IndexError):
        node.get_logger().error('Invalid motor ID. Must be 1-6')
        return
    
    # Test the specified motor
    success = node.test_motor(motor_index, movement_range=0.1)
    
    if success:
        node.get_logger().info('\n🎉 Motor test PASSED!')
    else:
        node.get_logger().error('\n❌ Motor test FAILED!')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
