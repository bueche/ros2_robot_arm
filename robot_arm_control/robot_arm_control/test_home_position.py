#!/usr/bin/env python3
"""
Test Home Position Movement
Safely moves the arm to the designated home position
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

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


class HomePositionTester(Node):
    """Test moving to home position"""
    
    def __init__(self):
        super().__init__('home_position_tester')
        
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/koch_v11_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('✓ Connected to action server')
    
    def move_to_home(self, duration=5.0):
        """Move to home position"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('MOVING TO HOME POSITION')
        self.get_logger().info('=' * 60)
        
        # Create home position array
        home_positions = [HOME_POSITION[name] for name in JOINT_NAMES]
        
        self.get_logger().info('Target home position:')
        for i, name in enumerate(JOINT_NAMES):
            self.get_logger().info(f'  {name:15s}: {home_positions[i]:+.4f} rad')
        
        self.get_logger().info(f'\nMoving (duration: {duration:.1f}s)...')
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = home_positions
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server!')
            return False
        
        self.get_logger().info('Goal accepted, executing...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('=' * 60)
            self.get_logger().info('✓ SUCCESSFULLY REACHED HOME POSITION')
            self.get_logger().info('=' * 60)
            return True
        else:
            self.get_logger().error(f'Movement failed with status: {result.status}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = HomePositionTester()
    
    print()
    print("=" * 60)
    print("IMPORTANT: Before running this test:")
    print("=" * 60)
    print("1. Make sure position limits are set in all Dynamixels")
    print("2. Keep your hand near the power switch")
    print("3. Be ready to power off if anything looks wrong")
    print("4. The arm will move SLOWLY over 5 seconds")
    print()
    input("Press ENTER when ready to move to home position...")
    print()
    
    success = node.move_to_home(duration=5.0)
    
    if success:
        print()
        print("🎉 Home position test SUCCESSFUL!")
        print()
        print("The arm should now be in a safe, stable configuration.")
        print("This position can be used as the starting point for all movements.")
    else:
        print()
        print("❌ Home position test FAILED!")
        print()
        print("Troubleshooting:")
        print("- Check that all motors are connected and powered")
        print("- Verify position limits are set in Dynamixels")
        print("- Check for any hardware obstructions")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
