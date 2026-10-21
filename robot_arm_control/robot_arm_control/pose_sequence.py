#!/usr/bin/env python3
"""
ROS2 node to move Koch v1.1 robot arm through a sequence of poses.
Publishes JointTrajectory messages with 2-second pauses between poses.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PoseSequenceNode(Node):
    def __init__(self):
        super().__init__('pose_sequence_node')
        
        # Create publisher for joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/koch_v11_controller/joint_trajectory',
            10
        )
        
        # Define joint names (6 joints total)
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift', 
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'pen_holder'
        ]
        
        # Define pose sequence 
        self.poses = [
            {
                'name': 'Pose 0',
                'positions': [ 1.1679296563996093, 2.607506552198339, -1.6037930235710798,
                               2.6873125208353694, 1.0190608302882267, 1.10 ],
                'duration': 2  # Time to reach pose from current position
            },
            {
                'name': 'Pose 1',
                'positions': [1.1617907357352224, 2.432547313263312, -1.4533894672936005, 
                             2.455568265754763, 0.9254422901563264, 1.58],
                'duration': 2  # Time to reach pose from current position
            },
           {
               'name': 'Pose 2',
               'positions': [1.9874755650952616, 2.332789852467025, -0.6522603205911089,
                            1.5908711139472399, -2.408349834228516, 0.21],
               'duration': 2
           },
           {
               'name': 'Pose 3',
               'positions': [1.4533894672936005, 2.5660688377137277, -0.405168763849536,
                            1.2992723823888618, -0.3727573310302735, 1.59],
               'duration': 2
           },
           {
               'name': 'Pose 4',
               'positions': [1.8125163261602346, 2.564534107547631, -1.6329528967269173,
                            2.440220964093796, 2.191594677186126, 0.5],
               'duration': 2
           },
           {
               'name': 'Pose 5',
               'positions': [1.8125163261602346, 2.564534107547631, -1.6329528967269173,
                            2.440220964093796, 2.191594677186126, 1.5],
              'duration': 2
           }
        ]
        
        self.get_logger().info('Pose Sequence Node initialized')
        self.get_logger().info(f'Will cycle through {len(self.poses)} poses')
    
    def create_trajectory_msg(self, positions, duration_sec):
        """Create a JointTrajectory message for the given positions."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        
        msg.points = [point]
        
        return msg
    
    def run_sequence(self):
        """Execute the pose sequence with pauses between poses."""
        self.get_logger().info('Starting pose sequence...')
        self.get_logger().warn('NOTE: 6th joint (pen_holder) using placeholder value 2.4')
        
        for i, pose in enumerate(self.poses):
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Moving to {pose['name']} ({i+1}/{len(self.poses)})")
            self.get_logger().info(f"Positions: {[f'{p:.3f}' for p in pose['positions']]}")
            
            # Create and publish trajectory message
            msg = self.create_trajectory_msg(pose['positions'], pose['duration'])
            self.publisher.publish(msg)
            
            self.get_logger().info(f"Published trajectory, duration: {pose['duration']}s")
            
            # Wait for movement to complete plus pause
            wait_time = pose['duration'] + 2.0  # Movement time + 2 second pause
            self.get_logger().info(f"Waiting {wait_time}s (movement + 2s pause)...")
            time.sleep(wait_time)
        
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info('Pose sequence complete!')
        self.get_logger().info('All poses executed successfully.')


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseSequenceNode()
    
    try:
        # Run the pose sequence
        node.run_sequence()
        
        # Keep node alive briefly to ensure last message is sent
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
