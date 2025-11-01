#!/usr/bin/env python3
"""
Draw Square with Koch v1.1 6-DOF Arm - ros2_control version
Adapted to use FollowJointTrajectory action interface

This version works with BOTH simulation and real Dynamixel hardware!
No code changes needed when switching between sim and real robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time
from writing_robot_control.koch_v11_ik_solver import KochWritingIK, validate_joint_angles


class KochSquareDrawer(Node):
    """Node to draw a square using Koch v1.1 arm with ros2_control"""
    
    def __init__(self):
        super().__init__('koch_square_drawer')
        
        # Action client for joint trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/koch_v11_controller/follow_joint_trajectory'
        )
        
        # Koch v1.1 joint names (in order)
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift', 
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'pen_holder'
        ]
        
        # Initialize IK solver with Koch v1.1 dimensions
        # TODO: UPDATE THESE WITH YOUR MEASURED VALUES!
        self.ik_solver = KochWritingIK(
            shoulder_height=0.04,    # Measure: base to shoulder_lift
            upper_arm_length=0.20,   # Measure: shoulder to elbow
            forearm_length=0.16,     # Measure: elbow to wrist
            wrist_to_pen=0.15        # Measure: wrist to pen tip
        )
        
        # Writing surface configuration (matches launch file surface)
        self.surface_x = 0.30       # X position of vertical surface (30cm from base)
        self.pen_touch_x = 0.30     # X position when pen touches (same as surface)
        self.pen_away_x = 0.32      # X position when pen is away (2cm back)
        
        # Square parameters
        self.square_center_y = 0.0  # Y center (left-right)
        self.square_center_z = 0.20 # Z center (height) - 20cm above base
        self.square_size = 0.05     # 5cm square
        
        # Motion parameters
        self.time_per_segment = 4.0     # Seconds to move between corners
        self.home_position_time = 4.0   # Seconds to move to home
        self.interpolation_points = 5   # Points between corners for smoother lines
        
        self.get_logger().info('Koch Square Drawer initialized (ros2_control version)')
        self.get_logger().info(f'Workspace: max_reach={self.ik_solver.max_reach:.3f}m')
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('✓ Action server connected')
    
    def send_trajectory(self, waypoints, time_per_segment):
        """
        Send a trajectory with multiple waypoints using action interface
        
        Args:
            waypoints: List of joint angle tuples (6 angles each)
            time_per_segment: Time in seconds for each segment
            
        Returns:
            Future for the action goal
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        current_time = 0.0
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = list(waypoint)
            point.time_from_start = Duration(
                sec=int(current_time),
                nanosec=int((current_time % 1.0) * 1e9)
            )
            goal_msg.trajectory.points.append(point)
            current_time += time_per_segment
        
        self.get_logger().info(f'Sending trajectory with {len(waypoints)} waypoints...')
        
        # Send goal and wait for result
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server!')
            return None
        
        self.get_logger().info('Goal accepted, executing trajectory...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Trajectory execution succeeded')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with status: {result.status}')
            return False
    
    def move_to_home(self):
        """Move arm to safe home position (vertical, arm up)"""
        self.get_logger().info('Moving to home position...')
        
        # Home position: arm pointing mostly vertical, safe height
        home_angles = (
            0.0,     # shoulder_pan: centered
            0.0,     # shoulder_lift: 0° (horizontal from shoulder)
            -1.57,   # elbow_flex: -90° (bends up)
            1.57,    # wrist_flex: +90° (compensates, keeps pen up)
            0.0,     # wrist_roll: 0° (neutral)
            0.0      # pen_holder: 0° (pen up/neutral)
        )
        
        if not validate_joint_angles(home_angles):
            self.get_logger().error('Home position out of joint limits!')
            return False
        
        success = self.send_trajectory([home_angles], self.home_position_time)
        
        if success:
            self.get_logger().info('✓ Reached home position')
        else:
            self.get_logger().error('Failed to reach home position')
        
        return success
    
    def interpolate_points(self, start_angles, end_angles, num_points):
        """
        Linearly interpolate between two joint configurations
        
        Args:
            start_angles: Starting joint angles (tuple of 6)
            end_angles: Ending joint angles (tuple of 6)
            num_points: Number of intermediate points
            
        Returns:
            List of interpolated joint angle tuples
        """
        interpolated = []
        for i in range(num_points + 1):
            t = i / num_points
            angles = tuple(
                start + t * (end - start)
                for start, end in zip(start_angles, end_angles)
            )
            interpolated.append(angles)
        return interpolated
    
    def compute_square_corners(self):
        """
        Compute the 4 corners of the square in Cartesian space
        
        Returns:
            List of (y, z) coordinates for square corners
        """
        half_size = self.square_size / 2.0
        
        corners = [
            # Start at top-left, go clockwise
            (self.square_center_y - half_size, self.square_center_z + half_size),  # Top-left
            (self.square_center_y + half_size, self.square_center_z + half_size),  # Top-right
            (self.square_center_y + half_size, self.square_center_z - half_size),  # Bottom-right
            (self.square_center_y - half_size, self.square_center_z - half_size),  # Bottom-left
            (self.square_center_y - half_size, self.square_center_z + half_size),  # Back to top-left
        ]
        
        return corners
    
    def validate_collision_safety(self, angles):
        """
        Check that the arm won't collide with the writing surface
        
        Args:
            angles: Tuple of 6 joint angles
            
        Returns:
            True if safe, False if potential collision
        """
        # This is a simplified check - in production you'd use proper collision detection
        # For now, just check if any joint is in a dangerous configuration
        
        # Check that pen is not trying to go through the surface
        # (This would require forward kinematics - simplified for now)
        
        # Check joint limits
        if not validate_joint_angles(angles):
            return False
        
        return True
    
    def draw_square(self):
        """Main function to draw the square"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting square drawing sequence')
        self.get_logger().info('=' * 60)
        
        # Step 1: Move to home position
        if not self.move_to_home():
            self.get_logger().error('Failed to reach home position')
            return False
        
        # Step 2: Compute square corners
        corners = self.compute_square_corners()
        self.get_logger().info(f'Square: {len(corners)} corners, size={self.square_size}m')
        
        # Step 3: Generate trajectory
        all_waypoints = []
        
        # Move to first corner with pen away
        self.get_logger().info('\nPhase 1: Moving to start position (pen away)...')
        y_start, z_start = corners[0]
        start_angles = self.ik_solver.solve_for_vertical_surface(
            self.pen_away_x, y_start, z_start
        )
        
        if not start_angles:
            self.get_logger().error('Start position unreachable!')
            return False
        
        all_waypoints.append(start_angles)
        
        # Draw the square
        self.get_logger().info('\nPhase 2: Drawing square...')
        for i in range(len(corners) - 1):
            y_from, z_from = corners[i]
            y_to, z_to = corners[i + 1]
            
            # Lift pen (move back in X)
            angles_pen_up = self.ik_solver.solve_for_vertical_surface(
                self.pen_away_x, y_from, z_from
            )
            
            if not angles_pen_up:
                self.get_logger().error(f'Pen up position {i} unreachable!')
                return False
            
            # Move pen to surface AND set pen_holder to indicate "pen down"
            angles_pen_down = self.ik_solver.solve_for_vertical_surface(
                self.pen_touch_x, y_from, z_from
            )
            
            if not angles_pen_down:
                self.get_logger().error(f'Pen down position {i} unreachable!')
                return False
            
            # Modify pen_holder to indicate pen is DOWN (small positive value)
            angles_pen_down = list(angles_pen_down)
            angles_pen_down[5] = 0.2  # pen_holder: positive = pen down
            angles_pen_down = tuple(angles_pen_down)
            
            # Validate safety
            if not self.validate_collision_safety(angles_pen_down):
                self.get_logger().error(f'Position {i} fails safety check!')
                return False
            
            # Add pen down waypoint
            all_waypoints.append(angles_pen_down)
            
            # Draw line to next corner (with interpolation for smoother motion)
            angles_next = self.ik_solver.solve_for_vertical_surface(
                self.pen_touch_x, y_to, z_to
            )
            
            if not angles_next:
                self.get_logger().error(f'Corner {i+1} unreachable!')
                return False
            
            # Set pen_holder to indicate pen DOWN for next corner too
            angles_next = list(angles_next)
            angles_next[5] = 0.2  # pen_holder: positive = pen down
            angles_next = tuple(angles_next)
            
            # Interpolate between current and next position (pen stays down)
            interpolated = self.interpolate_points(
                angles_pen_down, angles_next, self.interpolation_points
            )
            all_waypoints.extend(interpolated[1:])  # Skip first point (already added)
            
            self.get_logger().info(f'  ✓ Line {i+1}: ({y_from:.3f}, {z_from:.3f}) → '
                                 f'({y_to:.3f}, {z_to:.3f})')
        
        # Step 4: Lift pen and return to home
        self.get_logger().info('\nPhase 3: Lifting pen and returning home...')
        y_final, z_final = corners[-1]
        final_pen_up = self.ik_solver.solve_for_vertical_surface(
            self.pen_away_x, y_final, z_final
        )
        
        if final_pen_up:
            all_waypoints.append(final_pen_up)
        
        # Add home position (matches move_to_home function)
        home_angles = (0.0, 0.0, -1.57, 1.57, 0.0, 0.0)
        all_waypoints.append(home_angles)
        
        # Step 5: Execute trajectory
        self.get_logger().info(f'\nExecuting complete trajectory with {len(all_waypoints)} waypoints...')
        success = self.send_trajectory(all_waypoints, self.time_per_segment)
        
        if not success:
            self.get_logger().error('Trajectory execution failed!')
            return False
        
        total_time = len(all_waypoints) * self.time_per_segment
        self.get_logger().info(f'Trajectory completed in ~{total_time:.1f} seconds')
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('✓ Square drawing complete!')
        self.get_logger().info('=' * 60)
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = KochSquareDrawer()
    
    # Draw the square
    success = node.draw_square()
    
    if success:
        node.get_logger().info('\n🎉 SUCCESS! Square drawn successfully!')
    else:
        node.get_logger().error('\n❌ FAILED to draw square')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
