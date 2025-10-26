#!/usr/bin/env python3
"""
Draw Square with Koch v1.1 6-DOF Arm
Adapted from original 4-DOF draw_square_sim.py

This script demonstrates writing a square on a vertical surface using
the Koch v1.1 robot arm configuration.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time
from writing_robot_control.koch_v11_ik_solver import KochWritingIK, validate_joint_angles


class KochSquareDrawer(Node):
    """Node to draw a square using Koch v1.1 arm"""
    
    def __init__(self):
        super().__init__('koch_square_drawer')
        
        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/koch_v11_controller/joint_trajectory',
            10
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
        
        self.get_logger().info('Koch Square Drawer initialized')
        self.get_logger().info(f'Workspace: max_reach={self.ik_solver.max_reach:.3f}m')
    
    def send_trajectory(self, waypoints, time_per_segment):
        """
        Send a trajectory with multiple waypoints
        
        Args:
            waypoints: List of joint angle tuples (6 angles each)
            time_per_segment: Time in seconds for each segment
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        current_time = 0.0
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = list(waypoint)
            point.time_from_start = Duration(
                sec=int(current_time),
                nanosec=int((current_time % 1.0) * 1e9)
            )
            trajectory_msg.points.append(point)
            current_time += time_per_segment
        
        self.trajectory_pub.publish(trajectory_msg)
        self.get_logger().info(f'Published trajectory with {len(waypoints)} waypoints')
    
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
        
        self.send_trajectory([home_angles], self.home_position_time)
        time.sleep(self.home_position_time + 0.5)
        
        self.get_logger().info('✓ Reached home position')
        return True
    
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
        self.send_trajectory(all_waypoints, self.time_per_segment)
        
        total_time = len(all_waypoints) * self.time_per_segment
        self.get_logger().info(f'Trajectory duration: {total_time:.1f} seconds')
        
        # Wait for completion
        time.sleep(total_time + 1.0)
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('✓ Square drawing complete!')
        self.get_logger().info('=' * 60)
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = KochSquareDrawer()
    
    # Give time for connections to establish
    time.sleep(2.0)
    
    # Draw the square
    success = node.draw_square()
    
    if success:
        node.get_logger().info('\n🎉 SUCCESS! Square drawn successfully!')
    else:
        node.get_logger().error('\n❌ FAILED to draw square')
    
    # Keep node alive briefly to ensure messages are sent
    time.sleep(2.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
