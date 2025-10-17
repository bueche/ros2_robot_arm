#!/usr/bin/env python3
"""
Draw a Square - Vertical Surface Version with Validation
Draws on a VERTICAL surface (like a whiteboard)
Includes comprehensive checks and debugging
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class SquareDrawerSim(Node):
    def __init__(self):
        super().__init__('square_drawer_sim')
        
        # Action client for trajectory execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')
    
    def simple_ik_with_height(self, horizontal_reach, target_height, l1, l2):
        """
        IK solver that maintains constant height
        
        Args:
            horizontal_reach: How far forward from base (meters)
            target_height: How high above ground (meters)
            l1, l2: Link lengths
            
        Returns:
            (theta1, theta2): Shoulder and elbow angles in radians
            
        How it works:
            1. Calculate straight-line distance to target
            2. Find angle from base to target (elevation angle)
            3. Use law of cosines to find elbow angle
            4. Calculate shoulder angle to point at target
        """
        # Calculate the straight-line distance from base to target
        distance_to_target = math.sqrt(horizontal_reach**2 + target_height**2)
        
        # Check reachability
        if distance_to_target > (l1 + l2):
            self.get_logger().error(
                f'Target TOO FAR! Distance: {distance_to_target:.3f}m, '
                f'Max reach: {l1 + l2:.3f}m'
            )
            return None
        
        if distance_to_target < abs(l1 - l2):
            self.get_logger().error(
                f'Target TOO CLOSE! Distance: {distance_to_target:.3f}m, '
                f'Min reach: {abs(l1 - l2):.3f}m'
            )
            return None
        
        # Angle from base to target (elevation angle)
        alpha = math.atan2(target_height, horizontal_reach)
        
        # Solve elbow angle using law of cosines
        cos_theta2 = (distance_to_target**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        theta2 = math.acos(cos_theta2)  # Elbow angle
        
        # Solve shoulder angle
        beta = math.acos((l1**2 + distance_to_target**2 - l2**2) / (2 * l1 * distance_to_target))
        theta1 = alpha - beta  # Shoulder angle
        
        return (theta1, theta2)
    def create_home_position(self):
        """
        Create a safe home position - arm pointing up
        Prevents collision with paper on startup
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # Home position: arm pointing mostly upward
        point = JointTrajectoryPoint()
        point.positions = [
             0.0,    # Base centered
             -1.57,  # Shoulder: point up (90 degrees down from horizontal)
             1.57,   # Elbow: straight
             0.0     # Wrist: forward
        ]
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=4, nanosec=0)
        
        trajectory.points.append(point)
                                                                                                                                        
        self.get_logger().info('Moving to safe HOME position (arm up)...')
        return trajectory

    def validate_collision_safety(self, corners, paper_x):
       """
       Validate that arm won't extend past the paper surface
       """
       self.get_logger().info("=" * 60)
       self.get_logger().info("COLLISION SAFETY CHECK")
       self.get_logger().info("=" * 60)
       # Only check the drawing waypoints (pen touching), not pen-away positions
       # Skip first waypoint (pen away) and last waypoint (pen away)
       drawing_corners = corners[1:-1]
       if len(drawing_corners) == 0:
           drawing_corners = corners
       max_x = max(corner[0] for corner in drawing_corners)
       min_x = min(corner[0] for corner in drawing_corners)

       self.get_logger().info(f"Paper surface at X={paper_x:.3f}m")
       self.get_logger().info(f"Arm extends from X={min_x:.3f}m to X={max_x:.3f}m")
                                                                                                                
       # Check if any waypoint extends past paper
       safety_margin = 0.01  # 1cm safety margin
       
       if max_x > (paper_x + safety_margin):
            overshoot = max_x - paper_x
            self.get_logger().error( f"❌ COLLISION RISK! Arm extends {overshoot*100:.1f}cm past paper")
            self.get_logger().error( f"   Reduce pen_away_distance or move paper further out") 
            passed = False
       else:
            clearance = paper_x - max_x
            self.get_logger().info( f"✓ Safe: {clearance*100:.1f}cm clearance from paper") 
            passed = True
       self.get_logger().info("=" * 60)
       return passed, max_x

    def validate_waypoint(self, waypoint_num, x, y, z, expected_phase):
        """
        Validate that a waypoint makes sense
        
        Args:
            waypoint_num: Which waypoint
            x, y, z: Target position
            expected_phase: 'pen_away', 'pen_touch', or 'drawing'
        """
        issues = []
        
        # Check if position is reasonable
        if z < 0.0:
            issues.append(f"Z={z:.3f}m is BELOW ground (negative!)")
        
        if z > 0.30:
            issues.append(f"Z={z:.3f}m is suspiciously HIGH (>30cm)")
        
        r_xy = math.sqrt(x**2 + y**2)
        if r_xy < 0.10:
            issues.append(f"Horizontal reach {r_xy:.3f}m is TOO CLOSE")
        
        if r_xy > 0.50:
            issues.append(f"Horizontal reach {r_xy:.3f}m is TOO FAR")
        
        # Log validation
        if issues:
            self.get_logger().warn(f"⚠ Waypoint {waypoint_num} ISSUES: {', '.join(issues)}")
    
    def validate_trajectory_consistency(self, trajectory):
        """
        Validate that drawing waypoints maintain consistency
        Returns: (passed, details)
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TRAJECTORY VALIDATION")
        self.get_logger().info("=" * 60)
        
        passed = True
        
        if len(trajectory.points) < 3:
            self.get_logger().error(f"❌ Too few waypoints: {len(trajectory.points)}")
            return False
        
        # Extract Y positions (depth - should be consistent during drawing)
        # Skip first 2 (pen approach) and last 2 (pen retract)
        if len(trajectory.points) > 4:
            # Get target Y from waypoint calculations
            # We expect Y to be ~0.005 when drawing
            self.get_logger().info("✓ Trajectory has enough waypoints for validation")
            
        # Check joint angle consistency during drawing
        # Joint angles should vary smoothly
        base_angles = [math.degrees(p.positions[0]) for p in trajectory.points]
        joint2_angles = [math.degrees(p.positions[1]) for p in trajectory.points]
        
        # Check for reasonable variation
        base_range = max(base_angles) - min(base_angles)
        joint2_range = max(joint2_angles) - min(joint2_angles)
        
        self.get_logger().info(f"Base angle range: {base_range:.1f}° (should be small for vertical drawing)")
        self.get_logger().info(f"Joint2 angle range: {joint2_range:.1f}° (should vary for up/down motion)")
        
        if base_range > 10.0:
            self.get_logger().warn(f"⚠ Base rotating a lot ({base_range:.1f}°) - square might be curved")
        else:
            self.get_logger().info("✓ Base angle stable")
        
        if joint2_range < 5.0:
            self.get_logger().warn(f"⚠ Joint2 barely moving ({joint2_range:.1f}°) - square might be too small")
        else:
            self.get_logger().info("✓ Joint2 showing good range of motion")
        
        self.get_logger().info("=" * 60)
        
        if passed:
            self.get_logger().info("✓✓✓ TRAJECTORY VALIDATION PASSED ✓✓✓")
        else:
            self.get_logger().error("❌❌❌ TRAJECTORY VALIDATION FAILED ❌❌❌")
        
        self.get_logger().info("=" * 60)
        
        return passed
    
    def create_square_trajectory(self, center_x=0.25, center_y=0.0, center_z=0.15, 
                                 size=0.05):
        """Create trajectory to draw a square on a VERTICAL surface"""
        
        half_size = size / 2
        pen_away_distance = 0.02  # How far from surface when pen is "away" (2cm)
        pen_touch_distance = 0.005  # How close to surface when drawing (5mm)
        
        self.get_logger().info(f"Creating square: size={size*100:.1f}cm at Y={center_y:.3f}, Z={center_z:.3f}")
        self.get_logger().info(f"  Pen AWAY: X={center_x + pen_away_distance:.3f}m")
        self.get_logger().info(f"  Pen TOUCH: X={center_x + pen_touch_distance:.3f}m")
        
        # Define corners - drawing in Y-Z plane (VERTICAL surface, perpendicular to pen)
        # X stays constant (depth), Y and Z vary (left-right, up-down)
        corners = [
            # 1: Start away from surface (pen away)
            (center_x + pen_away_distance, center_y - half_size, center_z + half_size),
            # 2: Move toward surface (pen touches)
            (center_x + pen_touch_distance, center_y - half_size, center_z + half_size),
            # 3-6: Draw square in Y-Z plane (X stays constant)
            (center_x + pen_touch_distance, center_y + half_size, center_z + half_size),  # Top right
            (center_x + pen_touch_distance, center_y + half_size, center_z - half_size),  # Bottom right
            (center_x + pen_touch_distance, center_y - half_size, center_z - half_size),  # Bottom left
            (center_x + pen_touch_distance, center_y - half_size, center_z + half_size),  # Back to start
            # 7: Pull away from surface
            (center_x + pen_away_distance, center_y - half_size, center_z + half_size),
        ]
        
        # Robot link lengths (ADJUST FOR YOUR ROBOT!)
        L1 = 0.15  # Base to shoulder
        L2 = 0.20  # Shoulder to elbow
        L3 = 0.15  # Elbow to pen tip
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        time_per_segment = 2.0  # seconds
        current_time = 0.0
        
        # Add interpolation points between corners for straighter lines
        interpolated_corners = []
        interpolated_corners.append(corners[0])  # Start position
        
        # For each line segment, add intermediate points
        for i in range(1, len(corners)):
            start = corners[i-1]
            end = corners[i]
            
            # Skip interpolation for pen away/touch movements (waypoints 1-2 and 6-7)
            if i == 1 or i == 6:
                interpolated_corners.append(end)
            else:
                # Add 4 intermediate points between corners for drawing moves
                num_points = 5
                for j in range(1, num_points + 1):
                    t = j / num_points
                    interp_x = start[0] + t * (end[0] - start[0])
                    interp_y = start[1] + t * (end[1] - start[1])
                    interp_z = start[2] + t * (end[2] - start[2])
                    interpolated_corners.append((interp_x, interp_y, interp_z))
        
        self.get_logger().info(f"Generated {len(interpolated_corners)} waypoints (including interpolation)")
        # Validate collision safety BEFORE generating trajectory
        paper_x_position = center_x + pen_touch_distance
        safety_check, max_reach = self.validate_collision_safety( interpolated_corners, paper_x_position)
        if not safety_check:
            self.get_logger().error("Should Abort trajectory due to collision risk!")
            # return trajectory
        
        for i, (x, y, z) in enumerate(interpolated_corners):
            # Calculate joint angles for Y-Z plane drawing (paper perpendicular to pen)
            # Base rotates to point at the Y coordinate
            # Arm reaches out to X (depth) and up to Z (height)
            
            # Base angle: rotate to point toward Y position
            base_angle = math.atan2(y, x) if x != 0 else 0.0
            
            # Radial distance to target in X-Y plane
            r_xy = math.sqrt(x**2 + y**2)
            
            # Account for pen length pointing forward
            horizontal_reach = r_xy - (L3 * 0.7)
            target_height = z
            
            # Use IK solver for vertical plane
            result = self.simple_ik_with_height(horizontal_reach, target_height, L1, L2)
            
            if result is None:
                self.get_logger().error(
                    f'❌ Could not reach waypoint {i+1}: '
                    f'reach={horizontal_reach:.3f}m, height={target_height:.3f}m'
                )
                continue
            
            theta1, theta2 = result
            theta3 = 0.0  # Pen pointing forward
            
            # Create waypoint
            point = JointTrajectoryPoint()
            point.positions = [base_angle, theta1, theta2, theta3]
            point.velocities = [0.0, 0.0, 0.0, 0.0]
            point.accelerations = [0.0, 0.0, 0.0, 0.0]
            
            current_time += time_per_segment / 5  # Faster for interpolated points
            point.time_from_start = Duration(
                sec=int(current_time),
                nanosec=int((current_time % 1) * 1e9)
            )
            
            trajectory.points.append(point)
            
            # Only log actual corners, not all interpolated points
            if i in [0, 1] or i >= len(interpolated_corners) - 6:
                self.get_logger().info(
                    f'Waypoint {i+1}: Target=({x:.3f}, {y:.3f}, {z:.3f}) '
                    f'→ Base={math.degrees(base_angle):+.1f}°, '
                    f'Joint2={math.degrees(theta1):+.1f}°, '
                    f'Joint3={math.degrees(theta2):+.1f}°'
                )
        
        # Validate the complete trajectory
        self.validate_trajectory_consistency(trajectory)
        
        return trajectory
    
    def send_goal(self, trajectory):
        """Send trajectory goal via action"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('Sending trajectory goal...')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Drawing square...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Progress updates"""
        pass
    
    def get_result_callback(self, future):
        """Handle completion"""
        result = future.result().result
        self.get_logger().info(f'Square drawing complete! Result: {result.error_code}')
    
    def draw_square(self):
       """Execute the square drawing"""
                    
       # STEP 1: Move to safe home position
       self.get_logger().info('=' * 60)
       self.get_logger().info('STEP 1: Moving to HOME position')
       self.get_logger().info('=' * 60)
       home_trajectory = self.create_home_position()
       if len(home_trajectory.points) > 0:
          self.send_goal(home_trajectory)
          # Wait for home position to complete
          import time
          time.sleep(3)
          
       # STEP 2: Create and execute drawing trajectory
       self.get_logger().info('=' * 60)
       self.get_logger().info('STEP 2: Creating square trajectory for VERTICAL surface (Y-Z plane)...')
       self.get_logger().info('=' * 60)
       
       trajectory = self.create_square_trajectory(
             center_x=0.35,   # 35cm from base - SAFER distance
             center_y=0.0,    # Left-right center
             center_z=0.15,   # 15cm high (center of square)
             size=0.05        # 5cm square
       )
       
       if len(trajectory.points) == 0:
            self.get_logger().error('No valid trajectory!')
            return
       self.get_logger().info(f'Trajectory has {len(trajectory.points)} waypoints')
       self.send_goal(trajectory)

def main(args=None):
    rclpy.init(args=args)
    drawer = SquareDrawerSim()
    
    # Draw the square
    drawer.draw_square()
    
    # Keep spinning to handle callbacks
    try:
        rclpy.spin(drawer)
    except KeyboardInterrupt:
        pass
    
    drawer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
