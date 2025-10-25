#!/usr/bin/env python3
"""
6-DOF Inverse Kinematics Solver for Koch v1.1 Writing Arm

This IK solver is specifically designed for a writing application where:
- The arm needs to reach points on a surface
- The pen should be perpendicular to the writing surface
- Base rotation (shoulder_pan) handles horizontal positioning
- Shoulder, elbow, wrist handle vertical reach and approach angle
- Wrist roll orients the pen
- Pen holder stays mostly fixed

Koch v1.1 Kinematic Structure:
1. shoulder_pan: Base rotation (Z-axis)
2. shoulder_lift: Shoulder pitch (Y-axis)
3. elbow_flex: Elbow pitch (Y-axis)
4. wrist_flex: Wrist pitch (Y-axis)
5. wrist_roll: Wrist roll (Z-axis)
6. pen_holder: Pen angle adjustment (Y-axis) - typically fixed
"""

import numpy as np
from typing import Tuple, Optional
import math


class KochWritingIK:
    """Inverse kinematics solver for Koch v1.1 arm configured for writing"""
    
    def __init__(self, 
                 shoulder_height: float = 0.04,  # Height from base to shoulder_lift
                 upper_arm_length: float = 0.20,  # shoulder_lift to elbow_flex
                 forearm_length: float = 0.16,    # elbow_flex to wrist_flex
                 wrist_to_pen: float = 0.15):     # wrist_flex to pen tip
        """
        Initialize the IK solver with Koch v1.1 link dimensions
        
        Args:
            shoulder_height: Height from base_link to shoulder_lift joint
            upper_arm_length: Length of upper arm (shoulder to elbow)
            forearm_length: Length of forearm (elbow to wrist)
            wrist_to_pen: Distance from wrist_flex to pen tip
        
        NOTE: These are estimated values. Measure your actual assembled robot!
        """
        self.shoulder_height = shoulder_height
        self.L1 = upper_arm_length
        self.L2 = forearm_length
        self.L3 = wrist_to_pen
        
        # Maximum reach (fully extended)
        self.max_reach = self.L1 + self.L2 + self.L3
        
        # Minimum reach (fully retracted)
        self.min_reach = abs(self.L1 - self.L2 - self.L3)
        
    def solve_for_point(self, 
                       target_x: float, 
                       target_y: float, 
                       target_z: float,
                       pen_approach_angle: float = 0.0,
                       pen_roll: float = 0.0) -> Optional[Tuple[float, ...]]:
        """
        Solve IK for a target point with pen perpendicular to surface
        
        Args:
            target_x: X coordinate of pen tip (depth from base)
            target_y: Y coordinate of pen tip (left-right)
            target_z: Z coordinate of pen tip (height)
            pen_approach_angle: Angle of pen approach (0 = horizontal, pi/2 = vertical down)
            pen_roll: Roll angle of pen around its axis
            
        Returns:
            Tuple of 6 joint angles (shoulder_pan, shoulder_lift, elbow_flex, 
                                    wrist_flex, wrist_roll, pen_holder) or None if unreachable
        """
        # 1. Calculate base rotation (shoulder_pan) to point towards target
        shoulder_pan = math.atan2(target_y, target_x)
        
        # 2. Calculate horizontal distance from shoulder to target (in X-Y plane)
        horizontal_dist = math.sqrt(target_x**2 + target_y**2)
        
        # 3. Calculate vertical distance from shoulder_lift to target
        vertical_dist = target_z - self.shoulder_height
        
        # 4. Account for pen approach angle
        # The pen tip is offset from wrist based on approach angle
        pen_offset_horizontal = self.L3 * math.cos(pen_approach_angle)
        pen_offset_vertical = self.L3 * math.sin(pen_approach_angle)
        
        # Wrist position (where pen attaches)
        wrist_horizontal = horizontal_dist - pen_offset_horizontal
        wrist_vertical = vertical_dist - pen_offset_vertical
        
        # 5. Check if wrist position is reachable with shoulder + elbow
        wrist_dist = math.sqrt(wrist_horizontal**2 + wrist_vertical**2)
        
        if wrist_dist > (self.L1 + self.L2) or wrist_dist < abs(self.L1 - self.L2):
            print(f"Target unreachable: wrist_dist={wrist_dist:.3f}, "
                  f"max={self.L1+self.L2:.3f}, min={abs(self.L1-self.L2):.3f}")
            return None
        
        # 6. Solve for elbow angle using law of cosines
        cos_elbow = (wrist_dist**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_elbow = np.clip(cos_elbow, -1.0, 1.0)  # Numerical stability
        
        # Elbow up configuration (negative angle)
        elbow_flex = -math.acos(cos_elbow)
        
        # 7. Solve for shoulder angle
        # Angle from horizontal to wrist
        wrist_angle = math.atan2(wrist_vertical, wrist_horizontal)
        
        # Angle of upper arm segment
        alpha = math.atan2(self.L2 * math.sin(-elbow_flex), 
                          self.L1 + self.L2 * math.cos(-elbow_flex))
        
        shoulder_lift = wrist_angle - alpha
        
        # 8. Calculate wrist angle to achieve desired pen approach angle
        # Total angle from horizontal = shoulder + elbow + wrist
        wrist_flex = pen_approach_angle - shoulder_lift - elbow_flex
        
        # 9. Set wrist roll and pen holder
        wrist_roll = pen_roll
        pen_holder = 0.0  # Keep pen straight for writing
        
        return (shoulder_pan, shoulder_lift, elbow_flex, 
                wrist_flex, wrist_roll, pen_holder)
    
    def solve_for_vertical_surface(self,
                                   surface_x: float,
                                   target_y: float,
                                   target_z: float,
                                   pen_away_distance: float = 0.02) -> Optional[Tuple[float, ...]]:
        """
        Specialized IK for writing on a vertical surface (like your original project)
        
        Args:
            surface_x: X position of the vertical surface
            target_y: Y coordinate on surface (left-right)
            target_z: Z coordinate on surface (up-down)
            pen_away_distance: How far to keep pen from surface when not touching
            
        Returns:
            Tuple of 6 joint angles or None if unreachable
        """
        # For vertical surface, pen should approach horizontally (perpendicular to surface)
        pen_approach_angle = 0.0  # Horizontal approach
        
        target_x = surface_x + pen_away_distance
        
        return self.solve_for_point(target_x, target_y, target_z, 
                                    pen_approach_angle=pen_approach_angle)
    
    def get_workspace_limits(self) -> dict:
        """
        Calculate approximate workspace limits
        
        Returns:
            Dictionary with workspace boundaries
        """
        return {
            'max_reach': self.max_reach,
            'min_reach': self.min_reach,
            'max_height': self.max_reach + self.shoulder_height,
            'min_height': self.shoulder_height - self.max_reach,
            'shoulder_height': self.shoulder_height
        }


def validate_joint_angles(angles: Tuple[float, ...], 
                         limits: Optional[dict] = None) -> bool:
    """
    Validate that joint angles are within safe limits
    
    Args:
        angles: Tuple of 6 joint angles
        limits: Dictionary of joint limits (uses defaults if None)
    
    Returns:
        True if all angles are within limits
    """
    if limits is None:
        # Default Koch v1.1 joint limits (in radians)
        limits = {
            'shoulder_pan': (-3.14, 3.14),
            'shoulder_lift': (-2.0, 2.0),
            'elbow_flex': (-2.5, 2.5),
            'wrist_flex': (-2.0, 2.0),
            'wrist_roll': (-3.14, 3.14),
            'pen_holder': (-0.5, 0.5)
        }
    
    joint_names = list(limits.keys())
    
    for i, (angle, name) in enumerate(zip(angles, joint_names)):
        lower, upper = limits[name]
        if not (lower <= angle <= upper):
            print(f"Joint {name} out of range: {angle:.3f} not in [{lower:.3f}, {upper:.3f}]")
            return False
    
    return True


# Example usage
if __name__ == "__main__":
    # Create IK solver with Koch v1.1 dimensions
    # NOTE: These are estimated! Measure your actual robot!
    ik_solver = KochWritingIK(
        shoulder_height=0.04,
        upper_arm_length=0.20,
        forearm_length=0.16,
        wrist_to_pen=0.15
    )
    
    print("Koch v1.1 Writing IK Solver")
    print("=" * 50)
    
    # Print workspace limits
    workspace = ik_solver.get_workspace_limits()
    print(f"\nWorkspace limits:")
    print(f"  Max reach: {workspace['max_reach']:.3f} m")
    print(f"  Min reach: {workspace['min_reach']:.3f} m")
    print(f"  Max height: {workspace['max_height']:.3f} m")
    print(f"  Min height: {workspace['min_height']:.3f} m")
    
    # Test case: Point on vertical surface
    print("\nTest case: Point on vertical surface")
    surface_x = 0.35  # 35cm from base
    target_y = 0.0    # Center
    target_z = 0.20   # 20cm high
    
    result = ik_solver.solve_for_vertical_surface(surface_x, target_y, target_z)
    
    if result:
        joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 
                      'wrist_flex', 'wrist_roll', 'pen_holder']
        print("\nSolution found:")
        for name, angle in zip(joint_names, result):
            print(f"  {name:15s}: {angle:7.3f} rad ({math.degrees(angle):7.2f}°)")
        
        # Validate angles
        if validate_joint_angles(result):
            print("\n✓ All joint angles within safe limits")
        else:
            print("\n✗ Some joint angles exceed limits!")
    else:
        print("\n✗ Target unreachable!")
    
    # Test several points for a square
    print("\n" + "=" * 50)
    print("Testing square trajectory on vertical surface:")
    print("=" * 50)
    
    square_size = 0.05  # 5cm square
    square_points = [
        (0.0, 0.20),  # Center
        (-square_size/2, 0.20 + square_size/2),  # Top left
        (square_size/2, 0.20 + square_size/2),   # Top right
        (square_size/2, 0.20 - square_size/2),   # Bottom right
        (-square_size/2, 0.20 - square_size/2),  # Bottom left
        (-square_size/2, 0.20 + square_size/2),  # Back to top left
    ]
    
    all_reachable = True
    for i, (y, z) in enumerate(square_points):
        result = ik_solver.solve_for_vertical_surface(surface_x, y, z)
        status = "✓" if result else "✗"
        print(f"{status} Point {i}: y={y:6.3f}, z={z:6.3f} - "
              f"{'Reachable' if result else 'UNREACHABLE'}")
        if not result:
            all_reachable = False
    
    if all_reachable:
        print("\n✓ All square points are reachable!")
    else:
        print("\n✗ Some points are unreachable. Adjust square position or size.")
