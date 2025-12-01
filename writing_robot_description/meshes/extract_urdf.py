#!/usr/bin/env python3
"""
Extract URDF joint and link definitions from FreeCAD assembly
Can be run inside FreeCAD Python console or as standalone script
"""

import FreeCAD as App
import math

# ============================================================================
# CONFIGURATION - UPDATE THESE TO MATCH YOUR PARTS
# ============================================================================

# List your Parts in kinematic chain order (parent to child)
LINK_PARTS = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "hand_link",
    "pen_tip",
]

# Joint names (one fewer than links)
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "pen_holder",
]

# Joint types (usually "revolute" for servos, "fixed" for rigid connections)
JOINT_TYPES = [
    "revolute",
    "revolute",
    "revolute",
    "revolute",
    "revolute",
    "fixed"
]

# Joint limits (in radians) - [lower, upper, effort, velocity]
# Adjust based on your servo specs
JOINT_LIMITS = [
    [-3.14159, 3.14159, 2.0, 2.0],  # shoulder_pan
    [-1.57, 1.57, 1.5, 2.0],         # shoulder_lift
    [-1.57, 1.57, 1.5, 2.0],         # elbow_flex
    [-1.57, 1.57, 1.5, 2.0],         # wrist_flex
    [-1.57, 1.57, 1.5, 2.0],         # wrist_roll
    [0, 0, 0, 0]                     # pen_holder
]

# Shaft direction in Part's local frame (usually Z-axis for standard servo orientation)
# This is the direction the servo shaft points BEFORE any Part rotation
SHAFT_LOCAL_DIRECTION = App.Vector(0, 0, 1)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def euler_to_rpy(euler_deg):
    """Convert FreeCAD Euler angles (yaw, pitch, roll in degrees) to ROS RPY (radians)"""
    # FreeCAD: (yaw, pitch, roll) in degrees
    # ROS: (roll, pitch, yaw) in radians
    r = math.radians(euler_deg[2])  # roll
    p = math.radians(euler_deg[1])  # pitch
    y = math.radians(euler_deg[0])  # yaw
    return (r, p, y)

def format_xyz(vector_mm):
    """Convert Vector in mm to meters and format for URDF"""
    x = vector_mm.x / 1000.0
    y = vector_mm.y / 1000.0
    z = vector_mm.z / 1000.0
    return f"{x:.6f} {y:.6f} {z:.6f}"

def format_rpy(rpy_rad):
    """Format RPY tuple (in radians) for URDF"""
    return f"{rpy_rad[0]:.6f} {rpy_rad[1]:.6f} {rpy_rad[2]:.6f}"

def format_axis(vector):
    """Format axis vector for URDF"""
    # Normalize the vector
    length = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
    if length > 0:
        x = vector.x / length
        y = vector.y / length
        z = vector.z / length
    else:
        x, y, z = 0, 0, 1  # Default to Z
    return f"{x:.6f} {y:.6f} {z:.6f}"

# ============================================================================
# EXTRACTION FUNCTIONS
# ============================================================================

def extract_joint_data(parent_part, child_part, joint_name, joint_type, limits):
    """Extract joint definition from parent and child Parts"""
    
    # Calculate transform from parent to child
    pos_diff = child_part.Placement.Base - parent_part.Placement.Base
    
    # Calculate rotation difference
    rot_diff = child_part.Placement.Rotation.multiply(
        parent_part.Placement.Rotation.inverted()
    )
    
    # Convert to RPY
    euler_deg = rot_diff.toEuler()
    rpy_rad = euler_to_rpy(euler_deg)
    
    # Calculate joint axis from child's rotation
    # The shaft points along SHAFT_LOCAL_DIRECTION in the child's local frame
    axis_global = child_part.Placement.Rotation.multVec(SHAFT_LOCAL_DIRECTION)
    
    # Generate URDF
    urdf = []
    urdf.append(f'  <joint name="{joint_name}" type="{joint_type}">')
    urdf.append(f'    <parent link="{parent_part.Label}"/>')
    urdf.append(f'    <child link="{child_part.Label}"/>')
    urdf.append(f'    <origin xyz="{format_xyz(pos_diff)}" rpy="{format_rpy(rpy_rad)}"/>')
    urdf.append(f'    <axis xyz="{format_axis(axis_global)}"/>')
    
    if joint_type == "revolute" or joint_type == "prismatic":
        urdf.append(f'    <limit lower="{limits[0]}" upper="{limits[1]}" '
                   f'effort="{limits[2]}" velocity="{limits[3]}"/>')
    
    urdf.append(f'  </joint>')
    
    return '\n'.join(urdf)

def extract_link_data(part):
    """Extract link definition including visual meshes"""
    
    # Find all meshes in this Part
    meshes = []
    if hasattr(part, 'Group'):
        meshes = [obj for obj in part.Group if obj.TypeId == "Mesh::Feature"]
    
    urdf = []
    urdf.append(f'  <link name="{part.Label}">')
    
    # Add visual elements for each mesh
    for i, mesh in enumerate(meshes):
        # Get mesh position/rotation relative to Part origin
        pos = mesh.Placement.Base
        rot = mesh.Placement.Rotation
        euler_deg = rot.toEuler()
        rpy_rad = euler_to_rpy(euler_deg)
        
        # Mesh filename (you'll need to update paths)
        mesh_name = mesh.Label
        if not mesh_name.endswith('.stl'):
            mesh_name += '.stl'
        
        urdf.append(f'    <visual name="{mesh.Label}_visual">')
        urdf.append(f'      <origin xyz="{format_xyz(pos)}" rpy="{format_rpy(rpy_rad)}"/>')
        urdf.append(f'      <geometry>')
        urdf.append(f'        <mesh filename="package://writing_robot_description/meshes/{mesh_name}"/>')
        urdf.append(f'      </geometry>')
        urdf.append(f'    </visual>')
    
    # Add collision (simplified - usually same as visual)
    for i, mesh in enumerate(meshes):
        pos = mesh.Placement.Base
        rot = mesh.Placement.Rotation
        euler_deg = rot.toEuler()
        rpy_rad = euler_to_rpy(euler_deg)
        
        mesh_name = mesh.Label
        if not mesh_name.endswith('.stl'):
            mesh_name += '.stl'
        
        urdf.append(f'    <collision name="{mesh.Label}_collision">')
        urdf.append(f'      <origin xyz="{format_xyz(pos)}" rpy="{format_rpy(rpy_rad)}"/>')
        urdf.append(f'      <geometry>')
        urdf.append(f'        <mesh filename="package://writing_robot_description/meshes/{mesh_name}"/>')
        urdf.append(f'      </geometry>')
        urdf.append(f'    </collision>')
    
    urdf.append(f'  </link>')
    
    return '\n'.join(urdf)

# ============================================================================
# MAIN EXTRACTION
# ============================================================================

def generate_urdf():
    """Generate complete URDF from FreeCAD assembly"""
    
    doc = App.ActiveDocument
    if doc is None:
        print("ERROR: No document open in FreeCAD")
        return
    
    urdf = []
    
    # Header
    urdf.append('<?xml version="1.0"?>')
    urdf.append('<robot name="robot_arm">')
    urdf.append('')
    
    # Generate link definitions
    urdf.append('  <!-- ===== LINK DEFINITIONS ===== -->')
    urdf.append('')
    
    for link_name in LINK_PARTS:
        part = doc.getObject(link_name)
        if part is None:
            print(f"WARNING: Part '{link_name}' not found in document")
            urdf.append(f'  <!-- ERROR: {link_name} not found -->')
            continue
        
        link_urdf = extract_link_data(part)
        urdf.append(link_urdf)
        urdf.append('')
    
    # Generate joint definitions
    urdf.append('  <!-- ===== JOINT DEFINITIONS ===== -->')
    urdf.append('')
    
    for i in range(len(LINK_PARTS) - 1):
        parent_name = LINK_PARTS[i]
        child_name = LINK_PARTS[i + 1]
        joint_name = JOINT_NAMES[i]
        joint_type = JOINT_TYPES[i]
        limits = JOINT_LIMITS[i]
        
        parent = doc.getObject(parent_name)
        child = doc.getObject(child_name)
        
        if parent is None or child is None:
            print(f"WARNING: Cannot create joint - parent or child not found")
            continue
        
        joint_urdf = extract_joint_data(parent, child, joint_name, joint_type, limits)
        urdf.append(joint_urdf)
        urdf.append('')
    
    # Footer
    urdf.append('</robot>')
    
    return '\n'.join(urdf)

# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("URDF EXTRACTION FROM FREECAD ASSEMBLY")
    print("=" * 70)
    print()
    
    # Check configuration
    print("Configuration:")
    print(f"  Links: {len(LINK_PARTS)}")
    print(f"  Joints: {len(JOINT_NAMES)}")
    print()
    
    if len(JOINT_NAMES) != len(LINK_PARTS) - 1:
        print("ERROR: Number of joints should be one less than number of links")
    elif len(JOINT_TYPES) != len(JOINT_NAMES):
        print("ERROR: Number of joint types doesn't match number of joints")
    elif len(JOINT_LIMITS) != len(JOINT_NAMES):
        print("ERROR: Number of joint limits doesn't match number of joints")
    else:
        # Generate URDF
        urdf_output = generate_urdf()
        
        # Print to console
        print(urdf_output)
        print()
        print("=" * 70)
        
        # Optionally save to file
        save_to_file = True  # Set to True to save
        if save_to_file:
            output_path = "/tmp/robot_arm.urdf"  # UPDATE THIS PATH
            with open(output_path, 'w') as f:
                f.write(urdf_output)
            print(f"URDF saved to: {output_path}")
        
        print()
        print("NOTES:")
        print("  - Update mesh filenames and package names")
        print("  - Verify joint limits match your servo specs")
        print("  - Check joint axes directions in RViz")
        print("  - Add inertial and mass properties as needed")
