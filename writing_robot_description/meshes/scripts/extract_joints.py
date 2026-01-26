import FreeCAD as App
import math

doc = App.ActiveDocument


parts = [
    "Part006", # "base_link",
    "Part005", # "shoulder_link",
    "Part", # "upper_arm_link",
    "Part001", # "forearm_link",
    "Part002", # "wrist_link",
    "Part003", # hand_link",
    "Part004", # "pen_link"
]
joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "pen_holder"]

def euler_to_rpy(euler_deg):
    r = math.radians(euler_deg[2])
    p = math.radians(euler_deg[1])
    y = math.radians(euler_deg[0])
    return (r, p, y)

print("=== CORRECTED JOINT EXTRACTION (with parent-frame transform) ===\n")

for i in range(len(parts) - 1):
    if i >= len(joint_names):
        break
        
    parent = doc.getObject(parts[i])
    child = doc.getObject(parts[i+1])
    
    if not parent or not child:
        continue
    
    # Global position difference
    global_diff = child.Placement.Base - parent.Placement.Base
    
    # Transform into parent's reference frame - THIS IS CRITICAL!
    parent_frame_diff = parent.Placement.Rotation.inverted().multVec(global_diff)
    
    # Rotation difference
    rot_diff = child.Placement.Rotation.multiply(
        parent.Placement.Rotation.inverted()
    )
    euler = rot_diff.toEuler()
    rpy = euler_to_rpy(euler)
    
    # Axis in global frame
    shaft_local = App.Vector(0, 0, 1)
    shaft_global = child.Placement.Rotation.multVec(shaft_local)
    
    # Normalize
    length = math.sqrt(shaft_global.x**2 + shaft_global.y**2 + shaft_global.z**2)
    if length > 0.0001:
        shaft_normalized = App.Vector(shaft_global.x/length, shaft_global.y/length, shaft_global.z/length)
    else:
        shaft_normalized = App.Vector(0, 0, 1)
    
    print(f"=== {joint_names[i]}: {parts[i]} → {parts[i+1]} ===")
    print(f"Global diff:       ({global_diff.x:.1f}, {global_diff.y:.1f}, {global_diff.z:.1f}) mm")
    print(f"Parent-frame diff: ({parent_frame_diff.x:.1f}, {parent_frame_diff.y:.1f}, {parent_frame_diff.z:.1f}) mm")
    print()
    print(f'<joint name="{joint_names[i]}" type="revolute">')
    print(f'  <parent link="{parts[i]}"/>')
    print(f'  <child link="{parts[i+1]}"/>')
    print(f'  <origin xyz="{parent_frame_diff.x/1000:.6f} {parent_frame_diff.y/1000:.6f} {parent_frame_diff.z/1000:.6f}"')
    print(f'          rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>')
    print(f'  <axis xyz="{shaft_normalized.x:.6f} {shaft_normalized.y:.6f} {shaft_normalized.z:.6f}"/>')
    print(f'  <limit lower="-2.5" upper="2.5" effort="2.0" velocity="3.14"/>')
    print(f'</joint>\n')
