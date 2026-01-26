import FreeCAD as App
import math

doc = App.ActiveDocument

# List all your parts
# parts = ["base_link", "shoulder_link", "Part", "Part001", "wrist_link", "hand_link", "pen_tip"]
parts = [
    "Part006", # "base_link",
    "Part005", # "shoulder_link",
    "Part", # "upper_arm_link",
    "Part001", # "forearm_link",
    "Part002", # "wrist_link",
    "Part003", # hand_link",
    "Part004", # "pen_link"
]
def euler_to_rpy(euler_deg):
    """Convert FreeCAD Euler (Yaw, Pitch, Roll) to ROS RPY (Roll, Pitch, Yaw)"""
    roll_rad = math.radians(euler_deg[2])
    pitch_rad = math.radians(euler_deg[1])
    yaw_rad = math.radians(euler_deg[0])
    return (roll_rad, pitch_rad, yaw_rad)

print("=== LINK VISUAL DATA ===\n")

for part_name in parts:
    part = doc.getObject(part_name)
    if not part:
        print(f"{part_name}: NOT FOUND\n")
        continue
    
    if not hasattr(part, 'Group') or len(part.Group) == 0:
        print(f"{part_name}: No meshes found\n")
        continue
    
    print(f'<link name="{part_name}">')
    
    for obj in part.Group:
        if obj.TypeId == "Mesh::Feature":
            pos = obj.Placement.Base
            euler = obj.Placement.Rotation.toEuler()
            
            # Convert to meters
            x = pos.x / 1000.0
            y = pos.y / 1000.0
            z = pos.z / 1000.0
            
            # Convert rotation
            rpy = euler_to_rpy(euler)
            
            print(f'  <visual name="{obj.Label}_visual">')
            print(f'    <origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>')
            print(f'    <geometry>')
            print(f'      <mesh filename="package://writing_robot_description/meshes/{obj.Label}.stl"/>')
            print(f'    </geometry>')
            print(f'    <material name="grey">')
            print(f'      <color rgba="0.5 0.5 0.5 1.0"/>')
            print(f'    </material>')
            print(f'  </visual>')
    
    print(f'</link>\n')

print("\n=== VERIFICATION ===")
print("Check that:")
print("1. Connection points (brackets) should be at or near (0, 0, 0)")
print("2. Servos should be offset forward from origin")
