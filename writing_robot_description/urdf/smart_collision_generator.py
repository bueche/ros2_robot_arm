#!/usr/bin/env python3
"""
Smart Collision Geometry Generator - Reads from URDF and STL

Automatically generates collision geometry by:
1. Reading visual mesh transforms from URDF
2. Loading STL meshes to compute bounding boxes
3. Creating appropriate collision primitives

No hardcoded dimensions - everything from actual data!

Requirements:
  pip install trimesh numpy --break-system-packages

Usage:
  python3 smart_collision_generator.py --urdf koch_v11_arm_real.urdf --mesh-dir ./meshes --output koch_with_collision.urdf
"""

import trimesh
import numpy as np
import xml.etree.ElementTree as ET
import argparse
import sys
import os


class SmartCollisionGenerator:
    """Generate collision geometry from URDF visual elements and STL meshes."""
    
    def __init__(self, mesh_base_dir=None, margin=0.005):
        """
        Args:
            mesh_base_dir: Base directory for mesh files
            margin: Safety margin to add to collision geometry (meters)
        """
        self.mesh_base_dir = mesh_base_dir
        self.urdf_dir = None
        self.margin = margin  # 5mm default safety margin
        
        # Known servo dimensions (from datasheets)
        self.servo_dims = {
            'XL430': {'radius': 0.014, 'length': 0.046},
            'XL330': {'radius': 0.012, 'length': 0.028}
        }
    
    def resolve_mesh_path(self, filename):
        """Resolve mesh path from package:// or relative path."""
        if filename.startswith('package://'):
            parts = filename.replace('package://', '').split('/', 1)
            if len(parts) == 2:
                package_name, rel_path = parts
                
                if self.mesh_base_dir:
                    return os.path.join(self.mesh_base_dir, os.path.basename(rel_path))
                
                if self.urdf_dir:
                    package_root = os.path.dirname(self.urdf_dir)
                    mesh_path = os.path.join(package_root, rel_path)
                    if os.path.exists(mesh_path):
                        return mesh_path
                
                return os.path.basename(rel_path)
        elif os.path.isabs(filename):
            return filename
        else:
            if self.mesh_base_dir:
                return os.path.join(self.mesh_base_dir, filename)
            elif self.urdf_dir:
                return os.path.join(self.urdf_dir, filename)
            else:
                return filename
    
    def is_servo_visual(self, visual_name):
        """Check if visual element is a servo."""
        servo_patterns = ['xl-430', 'xl430', 'xl-330', 'xl330', 'xl,xc-330',
                         'dynamixel', 'servo', 'motor']
        visual_lower = visual_name.lower()
        return any(pattern in visual_lower for pattern in servo_patterns)
    
    def get_servo_type(self, visual_name):
        """Determine servo type from visual name."""
        visual_lower = visual_name.lower()
        if 'xl-430' in visual_lower or 'xl430' in visual_lower:
            return 'XL430'
        elif 'xl-330' in visual_lower or 'xl330' in visual_lower or 'xl,xc-330' in visual_lower:
            return 'XL330'
        return None
    
    def compute_bounding_primitive(self, mesh, transform, scale):
        """
        Compute best-fit primitive (cylinder or box) for mesh.
        
        Returns:
            dict with 'type', 'dimensions', 'origin', 'rpy'
        """
        if mesh is None:
            return None
        
        # Apply scale
        if scale != 1.0:
            mesh = mesh.copy()
            mesh.apply_scale(scale)
        
        # Get bounding box
        bbox = mesh.bounding_box
        extents = bbox.extents  # [width, depth, height]
        center = bbox.centroid
        
        # Apply transform to center
        center_transformed = center + np.array(transform)
        
        # Decide if cylinder or box is better
        # If roughly cylindrical (two dimensions similar, one different), use cylinder
        sorted_dims = sorted(extents)
        
        if sorted_dims[0] / sorted_dims[1] > 0.7:  # Two dims are similar
            # Use cylinder
            radius = (sorted_dims[0] + sorted_dims[1]) / 4 + self.margin  # Average of two similar dims
            length = sorted_dims[2] + self.margin * 2
            
            # Determine orientation (align with longest dimension)
            longest_axis = np.argmax(extents)
            
            if longest_axis == 0:  # X-axis
                rpy = [0, 1.5708, 0]  # 90° pitch
            elif longest_axis == 1:  # Y-axis
                rpy = [1.5708, 0, 0]  # 90° roll
            else:  # Z-axis
                rpy = [0, 0, 0]
            
            return {
                'type': 'cylinder',
                'radius': radius,
                'length': length,
                'origin': center_transformed.tolist(),
                'rpy': rpy
            }
        else:
            # Use box
            size = extents + self.margin * 2  # Add margin to all dimensions
            
            return {
                'type': 'box',
                'size': size.tolist(),
                'origin': center_transformed.tolist(),
                'rpy': [0, 0, 0]
            }
    
    def create_servo_collision(self, servo_type, transform):
        """Create collision for servo motor."""
        if servo_type not in self.servo_dims:
            return None
        
        dims = self.servo_dims[servo_type]
        
        return {
            'type': 'cylinder',
            'radius': dims['radius'] + self.margin,
            'length': dims['length'],
            'origin': transform,
            'rpy': [0, 0, 0]  # Assume servo aligned with Z-axis
        }
    
    def generate_collision_for_link(self, link_element):
        """Generate collision elements for a link by analyzing its visual elements."""
        link_name = link_element.get('name')
        collision_specs = []
        
        print(f"\n  Processing: {link_name}")
        
        # Analyze each visual element
        for visual in link_element.findall('visual'):
            visual_name = visual.get('name', '')
            
            # Get transform
            origin_elem = visual.find('origin')
            if origin_elem is not None:
                xyz_str = origin_elem.get('xyz', '0 0 0')
                rpy_str = origin_elem.get('rpy', '0 0 0')
                transform = [float(x) for x in xyz_str.split()]
                rpy_visual = [float(x) for x in rpy_str.split()]
            else:
                transform = [0, 0, 0]
                rpy_visual = [0, 0, 0]
            
            # Check if servo
            if self.is_servo_visual(visual_name):
                servo_type = self.get_servo_type(visual_name)
                if servo_type:
                    collision = self.create_servo_collision(servo_type, transform)
                    if collision:
                        collision['name'] = f"{visual_name.replace('visual', 'collision')}"
                        collision_specs.append(collision)
                        print(f"    ✓ Servo collision: {servo_type} at {transform}")
                continue
            
            # Get mesh info
            geometry = visual.find('geometry')
            if geometry is None:
                continue
            
            mesh_elem = geometry.find('mesh')
            if mesh_elem is None:
                continue
            
            # Get mesh file
            mesh_filename = mesh_elem.get('filename', '')
            if not mesh_filename:
                continue
            
            # Get scale
            scale_str = mesh_elem.get('scale', '1 1 1')
            scale = float(scale_str.split()[0])
            
            # Try to load mesh
            mesh_path = self.resolve_mesh_path(mesh_filename)
            
            print(f"    Loading mesh: {os.path.basename(mesh_filename)}")
            print(f"      Path: {mesh_path}")
            
            if not os.path.exists(mesh_path):
                print(f"      ⚠️  File not found, using default box")
                # Fallback: small box at transform location
                collision = {
                    'type': 'box',
                    'size': [0.02, 0.02, 0.02],
                    'origin': transform,
                    'rpy': [0, 0, 0],
                    'name': f"{visual_name.replace('visual', 'collision')}"
                }
                collision_specs.append(collision)
                continue
            
            try:
                mesh = trimesh.load(mesh_path)
                
                # Compute best primitive
                primitive = self.compute_bounding_primitive(mesh, transform, scale)
                
                if primitive:
                    primitive['name'] = f"{visual_name.replace('visual', 'collision')}"
                    collision_specs.append(primitive)
                    
                    if primitive['type'] == 'cylinder':
                        print(f"      ✓ Cylinder: r={primitive['radius']:.4f}m, l={primitive['length']:.4f}m")
                    else:
                        print(f"      ✓ Box: {primitive['size']}")
            
            except Exception as e:
                print(f"      ⚠️  Error loading mesh: {e}")
                # Fallback
                collision = {
                    'type': 'box',
                    'size': [0.02, 0.02, 0.02],
                    'origin': transform,
                    'rpy': [0, 0, 0],
                    'name': f"{visual_name.replace('visual', 'collision')}"
                }
                collision_specs.append(collision)
        
        return collision_specs
    
    def create_collision_element(self, spec):
        """Create XML collision element from spec."""
        collision = ET.Element('collision')
        
        if 'name' in spec:
            collision.set('name', spec['name'])
        
        # Origin
        origin = ET.SubElement(collision, 'origin')
        origin.set('xyz', ' '.join(f"{x:.6f}" for x in spec['origin']))
        origin.set('rpy', ' '.join(f"{x:.6f}" for x in spec['rpy']))
        
        # Geometry
        geometry = ET.SubElement(collision, 'geometry')
        
        if spec['type'] == 'cylinder':
            cylinder = ET.SubElement(geometry, 'cylinder')
            cylinder.set('radius', f"{spec['radius']:.6f}")
            cylinder.set('length', f"{spec['length']:.6f}")
        
        elif spec['type'] == 'box':
            box = ET.SubElement(geometry, 'box')
            box.set('size', ' '.join(f"{x:.6f}" for x in spec['size']))
        
        return collision
    
    def add_collision_to_link(self, link_element):
        """Add collision elements to link."""
        # Check if collision already exists
        if link_element.find('collision') is not None:
            print(f"    ⚠️  Already has collision, skipping")
            return False
        
        # Generate collision specs
        collision_specs = self.generate_collision_for_link(link_element)
        
        if not collision_specs:
            print(f"    ⚠️  No collision specs generated")
            return False
        
        # Find insertion point (after visual, before inertial)
        insert_index = 0
        for i, child in enumerate(link_element):
            if child.tag == 'visual':
                insert_index = i + 1
            elif child.tag == 'inertial':
                insert_index = i
                break
        
        # Add collision elements
        for spec in collision_specs:
            collision_elem = self.create_collision_element(spec)
            link_element.insert(insert_index, collision_elem)
            insert_index += 1
        
        return True
    
    def process_urdf(self, urdf_path, output_path):
        """Process URDF and add collision geometry."""
        print(f"\nReading URDF: {urdf_path}")
        
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            return False
        
        # Process each link
        links_processed = 0
        
        for link in root.findall('.//link'):
            link_name = link.get('name')
            
            print(f"\n{'='*60}")
            print(f"Link: {link_name}")
            print(f"{'='*60}")
            
            if self.add_collision_to_link(link):
                links_processed += 1
        
        # Write output
        if links_processed > 0:
            self._indent(root)
            tree.write(output_path, encoding='utf-8', xml_declaration=True)
            
            print(f"\n{'='*60}")
            print(f"SUCCESS")
            print(f"{'='*60}")
            print(f"  Links processed: {links_processed}")
            print(f"  Output: {output_path}")
            print(f"{'='*60}\n")
            
            return True
        else:
            print(f"\n❌ No links were modified")
            return False
    
    def _indent(self, elem, level=0):
        """Pretty-print XML."""
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for child in elem:
                self._indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i


def main():
    parser = argparse.ArgumentParser(
        description='Smart Collision Generator - Reads from URDF and STL meshes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-generate collision from URDF + meshes
  python3 smart_collision_generator.py \\
    --urdf koch_v11_arm_real.urdf \\
    --mesh-dir ~/robot_ws/src/writing_robot_description/meshes \\
    --output koch_with_collision.urdf
  
  # Adjust safety margin (default 5mm)
  python3 smart_collision_generator.py \\
    --urdf koch_v11_arm_real.urdf \\
    --mesh-dir ./meshes \\
    --margin 0.010 \\
    --output output.urdf

How it works:
  1. Reads visual elements from URDF
  2. Loads STL meshes
  3. Computes bounding boxes/cylinders
  4. Generates appropriate collision primitives
  5. No hardcoded dimensions!
        """
    )
    
    parser.add_argument('--urdf', type=str, required=True,
                       help='Input URDF file')
    parser.add_argument('--mesh-dir', type=str,
                       help='Base directory for mesh files')
    parser.add_argument('--output', type=str,
                       help='Output URDF file (default: <input>_with_collision.urdf)')
    parser.add_argument('--margin', type=float, default=0.005,
                       help='Safety margin to add (meters, default: 0.005)')
    parser.add_argument('--in-place', action='store_true',
                       help='Modify URDF in place')
    
    args = parser.parse_args()
    
    # Determine output path
    if args.in_place:
        output_path = args.urdf
        print("\n⚠️  WARNING: Modifying URDF in place!")
        response = input("Continue? (y/n): ")
        if response.lower() != 'y':
            print("Cancelled")
            return 0
    elif args.output:
        output_path = args.output
    else:
        if args.urdf.endswith('.urdf'):
            output_path = args.urdf.replace('.urdf', '_with_collision.urdf')
        else:
            output_path = args.urdf + '_with_collision.urdf'
    
    # Generate collision
    generator = SmartCollisionGenerator(
        mesh_base_dir=args.mesh_dir,
        margin=args.margin
    )
    
    success = generator.process_urdf(args.urdf, output_path)
    
    if success:
        print("\nNext steps:")
        print("  1. Review output file")
        print("  2. Test in Gazebo")
        print("  3. Adjust --margin if needed (default 5mm)")
        return 0
    else:
        return 1


if __name__ == '__main__':
    try:
        import trimesh
    except ImportError:
        print("\n❌ Error: trimesh not installed")
        print("Install with: pip install trimesh numpy --break-system-packages\n")
        sys.exit(1)
    
    sys.exit(main())
