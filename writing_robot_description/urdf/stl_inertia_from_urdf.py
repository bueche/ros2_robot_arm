#!/usr/bin/env python3
"""
STL-Based Inertial Property Calculator - URDF Direct Parser

Reads mesh paths, transforms, and scales directly from your URDF.
No separate config file needed! Single source of truth.

Requirements:
  pip install trimesh numpy scipy --break-system-packages

Usage:
  # Process entire URDF
  python3 stl_inertia_from_urdf.py --urdf koch_v11_arm_real.urdf --output inertials.xml
  
  # Single link
  python3 stl_inertia_from_urdf.py --urdf koch_v11_arm_real.urdf --link upper_arm_link
  
  # Specify mesh directory if not using ROS package paths
  python3 stl_inertia_from_urdf.py --urdf koch_v11_arm_real.urdf --mesh-dir ./meshes
"""

import trimesh
import numpy as np
import xml.etree.ElementTree as ET
import argparse
import sys
import os
from pathlib import Path
import re


class URDFInertiaCalculator:
    """Calculate inertial properties from URDF and STL meshes."""
    
    # Material densities (kg/m³)
    MATERIALS = {
        'pla': 1200,
        'pla+': 1240,
        'pla_plus': 1240,
        'abs': 1050,
        'petg': 1270,
        'tpu': 1210,
        'nylon': 1140,
        'aluminum': 2700,
        'steel': 7850
    }
    
    # Dynamixel servo specs (from datasheets)
    SERVOS = {
        'XL430-W250': {'mass': 0.059, 'radius': 0.014, 'length': 0.046},
        'XL330-M288': {'mass': 0.029, 'radius': 0.012, 'length': 0.028},
        'XL-430': {'mass': 0.059, 'radius': 0.014, 'length': 0.046},  # Alternate names
        'XL-330': {'mass': 0.029, 'radius': 0.012, 'length': 0.028},
        'XL330': {'mass': 0.029, 'radius': 0.012, 'length': 0.028},
        'XL430': {'mass': 0.059, 'radius': 0.014, 'length': 0.046}
    }
    
    def __init__(self, default_material='pla+', mesh_base_dir=None):
        self.default_material = default_material
        self.mesh_base_dir = mesh_base_dir
        self.urdf_dir = None
    
    def parse_urdf(self, urdf_path):
        """
        Parse URDF and extract mesh information for each link.
        
        Returns:
            dict: {link_name: {'meshes': [...], 'servos': [...]}}
        """
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # Store URDF directory for resolving relative paths
            self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            return None
        
        links_data = {}
        
        # Process each link
        for link in root.findall('.//link'):
            link_name = link.get('name')
            link_data = {'meshes': [], 'servos': []}
            
            # Extract visual meshes (not servo meshes)
            for visual in link.findall('visual'):
                visual_name = visual.get('name', '')
                
                # Skip servo visual meshes
                if self._is_servo_mesh(visual_name):
                    # Extract servo info instead
                    servo_info = self._extract_servo_info(visual)
                    if servo_info:
                        link_data['servos'].append(servo_info)
                    continue
                
                # Extract mesh info
                mesh_info = self._extract_mesh_info(visual)
                if mesh_info:
                    link_data['meshes'].append(mesh_info)
            
            links_data[link_name] = link_data
        
        return links_data
    
    def _is_servo_mesh(self, visual_name):
        """Check if visual element is a servo (not a structural part)."""
        servo_patterns = [
            'xl-430', 'xl430', 'xl-330', 'xl330',
            'dynamixel', 'servo', 'motor',
            'XL-430', 'XL430', 'XL-330', 'XL330'
        ]
        
        visual_lower = visual_name.lower()
        return any(pattern.lower() in visual_lower for pattern in servo_patterns)
    
    def _extract_mesh_info(self, visual_element):
        """Extract mesh file path, origin, and scale from visual element."""
        geometry = visual_element.find('geometry')
        if geometry is None:
            return None
        
        mesh_elem = geometry.find('mesh')
        if mesh_elem is None:
            return None
        
        # Get mesh filename
        filename = mesh_elem.get('filename', '')
        if not filename:
            return None
        
        # Parse scale
        scale_str = mesh_elem.get('scale', '1 1 1')
        scale = [float(x) for x in scale_str.split()]
        scale_factor = scale[0] if scale else 1.0  # Use first value
        
        # Parse origin (xyz and rpy)
        origin = visual_element.find('origin')
        if origin is not None:
            xyz_str = origin.get('xyz', '0 0 0')
            xyz = [float(x) for x in xyz_str.split()]
        else:
            xyz = [0, 0, 0]
        
        return {
            'filename': filename,
            'scale': scale_factor,
            'origin': xyz
        }
    
    def _extract_servo_info(self, visual_element):
        """Extract servo type and position from visual element."""
        visual_name = visual_element.get('name', '')
        
        # Determine servo type from name
        servo_type = None
        if 'xl-430' in visual_name.lower() or 'xl430' in visual_name.lower():
            servo_type = 'XL430-W250'
        elif 'xl-330' in visual_name.lower() or 'xl330' in visual_name.lower():
            servo_type = 'XL330-M288'
        
        if not servo_type:
            return None
        
        # Get position
        origin = visual_element.find('origin')
        if origin is not None:
            xyz_str = origin.get('xyz', '0 0 0')
            position = [float(x) for x in xyz_str.split()]
        else:
            position = [0, 0, 0]
        
        return {
            'type': servo_type,
            'position': position
        }
    
    def resolve_mesh_path(self, filename):
        """
        Resolve mesh path from package:// or relative path.
        
        Args:
            filename: Path from URDF (e.g., "package://pkg/meshes/file.stl")
        
        Returns:
            Absolute path to mesh file
        """
        # Handle package:// paths
        if filename.startswith('package://'):
            # Extract package name and relative path
            # package://writing_robot_description/meshes/file.stl
            parts = filename.replace('package://', '').split('/', 1)
            
            if len(parts) == 2:
                package_name, rel_path = parts
                
                # If mesh_base_dir specified, use it
                if self.mesh_base_dir:
                    return os.path.join(self.mesh_base_dir, os.path.basename(rel_path))
                
                # Try to find in ROS workspace
                # Common pattern: ~/robot_ws/src/package_name/...
                if 'COLCON_PREFIX_PATH' in os.environ:
                    ws_paths = os.environ['COLCON_PREFIX_PATH'].split(':')
                    for ws_path in ws_paths:
                        # Go up from install/lib/... to src/
                        ws_root = os.path.dirname(os.path.dirname(ws_path))
                        mesh_path = os.path.join(ws_root, 'src', package_name, rel_path)
                        if os.path.exists(mesh_path):
                            return mesh_path
                
                # Fallback: look relative to URDF directory
                if self.urdf_dir:
                    # Go up to package root (assume URDF in urdf/ subdirectory)
                    package_root = os.path.dirname(self.urdf_dir)
                    mesh_path = os.path.join(package_root, rel_path)
                    if os.path.exists(mesh_path):
                        return mesh_path
                
                # Last resort: just use the basename
                return os.path.basename(rel_path)
        
        # Handle absolute paths
        elif os.path.isabs(filename):
            return filename
        
        # Handle relative paths
        else:
            if self.mesh_base_dir:
                return os.path.join(self.mesh_base_dir, filename)
            elif self.urdf_dir:
                return os.path.join(self.urdf_dir, filename)
            else:
                return filename
    
    def load_mesh(self, mesh_path, scale=1.0):
        """Load and optionally scale STL mesh."""
        try:
            mesh = trimesh.load(mesh_path)
            
            if scale != 1.0:
                mesh.apply_scale(scale)
            
            if not mesh.is_watertight:
                print(f"    ⚠️  Mesh not watertight, attempting fix...")
                mesh.fill_holes()
                if mesh.is_watertight:
                    print(f"    ✓ Fixed")
            
            return mesh
        except Exception as e:
            print(f"    ❌ Error loading mesh: {e}")
            return None
    
    def calculate_mesh_properties(self, mesh, density):
        """Calculate mass, COM, and inertia from mesh."""
        volume = mesh.volume
        mass = volume * density
        com = mesh.center_mass
        
        # Get inertia tensor
        I = mesh.moment_inertia
        
        # Scale by mass/volume ratio
        Ixx = I[0, 0] * mass / volume if volume > 0 else 0
        Iyy = I[1, 1] * mass / volume if volume > 0 else 0
        Izz = I[2, 2] * mass / volume if volume > 0 else 0
        
        return {
            'mass': mass,
            'com': com.tolist(),
            'volume': volume,
            'inertia': {
                'Ixx': Ixx, 'Iyy': Iyy, 'Izz': Izz,
                'Ixy': 0.0, 'Ixz': 0.0, 'Iyz': 0.0
            }
        }
    
    def servo_properties(self, servo_type):
        """Get servo inertial properties."""
        if servo_type not in self.SERVOS:
            print(f"    ⚠️  Unknown servo type: {servo_type}")
            return None
        
        servo = self.SERVOS[servo_type]
        mass = servo['mass']
        r = servo['radius']
        L = servo['length']
        
        # Cylinder inertia
        Ixx = Iyy = (1/12) * mass * (3*r**2 + L**2)
        Izz = 0.5 * mass * r**2
        
        return {
            'mass': mass,
            'inertia': {
                'Ixx': Ixx, 'Iyy': Iyy, 'Izz': Izz,
                'Ixy': 0.0, 'Ixz': 0.0, 'Iyz': 0.0
            }
        }
    
    def parallel_axis_theorem(self, I_com, mass, offset):
        """Apply parallel axis theorem to shift inertia tensor."""
        dx, dy, dz = offset
        
        Ixx = I_com['Ixx'] + mass * (dy**2 + dz**2)
        Iyy = I_com['Iyy'] + mass * (dx**2 + dz**2)
        Izz = I_com['Izz'] + mass * (dx**2 + dy**2)
        
        return {'Ixx': Ixx, 'Iyy': Iyy, 'Izz': Izz,
                'Ixy': 0.0, 'Ixz': 0.0, 'Iyz': 0.0}
    
    def combine_components(self, components):
        """Combine multiple components into single inertial properties."""
        if not components:
            return None
        
        total_mass = sum(c['mass'] for c in components)
        
        if total_mass == 0:
            return None
        
        # Combined COM
        com = [
            sum(c['mass'] * c['com'][i] for c in components) / total_mass
            for i in range(3)
        ]
        
        # Combined inertia
        Ixx = Iyy = Izz = 0
        
        for c in components:
            dx = c['com'][0] - com[0]
            dy = c['com'][1] - com[1]
            dz = c['com'][2] - com[2]
            
            I_shifted = self.parallel_axis_theorem(c['inertia'], c['mass'], [dx, dy, dz])
            
            Ixx += I_shifted['Ixx']
            Iyy += I_shifted['Iyy']
            Izz += I_shifted['Izz']
        
        return {
            'mass': total_mass,
            'com': com,
            'inertia': {
                'Ixx': Ixx, 'Iyy': Iyy, 'Izz': Izz,
                'Ixy': 0.0, 'Ixz': 0.0, 'Iyz': 0.0
            }
        }
    
    def process_link(self, link_name, link_data):
        """Process a single link from URDF data."""
        print(f"\n{'='*60}")
        print(f"Processing: {link_name}")
        print(f"{'='*60}")
        
        components = []
        
        # Process meshes
        for mesh_info in link_data['meshes']:
            filename = mesh_info['filename']
            mesh_path = self.resolve_mesh_path(filename)
            
            print(f"\n  Mesh: {os.path.basename(filename)}")
            print(f"    Path: {mesh_path}")
            
            if not os.path.exists(mesh_path):
                print(f"    ❌ File not found!")
                continue
            
            # Load mesh
            mesh = self.load_mesh(mesh_path, mesh_info['scale'])
            if mesh is None:
                continue
            
            # Calculate properties
            density = self.MATERIALS[self.default_material]
            props = self.calculate_mesh_properties(mesh, density)
            
            # Apply origin offset
            props['com'] = [
                props['com'][0] + mesh_info['origin'][0],
                props['com'][1] + mesh_info['origin'][1],
                props['com'][2] + mesh_info['origin'][2]
            ]
            
            components.append(props)
            
            print(f"    ✓ Mass: {props['mass']*1000:.1f}g")
            print(f"    ✓ Volume: {props['volume']*1e6:.2f}cm³")
            print(f"    ✓ COM: [{props['com'][0]*1000:.1f}, {props['com'][1]*1000:.1f}, {props['com'][2]*1000:.1f}]mm")
        
        # Process servos
        for servo_info in link_data['servos']:
            servo_type = servo_info['type']
            position = servo_info['position']
            
            print(f"\n  Servo: {servo_type}")
            print(f"    Position: {position}")
            
            props = self.servo_properties(servo_type)
            if props is None:
                continue
            
            props['com'] = position
            components.append(props)
            
            print(f"    ✓ Mass: {props['mass']*1000:.1f}g")
        
        # Combine
        if not components:
            print(f"\n  ❌ No valid components found!")
            return None
        
        combined = self.combine_components(components)
        
        print(f"\n{'='*60}")
        print(f"COMBINED PROPERTIES")
        print(f"{'='*60}")
        print(f"  Mass: {combined['mass']*1000:.1f}g")
        print(f"  COM: [{combined['com'][0]*1000:.1f}, {combined['com'][1]*1000:.1f}, {combined['com'][2]*1000:.1f}]mm")
        print(f"  Ixx: {combined['inertia']['Ixx']:.9f} kg⋅m²")
        print(f"  Iyy: {combined['inertia']['Iyy']:.9f} kg⋅m²")
        print(f"  Izz: {combined['inertia']['Izz']:.9f} kg⋅m²")
        
        return combined
    
    def generate_urdf_snippet(self, link_name, properties):
        """Generate URDF inertial XML."""
        if properties is None:
            return ""
        
        mass = properties['mass']
        com = properties['com']
        I = properties['inertia']
        
        return f'''<!-- {link_name} inertial properties (from STL + servos) -->
<inertial>
  <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>
  <mass value="{mass:.6f}"/>
  <inertia ixx="{I['Ixx']:.9f}" ixy="{I['Ixy']:.9f}" ixz="{I['Ixz']:.9f}"
           iyy="{I['Iyy']:.9f}" iyz="{I['Iyz']:.9f}"
           izz="{I['Izz']:.9f}"/>
</inertial>
'''


def main():
    parser = argparse.ArgumentParser(
        description='Calculate inertial properties from URDF and STL meshes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process entire URDF
  python3 stl_inertia_from_urdf.py --urdf koch_v11_arm_real.urdf --output inertials.xml
  
  # Single link only
  python3 stl_inertia_from_urdf.py --urdf koch_v11_arm_real.urdf --link upper_arm_link
  
  # Specify mesh directory
  python3 stl_inertia_from_urdf.py --urdf koch_v11_arm_real.urdf --mesh-dir ~/robot_ws/src/writing_robot_description/meshes
        """
    )
    
    parser.add_argument('--urdf', type=str, required=True,
                       help='Path to URDF file')
    parser.add_argument('--link', type=str,
                       help='Process only this link')
    parser.add_argument('--mesh-dir', type=str,
                       help='Base directory for mesh files (overrides package paths)')
    parser.add_argument('--material', type=str, default='pla+',
                       help='Material for 3D printed parts (default: pla+)')
    parser.add_argument('--output', type=str,
                       help='Output file for URDF snippets')
    
    args = parser.parse_args()
    
    # Initialize calculator
    calc = URDFInertiaCalculator(
        default_material=args.material,
        mesh_base_dir=args.mesh_dir
    )
    
    # Parse URDF
    print(f"\nParsing URDF: {args.urdf}")
    links_data = calc.parse_urdf(args.urdf)
    
    if not links_data:
        print("Failed to parse URDF!")
        return 1
    
    print(f"Found {len(links_data)} links")
    
    # Process links
    results = {}
    
    for link_name, link_data in links_data.items():
        if args.link and link_name != args.link:
            continue
        
        # Skip if no meshes or servos
        if not link_data['meshes'] and not link_data['servos']:
            print(f"\nSkipping {link_name} (no meshes or servos)")
            continue
        
        properties = calc.process_link(link_name, link_data)
        if properties:
            results[link_name] = properties
    
    # Output results
    if args.output and results:
        with open(args.output, 'w') as f:
            f.write('<!-- Generated inertial properties from STL meshes -->\n')
            f.write('<!-- Material: {} (density: {} kg/m³) -->\n\n'.format(
                args.material, calc.MATERIALS[args.material]
            ))
            
            for link_name, props in results.items():
                f.write(calc.generate_urdf_snippet(link_name, props))
                f.write('\n')
        
        print(f"\n{'='*60}")
        print(f"✓ Saved to: {args.output}")
        print(f"{'='*60}\n")
    
    # Print summary
    if results:
        print(f"\n{'='*60}")
        print("SUMMARY")
        print(f"{'='*60}\n")
        
        total_mass = sum(r['mass'] for r in results.values())
        
        for link_name, props in results.items():
            print(f"{link_name}:")
            print(f"  {props['mass']*1000:.1f}g")
        
        print(f"\nTotal robot mass: {total_mass*1000:.0f}g ({total_mass:.3f}kg)")
    
    return 0


if __name__ == '__main__':
    try:
        import trimesh
    except ImportError:
        print("\n❌ Error: trimesh not installed")
        print("Install with: pip install trimesh numpy scipy --break-system-packages\n")
        sys.exit(1)
    
    sys.exit(main())
