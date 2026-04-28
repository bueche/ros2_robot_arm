from setuptools import find_packages, setup
import os

package_name = 'balance'

# Collect all files in models/ directory
models_files = []
models_dir = os.path.join('models')
if os.path.isdir(models_dir):
    for f in os.listdir(models_dir):
        models_files.append(os.path.join(models_dir, f))

# Collect launch files if launch/ directory exists
launch_files = []
launch_dir = 'launch'
if os.path.isdir(launch_dir):
    for f in os.listdir(launch_dir):
        if f.endswith('.py') or f.endswith('.yaml'):
            launch_files.append(os.path.join(launch_dir, f))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install model files so nodes can find them via share path
        ('share/' + package_name + '/models', models_files),
        # Install launch files if they exist
        *([('share/' + package_name + '/launch', launch_files)]
          if launch_files else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='bueche64@gmail.com',
    description='Ball balancing vision and control nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # USB camera detector (Jazzy / Pi 5)
            'ball_detector_node = balance.ball_detector_node:main',
            # OAK-D Lite detector (Humble / Orin Nano or Pi 5)
            # Safe to install on both — depthai import is guarded
            'ball_detector_oak = balance.ball_detector_oak:main',
            # nvidia inference approach (needs special hybrid container)
            'ball_detector_nvidia = balance.ball_detector_nvidia:main',
            # Marker node (any platform)
            'ball_marker_node = balance.ball_marker_node:main',
        ],
    },
)
