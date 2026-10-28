from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config', 'balance'), glob('config/balance/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='bueche64@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
            'draw_square_sim = robot_arm_control.draw_square_sim:main',
            'koch_v11_draw_square = robot_arm_control.koch_v11_draw_square:main',
            'pose_sequence = robot_arm_control.pose_sequence:main',
            'pose_test = robot_arm_control.pose_test:main',
            'load_tester = robot_arm_control.load_tester:main',
            'power_monitor_node = robot_arm_control.power_monitor_node:main',
            'power_logger = robot_arm_control.power_logger:main',
            'imu_balance_node = robot_arm_control.imu_balance_node:main',
            'balance_test_injector = robot_arm_control.balance_test_injector:main',
            'wrist_balance_controller = robot_arm_control.wrist_balance_controller:main',
            'joint_nudge = robot_arm_control.joint_nudge:main',
            'ball_balance_node = robot_arm_control.ball_balance_node:main',
            'system_watchdog = robot_arm_control.system_watchdog:main',
            'pid_logger = robot_arm_control.pid_logger:main',
       ],
    },
)
