from setuptools import find_packages, setup

package_name = 'writing_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
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
            'draw_square_sim = writing_robot_control.draw_square_sim:main',
            'koch_v11_draw_square = writing_robot_control.koch_v11_draw_square:main',
            'pose_sequence = writing_robot_control.pose_sequence:main',
            'pose_test = writing_robot_control.pose_test:main',
            'load_tester = writing_robot_control.load_tester:main',
            'power_monitor_node = writing_robot_control.power_monitor_node:main',
            'power_logger = writing_robot_control.power_logger:main',
            'imu_balance_node = writing_robot_control.imu_balance_node:main',
            'balance_test_injector = writing_robot_control.balance_test_injector:main',
            'wrist_balance_controller = writing_robot_control.wrist_balance_controller:main',
            'joint_nudge = writing_robot_control.joint_nudge:main',
            'ball_balance_node = writing_robot_control.ball_balance_node:main',
            'system_watchdog = writing_robot_control.system_watchdog:main',
       ],
    },
)
