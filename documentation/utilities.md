# Robot Arm Pose Driving Utilities and Analysis
In this section we outline some of the software utilities used to cause the robot to move through and analyze a set of poses. 

## Sections
- [Tools and Utilities](#tools-and-utilities): This section outlines the utilities and tools built in the repository to support research and testing. This points to more detailed execution examples for each tool.
- [Analysis](#analysis-notes): In this section outlines and points to more detailed analysis of the robot using the tools noted above.
- [Next Steps](#next-steps): Some possible next steps for building utilities and analysis.

## Tools and Utilities
- [Running the pose_test utility](./pose_test.md): The pose test utility will iterate through a series of predefined input poses and take measurements of the physical robot during this time.
Physical characteristics like input current, voltage, load, and temperature are measured as well as movement limits (as defined by the URDF file (see [discussion
movment boundaries](./firmware.md#dynamixel-servos))
- [Running the servo_monitor tool](./servo_monitor.md): The servo monitor tool holds the robot in a pose and measures the temp, current, and voltage as this pose is held.
- [Running the pose load_test analysis tool](./load_test.md): Making projections on limits of the robot joints based on pose transitions.

## Analysis Notes
- [Power monitoring using INA219, INA226, and servo-internal sensors](./power_monitoring.md): The utilities include a ros2 node that taps into INA219 sensors, INA226 sensors, and Dynamixel servo-internal sensors that have been added to measure voltage and current draw on the 5v and 12v set of servos. This provides insights into the servo dynamics and the corresponding power needs.

  
<p align="center">
 
  <img src="../images/story_board_for_example_pose_test.jpg" alt="story board for example pose test" width="900">

</p>

## Next Steps
- TBD
