# MultiRobotMappingDevLog
Author: Pushkar Dave
This DevLog documents the progress of the project [MultiRobotMapping](https://github.com/rdlynx19/MultiRobotMapping).

### Week 1: 1/6 - 1/10
- Finished setting up Docker container for ROS2 Humble + Intel Realsense SDK + RTAB Map package
- Tested a few RTAB Map examples, a detailed exploration should start next week
- Assembled and calibrated quadrotor using QGroundControl
- Designed and 3D printed [Landing Gear](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/LandingGear.png) and [RealsenseCameraMount](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/RealsenseMount.png) using Onshape
- Set up and debugged Docker container for ROS2 Humble + PX4 Autopilot

### Week 2: 1/13 - 1/17
- Set up mount for Raspberry Pi on quadrotor
- Tested manual flight mode on quadrotor
- Designed [YBatterySplitter](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/YBatterySplitter.jpg) between RPi and quadrotor
- Designed [TX-RXConnector](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/TELEM2Pi.jpg) between RPi and PX4
- Setup serial communication between RPi and PX4
