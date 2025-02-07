# MultiRobotMappingDevLog
Author: Pushkar Dave \
This DevLog documents the progress of the project [MultiRobotMapping](https://github.com/rdlynx19/MultiRobotMapping). I will try to summarise the thought process behind each of my decisions, as I progress through the project.

Current Objectives:
Writing a package to autonomously fly the quadrotor using MoCap system
Exploring RTABMap topics, parameters and launch files for the Oak Camera

Next Objectives:
Generating a valid map with the quadrotor by manually exploring the environment


### Week 1: 1/6 - 1/10
- Finished setting up Docker container for ROS2 Humble + Intel Realsense SDK + RTAB Map package
    - Repository: https://github.com/rdlynx19/docker-ros2-librealsense
    - This was required for compatibility issues, since System76 runs ROS2 Jazzy, and all the packages had verified support upto Humble
- Tested a few RTAB Map examples, a detailed exploration should start next week
    - Update: As of 1/24, I am yet to get into the details of RTAB Map
    - Documentation for RTAB Map is scattered all around, since the package was initially developed for ROS1, all the tutorials have outdated instructions.
- Assembled and calibrated quadrotor using QGroundControl
    - The RC receiver which was selected earlier (FrSky X8R), was incompatible with the Taranis QX7 transmitter and receiver
    - I resolved this issue by flashing new firmware on both the transmitter and receiver using an SD Card. [Transmitter](https://www.youtube.com/watch?v=dJ4dSKjrLxk)
    [Receiver](https://www.youtube.com/watch?v=On-vrm4KWxY)
- Designed and 3D printed [Landing Gear](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/LandingGear.png) and [RealsenseCameraMount](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/RealsenseMount.png) using Onshape
    - Update: As of 1/24, the Realsense will be switched out for an Oak Camera
- Set up and debugged Docker container for ROS2 Humble + PX4 Autopilot
    - This was an attempt at running the PX4 ROS simulation, however more details revealed that this system builds without errors in Jazzy
    - Update: As of 1/24, I have built the px4 packages on Jazzy, but I am yet to test offboard control

### Week 2: 1/13 - 1/17
- Set up mount for Raspberry Pi 4B on quadrotor
    - This involved modifying the case which shipped with RPi 4B, to securely fit it under the quadrotor
    - Update: As of 1/24, I am trying to switch from RPi 4B to RPi 5. Initial setup looks promising.
- Tested manual flight mode on quadrotor
    - Did a couple of manual indoor flights on the quadrotor, balancing the COM will be a crucial factor, but the drone seems able to lift off its weight
- Designed [YBatterySplitter](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/YBatterySplitter.jpg) between RPi and quadrotor
    - This is designed to power the RPi 4B from the LiPo which powers the quadrotor. A 5V/3A UBEC is used to step down the voltage from 14.8V of the LiPo battery.
    - Update: As of 1/24, a new version of this will be assembled (soldered) to support the RPi 5 which uses a 5V/5A UBEC.
- Designed [TX-RXConnector](https://github.com/rdlynx19/MultiRobotMapping/blob/main/images/TELEM2Pi.jpg) between RPi and PX4
    - This allows serial communication between RPi 4B and PX4 to enable offboard control for autonomous flight.
- Setup serial communication between RPi and PX4
    - This was verified by arming the quadrotor with an offboard control node running on the RPi 4B.

### Week 3: 1/20 - 1/24
- Setup and debugged the OptiTrack system to track the quadrotor
    - With the help of Drew Curtis, and the package developed by Aditya Nair, I was able to send the Motion Capture systems' data to the System76 laptop
    - With minor modifications, I was also able to send the position data from the MoCap system to the RPi 4B
    - Currently, my plan is to write a ROS package in C++, which obtains the data from the MoCap, converts it into PX4 messages, and sends it as an odometry message to the quadrotor
    - Note: This was tested on RPi 4B, but I should easily be able to replicate this on RPi 5 this weekend
- Switched from RPi 4B to RPi 5 and Realsense to Oak D
    - While testing the mapping pipeline, I discovered the slow FPS and dependency issues arising due to the limited compute of RPi 4B and the Realsense libraries
    - With RTAB Map building correctly with Jazzy, I made the decision to shift to RPi 5 which supports Ubuntu 24.04 and ROS2 Jazzy
    - Additionally, the Oak camera has a simpler setup than the Realsense, and has worked smoothly in all of the current tests
    - The main requirement of RTAB Map is a stereo or RGBD camera setup, which is satisfied by the Oak camera
- Setup and tested remote image data transfer from RPi 5 to System 76
    - RTAB Map is an intensive process, so the plan is to run only the camera nodes on the RPi, and run the mapping nodes remotely on System76
    - Initial attempt was utilising the ROS2 DDS setup, and exporting the ROS_DOMAIN_ID to share the nodes and topics
    - This was unsuccessful, as `ros2 topic list` returned a list of valid topics but `ros2 node list` returned an empty list
    - Some [forums](https://robotics.stackexchange.com/questions/113412/ros2-humble-nodes-and-topics-dont-show-up) suggested switching the middleware to CycloneDDS which solved the node issue, but I still was not able to echo the topics on my remote system
    - After digging a little more, I understood this was caused because the default ROS2 DDS setup uses **udp** protocol, which had no acknowledgement system to confirm the data was received. A better choice would be to have the data transferred using **tcp**
    - This was confirmed and verified using this tcp tunneling package: https://github.com/norlab-ulaval/ros2_tcp_tunnel. However, this package only supports Humble and ROS2 does not support cross platform communication, which forced me to use a docker container again.
    - Other packages seemed to work well with Jazzy, so I figured out a new middleware which was supported in ROS2 Jazzy, which is [zenoh](https://github.com/norlab-ulaval/ros2_tcp_tunnel)
    - I tested this by launching the Oak camera node on RPi 5, and visualising it in RViz in System76

### Week 4: 1/27 - 1/31
- Wrote custom node for offboard control based on the offboard example. Added addtional service calls to arm and disarm the drone
- Solved the lag when using RPi with SSH while being powered by the LiPo battery.
  - The RPi 5 communicates with its power source to determine if its capable of supplying 5V/5A. If it is convinced the source can supply 5V/5A, it boots into standard mode, otherwise it will boot into low power mode. This makes it super slow.
  - The solution to this problem was found on this [page](https://pichondria.com/usb-pd-2-0-3-0-to-5v-5a-converter-for-raspberrypi-5-tutorial/), which suggested to edit the `rpi-eeprom-config` and tell the firmware to skip power delivery negotiation.
- Tested autonomous takeoff and trajectory setpoint
    - Before solving the RPi power issue, with multiple reboots, I was able to access one session when the RPi booted into standard mode
    - I conducted two runs each involving: arming the drone, takeoff to a height of around 1m, and land by manually switching mode on RC transmitter
    - In the first run, the drone armed fine, took off to a perfect estimate of around 1m, and when I switched mode on RC stick, softly landed on its own.
    - In the second run, the drone armed fine, struggled to take off and drifted away from its original position, took off, tried to adjust to reach the setpoint position, but due to safety concerns I had to manually trigger the kill switch, which caused it to crash immediately.
    - No critical damage was observed, except broken landing gear.
- Update: As of 1/31, I have observed QGC throwing the error: `Crash Dumps present on SD Card`. After digging deep into this, I found out the crash caused a hard fault in the Pixhawk and will need a manually debugging using a physical adapter and an in-circuit debugger.

### Week 5: 2/3 - 2/7
- Solved rtabmap issue, which caused `/rgbd_image` topic to not show up on System76
    - rtabmap needs camera data at atleast 10Hz to start the odometry and mapping nodes, this is not possible if you plan to transport the entire image data over the network
    - The solution to this problem is to transport the compressed image topics over the network and then possibly decompress them on the laptop
    - Another useful improvement is to use a topic relay, which eliminates multiple subscription copies, and creates a single subscription for all the nodes
    - And finally, switching to a different router with less traffic, greatly improved the data transmission
    - After implementing all these fixes, I was able to successfully receive camera data from the RPi to the System 76
- Tested out example runs of handheld, Go1 and Go2 mounted mapping using manual control modes
- Experimented with map merging, by using rtabmap's default processing two merge two recorded databases
- Setup prelimnary RViz to visualise octo and binary occupancy grid
