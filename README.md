# DJI Tello EDU with ROS2

This repository contains ROS2 nodes developed to control DJI Tello drones.

## Project Overview

This project involves controlling DJI Tello drones using ROS2. The nodes include functionalities for communication and control.

## Setup Drone

### 1. Update the Drone Firmware

1. Download the Tello App using the following link: [Tello App Download](https://www.dji.com/ca/downloads/djiapp/tello).
2. Ignore the "connect tello" button by pressing exit on the top right side of the screen and download the latest firmware.
3. Turn off the cellular data, turn on the drone, and connect the phone Wi-Fi to the drone's SSID.
4. Open the Tello app and press "connect tello."
5. Go to the `Setting > More > ... > Firmware Version - Update` to update the firmware.

### 2. Wi-Fi Setting

1. Restart the drone and use the `wifi_setup.py` code to set the SSID and Password of your modem (or Wi-Fi hotspot). You should change the `your_SSID` and `your_password` in the code.
2. Get the drone IP using `ip neighbor` in Linux terminal.

### 3. Installing and Using mocap4r2

mocap4r2 is a ROS2 package designed to integrate motion capture systems with ROS2. It provides tools and nodes to interface with OptiTrack systems, allowing for real-time tracking and data integration in ROS2 applications. This project uses mocap4r2 to integrate OptiTrack Flex 13 cameras.

For more information, visit the [mocap4r2 GitHub repository](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack).

1. Ensure your motion capture cameras are set up and calibrated correctly.
2. Make sure that configuration file is correct. The server_address is the IP of the PC that runs the motion capture software and local_addresses is the IP addresses of the PC that runs the mocap4r2.
3. Launch the mocap4r2 nodes to start streaming motion capture data to ROS2:
```
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

### 4. Run the ROS2 Node
3.1 Clone the repository:
```
git clone git@github.com:NickTayefe/DJI-Tello-EDU.git
cd DJI-Tello-EDU
```
3.2 Change your directory to your ROS2 workspace and launch the driver:
```
cd ~/'your_ros2_workspace'
colcon build --packages-select tello_driver
source install/setup.bash
ros2 launch tello_driver tello_launch.py

```

### Issues and Contributions

If you encounter any issues, please open an issue on GitHub. Contributions are welcome!
This code is tested on Ubuntu 22 with ROS2 Humble installed on it.

### License

This project is licensed under the MIT License.

