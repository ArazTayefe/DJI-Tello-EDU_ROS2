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

### wifi_setup.py
```python
import socket
import time

# IP and port of the Tello drone's command interface
tello_address = ('192.168.10.1', 8889)

# Create a UDP connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind to an address and port to receive messages from the drone
sock.bind(('', 9000))  # Ensure this port is available and not blocked by your firewall

# Command to switch the Tello to station mode: command to connect to your Wi-Fi
command = 'command'
wifi_command = f'ap your_SSID your_password.'
# Send commands
sock.sendto(command.encode(), tello_address)
time.sleep(1)  # Wait for the drone to be ready to receive next command

response, _ = sock.recvfrom(1024)  # Waiting for response from drone
print(f'Received: {response.decode()}')

sock.sendto(wifi_command.encode(), tello_address)
time.sleep(1)  # Give the drone some time to process the command

response, _ = sock.recvfrom(1024)  # Waiting for response from drone
print(f'Received: {response.decode()}')

sock.close()  # Close the socket when done
```
This code is tested on Ubuntu 22 with ROS2 Humble installed on it.
