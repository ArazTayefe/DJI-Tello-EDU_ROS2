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
2. Get the drone IP using `ip neighbor`.

### wifi_setup.py
```python
import socket

def send_wifi_setup_command(ssid, password):
    command = f'command;wifi ssid {ssid} password {password}'
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.sendto(command.encode(), ('192.168.10.1', 8889))
    udp_socket.close()

if __name__ == "__main__":
    your_SSID = "your_SSID"
    your_password = "your_password"
    send_wifi_setup_command(your_SSID, your_password)
