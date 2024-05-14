import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import threading
import math

class TelloNode(Node):
    def __init__(self):
        super().__init__('tello_node')
        self.drones_info = {
            'Alpha': {'ip': '192.168.10.97', 'command_port': 8889, 'topic': '/alpha/pose'},
            'Bravo': {'ip': '192.168.10.204', 'command_port': 8889, 'topic': '/bravo/pose'},
            'Charlie': {'ip': '192.168.10.95', 'command_port': 8889, 'topic': '/charlie/pose'}
        }
        self.current_poses = {}
        self.bridge = CvBridge()
        self.initialize_drones()
        self.initialize_subscribers()
        self.timer = self.create_timer(2.0, self.sinusoidal_motion)  # Adjust timing based on actual dynamics

    def initialize_drones(self):
        for name in self.drones_info:
            self.current_poses[name] = None  # Initialize pose storage
            self.create_subscription(PoseStamped, self.drones_info[name]['topic'], lambda msg, name=name: self.pose_callback(msg, name), 10)

    def pose_callback(self, msg, drone_name):
        self.current_poses[drone_name] = msg.pose  # Store the latest pose

    def sinusoidal_motion(self):
        amplitude = 50  # in cm
        frequency = 0.1  # in Hz
        for name in self.drones_info:
            if self.current_poses[name] is not None:
                desired_z = amplitude * math.sin(2 * math.pi * frequency * self.t) + self.initial_z[name]
                actual_z = self.current_poses[name].position.z
                error = desired_z - actual_z
                # Implementing a simple proportional controller for demonstration
                correction = int(error * 0.1)  # Proportional gain
                command = 'up ' + str(correction) if correction > 0 else 'down ' + str(-correction)
                self.send_command(name, command)
        self.t += 2.0  # Increment time by the period of the timer

    def send_command(self, name, command):
        # Send commands directly via socket
        sock = self.command_sockets[name]
        address = (self.drones_info[name]['ip'], self.drones_info[name]['command_port'])
        sock.sendto(command.encode(), address)
        self.get_logger().info(f'Sent command "{command}" to {name}')

def main(args=None):
    rclpy.init(args=args)
    tello_node = TelloNode()
    rclpy.spin(tello_node)
    tello_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

