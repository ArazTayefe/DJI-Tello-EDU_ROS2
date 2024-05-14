import rclpy
from rclpy.node import Node
import socket
import threading
import math

class TelloNode(Node):
    def __init__(self):
        super().__init__('tello_node')
        self.drones_info = {
            'Alpha': {'ip': '192.168.10.97', 'command_port': 8889},
            'Bravo': {'ip': '192.168.10.204', 'command_port': 8889},
            'Charlie': {'ip': '192.168.10.95', 'command_port': 8889}
        }
        self.command_sockets = {}
        self.initialize_drones()
        self.timer = self.create_timer(2.0, self.sinusoidal_motion)  # Trigger every 2 seconds
        self.t = 0

    def initialize_drones(self):
        for name, settings in self.drones_info.items():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('', 0))  # Bind to any free local port
            self.command_sockets[name] = sock
            self.send_command(name, 'command')
            self.send_command(name, 'takeoff')

    def send_command(self, name, command):
        sock = self.command_sockets[name]
        address = (self.drones_info[name]['ip'], self.drones_info[name]['command_port'])
        # Send commands in a separate thread to improve responsiveness
        thread = threading.Thread(target=self._send_command, args=(sock, command, address))
        thread.start()

    def _send_command(self, sock, command, address):
        sock.sendto(command.encode(), address)
        self.get_logger().info(f'Sent command "{command}" to {address}')

    def sinusoidal_motion(self):
        amplitude = 50  # in cm
        frequency = 0.1  # in Hz
        z = amplitude * math.sin(2 * math.pi * frequency * self.t)
        command = f'up {abs(int(z))}' if z >= 0 else f'down {abs(int(z))}'
        for name in self.drones_info:
            self.send_command(name, command)
        self.t += 2.0  # Increment time by the period of the timer

    def on_shutdown(self):
        for name in self.drones_info:
            self.send_command(name, 'land')
            self.command_sockets[name].close()
            self.get_logger().info(f'Closed socket and sent land command to drone {name}')

def main(args=None):
    rclpy.init(args=args)
    tello_node = TelloNode()
    rclpy.spin(tello_node)
    tello_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
