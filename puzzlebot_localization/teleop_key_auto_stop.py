#!/usr/bin/env python3
"""Teleoperación simple con auto-stop si no hay teclas nuevas."""

import sys
import termios
import tty
import select
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopAutoStop(Node):
    def __init__(self):
        super().__init__('teleop_auto_stop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.cmd = (0.0, 0.0)
        self.stop_timeout = 0.3  # segundos sin tecla -> stop
        self.last_key_time = time.time()
        self.running = True

        self.get_logger().info('Teleop iniciado. Controles:')
        self.get_logger().info('  i=avanzar  j=girar izq  l=girar der  ,=retroceder  k=stop')
        self.get_logger().info('  q/z=vel +/-  w/x=linear +/-  e/c=angular +/-  Ctrl-C=salir')

        # Thread para leer teclado
        self.input_thread = threading.Thread(target=self.read_keys)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Timer para publicar cmd_vel
        self.timer = self.create_timer(0.1, self.update)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def read_keys(self):
        while self.running:
            key = self.get_key()
            if key:
                self.last_key_time = time.time()
                self.handle_key(key)

    def handle_key(self, key):
        if key == 'q':
            self.speed *= 1.1
        elif key == 'z':
            self.speed *= 0.9
        elif key == 'w':
            self.speed *= 1.1
        elif key == 'x':
            self.speed *= 0.9
        elif key == 'e':
            self.turn *= 1.1
        elif key == 'c':
            self.turn *= 0.9
        elif key in ('i', 'I'):
            self.cmd = (self.speed, 0.0)
        elif key in (',', '<'):
            self.cmd = (-self.speed, 0.0)
        elif key in ('j', 'J'):
            self.cmd = (0.0, self.turn)
        elif key in ('l', 'L'):
            self.cmd = (0.0, -self.turn)
        elif key in ('u', 'U'):
            self.cmd = (self.speed, self.turn)
        elif key in ('o', 'O'):
            self.cmd = (self.speed, -self.turn)
        elif key in ('m', 'M'):
            self.cmd = (-self.speed, self.turn)
        elif key in ('.', '>'):
            self.cmd = (-self.speed, -self.turn)
        elif key == 'k':
            self.cmd = (0.0, 0.0)
        else:
            return
        self.get_logger().info(f'Key: {key} | speed={self.speed:.2f} turn={self.turn:.2f}')

    def update(self):
        if time.time() - self.last_key_time > self.stop_timeout:
            self.cmd = (0.0, 0.0)
        msg = Twist()
        msg.linear.x = float(self.cmd[0])
        msg.angular.z = float(self.cmd[1])
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.input_thread.join(timeout=0.5)
        super().destroy_node()


def main():
    rclpy.init()
    node = TeleopAutoStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
