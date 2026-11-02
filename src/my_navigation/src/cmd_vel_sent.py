#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

import socket
import struct

# ============ XOR encrypt/decrypt function (example) ============
def simple_xor_encrypt(data, key=0x42):
    """Simple XOR on byte array data; encryption and decryption work the same way"""
    return bytes([b ^ key for b in data])


class CmdVelSender(object):
    def __init__(self):
        # Read some params (optional), such as IP and port; they can also be hard-coded.
        self.roborio_ip = rospy.get_param('~roborio_ip', '10.1.14.2')
        self.roborio_port = rospy.get_param('~roborio_port', 7777)
        self.key = rospy.get_param('~xor_key', 0x42)

        # Initialize the TCP connection
        self.socket = None
        self.connect_to_roborio()

        # Subscribe to /cmd_vel
        self.sub_cmd_vel = rospy.Subscriber(
            '/cmd_vel',  # The topic name you need
            Twist,
            self.cmd_vel_callback
        )
        rospy.loginfo("CmdVelSender inited, subscribing to /cmd_vel")

    def connect_to_roborio(self):
        """Try to connect to the RoboRIO at the given IP:Port."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.roborio_ip, self.roborio_port))
            rospy.loginfo("Connected to RoboRIO: {}:{}".format(
                self.roborio_ip, self.roborio_port
            ))
        except Exception as e:
            rospy.logerr("Failed to connect RoboRIO: {}".format(e))
            self.socket = None

    def cmd_vel_callback(self, msg):
        """
        Subscriber callback: runs when a new /cmd_vel arrives.
        We only demo sending these three floats (msg.linear.x, msg.linear.y, msg.angular.z).
        More fields can be sent if needed.
        """
        if not self.socket:
            # If not connected yet or the connection dropped, reconnect logic can go here
            return

        # Pack into 3 floats = 12 bytes
        data_bytes = struct.pack('fff', msg.linear.x, msg.linear.y, msg.angular.z)

        # Simple XOR encryption
        encrypted = simple_xor_encrypt(data_bytes, self.key)

        # Send
        try:
            self.socket.sendall(encrypted)
        except Exception as e:
            rospy.logerr("Send failed: {}".format(e))
            # On error, can try to close and reconnect
            self.socket.close()
            self.socket = None


def main():
    rospy.init_node('cmd_vel_sender', anonymous=True)
    sender = CmdVelSender()

    # rospy.spin() keeps the node running
    rospy.spin()

if __name__ == '__main__':
    main()
