#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

import socket
import struct

# ============ XOR加密/解密函数(示例) ============
def simple_xor_encrypt(data, key=0x42):
    """对字节数组 data 进行简单 XOR，加解密同理"""
    return bytes([b ^ key for b in data])


class CmdVelSender(object):
    def __init__(self):
        # 读取一些参数（可选），比如 IP、端口；也可以直接写死。
        self.roborio_ip = rospy.get_param('~roborio_ip', '10.1.14.2')
        self.roborio_port = rospy.get_param('~roborio_port', 7777)
        self.key = rospy.get_param('~xor_key', 0x42)

        # 初始化 TCP 连接
        self.socket = None
        self.connect_to_roborio()

        # 订阅 /cmd_vel
        self.sub_cmd_vel = rospy.Subscriber(
            '/cmd_vel',  # 你需要的 topic 名称
            Twist,
            self.cmd_vel_callback
        )
        rospy.loginfo("CmdVelSender inited, subscribing to /cmd_vel")

    def connect_to_roborio(self):
        """尝试连接到 RoboRIO 的指定 IP:Port."""
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
        订阅回调函数：当有新的 /cmd_vel 时，会进入这里。
        我们只演示发送 (msg.linear.x, msg.linear.y, msg.angular.z) 这三个 float。
        根据需求，也可发送更多字段。
        """
        if not self.socket:
            # 如果还没连上或连接中断，可以在这里做重连逻辑
            return

        # 打包成 3 个 float = 12 字节
        data_bytes = struct.pack('fff', msg.linear.x, msg.linear.y, msg.angular.z)

        # 简单 XOR 加密
        encrypted = simple_xor_encrypt(data_bytes, self.key)

        # 发送
        try:
            self.socket.sendall(encrypted)
        except Exception as e:
            rospy.logerr("Send failed: {}".format(e))
            # 出错后可以尝试关闭并重连
            self.socket.close()
            self.socket = None


def main():
    rospy.init_node('cmd_vel_sender', anonymous=True)
    sender = CmdVelSender()

    # rospy.spin() 会让节点保持运行
    rospy.spin()

if __name__ == '__main__':
    main()
