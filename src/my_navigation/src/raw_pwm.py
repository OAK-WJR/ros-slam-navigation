#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
import serial
import yaml


class RawPWMReceiverNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('raw_pwm_receiver', anonymous=True)

        # 加载配置文件以获取串口信息
        config_file = rospy.get_param("~config_file", "/home/jetson/catkin_ws/src/my_navigation/config/robot_info.yaml")
        self.serial_port_name = "/dev/ttyUSB0"  # 默认值

        try:
            with open(config_file, "r") as f:
                config = yaml.safe_load(f)
                self.serial_port_name = config["arduino"]["serial_port"]
                rospy.loginfo(f"Loaded serial port from config: {self.serial_port_name}")
        except Exception as e:
            rospy.logwarn(f"Failed to load config file: {e}. Using default port: {self.serial_port_name}")

        # 初始化串口
        try:
            self.serial_port = serial.Serial(self.serial_port_name, 9600, timeout=1)
            self.serial_port.flush()
            rospy.loginfo("Serial port opened successfully.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            rospy.signal_shutdown("Serial Port Error")
            return

        # 订阅 /raw_pwm 话题
        rospy.Subscriber('/raw_pwm', Int16MultiArray, self.raw_pwm_callback)
        rospy.loginfo("Subscribed to /raw_pwm.")

    def raw_pwm_callback(self, msg):
        """
        接收 /raw_pwm 消息并发送 PWM 值到串口
        消息格式: Int16MultiArray.data = [left_pwm, right_pwm]
        """
        if len(msg.data) != 2:
            rospy.logerr("Invalid message length, expected 2 values (left_pwm, right_pwm).")
            return

        left_pwm, right_pwm = msg.data

        # 构造串口指令字符串
        command = f"{left_pwm} {right_pwm}\n"

        try:
            # 发送到串口
            self.serial_port.write(command.encode('utf-8'))
            rospy.loginfo(f"Sent to hardware: {command.strip()}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to write to serial port: {e}")


def main(args=None):
    # 等待serial_port_ready
    rospy.loginfo("raw_pwm: Waiting for /serial_port_ready param...")
    while not rospy.has_param("/serial_port_ready"):
#        rospy.loginfo("raw_pwm: Waiting for /serial_port_ready param...")
        rospy.sleep(1.0)
    while not rospy.get_param("/serial_port_ready", False):
        rospy.loginfo("raw_pwm: /serial_port_ready is false, retrying...")
        rospy.sleep(1.0)
    rospy.loginfo("raw_pwm: /serial_port_ready == true, starting raw_pwm initialization...")

    node = RawPWMReceiverNode()

    rospy.set_param("/raw_pwm_ready", True)
    rospy.loginfo("raw_pwm: /raw_pwm_ready = True")

    rospy.spin()


if __name__ == '__main__':
    main()
