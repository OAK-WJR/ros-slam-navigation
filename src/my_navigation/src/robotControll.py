#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial
import time

class MotorControlNode:
    def __init__(self):
        rospy.init_node('motor_control_node')
        
        # 可在ROS参数服务器里设置，也可以写死在代码中
        self.serial_port_name = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.timeout_duration = rospy.get_param('~timeout_duration', 0.5)  # 超时时间（秒）

        # 串口初始化
        try:
            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.serial_port.flush()
            rospy.loginfo(f"Successfully connected to serial port {self.serial_port_name}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port {self.serial_port_name}: {e}")
            rospy.signal_shutdown("Serial port initialization failed")  # 若无法初始化，直接关闭节点
            return

        # 电机中心值
        self.center_left = 1525
        self.center_right = 1545

        # 记录上一次接收 cmd_vel 的时间，初始为当前时间
        self.last_cmd_time = time.time()

        # 订阅 /cmd_vel 话题
        self.subscriber = rospy.Subscriber(
            '/cmd_vel',
            Twist,
            self.cmd_vel_callback,
            queue_size=10
        )
        rospy.loginfo("Motor Control Node has started and subscribed to /cmd_vel")

        # 设置一个定时器，周期性检查是否超时
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timeout_check_callback)  # 0.1s 执行一次

    def cmd_vel_callback(self, msg):
        # 更新 last_cmd_time
        self.last_cmd_time = time.time()

        # 从 /cmd_vel 获取线速度和角速度
        linear_x = msg.linear.x  # 前进/后退速度
        angular_z = msg.angular.z  # 转向速度

        # 转换为左右电机的 PWM 值
        left_speed = int(self.center_left + linear_x * 200 + angular_z * 100)
        right_speed = int(self.center_right + linear_x * 200 - angular_z * 100)  # 注意这里改为减号

        # 限制 PWM 范围在 1000-2000 微秒之间
        left_speed = max(1000, min(2000, left_speed))
        right_speed = max(1000, min(2000, right_speed))

        # 发送给Arduino
        self.send_pwm(left_speed, right_speed)

    def timeout_check_callback(self, event):
        """
        定时器回调：检测是否超过超时时间没收到新命令。
        如果超时，则发送中心值(停止命令)到电机。
        """
        current_time = time.time()
        if (current_time - self.last_cmd_time) > self.timeout_duration:
            rospy.logwarn("No new /cmd_vel received. Stopping motors.")
            self.send_pwm(self.center_left, self.center_right)

    def send_pwm(self, left_speed, right_speed):
        """
        统一封装发送PWM的操作，并带串口异常处理
        """
        try:
            command = f"{left_speed} {right_speed}\n"
            self.serial_port.write(command.encode('utf-8'))
            rospy.loginfo(f"Sent to Arduino: {command.strip()}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to write to serial port: {e}")

    def shutdown(self):
        """
        节点关闭时：停止电机并关闭串口
        """
        rospy.loginfo("Shutting down motor control node...")
        # 停止电机
        self.send_pwm(self.center_left, self.center_right)

        # 停止定时器
        self.timer.shutdown()

        if self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("Serial port closed.")

def main():
    try:
        node = MotorControlNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
