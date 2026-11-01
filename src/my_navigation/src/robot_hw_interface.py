#!/usr/bin/env python
import rospy
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class RobotHWInterface:
    def __init__(self):
        rospy.init_node("robot_hw_interface")

        # 左右轮速度占位
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        # 订阅 /joint_states 以更新速度
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        # 发布 /raw_pwm
        self.raw_pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=10)

        # 订阅 /cmd_vel (来自 diff_drive_controller)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

    def joint_states_callback(self, msg):
        # 解析 /joint_states，更新当前速度
        try:
            self.left_velocity = msg.velocity[msg.name.index("left_wheel_joint")]
            self.right_velocity = msg.velocity[msg.name.index("right_wheel_joint")]
        except ValueError:
            pass  # 如果找不到 joint 名字，忽略

    def cmd_vel_callback(self, msg):
        # 将 /cmd_vel 转换为左右轮 PWM
        left_pwm = int(1398 + 100 * msg.linear.x - 50 * msg.angular.z)
        right_pwm = int(1398 + 100 * msg.linear.x + 50 * msg.angular.z)

        # 限幅 PWM 范围
        left_pwm = max(1000, min(2000, left_pwm))
        right_pwm = max(1000, min(2000, right_pwm))

        # 发布到 /raw_pwm
        pwm_msg = Int16MultiArray(data=[left_pwm, right_pwm])
        self.raw_pwm_pub.publish(pwm_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    hw = RobotHWInterface()
    hw.spin()
