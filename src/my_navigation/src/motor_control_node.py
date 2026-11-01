#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray

class CmdVelToPWMNode(object):
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node("cmd_vel_to_raw_pwm_node", anonymous=True)

        # 2. 从参数服务器获取 config_file 路径，或者使用默认路径
        config_file = rospy.get_param(
            "~config_file", 
            "/home/jetson/catkin_ws/src/my_navigation/config/robot_info.yaml"
        )

        # 读取配置文件，获取电机中点 PWM 值等
        self.pwm_center_left = 1500
        self.pwm_center_right = 1500
        self.serial_port_name = "/dev/ttyUSB0"   # 备用串口名

        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    self.serial_port_name = config["arduino"]["serial_port"]
                    self.pwm_center_left  = config["motor_control_node"]["pwm_center_left"]
                    self.pwm_center_right = config["motor_control_node"]["pwm_center_right"]
                    rospy.loginfo("[cmd_vel_to_raw_pwm] Loaded config successfully.")
                    rospy.loginfo(f"serial_port_name = {self.serial_port_name}")
                    rospy.loginfo(f"pwm_center_left = {self.pwm_center_left}, pwm_center_right = {self.pwm_center_right}")
            except Exception as e:
                rospy.logwarn(f"[cmd_vel_to_raw_pwm] Failed to load config file: {e}. Using default values.")
        else:
            rospy.logwarn(f"[cmd_vel_to_raw_pwm] Config file {config_file} does not exist. Using default values.")

        # 3. 机器人本体参数（需要与实际机器人匹配）
        self.wheel_radius = 0.033   # 轮子半径(m)
        self.wheel_base   = 0.100   # 左右轮中心间距(m)

        # 4. 简单的速度-PWM 转换系数（根据实际测量或标定）
        #   假设线速度( m/s )经过一定系数映射到 PWM 的偏移量( e.g. 100 PWM每 1 m/s )
        #   请根据实际测量做调整
        self.K_v = 100.0  # 线速度 -> PWM 增量系数
        self.K_w = 50.0   # 角速度 -> PWM 增量系数

        # 5. 建立简单的左右轮速度校正增益，用于“回正” (只作演示，可改进为PID)
        #   如果发现左轮速度比期望大了，就调低一点 PWM；右轮同理
        #   这里仅用一个简单的 P (比例) 控制示例
        self.Kp = 50.0   # 速度误差 -> PWM 修正的比例系数 (需要自己调参)

        # 6. 初始化存储当前测得的左右轮角速度
        self.left_vel_meas  = 0.0
        self.right_vel_meas = 0.0

        # 7. 发布者：将计算好的左右电机 PWM 指令发布到 /raw_pwm 话题
        self.raw_pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=10)

        # 8. 订阅 /cmd_vel 和 /joint_states
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        rospy.loginfo("[cmd_vel_to_raw_pwm] Node is initialized.")

    def joint_states_callback(self, msg):
        """
        回调函数：订阅 /joint_states，获取左右轮当前实际角速度(rad/s)
        假设 msg.name = ["left_wheel_joint", "right_wheel_joint"]
        msg.velocity 对应左右轮的角速度 (rad/s)，需要转换为线速度(m/s)
        """
        try:
            # 根据顺序读取
            idx_left  = msg.name.index("left_wheel_joint")
            idx_right = msg.name.index("right_wheel_joint")

            # joint_states 里给的是角速度(rad/s)，换成线速度(m/s) = 角速度 * 轮子半径
            self.left_vel_meas  = msg.velocity[idx_left]  * self.wheel_radius
            self.right_vel_meas = msg.velocity[idx_right] * self.wheel_radius
        except ValueError:
            # 如果找不到对应索引，忽略
            pass

    def cmd_vel_callback(self, msg):
        """
        回调函数：订阅 /cmd_vel
        根据期望的线速度(v)和角速度(w)，
        先计算“理想”左右轮线速度，再转换成PWM，再根据实际测得速度与目标速度的偏差做回正。
        最终发布到 /raw_pwm。
        """
        # 1. 从 /cmd_vel 中获取期望线速度与角速度
        v = msg.linear.x   # 单位: m/s
        w = msg.angular.z  # 单位: rad/s

        # 2. 理论上左右轮线速度
        #    left_speed  = v - w*(wheel_base/2)
        #    right_speed = v + w*(wheel_base/2)
        #    这里不做额外限幅，你可自行加以限制
        left_target_speed  = v - (w * (self.wheel_base / 2.0))
        right_target_speed = v + (w * (self.wheel_base / 2.0))

        # 3. 将目标线速度转换成 “基础PWM增量” (不考虑回正校正时)
        #    注意：这里非常依赖实际测量，需要你自己调参 K_v、K_w
        #    例如：left_pwm = pwm_center_left + K_v * left_target_speed
        #           right_pwm = pwm_center_right + K_v * right_target_speed
        #    如果要细分线速度和角速度增量，也可以拆分，但这里做简化处理
        left_pwm_base  = self.pwm_center_left  + int(self.K_v * left_target_speed)
        right_pwm_base = self.pwm_center_right + int(self.K_v * right_target_speed)

        # 4. 根据实际速度和目标速度的误差，做简单比例修正
        #    err_left  = left_target_speed  - left_vel_meas
        #    err_right = right_target_speed - right_vel_meas
        err_left  = left_target_speed  - self.left_vel_meas
        err_right = right_target_speed - self.right_vel_meas

        # 将速度误差转换成 PWM 修正量
        pwm_offset_left  = int(self.Kp * err_left)
        pwm_offset_right = int(self.Kp * err_right)

        # 最终 PWM
        left_pwm  = left_pwm_base  + pwm_offset_left
        right_pwm = right_pwm_base + pwm_offset_right

        # 5. 限幅，确保 PWM 在 [1000, 2000] 区间
        left_pwm  = max(1000, min(2000, left_pwm))
        right_pwm = max(1000, min(2000, right_pwm))

        # 6. 打包消息并发布到 /raw_pwm
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [left_pwm, right_pwm]
        self.raw_pwm_pub.publish(pwm_msg)

        rospy.logdebug(f"[cmd_vel_to_raw_pwm] v={v:.2f}, w={w:.2f}, Lpwm={left_pwm}, Rpwm={right_pwm}, "
                       f"ErrL={err_left:.2f}, ErrR={err_right:.2f}")

    def spin(self):
        rospy.loginfo("[cmd_vel_to_raw_pwm] Start spinning...")
        rospy.spin()

if __name__ == "__main__":
    try:
        # 等待 /serial_port_ready (若有需要，可参照 raw_pwm.py 的处理方式)
        rospy.loginfo("cmd_vel_to_raw_pwm: Waiting for /serial_port_ready param...")
        while not rospy.has_param("/serial_port_ready"):
            rospy.sleep(1.0)
        while not rospy.get_param("/serial_port_ready", False):
            rospy.loginfo("cmd_vel_to_raw_pwm: /serial_port_ready is false, retrying...")
            rospy.sleep(1.0)
        rospy.loginfo("cmd_vel_to_raw_pwm: /serial_port_ready == true, starting initialization...")

        node = CmdVelToPWMNode()
        rospy.set_param("/cmd_vel_to_raw_pwm_ready", True)
        rospy.loginfo("[cmd_vel_to_raw_pwm] /cmd_vel_to_raw_pwm_ready = True")

        node.spin()

    except rospy.ROSInterruptException:
        pass
