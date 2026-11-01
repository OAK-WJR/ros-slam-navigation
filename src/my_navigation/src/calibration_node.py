#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray  # 用于发布左右PWM值
import os
import yaml
import time

class CalibrationNode(object):
    def __init__(self):
        rospy.init_node("calibration_node", anonymous=True)

        # ========== 1. 读取/设置相关参数 ==========
        # PWM 最大最小值（左右独立）
        self.pwm_min_left_default  = rospy.get_param("~pwm_min_left_default", 0)
        self.pwm_max_left_default  = rospy.get_param("~pwm_max_left_default", 3000)
        self.pwm_min_right_default = rospy.get_param("~pwm_min_right_default", 0)
        self.pwm_max_right_default = rospy.get_param("~pwm_max_right_default", 3000)
        
        # 初始化 PWM 范围，稍后将通过测试进行调整
        self.pwm_min_left  = self.pwm_min_left_default
        self.pwm_max_left  = self.pwm_max_left_default
        self.pwm_min_right = self.pwm_min_right_default
        self.pwm_max_right = self.pwm_max_right_default

        # 每次微调步幅（us）
        self.adjust_step = rospy.get_param("~adjust_step", 1)
        # 速度阈值（m/s 以内视为“静止”）
        self.speed_thresh = rospy.get_param("~speed_thresh", 0.001)
        # 轮子半径，用于将角速度(rad/s)转换成线速度(m/s)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.033)

        # 可从上次校准结果或用户给定参数读取初始中位值；若没有，就默认 1500
        self.calib_left  = rospy.get_param("~init_center_left",  1500)
        self.calib_right = rospy.get_param("~init_center_right", 1500)

        # 用于存放当前左右轮的线速度
        self.left_vel = 0.0
        self.right_vel = 0.0

        # ========== 2. 创建Publisher，用于发布左右轮PWM ==========
        # 本示例用 Int16MultiArray, data=[left_pwm, right_pwm]
        # 你可以改成自己硬件需要的消息类型和话题
        self.pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=1)

        # ========== 3. 订阅 /joint_states，读取当前轮子转速 ==========
        # 确保 joint_states 中包含 "left_wheel_joint" 和 "right_wheel_joint"
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        # ========== 启动校准流程 ==========
        rospy.loginfo("[Calibration] Node started. Will test and calibrate left & right wheels...")

        # 等待一小会儿，让 /joint_states 开始发布
        rospy.sleep(1.0)

        # 测试并设置 PWM 范围
        self.test_pwm_range(is_left=True)
        self.test_pwm_range(is_left=False)

        # 先校准左轮，再校准右轮
        self.calibrate_wheel(is_left=True)
        self.calibrate_wheel(is_left=False)

        # 校准完成
        rospy.loginfo("[Calibration] Done! Left Center=%d, Right Center=%d",
                      self.calib_left, self.calib_right)
        rospy.loginfo("[Calibration] PWM Ranges: Left [%d, %d], Right [%d, %d]",
                      self.pwm_min_left, self.pwm_max_left,
                      self.pwm_min_right, self.pwm_max_right)

        # 把结果写到参数服务器 (供其他节点使用)
        rospy.set_param("/motor_control_node/pwm_center_left",  self.calib_left)
        rospy.set_param("/motor_control_node/pwm_center_right", self.calib_right)
        rospy.set_param("/motor_control_node/pwm_min_left", self.pwm_min_left)
        rospy.set_param("/motor_control_node/pwm_max_left", self.pwm_max_left)
        rospy.set_param("/motor_control_node/pwm_min_right", self.pwm_min_right)
        rospy.set_param("/motor_control_node/pwm_max_right", self.pwm_max_right)

        # 如果需要，还可以保存到本地文件
        self.save_to_config_file(self.calib_left, self.calib_right)

        # 校准完成后，关闭节点
        rospy.signal_shutdown("Calibration completed.")

    def joint_state_cb(self, msg):
        """
        从 JointState 获取左右轮角速度 => 线速度
        假设 joint_states 里轮子的 name 分别是 'left_wheel_joint' & 'right_wheel_joint'
        """
        try:
            left_index  = msg.name.index("left_wheel_joint")
            right_index = msg.name.index("right_wheel_joint")
        except ValueError:
            # 说明这次消息里没有这俩 joint，忽略
            return

        left_omega  = msg.velocity[left_index]   # 左轮角速度 (rad/s)
        right_omega = msg.velocity[right_index]  # 右轮角速度 (rad/s)

        self.left_vel  = left_omega  * self.wheel_radius  # (m/s)
        self.right_vel = right_omega * self.wheel_radius  # (m/s)

    def test_pwm_range(self, is_left=True):
        """
        测试单个轮子的 PWM 范围（最低值和最高值）。
        通过逐步增加或减少 PWM，检测轮子是否开始转动，以确定有效的 PWM 范围。
        """
        wheel = "Left" if is_left else "Right"
        rospy.loginfo(f"[Calibration] Testing {wheel} wheel PWM range...")

        # 测试最低 PWM
        rospy.loginfo(f"[Calibration] Testing {wheel} wheel minimum PWM...")
        pwm_min = self.pwm_min_left_default if is_left else self.pwm_min_right_default
        test_pwm_min = self.find_pwm_boundary(is_left, pwm_min, direction='increase')
        if is_left:
            self.pwm_min_left = test_pwm_min
        else:
            self.pwm_min_right = test_pwm_min
        rospy.loginfo(f"[Calibration] {wheel} wheel minimum PWM set to {test_pwm_min}")

        # 测试最高 PWM
        rospy.loginfo(f"[Calibration] Testing {wheel} wheel maximum PWM...")
        pwm_max = self.pwm_max_left_default if is_left else self.pwm_max_right_default
        test_pwm_max = self.find_pwm_boundary(is_left, pwm_max, direction='decrease')
        if is_left:
            self.pwm_max_left = test_pwm_max
        else:
            self.pwm_max_right = test_pwm_max
        rospy.loginfo(f"[Calibration] {wheel} wheel maximum PWM set to {test_pwm_max}")

    def find_pwm_boundary(self, is_left, start_pwm, direction='increase'):
        """
        寻找 PWM 边界值，根据速度变化趋势检测速度停止增加或开始下降的位置。
        改进：引入速度变化的允许区间 (delta)。

        :param is_left: True 表示左轮，False 表示右轮
        :param start_pwm: 起始 PWM 值（推荐设置为中间值）
        :param direction: 'increase' 或 'decrease'
        :return: 找到的边界 PWM 值
        """
        step = 10  # PWM 调整步幅
        pwm = start_pwm
        max_attempts = 1000  # 限制最大尝试次数
        prev_speed = 0  # 上一次的速度，用于计算速度增量
        stable_speed_count = 0  # 稳定速度计数器，用于检测速度变化停止

        # 定义速度变化允许的区间
        delta = 0.08  # 允许的速度变化范围

        rospy.loginfo(f"[Calibration] Testing {'Left' if is_left else 'Right'} wheel, direction: {direction}")

        for attempt in range(max_attempts):
            # 设置当前轮的 PWM 值
            if is_left:
                left_pwm = pwm
                right_pwm = self.calib_right  # 右轮保持不变
            else:
                left_pwm = self.calib_left  # 左轮保持不变
                right_pwm = pwm

            # 发布 PWM 命令
            pwm_msg = Int16MultiArray()
            pwm_msg.data = [left_pwm, right_pwm]
            self.pwm_pub.publish(pwm_msg)

            # 等待系统响应
            rospy.sleep(0.1)

            # 获取当前速度
            current_speed = self.left_vel if is_left else self.right_vel
            speed_diff = abs(current_speed - prev_speed)

            rospy.logdebug(f"[Calibration] PWM: {pwm}, Current Speed: {current_speed:.4f}, Speed Diff: {speed_diff:.4f}")

            # 检查速度是否超出允许变化区间
            if not (prev_speed - delta <= current_speed <= prev_speed + delta):
                stable_speed_count += 1
                if stable_speed_count >= 3:  # 连续3次超出区间，认为达到边界
                    rospy.loginfo(f"[Calibration] Boundary found: PWM = {pwm}, Speed = {current_speed:.4f}")
                    return pwm
            else:
                stable_speed_count = 0  # 重置稳定计数器

            # 调整 PWM 值
            if direction == 'increase':
                pwm += step
            elif direction == 'decrease':
                pwm -= step

            # 更新上一次速度
            prev_speed = current_speed

            # 限制 PWM 在合理范围内
            if is_left:
                pwm = max(self.pwm_min_left_default, min(self.pwm_max_left_default, pwm))
            else:
                pwm = max(self.pwm_min_right_default, min(self.pwm_max_right_default, pwm))

        rospy.logwarn(f"[Calibration] Failed to find PWM boundary after {max_attempts} attempts.")
        return pwm


    def calibrate_wheel(self, is_left=True):
        """
        对单个轮子进行校准：
         - 当检测到该轮速度 > 阈值时 => 调整其中位值(增/减)
         - 直到速度 < 阈值 => 结束校准
         - 另一个轮子始终保持它的中心值，避免干扰
        """
        wheel_name = "Left" if is_left else "Right"
        rospy.loginfo(f"[Calibration] Calibrating {wheel_name} wheel center...")

        r = rospy.Rate(10)  # 循环频率 (Hz)
        max_iterations = 3000  # 最多迭代次数，防止卡住

        for i in range(max_iterations):
            # 根据当前校准状态设置 PWM 值
            if is_left:
                left_pwm = self.calib_left
                right_pwm = self.calib_right  # 右轮保持不变
            else:
                left_pwm = self.calib_left  # 左轮保持不变
                right_pwm = self.calib_right

            # 发布PWM命令
            pwm_msg = Int16MultiArray()
            pwm_msg.data = [left_pwm, right_pwm]
            self.pwm_pub.publish(pwm_msg)

            rospy.sleep(0.2)

            # 当前轮子速度
            current_speed = abs(self.left_vel if is_left else self.right_vel)

            if current_speed < self.speed_thresh:
                rospy.loginfo(f"[Calibration] {wheel_name} wheel speed < threshold => rough calibration done!")
                break
            else:
                # 速度还在阈值之上 => 判断方向并做微调
                if is_left:
                    if self.left_vel > 0:
                        self.calib_left -= self.adjust_step
                        # 确保不低于左轮最小值
                        self.calib_left = max(self.pwm_min_left, self.calib_left)
                    else:
                        self.calib_left += self.adjust_step
                        # 确保不超过左轮最大值
                        self.calib_left = min(self.pwm_max_left, self.calib_left)
                else:
                    if self.right_vel > 0:
                        self.calib_right -= self.adjust_step
                        # 确保不低于右轮最小值
                        self.calib_right = max(self.pwm_min_right, self.calib_right)
                    else:
                        self.calib_right += self.adjust_step
                        # 确保不超过右轮最大值
                        self.calib_right = min(self.pwm_max_right, self.calib_right)

            r.sleep()

        # 粗调完成后，调用精调
        self.fine_calibration(is_left=is_left)

    def fine_calibration(self, is_left=True, max_iterations=50, 
                         speed_collect_threshold=0.05,  # 当前轮速小于该值才认为进入精调区间
                         final_adjust_threshold=0.001,  # 最终平均速度若大于该值，需再微调
                         adjust_step=1):
        """
        针对某个轮子进行精细校准：
        1. 使用已经得到的 self.calib_left / self.calib_right 作为“初步的”中心PWM
        2. 在循环中持续发布该PWM，统计所有速度 < speed_collect_threshold 的数据
        3. 计算平均速度，若仍偏大(正或负)，做最后微调

        :param is_left: True表示左轮校准；False表示右轮校准
        :param max_iterations: 循环次数上限
        :param speed_collect_threshold: 小于这个阈值的速度将被收集做平均
        :param final_adjust_threshold: 最终平均速度若大于该值，需再微调
        :param adjust_step: 最后微调的步进大小
        """
        wheel_name = "left" if is_left else "right"
        rospy.loginfo(f"[Calibration] Start fine calibration for {wheel_name} wheel ...")

        r = rospy.Rate(10)  # 10Hz
        speeds_collected = []

        # 1. 连续发送“初步”中心PWM，记录速度
        for i in range(max_iterations):
            # 发布PWM命令
            pwm_msg = Int16MultiArray()
            if is_left:
                pwm_msg.data = [self.calib_left, self.calib_right]
            else:
                pwm_msg.data = [self.calib_left, self.calib_right]
            self.pwm_pub.publish(pwm_msg)

            rospy.sleep(0.03)
            # 获取当前轮的速度
            current_speed = self.left_vel if is_left else self.right_vel
            rospy.logdebug(f"[Calibration] {wheel_name} wheel speed = {current_speed:.4f} m/s")

            # 若当前速度的绝对值小于阈值，记录下来
            if abs(current_speed) < speed_collect_threshold:
                speeds_collected.append(current_speed)

            r.sleep()

        # 2. 计算平均速度
        if len(speeds_collected) == 0:
            rospy.logwarn(f"[Calibration] No speed below threshold={speed_collect_threshold:.3f} was collected for {wheel_name} wheel. "
                          "Maybe calibration is not finished or threshold too small.")
            return  # 直接返回，或者自行决定是否要再做其他处理

        avg_speed = sum(speeds_collected) / len(speeds_collected)
        rospy.loginfo(f"[Calibration] {wheel_name} wheel collected {len(speeds_collected)} speeds, avg={avg_speed:.6f} m/s")

        # 3. 依据平均速度再做一次微调
        #    如果 avg_speed > 0 表示轮子依然略微往正方向转 => 中心PWM再减一点
        #    如果 avg_speed < 0 表示轮子略微往负方向转 => 中心PWM再加一点
        if abs(avg_speed) > final_adjust_threshold:
            if is_left:
                if avg_speed > 0:
                    rospy.loginfo(f"[Calibration] Avg speed > 0 => reduce left wheel PWM by {adjust_step}")
                    self.calib_left -= adjust_step
                    # 确保不低于左轮最小值
                    self.calib_left = max(self.pwm_min_left, self.calib_left)
                else:
                    rospy.loginfo(f"[Calibration] Avg speed < 0 => increase left wheel PWM by {adjust_step}")
                    self.calib_left += adjust_step
                    # 确保不超过左轮最大值
                    self.calib_left = min(self.pwm_max_left, self.calib_left)
                rospy.loginfo(f"[Calibration] New left wheel center PWM = {self.calib_left}")
            else:
                if avg_speed > 0:
                    rospy.loginfo(f"[Calibration] Avg speed > 0 => reduce right wheel PWM by {adjust_step}")
                    self.calib_right -= adjust_step
                    # 确保不低于右轮最小值
                    self.calib_right = max(self.pwm_min_right, self.calib_right)
                else:
                    rospy.loginfo(f"[Calibration] Avg speed < 0 => increase right wheel PWM by {adjust_step}")
                    self.calib_right += adjust_step
                    # 确保不超过右轮最大值
                    self.calib_right = min(self.pwm_max_right, self.calib_right)
                rospy.loginfo(f"[Calibration] New right wheel center PWM = {self.calib_right}")
        else:
            rospy.loginfo(f"[Calibration] Avg speed is small enough (|avg|<= {final_adjust_threshold:.3f}), no final adjustment needed.")

    def save_to_config_file(self, left_val, right_val):
        """
        保存校准结果到指定的 YAML 配置文件中。

        参数:
        - left_val: 左轮校准 PWM 值。
        - right_val: 右轮校准 PWM 值。
        """
        config_file = "/home/jetson/catkin_ws/src/my_navigation/config/robot_info.yaml"

        # 确保目录存在
        os.makedirs(os.path.dirname(config_file), exist_ok=True)

        # 读取现有配置（如果存在）
        config_data = {}
        if os.path.exists(config_file):
            with open(config_file, "r") as f:
                try:
                    config_data = yaml.safe_load(f) or {}
                except yaml.YAMLError as e:
                    rospy.logwarn(f"[Calibration] Failed to parse existing config file: {e}")

        # 更新校准数据
        calibration_data = {
            "pwm_center_left": left_val,
            "pwm_center_right": right_val,
            "pwm_min_left": self.pwm_min_left,
            "pwm_max_left": self.pwm_max_left,
            "pwm_min_right": self.pwm_min_right,
            "pwm_max_right": self.pwm_max_right
        }
        if "motor_control_node" not in config_data:
            config_data["motor_control_node"] = {}
        config_data["motor_control_node"].update(calibration_data)

        # 写入文件
        with open(config_file, "w") as f:
            yaml.dump(config_data, f, default_flow_style=False)

        rospy.loginfo(f"[Calibration] Calibration results saved to {config_file}")

def wait_for_joint_states():
    """等待 /joint_states 数据发布"""
    try:
        rospy.wait_for_message("/joint_states", JointState, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /joint_states topic. Exiting.")
        rospy.signal_shutdown("No /joint_states data available.")
        return

def main():
    # 等待 /raw_pwm_ready
    rospy.loginfo("[Calibration] Waiting for /raw_pwm_ready param...")
    while not rospy.has_param("/raw_pwm_ready"):
        rospy.sleep(1.0)
    while not rospy.get_param("/raw_pwm_ready", False):
        rospy.loginfo("[Calibration] /raw_pwm_ready is false, retrying...")
        rospy.sleep(1.0)

    rospy.loginfo("[Calibration] /raw_pwm_ready == true, start calibrating...")

    node = CalibrationNode()

    rospy.set_param("/calibration_done", True)
    rospy.loginfo("[Calibration] /calibration_done = True")
    rospy.spin()

if __name__ == "__main__":
    main()
