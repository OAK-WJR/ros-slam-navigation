#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
import tf2_ros
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

class DiffDriveOdomPublisher(object):
    def __init__(self):
        # -----------------------------
        #      1. 从参数服务器读取
        # -----------------------------
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.033)   # 轮子半径 (m)
        self.wheel_base   = rospy.get_param("~wheel_base",   0.1075)   # 左右轮中心间距 (m)

        # 若左右轮安装反向，则可通过参数控制是否反转
        self.reverse_left_wheel  = rospy.get_param("~reverse_left_wheel",  False)
        self.reverse_right_wheel = rospy.get_param("~reverse_right_wheel", False)

        # 设置最大允许增量阈值，过滤严重跳变
        self.max_delta_threshold = rospy.get_param("~max_delta_threshold", 1.0)

        # 设置滤波窗口大小
        self.filter_window = rospy.get_param("~filter_window", 3)

        # -----------------------------
        #      2. 初始化位姿与缓存
        # -----------------------------
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.prev_left_rad  = None
        self.prev_right_rad = None
        self.prev_time      = None

        # 存放增量用于滑动平均滤波
        self.left_deltas  = []
        self.right_deltas = []

        # -----------------------------
        #      3. 创建发布者和订阅者
        # -----------------------------
        # 发布里程计
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        # TF 广播器
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

        # 订阅/joint_states
        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)

    def jointStateCallback(self, msg):
        """
        从 JointState 中提取左右轮的角度(位置)，
        计算机器人位姿增量，发布里程计 & TF。
        """
        # 查找左右轮索引
        try:
            left_index  = msg.name.index("left_wheel_joint")
            right_index = msg.name.index("right_wheel_joint")
        except ValueError:
            rospy.logwarn("Cannot find wheel_joint in JointState!")
            return

        # 读取左右轮位置
        left_pos  = msg.position[left_index]
        right_pos = msg.position[right_index]

        # 若轮子安装反向，可在此反转数据
        if self.reverse_left_wheel:
            left_pos = -left_pos
        if self.reverse_right_wheel:
            right_pos = -right_pos

        current_time = rospy.Time.now()
        if self.prev_time is None:
            # 第一次回调，无法计算速度和位移，直接记录
            self.prev_time       = current_time
            self.prev_left_rad   = left_pos
            self.prev_right_rad  = right_pos
            return

        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0.0:
            # 时间异常，直接返回
            return

        # 计算前后两次角度差值
        delta_left  = left_pos  - self.prev_left_rad
        delta_right = right_pos - self.prev_right_rad

        # 更新上一次
        self.prev_left_rad  = left_pos
        self.prev_right_rad = right_pos
        self.prev_time      = current_time

        # -----------------------------
        #   4. 数据滤波 & 异常检测
        # -----------------------------
        # （1）滑动平均滤波
        self.left_deltas.append(delta_left)
        self.right_deltas.append(delta_right)
        if len(self.left_deltas)  > self.filter_window:
            self.left_deltas.pop(0)
        if len(self.right_deltas) > self.filter_window:
            self.right_deltas.pop(0)

        # 将窗口内增量进行平均
        delta_left  = sum(self.left_deltas)  / len(self.left_deltas)
        delta_right = sum(self.right_deltas) / len(self.right_deltas)

        # （2）异常值过滤
        if abs(delta_left) > self.max_delta_threshold or abs(delta_right) > self.max_delta_threshold:
            rospy.logwarn("Abnormal wheel delta detected, skipping this update.")
            return

        # -----------------------------
        #      5. 计算里程计
        # -----------------------------
        # 单个轮子的线位移
        dist_left  = delta_left  * self.wheel_radius
        dist_right = delta_right * self.wheel_radius

        # 平均线位移
        dist = (dist_left + dist_right) / 2.0

        # 转动角度 (弧度)
        theta = (dist_left - dist_right) / self.wheel_base

        # ---- 精确积分 (Exact Integration) ----
        if abs(theta) < 1e-9:
            # 近似直线运动
            self.x += dist * math.cos(self.yaw)
            self.y += dist * math.sin(self.yaw)
        else:
            # 弧线运动
            radius = dist / theta
            self.x += radius * (math.sin(self.yaw + theta) - math.sin(self.yaw))
            self.y += radius * (math.cos(self.yaw) - math.cos(self.yaw + theta))

        # 更新偏航角
        self.yaw += theta
        # 归一化 yaw 到 -pi ~ pi
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # 计算线速度 & 角速度
        vx  = dist / dt
        vy  = 0.0
        vth = theta / dt

        # -----------------------------
        #      6. 发布里程计
        # -----------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id  = "base_link"

        # 位置 (x, y)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # 姿态 (yaw -> 四元数)
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(*q)

        # 速度
        odom_msg.twist.twist.linear.x  = vx
        odom_msg.twist.twist.linear.y  = vy
        odom_msg.twist.twist.angular.z = vth

        # (可选) 设置里程计协方差
        # 根据实际情况可以适当调整
        odom_msg.pose.covariance = [
            0.001, 0,     0,     0,     0,     0,
            0,     0.001, 0,     0,     0,     0,
            0,     0,     0.001, 0,     0,     0,
            0,     0,     0,     0.001, 0,     0,
            0,     0,     0,     0,     0.001, 0,
            0,     0,     0,     0,     0,     0.001
        ]
        odom_msg.twist.covariance = [
            0.001, 0,     0,     0,     0,     0,
            0,     0.001, 0,     0,     0,     0,
            0,     0,     0.001, 0,     0,     0,
            0,     0,     0,     0.001, 0,     0,
            0,     0,     0,     0,     0.001, 0,
            0,     0,     0,     0,     0,     0.001
        ]

        # 发布里程计消息
        self.odom_pub.publish(odom_msg)

        # -----------------------------
        #      7. 发布 TF (odom->base_link)
        # -----------------------------
        odom_tf = TransformStamped()
        odom_tf.header.stamp = current_time
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id  = "base_link"
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation      = Quaternion(*q)

        self.odom_broadcaster.sendTransform(odom_tf)

        rospy.loginfo_throttle(1.0,  # 1s节流打印一次
            f"[Odom] x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f} deg, vx={vx:.3f}, vth={vth:.3f}"
        )

def main():
    rospy.init_node("diffdrive_odom_publisher")
    rospy.loginfo("DiffDrive Odom Publisher started.")
    node = DiffDriveOdomPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
