#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Transform
import PyKDL  # PyKDL 用于变换、矩阵等计算

class MapOdomTfBroadcaster(object):
    def __init__(self):
        rospy.init_node("map_odom_tf_broadcaster", anonymous=True)

        # 1) TF Buffer/Listener，用于查询 odom->base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 2) 用于发布 map->odom 的广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 3) 存储并初始化 map->odom
        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = "map"
        self.map_to_odom.child_frame_id = "odom"
        self.map_to_odom.transform.rotation.w = 1.0  # 单位旋转

        # 4) 订阅 /initialpose
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialpose_callback)

        # 以固定频率持续发布
        self.rate = rospy.Rate(20)
        self.publish_loop()

    def initialpose_callback(self, msg):
        """当收到新的初始位姿时，更新 map->odom，使 base_link 放到你指定的初始位置"""
        # 1) 从 /initialpose 中获取 期望的 "map->base_link"
        init_x = msg.pose.pose.position.x
        init_y = msg.pose.pose.position.y
        init_z = msg.pose.pose.position.z
        init_qx = msg.pose.pose.orientation.x
        init_qy = msg.pose.pose.orientation.y
        init_qz = msg.pose.pose.orientation.z
        init_qw = msg.pose.pose.orientation.w

        rospy.loginfo("Received new initial pose: (%.2f, %.2f, %.2f)", init_x, init_y, init_z)

        # 构造一个 Transform: map->base_link
        t_map_base = Transform()
        t_map_base.translation.x = init_x
        t_map_base.translation.y = init_y
        t_map_base.translation.z = init_z
        t_map_base.rotation.x = init_qx
        t_map_base.rotation.y = init_qy
        t_map_base.rotation.z = init_qz
        t_map_base.rotation.w = init_qw

        # 2) 查询 odom->base_link
        try:
            t_odom_base = self.tf_buffer.lookup_transform(
                "odom", "base_link", rospy.Time(0), rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logwarn("Failed to lookup odom->base_link, error: %s", e)
            return

        # 3) 转为 KDL.Frame
        kdl_map_base  = transformToKDL(t_map_base)
        kdl_odom_base = transformToKDL(t_odom_base.transform)

        # 4) 计算 map->odom = map->base_link * (odom->base_link)^(-1)
        kdl_map_odom = kdl_map_base * kdl_odom_base.Inverse()

        # 5) 转回 geometry_msgs/Transform
        updated_transform = kdlToTransform(kdl_map_odom)

        # 6) 存到 self.map_to_odom，等待后续发布
        self.map_to_odom.header.stamp = rospy.Time.now()
        self.map_to_odom.transform = updated_transform

        # 打印调试
        tx = updated_transform.translation.x
        ty = updated_transform.translation.y
        tz = updated_transform.translation.z
        qx = updated_transform.rotation.x
        qy = updated_transform.rotation.y
        qz = updated_transform.rotation.z
        qw = updated_transform.rotation.w
        rospy.loginfo("Updated map->odom => t(%.2f, %.2f, %.2f), q(%.2f, %.2f, %.2f, %.2f)",
                      tx, ty, tz, qx, qy, qz, qw)

    def publish_loop(self):
        """持续发布 map->odom"""
        while not rospy.is_shutdown():
            self.map_to_odom.header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(self.map_to_odom)
            self.rate.sleep()

def transformToKDL(t):
    """
    将 geometry_msgs/Transform 转成 PyKDL.Frame
    t: geometry_msgs/Transform
    return: PyKDL.Frame
    """
    # 平移
    trans = PyKDL.Vector(t.translation.x,
                         t.translation.y,
                         t.translation.z)
    # 旋转(四元数 -> KDL.Rotation)
    rot = PyKDL.Rotation.Quaternion(t.rotation.x,
                                    t.rotation.y,
                                    t.rotation.z,
                                    t.rotation.w)
    return PyKDL.Frame(rot, trans)

def kdlToTransform(kdl_frame):
    """
    将 PyKDL.Frame 转成 geometry_msgs/Transform
    kdl_frame: PyKDL.Frame
    return: geometry_msgs/Transform
    """
    t = Transform()
    # 平移
    t.translation.x = kdl_frame.p[0]
    t.translation.y = kdl_frame.p[1]
    t.translation.z = kdl_frame.p[2]
    # 旋转 (KDL.Rotation -> 四元数)
    x, y, z, w = kdl_frame.M.GetQuaternion()
    t.rotation.x = x
    t.rotation.y = y
    t.rotation.z = z
    t.rotation.w = w
    return t

if __name__ == "__main__":
    try:
        MapOdomTfBroadcaster()
    except rospy.ROSInterruptException:
        pass
