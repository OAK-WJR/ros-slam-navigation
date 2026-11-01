#!/usr/bin/env python3
import rospy
import tf
import math
import nav_msgs.msg
import geometry_msgs.msg
from networktables import NetworkTables
import time

# ROS 节点 & Publisher
rospy.init_node("odom_receiver_nt", anonymous=True)
odom_pub = rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=10)
tf_broadcaster = tf.TransformBroadcaster()

def conn_listener(connected, info):
    if connected:
        rospy.loginfo("NetworkTables 已连接: %s", getattr(info, "remote_id", "unknown"))
    else:
        rospy.logwarn("NetworkTables 已断开: %s", getattr(info, "remote_id", "unknown"))

# NetworkTables 初始化（RoboRIO IP，可通过 rosparam 覆盖）
server_ip = rospy.get_param("~server_ip", "10.1.14.2")
table_name = rospy.get_param("~table_name", "odom")
NetworkTables.initialize(server=server_ip)
NetworkTables.addConnectionListener(conn_listener, immediateNotify=True)
nt = NetworkTables.getTable(table_name)

rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # 连接状态提示（节流）
    try:
        if not NetworkTables.isConnected():
            rospy.logwarn_throttle(5.0, "NetworkTables 未连接（server_ip=%s, table=%s），/odom 可能为默认值", server_ip, table_name)
    except Exception:
        pass

    # 从 NT 读取里程计
    timestamp = nt.getNumber('timestamp', rospy.Time.now().to_sec())
    x = nt.getNumber('x', 0.0)
    y = nt.getNumber('y', 0.0)
    theta = nt.getNumber('theta', 0.0)
    vx = nt.getNumber('vx', 0.0)
    omega = nt.getNumber('omega', 0.0)

    # 发布 Odometry 消息
    odom = nav_msgs.msg.Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose = geometry_msgs.msg.Pose(
        geometry_msgs.msg.Point(x, y, 0.0),
        geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))
    )
    odom.twist.twist = geometry_msgs.msg.Twist(
        geometry_msgs.msg.Vector3(vx, 0.0, 0.0),
        geometry_msgs.msg.Vector3(0.0, 0.0, omega)
    )
    odom_pub.publish(odom)

    rospy.loginfo_throttle(1.0, "[NT Odom] x=%.3f y=%.3f theta=%.3f vx=%.3f omega=%.3f (server=%s)",
                           x, y, theta, vx, omega, server_ip)

    # 发布 TF
    tf_broadcaster.sendTransform(
        (x, y, 0),
        tf.transformations.quaternion_from_euler(0, 0, theta),
        rospy.Time.now(),
        "base_link",
        "odom"
    )

    rate.sleep()