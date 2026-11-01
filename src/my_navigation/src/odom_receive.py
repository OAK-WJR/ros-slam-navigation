#!/usr/bin/env python
import rospy
import socket
import tf
import math
import nav_msgs.msg
import geometry_msgs.msg

# 端口配置
PORT_ODOM = 8888
BUFFER_SIZE = 1024

rospy.init_node("odom_receiver", anonymous=True)
odom_pub = rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=10)
tf_broadcaster = tf.TransformBroadcaster()

# 创建 TCP 服务器，监听指定端口
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", PORT_ODOM))
server_socket.listen(1)

rospy.loginfo("等待在端口 %d 上接收 Odom 数据...", PORT_ODOM)
conn, addr = server_socket.accept()
rospy.loginfo("已接收来自 %s 的 Odom 连接", addr)

buffer = ""
while not rospy.is_shutdown():
    try:
        # 接收数据并追加到缓冲区
        data = conn.recv(BUFFER_SIZE).decode("utf-8")
        if not data:
            continue
        buffer += data

        # 按换行符拆分数据，得到完整的数据行列表
        lines = buffer.split("\n")
        # 如果最后一行不完整，则保留到下次继续接收
        if buffer[-1] != "\n":
            buffer = lines[-1]
            lines = lines[:-1]
        else:
            buffer = ""

        # 对每一完整行数据进行处理
        for line in lines:
            line = line.strip()
            if not line:
                continue

            # 将 CSV 数据按逗号拆分
            values = line.split(",")
            if len(values) < 7:
                rospy.logwarn("不完整的 Odom 数据: %s", line)
                continue

            # CSV 格式：[timestamp, x, y, yawDeg, vx, vy, angVel]
            try:
                timestamp = float(values[0])
                x = float(values[1])
                y = float(values[2])
                yawDeg = float(values[3])  # 假设角度为度
                vx = float(values[4])
                vy = float(values[5])
                angVel = float(values[6])
            except Exception as parse_e:
                rospy.logwarn("数据转换错误: %s, 行数据: %s", parse_e, line)
                continue

            # 将角度转换为弧度
            yawRad = yawDeg
            quat = tf.transformations.quaternion_from_euler(0, 0, yawRad)

            # 构建 odom 消息
            odom_msg = nav_msgs.msg.Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(*quat)

            # 设置线速度
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            # 设置角速度（将角速度从度/s转换为弧度/s）
            odom_msg.twist.twist.angular.z = angVel

            # 发布消息
            odom_pub.publish(odom_msg)

            # 发布 TF 变换
            # 组合里程计位置和IMU的偏移
            tf_broadcaster.sendTransform(
                (x, y, 0),  # 结合里程计位置和IMU偏移
                tf.transformations.quaternion_from_euler(0, 0, yawRad),  # 结合里程计方向和IMU旋转
                rospy.Time.now(),
                "base_link",
                "odom"
            )

    except Exception as e:
        rospy.logwarn("Odom 数据处理错误: %s", e)
        break

conn.close()
