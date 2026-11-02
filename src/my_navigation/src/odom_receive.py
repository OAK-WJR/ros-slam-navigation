#!/usr/bin/env python
import rospy
import socket
import tf
import math
import nav_msgs.msg
import geometry_msgs.msg

# Port config
PORT_ODOM = 8888
BUFFER_SIZE = 1024

rospy.init_node("odom_receiver", anonymous=True)
odom_pub = rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=10)
tf_broadcaster = tf.TransformBroadcaster()

# Create a TCP server listening on the given port
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", PORT_ODOM))
server_socket.listen(1)

rospy.loginfo("Waiting to receive Odom data on port %d...", PORT_ODOM)
conn, addr = server_socket.accept()
rospy.loginfo("Accepted Odom connection from %s", addr)

buffer = ""
while not rospy.is_shutdown():
    try:
        # Receive data and append it to the buffer
        data = conn.recv(BUFFER_SIZE).decode("utf-8")
        if not data:
            continue
        buffer += data

        # Split the data on newlines to get a list of complete data lines
        lines = buffer.split("\n")
        # If the last line is incomplete, keep it for the next receive
        if buffer[-1] != "\n":
            buffer = lines[-1]
            lines = lines[:-1]
        else:
            buffer = ""

        # Process each complete line of data
        for line in lines:
            line = line.strip()
            if not line:
                continue

            # Split the CSV data on commas
            values = line.split(",")
            if len(values) < 7:
                rospy.logwarn("Incomplete Odom data: %s", line)
                continue

            # CSV format: [timestamp, x, y, yawDeg, vx, vy, angVel]
            try:
                timestamp = float(values[0])
                x = float(values[1])
                y = float(values[2])
                yawDeg = float(values[3])  # Assume the angle is in degrees
                vx = float(values[4])
                vy = float(values[5])
                angVel = float(values[6])
            except Exception as parse_e:
                rospy.logwarn("Data conversion error: %s, line data: %s", parse_e, line)
                continue

            # Convert the angle to radians
            yawRad = yawDeg
            quat = tf.transformations.quaternion_from_euler(0, 0, yawRad)

            # Build the odom message
            odom_msg = nav_msgs.msg.Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(*quat)

            # Set linear velocity
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            # Set angular velocity (convert angular velocity from deg/s to rad/s)
            odom_msg.twist.twist.angular.z = angVel

            # Publish the message
            odom_pub.publish(odom_msg)

            # Publish the TF transform
            # Combine odometry position and IMU offset
            tf_broadcaster.sendTransform(
                (x, y, 0),  # Combine odometry position and IMU offset
                tf.transformations.quaternion_from_euler(0, 0, yawRad),  # Combine odometry heading and IMU rotation
                rospy.Time.now(),
                "base_link",
                "odom"
            )

    except Exception as e:
        rospy.logwarn("Odom data processing error: %s", e)
        break

conn.close()
