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
        #      1. Read from the parameter server
        # -----------------------------
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.033)   # Wheel radius (m)
        self.wheel_base   = rospy.get_param("~wheel_base",   0.1075)   # Distance between left and right wheel centers (m)

        # If the left/right wheels are mounted reversed, params control whether to invert them
        self.reverse_left_wheel  = rospy.get_param("~reverse_left_wheel",  False)
        self.reverse_right_wheel = rospy.get_param("~reverse_right_wheel", False)

        # Set the max allowed delta threshold to filter out large jumps
        self.max_delta_threshold = rospy.get_param("~max_delta_threshold", 1.0)

        # Set the filter window size
        self.filter_window = rospy.get_param("~filter_window", 3)

        # -----------------------------
        #      2. Initialize pose and buffers
        # -----------------------------
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.prev_left_rad  = None
        self.prev_right_rad = None
        self.prev_time      = None

        # Store deltas for the moving average filter
        self.left_deltas  = []
        self.right_deltas = []

        # -----------------------------
        #      3. Create publishers and subscribers
        # -----------------------------
        # Publish odometry
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        # TF broadcaster
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to /joint_states
        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)

    def jointStateCallback(self, msg):
        """
        Get the left/right wheel angles (positions) from JointState,
        compute the robot pose change, publish odometry & TF.
        """
        # Find the left/right wheel indexes
        try:
            left_index  = msg.name.index("left_wheel_joint")
            right_index = msg.name.index("right_wheel_joint")
        except ValueError:
            rospy.logwarn("Cannot find wheel_joint in JointState!")
            return

        # Read the left/right wheel positions
        left_pos  = msg.position[left_index]
        right_pos = msg.position[right_index]

        # If a wheel is mounted reversed, invert the data here
        if self.reverse_left_wheel:
            left_pos = -left_pos
        if self.reverse_right_wheel:
            right_pos = -right_pos

        current_time = rospy.Time.now()
        if self.prev_time is None:
            # First callback: cannot compute speed or displacement, just record
            self.prev_time       = current_time
            self.prev_left_rad   = left_pos
            self.prev_right_rad  = right_pos
            return

        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0.0:
            # Bad time value, return directly
            return

        # Compute the angle difference between the two readings
        delta_left  = left_pos  - self.prev_left_rad
        delta_right = right_pos - self.prev_right_rad

        # Update the previous values
        self.prev_left_rad  = left_pos
        self.prev_right_rad = right_pos
        self.prev_time      = current_time

        # -----------------------------
        #   4. Data filtering & outlier detection
        # -----------------------------
        # (1) Moving average filter
        self.left_deltas.append(delta_left)
        self.right_deltas.append(delta_right)
        if len(self.left_deltas)  > self.filter_window:
            self.left_deltas.pop(0)
        if len(self.right_deltas) > self.filter_window:
            self.right_deltas.pop(0)

        # Average the deltas in the window
        delta_left  = sum(self.left_deltas)  / len(self.left_deltas)
        delta_right = sum(self.right_deltas) / len(self.right_deltas)

        # (2) Outlier filtering
        if abs(delta_left) > self.max_delta_threshold or abs(delta_right) > self.max_delta_threshold:
            rospy.logwarn("Abnormal wheel delta detected, skipping this update.")
            return

        # -----------------------------
        #      5. Compute odometry
        # -----------------------------
        # Linear displacement of each wheel
        dist_left  = delta_left  * self.wheel_radius
        dist_right = delta_right * self.wheel_radius

        # Average linear displacement
        dist = (dist_left + dist_right) / 2.0

        # Rotation angle (radians)
        theta = (dist_left - dist_right) / self.wheel_base

        # ---- Exact Integration ----
        if abs(theta) < 1e-9:
            # Approximately straight-line motion
            self.x += dist * math.cos(self.yaw)
            self.y += dist * math.sin(self.yaw)
        else:
            # Arc motion
            radius = dist / theta
            self.x += radius * (math.sin(self.yaw + theta) - math.sin(self.yaw))
            self.y += radius * (math.cos(self.yaw) - math.cos(self.yaw + theta))

        # Update the yaw angle
        self.yaw += theta
        # Normalize yaw to -pi ~ pi
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Compute linear & angular speed
        vx  = dist / dt
        vy  = 0.0
        vth = theta / dt

        # -----------------------------
        #      6. Publish odometry
        # -----------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id  = "base_link"

        # Position (x, y)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (yaw -> quaternion)
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(*q)

        # Velocity
        odom_msg.twist.twist.linear.x  = vx
        odom_msg.twist.twist.linear.y  = vy
        odom_msg.twist.twist.angular.z = vth

        # (Optional) set the odometry covariance
        # Can be adjusted to fit the real situation
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

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # -----------------------------
        #      7. Publish TF (odom->base_link)
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

        rospy.loginfo_throttle(1.0,  # Throttled: print once per 1 s
            f"[Odom] x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f} deg, vx={vx:.3f}, vth={vth:.3f}"
        )

def main():
    rospy.init_node("diffdrive_odom_publisher")
    rospy.loginfo("DiffDrive Odom Publisher started.")
    node = DiffDriveOdomPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
