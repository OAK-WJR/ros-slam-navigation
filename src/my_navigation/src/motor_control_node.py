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
        # 1. Initialize the ROS node
        rospy.init_node("cmd_vel_to_raw_pwm_node", anonymous=True)

        # 2. Get the config_file path from the parameter server, or use the default path
        config_file = rospy.get_param(
            "~config_file", 
            "/home/jetson/catkin_ws/src/my_navigation/config/robot_info.yaml"
        )

        # Read the config file to get the motor center PWM values, etc.
        self.pwm_center_left = 1500
        self.pwm_center_right = 1500
        self.serial_port_name = "/dev/ttyUSB0"   # Fallback serial port name

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

        # 3. Robot body parameters (must match the real robot)
        self.wheel_radius = 0.033   # Wheel radius (m)
        self.wheel_base   = 0.100   # Distance between left and right wheel centers (m)

        # 4. Simple speed-to-PWM conversion factors (from real measurement or calibration)
        #   Assume linear speed (m/s) maps to a PWM offset through some factor (e.g. 100 PWM per 1 m/s)
        #   Adjust based on real measurement
        self.K_v = 100.0  # Linear speed -> PWM increment factor
        self.K_w = 50.0   # Angular speed -> PWM increment factor

        # 5. Set up a simple left/right wheel speed gain for "correction" (demo only, could be improved to PID)
        #   If the left wheel is faster than desired, lower its PWM a bit; same for the right wheel
        #   This only uses a simple P (proportional) control example
        self.Kp = 50.0   # Proportional gain from speed error -> PWM correction (needs your own tuning)

        # 6. Initialize storage for the currently measured left/right wheel angular speeds
        self.left_vel_meas  = 0.0
        self.right_vel_meas = 0.0

        # 7. Publisher: publish the computed left/right motor PWM commands to the /raw_pwm topic
        self.raw_pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=10)

        # 8. Subscribe to /cmd_vel and /joint_states
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        rospy.loginfo("[cmd_vel_to_raw_pwm] Node is initialized.")

    def joint_states_callback(self, msg):
        """
        Callback: subscribes to /joint_states and gets the current actual angular speed of the left and right wheels (rad/s)
        Assume msg.name = ["left_wheel_joint", "right_wheel_joint"]
        msg.velocity holds the left/right wheel angular speeds (rad/s), which must be converted to linear speed (m/s)
        """
        try:
            # Read in order
            idx_left  = msg.name.index("left_wheel_joint")
            idx_right = msg.name.index("right_wheel_joint")

            # joint_states gives angular speed (rad/s); linear speed (m/s) = angular speed * wheel radius
            self.left_vel_meas  = msg.velocity[idx_left]  * self.wheel_radius
            self.right_vel_meas = msg.velocity[idx_right] * self.wheel_radius
        except ValueError:
            # If the matching index is not found, ignore it
            pass

    def cmd_vel_callback(self, msg):
        """
        Callback: subscribes to /cmd_vel
        From the desired linear speed (v) and angular speed (w),
        first compute the "ideal" left/right wheel linear speeds, then convert them to PWM, then correct using the error between the measured and target speeds.
        Finally publish to /raw_pwm.
        """
        # 1. Get the desired linear and angular speed from /cmd_vel
        v = msg.linear.x   # Unit: m/s
        w = msg.angular.z  # Unit: rad/s

        # 2. Theoretical left/right wheel linear speeds
        #    left_speed  = v - w*(wheel_base/2)
        #    right_speed = v + w*(wheel_base/2)
        #    No extra limiting here; you can add your own limits
        left_target_speed  = v - (w * (self.wheel_base / 2.0))
        right_target_speed = v + (w * (self.wheel_base / 2.0))

        # 3. Convert the target linear speed to a "base PWM increment" (without the correction)
        #    Note: this depends heavily on real measurement; you need to tune K_v, K_w yourself
        #    For example: left_pwm = pwm_center_left + K_v * left_target_speed
        #           right_pwm = pwm_center_right + K_v * right_target_speed
        #    To separate the linear and angular speed increments you can split them, but this is simplified here
        left_pwm_base  = self.pwm_center_left  + int(self.K_v * left_target_speed)
        right_pwm_base = self.pwm_center_right + int(self.K_v * right_target_speed)

        # 4. Simple proportional correction from the error between actual and target speed
        #    err_left  = left_target_speed  - left_vel_meas
        #    err_right = right_target_speed - right_vel_meas
        err_left  = left_target_speed  - self.left_vel_meas
        err_right = right_target_speed - self.right_vel_meas

        # Convert the speed error to a PWM correction
        pwm_offset_left  = int(self.Kp * err_left)
        pwm_offset_right = int(self.Kp * err_right)

        # Final PWM
        left_pwm  = left_pwm_base  + pwm_offset_left
        right_pwm = right_pwm_base + pwm_offset_right

        # 5. Clamp so PWM stays in [1000, 2000]
        left_pwm  = max(1000, min(2000, left_pwm))
        right_pwm = max(1000, min(2000, right_pwm))

        # 6. Pack the message and publish it to /raw_pwm
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
        # Wait for /serial_port_ready (if needed, follow how raw_pwm.py does it)
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
