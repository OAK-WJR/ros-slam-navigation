#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray  # Used to publish left/right PWM values
import os
import yaml
import time

class CalibrationNode(object):
    def __init__(self):
        rospy.init_node("calibration_node", anonymous=True)

        # ========== 1. Read/set related parameters ==========
        # PWM max/min values (left and right separate)
        self.pwm_min_left_default  = rospy.get_param("~pwm_min_left_default", 0)
        self.pwm_max_left_default  = rospy.get_param("~pwm_max_left_default", 3000)
        self.pwm_min_right_default = rospy.get_param("~pwm_min_right_default", 0)
        self.pwm_max_right_default = rospy.get_param("~pwm_max_right_default", 3000)
        
        # Initialize PWM range; it will be adjusted by testing later
        self.pwm_min_left  = self.pwm_min_left_default
        self.pwm_max_left  = self.pwm_max_left_default
        self.pwm_min_right = self.pwm_min_right_default
        self.pwm_max_right = self.pwm_max_right_default

        # Step size of each fine adjustment (us)
        self.adjust_step = rospy.get_param("~adjust_step", 1)
        # Speed threshold (within this many m/s counts as "stationary")
        self.speed_thresh = rospy.get_param("~speed_thresh", 0.001)
        # Wheel radius, used to convert angular speed (rad/s) to linear speed (m/s)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.033)

        # The initial center value can be read from the last calibration result or a user-given param; if none, default 1500
        self.calib_left  = rospy.get_param("~init_center_left",  1500)
        self.calib_right = rospy.get_param("~init_center_right", 1500)

        # Holds the current linear speed of the left and right wheels
        self.left_vel = 0.0
        self.right_vel = 0.0

        # ========== 2. Create a Publisher to publish left/right wheel PWM ==========
        # This example uses Int16MultiArray, data=[left_pwm, right_pwm]
        # You can change it to the message type and topic your hardware needs
        self.pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=1)

        # ========== 3. Subscribe to /joint_states to read the current wheel speed ==========
        # Make sure joint_states includes "left_wheel_joint" and "right_wheel_joint"
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        # ========== Start calibration ==========
        rospy.loginfo("[Calibration] Node started. Will test and calibrate left & right wheels...")

        # Wait a moment so /joint_states starts publishing
        rospy.sleep(1.0)

        # Test and set the PWM range
        self.test_pwm_range(is_left=True)
        self.test_pwm_range(is_left=False)

        # Calibrate the left wheel first, then the right wheel
        self.calibrate_wheel(is_left=True)
        self.calibrate_wheel(is_left=False)

        # Calibration done
        rospy.loginfo("[Calibration] Done! Left Center=%d, Right Center=%d",
                      self.calib_left, self.calib_right)
        rospy.loginfo("[Calibration] PWM Ranges: Left [%d, %d], Right [%d, %d]",
                      self.pwm_min_left, self.pwm_max_left,
                      self.pwm_min_right, self.pwm_max_right)

        # Write the results to the parameter server (for other nodes to use)
        rospy.set_param("/motor_control_node/pwm_center_left",  self.calib_left)
        rospy.set_param("/motor_control_node/pwm_center_right", self.calib_right)
        rospy.set_param("/motor_control_node/pwm_min_left", self.pwm_min_left)
        rospy.set_param("/motor_control_node/pwm_max_left", self.pwm_max_left)
        rospy.set_param("/motor_control_node/pwm_min_right", self.pwm_min_right)
        rospy.set_param("/motor_control_node/pwm_max_right", self.pwm_max_right)

        # If needed, can also save to a local file
        self.save_to_config_file(self.calib_left, self.calib_right)

        # After calibration, shut down the node
        rospy.signal_shutdown("Calibration completed.")

    def joint_state_cb(self, msg):
        """
        Get left/right wheel angular speed from JointState => linear speed
        Assume the wheel names in joint_states are 'left_wheel_joint' & 'right_wheel_joint'
        """
        try:
            left_index  = msg.name.index("left_wheel_joint")
            right_index = msg.name.index("right_wheel_joint")
        except ValueError:
            # This message does not have these two joints; ignore it
            return

        left_omega  = msg.velocity[left_index]   # Left wheel angular speed (rad/s)
        right_omega = msg.velocity[right_index]  # Right wheel angular speed (rad/s)

        self.left_vel  = left_omega  * self.wheel_radius  # (m/s)
        self.right_vel = right_omega * self.wheel_radius  # (m/s)

    def test_pwm_range(self, is_left=True):
        """
        Test the PWM range of a single wheel (lowest and highest values).
        Step the PWM up or down and detect whether the wheel starts turning, to find the valid PWM range.
        """
        wheel = "Left" if is_left else "Right"
        rospy.loginfo(f"[Calibration] Testing {wheel} wheel PWM range...")

        # Test the lowest PWM
        rospy.loginfo(f"[Calibration] Testing {wheel} wheel minimum PWM...")
        pwm_min = self.pwm_min_left_default if is_left else self.pwm_min_right_default
        test_pwm_min = self.find_pwm_boundary(is_left, pwm_min, direction='increase')
        if is_left:
            self.pwm_min_left = test_pwm_min
        else:
            self.pwm_min_right = test_pwm_min
        rospy.loginfo(f"[Calibration] {wheel} wheel minimum PWM set to {test_pwm_min}")

        # Test the highest PWM
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
        Find the PWM boundary value: use the speed change trend to detect where speed stops increasing or starts dropping.
        Improvement: add an allowed range for speed change (delta).

        :param is_left: True means left wheel, False means right wheel
        :param start_pwm: Starting PWM value (recommended: the middle value)
        :param direction: 'increase' or 'decrease'
        :return: The boundary PWM value found
        """
        step = 10  # PWM adjustment step
        pwm = start_pwm
        max_attempts = 1000  # Limit on the number of attempts
        prev_speed = 0  # Previous speed, used to compute the speed change
        stable_speed_count = 0  # Stable speed counter, used to detect when the speed stops changing

        # Define the allowed range of speed change
        delta = 0.08  # Allowed speed change range

        rospy.loginfo(f"[Calibration] Testing {'Left' if is_left else 'Right'} wheel, direction: {direction}")

        for attempt in range(max_attempts):
            # Set the PWM value of the current wheel
            if is_left:
                left_pwm = pwm
                right_pwm = self.calib_right  # Right wheel stays unchanged
            else:
                left_pwm = self.calib_left  # Left wheel stays unchanged
                right_pwm = pwm

            # Publish PWM command
            pwm_msg = Int16MultiArray()
            pwm_msg.data = [left_pwm, right_pwm]
            self.pwm_pub.publish(pwm_msg)

            # Wait for the system to respond
            rospy.sleep(0.1)

            # Get the current speed
            current_speed = self.left_vel if is_left else self.right_vel
            speed_diff = abs(current_speed - prev_speed)

            rospy.logdebug(f"[Calibration] PWM: {pwm}, Current Speed: {current_speed:.4f}, Speed Diff: {speed_diff:.4f}")

            # Check whether the speed is outside the allowed change range
            if not (prev_speed - delta <= current_speed <= prev_speed + delta):
                stable_speed_count += 1
                if stable_speed_count >= 3:  # Outside the range 3 times in a row: treat as boundary reached
                    rospy.loginfo(f"[Calibration] Boundary found: PWM = {pwm}, Speed = {current_speed:.4f}")
                    return pwm
            else:
                stable_speed_count = 0  # Reset the stable counter

            # Adjust the PWM value
            if direction == 'increase':
                pwm += step
            elif direction == 'decrease':
                pwm -= step

            # Update the previous speed
            prev_speed = current_speed

            # Keep PWM within a reasonable range
            if is_left:
                pwm = max(self.pwm_min_left_default, min(self.pwm_max_left_default, pwm))
            else:
                pwm = max(self.pwm_min_right_default, min(self.pwm_max_right_default, pwm))

        rospy.logwarn(f"[Calibration] Failed to find PWM boundary after {max_attempts} attempts.")
        return pwm


    def calibrate_wheel(self, is_left=True):
        """
        Calibrate a single wheel:
         - When this wheel's speed > threshold => adjust its center value (up/down)
         - Until speed < threshold => end calibration
         - The other wheel always keeps its center value, to avoid interference
        """
        wheel_name = "Left" if is_left else "Right"
        rospy.loginfo(f"[Calibration] Calibrating {wheel_name} wheel center...")

        r = rospy.Rate(10)  # Loop rate (Hz)
        max_iterations = 3000  # Max iterations, to avoid getting stuck

        for i in range(max_iterations):
            # Set PWM values from the current calibration state
            if is_left:
                left_pwm = self.calib_left
                right_pwm = self.calib_right  # Right wheel stays unchanged
            else:
                left_pwm = self.calib_left  # Left wheel stays unchanged
                right_pwm = self.calib_right

            # Publish PWM command
            pwm_msg = Int16MultiArray()
            pwm_msg.data = [left_pwm, right_pwm]
            self.pwm_pub.publish(pwm_msg)

            rospy.sleep(0.2)

            # Current wheel speed
            current_speed = abs(self.left_vel if is_left else self.right_vel)

            if current_speed < self.speed_thresh:
                rospy.loginfo(f"[Calibration] {wheel_name} wheel speed < threshold => rough calibration done!")
                break
            else:
                # Speed is still above the threshold => find the direction and fine-tune
                if is_left:
                    if self.left_vel > 0:
                        self.calib_left -= self.adjust_step
                        # Make sure it is not below the left wheel minimum
                        self.calib_left = max(self.pwm_min_left, self.calib_left)
                    else:
                        self.calib_left += self.adjust_step
                        # Make sure it does not exceed the left wheel maximum
                        self.calib_left = min(self.pwm_max_left, self.calib_left)
                else:
                    if self.right_vel > 0:
                        self.calib_right -= self.adjust_step
                        # Make sure it is not below the right wheel minimum
                        self.calib_right = max(self.pwm_min_right, self.calib_right)
                    else:
                        self.calib_right += self.adjust_step
                        # Make sure it does not exceed the right wheel maximum
                        self.calib_right = min(self.pwm_max_right, self.calib_right)

            r.sleep()

        # After coarse calibration, run fine calibration
        self.fine_calibration(is_left=is_left)

    def fine_calibration(self, is_left=True, max_iterations=50, 
                         speed_collect_threshold=0.05,  # Only when the wheel speed is below this is it in the fine-calibration range
                         final_adjust_threshold=0.001,  # If the final average speed is above this, fine-tune again
                         adjust_step=1):
        """
        Fine calibration for one wheel:
        1. Use the self.calib_left / self.calib_right already found as the "initial" center PWM
        2. Keep publishing this PWM in a loop and collect all speeds < speed_collect_threshold
        3. Compute the average speed; if it is still too large (positive or negative), do a final fine-tune

        :param is_left: True means calibrate the left wheel; False means calibrate the right wheel
        :param max_iterations: Max number of loops
        :param speed_collect_threshold: Speeds below this threshold are collected for the average
        :param final_adjust_threshold: If the final average speed is above this, fine-tune again
        :param adjust_step: Step size of the final fine-tune
        """
        wheel_name = "left" if is_left else "right"
        rospy.loginfo(f"[Calibration] Start fine calibration for {wheel_name} wheel ...")

        r = rospy.Rate(10)  # 10Hz
        speeds_collected = []

        # 1. Keep sending the "initial" center PWM and record the speeds
        for i in range(max_iterations):
            # Publish PWM command
            pwm_msg = Int16MultiArray()
            if is_left:
                pwm_msg.data = [self.calib_left, self.calib_right]
            else:
                pwm_msg.data = [self.calib_left, self.calib_right]
            self.pwm_pub.publish(pwm_msg)

            rospy.sleep(0.03)
            # Get the current wheel's speed
            current_speed = self.left_vel if is_left else self.right_vel
            rospy.logdebug(f"[Calibration] {wheel_name} wheel speed = {current_speed:.4f} m/s")

            # If the current speed's absolute value is below the threshold, record it
            if abs(current_speed) < speed_collect_threshold:
                speeds_collected.append(current_speed)

            r.sleep()

        # 2. Compute the average speed
        if len(speeds_collected) == 0:
            rospy.logwarn(f"[Calibration] No speed below threshold={speed_collect_threshold:.3f} was collected for {wheel_name} wheel. "
                          "Maybe calibration is not finished or threshold too small.")
            return  # Return directly, or decide yourself whether to do anything else

        avg_speed = sum(speeds_collected) / len(speeds_collected)
        rospy.loginfo(f"[Calibration] {wheel_name} wheel collected {len(speeds_collected)} speeds, avg={avg_speed:.6f} m/s")

        # 3. Fine-tune once more based on the average speed
        #    If avg_speed > 0, the wheel still turns slightly in the positive direction => lower the center PWM a bit more
        #    If avg_speed < 0, the wheel turns slightly in the negative direction => raise the center PWM a bit more
        if abs(avg_speed) > final_adjust_threshold:
            if is_left:
                if avg_speed > 0:
                    rospy.loginfo(f"[Calibration] Avg speed > 0 => reduce left wheel PWM by {adjust_step}")
                    self.calib_left -= adjust_step
                    # Make sure it is not below the left wheel minimum
                    self.calib_left = max(self.pwm_min_left, self.calib_left)
                else:
                    rospy.loginfo(f"[Calibration] Avg speed < 0 => increase left wheel PWM by {adjust_step}")
                    self.calib_left += adjust_step
                    # Make sure it does not exceed the left wheel maximum
                    self.calib_left = min(self.pwm_max_left, self.calib_left)
                rospy.loginfo(f"[Calibration] New left wheel center PWM = {self.calib_left}")
            else:
                if avg_speed > 0:
                    rospy.loginfo(f"[Calibration] Avg speed > 0 => reduce right wheel PWM by {adjust_step}")
                    self.calib_right -= adjust_step
                    # Make sure it is not below the right wheel minimum
                    self.calib_right = max(self.pwm_min_right, self.calib_right)
                else:
                    rospy.loginfo(f"[Calibration] Avg speed < 0 => increase right wheel PWM by {adjust_step}")
                    self.calib_right += adjust_step
                    # Make sure it does not exceed the right wheel maximum
                    self.calib_right = min(self.pwm_max_right, self.calib_right)
                rospy.loginfo(f"[Calibration] New right wheel center PWM = {self.calib_right}")
        else:
            rospy.loginfo(f"[Calibration] Avg speed is small enough (|avg|<= {final_adjust_threshold:.3f}), no final adjustment needed.")

    def save_to_config_file(self, left_val, right_val):
        """
        Save the calibration results to the given YAML config file.

        Parameters:
        - left_val: Left wheel calibrated PWM value.
        - right_val: Right wheel calibrated PWM value.
        """
        config_file = "/home/jetson/catkin_ws/src/my_navigation/config/robot_info.yaml"

        # Make sure the directory exists
        os.makedirs(os.path.dirname(config_file), exist_ok=True)

        # Read the existing config (if it exists)
        config_data = {}
        if os.path.exists(config_file):
            with open(config_file, "r") as f:
                try:
                    config_data = yaml.safe_load(f) or {}
                except yaml.YAMLError as e:
                    rospy.logwarn(f"[Calibration] Failed to parse existing config file: {e}")

        # Update calibration data
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

        # Write the file
        with open(config_file, "w") as f:
            yaml.dump(config_data, f, default_flow_style=False)

        rospy.loginfo(f"[Calibration] Calibration results saved to {config_file}")

def wait_for_joint_states():
    """Wait for /joint_states data to be published"""
    try:
        rospy.wait_for_message("/joint_states", JointState, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /joint_states topic. Exiting.")
        rospy.signal_shutdown("No /joint_states data available.")
        return

def main():
    # Wait for /raw_pwm_ready
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
