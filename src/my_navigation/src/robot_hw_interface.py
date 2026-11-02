#!/usr/bin/env python
import rospy
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class RobotHWInterface:
    def __init__(self):
        rospy.init_node("robot_hw_interface")

        # Left/right wheel speed placeholders
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        # Subscribe to /joint_states to update speeds
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        # Publish /raw_pwm
        self.raw_pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=10)

        # Subscribe to /cmd_vel (from diff_drive_controller)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

    def joint_states_callback(self, msg):
        # Parse /joint_states, update the current speeds
        try:
            self.left_velocity = msg.velocity[msg.name.index("left_wheel_joint")]
            self.right_velocity = msg.velocity[msg.name.index("right_wheel_joint")]
        except ValueError:
            pass  # If the joint name is not found, ignore it

    def cmd_vel_callback(self, msg):
        # Convert /cmd_vel to left/right wheel PWM
        left_pwm = int(1398 + 100 * msg.linear.x - 50 * msg.angular.z)
        right_pwm = int(1398 + 100 * msg.linear.x + 50 * msg.angular.z)

        # Clamp the PWM range
        left_pwm = max(1000, min(2000, left_pwm))
        right_pwm = max(1000, min(2000, right_pwm))

        # Publish to /raw_pwm
        pwm_msg = Int16MultiArray(data=[left_pwm, right_pwm])
        self.raw_pwm_pub.publish(pwm_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    hw = RobotHWInterface()
    hw.spin()
