#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Initialize speed variables
linear_speed = 0.1  # Linear speed added per key press
angular_speed = 0.1  # Angular speed added per key press
linear = 0.0  # Current linear speed
angular = 0.0  # Current angular speed


def get_key():
    """
    Read one character from the terminal, non-blocking
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def move_robot(linear, angular):
    """
    Create a Twist message
    """
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    rospy.loginfo_throttle(1, f"Current motion: linear={linear:.3f}, angular={angular:.3f}")
    return msg


def main():
    global linear, angular
    rospy.init_node("wsad_robot_controller")
    vel_pub = rospy.Publisher("/diff_drive_controller/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Keyboard control activated! Use W/A/S/D to move, Q to stop.")

    try:
        while not rospy.is_shutdown():
            # Keep publishing the current speed
            vel_pub.publish(move_robot(linear, angular))

            # Listen for keyboard input
            try:
                key = get_key()
                if key.lower() == 'w':  # Forward
                    linear += linear_speed
                elif key.lower() == 's':  # Backward
                    linear -= linear_speed
                elif key.lower() == 'a':  # Turn left
                    angular += angular_speed
                elif key.lower() == 'd':  # Turn right
                    angular -= angular_speed
                elif key.lower() == 'q':  # Stop
                    linear = 0.0
                    angular = 0.0
                    rospy.loginfo("Robot stopped. Press any key to continue.")
                else:
                    rospy.loginfo("Unknown command. Use W/A/S/D to move, Q to stop.")
            except Exception:
                pass

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down robot control.")
        vel_pub.publish(move_robot(0.0, 0.0))


if __name__ == "__main__":
    main()
