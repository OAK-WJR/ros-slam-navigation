#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# 初始化速度变量
linear_speed = 0.1  # 每次按键增加的线速度
angular_speed = 0.1  # 每次按键增加的角速度
linear = 0.0  # 当前线速度
angular = 0.0  # 当前角速度


def get_key():
    """
    从终端读取一个字符，非阻塞
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
    创建 Twist 消息
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
            # 持续发布当前速度
            vel_pub.publish(move_robot(linear, angular))

            # 监听键盘输入
            try:
                key = get_key()
                if key.lower() == 'w':  # 前进
                    linear += linear_speed
                elif key.lower() == 's':  # 后退
                    linear -= linear_speed
                elif key.lower() == 'a':  # 左转
                    angular += angular_speed
                elif key.lower() == 'd':  # 右转
                    angular -= angular_speed
                elif key.lower() == 'q':  # 停止
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
