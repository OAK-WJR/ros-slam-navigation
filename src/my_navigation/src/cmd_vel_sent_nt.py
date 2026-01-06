#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from networktables import NetworkTables

class CmdVelSenderNT:
    def __init__(self):
        # 初始化 NT（RoboRIO IP，可通过 rosparam 覆盖）
        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.table_name = rospy.get_param("~table_name", "cmd_vel")
        NetworkTables.initialize(server=self.server_ip)
        self.cmd_table = NetworkTables.getTable(self.table_name)

        # 订阅 ROS /cmd_vel 话题
        rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel)

    def cb_cmd_vel(self, msg: Twist):
        # 发布到 NT
        self.cmd_table.putNumber('linear', msg.linear.x)
        self.cmd_table.putNumber('lateral', msg.linear.y)
        self.cmd_table.putNumber('angular', msg.angular.z)
        rospy.logdebug(f"NT({self.server_ip}/{self.table_name}) Send cmd_vel: linear={msg.linear.x}, lateral={msg.linear.y}, angular={msg.angular.z}")

def main():
    rospy.init_node('cmd_vel_sender_nt', anonymous=True)
    sender = CmdVelSenderNT()
    rospy.spin()

if __name__ == '__main__':
    main()