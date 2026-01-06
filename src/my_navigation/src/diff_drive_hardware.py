#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from controller_manager_msgs.srv import LoadController, SwitchController

class DiffDriveHardware:
    def __init__(self):
        rospy.init_node("diff_drive_hardware")

        # 定义关节名称
        self.joint_names = ["left_wheel_joint", "right_wheel_joint"]

        # 关节状态（初始化为 0）
        self.joint_positions = [0.0, 0.0]
        self.joint_velocities = [0.0, 0.0]
        self.joint_efforts = [0.0, 0.0]

        # 速度命令
        self.joint_commands = [0.0, 0.0]

        # 发布和订阅
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.command_subs = [
            rospy.Subscriber("/left_wheel_velocity_controller/command", Float64, self.left_wheel_command_callback),
            rospy.Subscriber("/right_wheel_velocity_controller/command", Float64, self.right_wheel_command_callback)
        ]

    def left_wheel_command_callback(self, msg):
        """左轮速度命令回调"""
        self.joint_commands[0] = msg.data

    def right_wheel_command_callback(self, msg):
        """右轮速度命令回调"""
        self.joint_commands[1] = msg.data

    def publish_joint_states(self):
        """发布关节状态"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions
        joint_state.velocity = self.joint_velocities
        self.joint_state_pub.publish(joint_state)

    def update_joint_states(self, dt):
        """更新关节状态"""
        self.joint_positions[0] += self.joint_commands[0] * dt
        self.joint_positions[1] += self.joint_commands[1] * dt
        self.joint_velocities = self.joint_commands

    def run(self):
        rate = rospy.Rate(50)  # 50Hz
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - prev_time).to_sec()
            prev_time = current_time

            # 更新关节状态并发布
            self.update_joint_states(dt)
            self.publish_joint_states()

            rate.sleep()

if __name__ == "__main__":
    try:
        hardware = DiffDriveHardware()
        hardware.run()
    except rospy.ROSInterruptException:
        pass
