#!/usr/bin/env python
import rospy
import socket
import math

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
import tf  # ROS 1 使用 tf 而非 tf2_ros
from tf.transformations import quaternion_from_euler


class IMUReceiverNode:
    def __init__(self):
        rospy.init_node('imu_receiver_node')

        # 创建Publisher
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 启动 Socket server (监听 0.0.0.0:9999, 等待 RoboRio 连进来)
        self.host = '0.0.0.0'
        self.port = 9999
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        rospy.loginfo(f'Listening on {self.host}:{self.port}...')

        self.client_socket = None

        # 主循环
        self.run()

    def run(self):
        rate = rospy.Rate(1000)  # 每秒检查 1000 次
        while not rospy.is_shutdown():
            self.check_connection()
            rate.sleep()

    def check_connection(self):
        # 如果还没连接, accept 一次
        if self.client_socket is None:
            try:
                self.client_socket, addr = self.server_socket.accept()
                rospy.loginfo(f'Accepted connection from {addr}')
                self.client_socket.setblocking(False)
            except BlockingIOError:
                # 没有新的连接就算了
                return

        # 如果已经连接, 尝试读数据
        if self.client_socket:
            try:
                data = self.client_socket.recv(4096)
                if not data:
                    self.client_socket.close()
                    self.client_socket = None
                    rospy.logwarn('Client disconnected.')
                    return

                lines = data.decode('utf-8').strip().split('\n')
                for line in lines:
                    if line:
                        self.process_csv_line(line)
            except BlockingIOError:
                pass
            except Exception as e:
                rospy.logerr(f'Recv error: {e}')
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None

    def process_csv_line(self, line: str):
        vals = line.split(',')
        if len(vals) != 13:
            rospy.logwarn(f'CSV parse error, expect 13 fields, got {len(vals)}: {line}')
            return
        try:
            timestamp_s = float(vals[0])
            ax = float(vals[1])
            ay = float(vals[2])
            az = float(vals[3])
            gx_deg = float(vals[4])
            gy_deg = float(vals[5])
            gz_deg = float(vals[6])
            mx = float(vals[7])
            my = float(vals[8])
            mz = float(vals[9])
            rollDeg = float(vals[10])
            pitchDeg = float(vals[11])
            yawDeg = float(vals[12])

            gx = math.radians(gx_deg)
            gy = math.radians(gy_deg)
            gz = math.radians(gz_deg)

            roll = math.radians(rollDeg)
            pitch = math.radians(pitchDeg)
            yaw = math.radians(yawDeg)

            q = quaternion_from_euler(roll, pitch, yaw)

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            self.imu_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6

            self.mag_pub.publish(mag_msg)

            #self.tf_broadcaster.sendTransform(
            #    (0.0, 0.0, 0.0),  # 位移
            #    q,  # 四元数
            #    rospy.Time.now(),
            #    'imu_link',  # 子坐标系
            #    'base_link'  # 父坐标系
            #)

        except ValueError:
            rospy.logwarn(f'Value error parsing CSV line: {line}')
        except Exception as e:
            rospy.logerr(f'Unexpected error in process_csv_line: {e}')


if __name__ == '__main__':
    try:
        IMUReceiverNode()
    except rospy.ROSInterruptException:
        pass
