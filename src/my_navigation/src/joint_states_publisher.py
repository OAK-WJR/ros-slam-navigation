#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import time
from smbus import SMBus
import math

# Initialize I2C bus
bus = SMBus(1)

# Constants
TCA9548A_ADDR = 0x70
AS5600_ADDRESS = 0x36
ANGLE_REGISTER = 0x0E  # Angle register

WHEEL_RADIUS = 33 / 1000.0  # Meters
WHEEL_BASE = 107.5 / 1000.0  # Distance between wheels, meters

ENCODER_MAX_ANGLE = 2 * math.pi  # Assuming 0 to 2π radians

VELOCITY_FILTER_ALPHA = 0.9  # Low-pass filter coefficient
filtered_left_velocity = 0.0
filtered_right_velocity = 0.0

prev_left_angle = 0.0
prev_right_angle = 0.0


def select_channel(channel):
    """Select I2C channel."""
    assert 0 <= channel <= 7, "Channel must be between 0 and 7"
    bus.write_byte(TCA9548A_ADDR, 1 << channel)


def read_encoder():
    """Read encoder angle."""
    try:
        data = bus.read_i2c_block_data(AS5600_ADDRESS, ANGLE_REGISTER, 2)
        raw_angle = (data[0] << 8) | data[1]
        return (raw_angle * ENCODER_MAX_ANGLE) / 4096.0  # Convert to radians
    except Exception as e:
        rospy.logwarn(f"Failed to read encoder: {e}")
        return None


def compute_angle_diff(current, previous):
    """Compute angle difference considering wraparound."""
    diff = current - previous
    diff = (diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]
    return diff


def main():
    global prev_left_angle, prev_right_angle, filtered_left_velocity, filtered_right_velocity

    # Initialize ROS node
    rospy.init_node("joint_states_publisher")

    # Initialize time after ROS is initialized
    prev_time = rospy.Time.now()

    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - prev_time).to_sec()
        if dt <= 0.0001 or dt > 1.0:  # Avoid divide-by-zero or large intervals
            rospy.logwarn_throttle(1.0, f"Invalid time interval dt={dt:.6f}s. Skipping iteration.")
            prev_time = current_time
            continue
        prev_time = current_time

        # Read left encoder
        select_channel(2)
        left_angle = read_encoder()

        # Read right encoder
        select_channel(1)
        right_angle = read_encoder()

        if left_angle is None or right_angle is None:
            continue

        left_angle_diff = compute_angle_diff(left_angle, prev_left_angle)
        right_angle_diff = compute_angle_diff(right_angle, prev_right_angle)

        left_velocity = left_angle_diff / dt
        right_velocity = right_angle_diff / dt  # Assuming opposite direction

        # Smooth velocities
        filtered_left_velocity = VELOCITY_FILTER_ALPHA * filtered_left_velocity + (1 - VELOCITY_FILTER_ALPHA) * left_velocity
        filtered_right_velocity = VELOCITY_FILTER_ALPHA * filtered_right_velocity + (1 - VELOCITY_FILTER_ALPHA) * right_velocity

        prev_left_angle = left_angle
        prev_right_angle = right_angle

        # Publish JointState message
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_state.position = [left_angle, -right_angle]
        joint_state.velocity = [filtered_left_velocity, filtered_right_velocity]
        joint_state.effort = []

        joint_pub.publish(joint_state)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
