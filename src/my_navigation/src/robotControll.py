#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial
import time

class MotorControlNode:
    def __init__(self):
        rospy.init_node('motor_control_node')
        
        # Can be set on the ROS parameter server, or hard-coded
        self.serial_port_name = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.timeout_duration = rospy.get_param('~timeout_duration', 0.5)  # Timeout (seconds)

        # Serial port init
        try:
            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.serial_port.flush()
            rospy.loginfo(f"Successfully connected to serial port {self.serial_port_name}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port {self.serial_port_name}: {e}")
            rospy.signal_shutdown("Serial port initialization failed")  # If init fails, shut down the node right away
            return

        # Motor center values
        self.center_left = 1525
        self.center_right = 1545

        # Record when cmd_vel was last received; starts as the current time
        self.last_cmd_time = time.time()

        # Subscribe to the /cmd_vel topic
        self.subscriber = rospy.Subscriber(
            '/cmd_vel',
            Twist,
            self.cmd_vel_callback,
            queue_size=10
        )
        rospy.loginfo("Motor Control Node has started and subscribed to /cmd_vel")

        # Set a timer to check for timeout periodically
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timeout_check_callback)  # Runs once every 0.1s

    def cmd_vel_callback(self, msg):
        # Update last_cmd_time
        self.last_cmd_time = time.time()

        # Get linear and angular speed from /cmd_vel
        linear_x = msg.linear.x  # Forward/backward speed
        angular_z = msg.angular.z  # Turning speed

        # Convert to left/right motor PWM values
        left_speed = int(self.center_left + linear_x * 200 + angular_z * 100)
        right_speed = int(self.center_right + linear_x * 200 - angular_z * 100)  # Note: changed to a minus sign here

        # Limit PWM to the 1000-2000 microsecond range
        left_speed = max(1000, min(2000, left_speed))
        right_speed = max(1000, min(2000, right_speed))

        # Send to the Arduino
        self.send_pwm(left_speed, right_speed)

    def timeout_check_callback(self, event):
        """
        Timer callback: check whether no new command has arrived within the timeout.
        If timed out, send the center values (stop command) to the motors.
        """
        current_time = time.time()
        if (current_time - self.last_cmd_time) > self.timeout_duration:
            rospy.logwarn("No new /cmd_vel received. Stopping motors.")
            self.send_pwm(self.center_left, self.center_right)

    def send_pwm(self, left_speed, right_speed):
        """
        Wraps sending PWM in one place, with serial port exception handling
        """
        try:
            command = f"{left_speed} {right_speed}\n"
            self.serial_port.write(command.encode('utf-8'))
            rospy.loginfo(f"Sent to Arduino: {command.strip()}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to write to serial port: {e}")

    def shutdown(self):
        """
        On node shutdown: stop the motors and close the serial port
        """
        rospy.loginfo("Shutting down motor control node...")
        # Stop the motors
        self.send_pwm(self.center_left, self.center_right)

        # Stop the timer
        self.timer.shutdown()

        if self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("Serial port closed.")

def main():
    try:
        node = MotorControlNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
