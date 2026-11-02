#!/usr/bin/env python3

import os
import yaml
import serial
import serial.tools.list_ports
import rospy

def detect_arduino():
    """
    Auto-detect the connected Arduino serial device.
    Returns the detected serial port path, e.g. /dev/ttyUSB0, or None.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 9600, timeout=2)
            ser.write(b'\n')  # Try sending an empty line to trigger an Arduino response
            response = ser.readline().decode('utf-8').strip()
            ser.close()

            if response:  # If there is a response
                print(f"Detected Arduino on {port.device}: {response}")
                return port.device

        except (serial.SerialException, UnicodeDecodeError) as e:
            print(f"Error checking port {port.device}: {e}")

    print("No Arduino detected.")
    return None

def update_or_create_yaml(filename, key, value):
    """
    Update or create config info in a YAML file.

    Parameters:
    - filename: Config file path.
    - key: Top-level key to update (e.g. "motor_control_node").
    - value: Value to write (can be a dict).
    """
    # Make sure the target directory exists
    os.makedirs(os.path.dirname(filename), exist_ok=True)

    # If the file exists, load its current content first
    data = {}
    if os.path.exists(filename):
        with open(filename, 'r') as yaml_file:
            try:
                data = yaml.safe_load(yaml_file) or {}
            except yaml.YAMLError as e:
                print(f"Error reading YAML file: {e}")

    # Update the given key
    data[key] = value

    # Write back to the file
    with open(filename, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)
    print(f"Configuration updated in {filename}")

def main():
    print("Starting Arduino detection...")
    arduino_port = detect_arduino()
    filename = "/home/jetson/catkin_ws/src/my_navigation/config/robot_info.yaml"

    if arduino_port:
        update_or_create_yaml(filename, "arduino", {"serial_port": arduino_port})
        print(f"Arduino detected on {arduino_port}. Configuration saved.")
        rospy.set_param("/serial_port_ready", True)  # Set the param to notify other nodes
    else:
        print("No Arduino detected. YAML file not written.")
        rospy.set_param("/serial_port_ready", False)  # Set the failure state

    # Keep the node running to avoid triggering a ROS system shutdown
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("serial_port_path_detect", anonymous=True)
    main()
