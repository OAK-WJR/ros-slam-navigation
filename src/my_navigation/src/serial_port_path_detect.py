#!/usr/bin/env python3

import os
import yaml
import serial
import serial.tools.list_ports
import rospy

def detect_arduino():
    """
    自动检测连接的 Arduino 串口设备。
    返回检测到的串口路径，例如 /dev/ttyUSB0 或 None。
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 9600, timeout=2)
            ser.write(b'\n')  # 尝试发送空行，触发 Arduino 响应
            response = ser.readline().decode('utf-8').strip()
            ser.close()

            if response:  # 如果有响应
                print(f"Detected Arduino on {port.device}: {response}")
                return port.device

        except (serial.SerialException, UnicodeDecodeError) as e:
            print(f"Error checking port {port.device}: {e}")

    print("No Arduino detected.")
    return None

def update_or_create_yaml(filename, key, value):
    """
    更新或创建 YAML 文件中的配置信息。

    参数:
    - filename: 配置文件路径。
    - key: 要更新的顶级键（如 "motor_control_node"）。
    - value: 要写入的值（可以是字典类型）。
    """
    # 确保目标目录存在
    os.makedirs(os.path.dirname(filename), exist_ok=True)

    # 如果文件存在，先加载已有内容
    data = {}
    if os.path.exists(filename):
        with open(filename, 'r') as yaml_file:
            try:
                data = yaml.safe_load(yaml_file) or {}
            except yaml.YAMLError as e:
                print(f"Error reading YAML file: {e}")

    # 更新指定的键值
    data[key] = value

    # 写回文件
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
        rospy.set_param("/serial_port_ready", True)  # 设置参数，通知其他节点
    else:
        print("No Arduino detected. YAML file not written.")
        rospy.set_param("/serial_port_ready", False)  # 设置失败状态

    # 保持节点运行以避免触发 ROS 系统关闭
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("serial_port_path_detect", anonymous=True)
    main()
