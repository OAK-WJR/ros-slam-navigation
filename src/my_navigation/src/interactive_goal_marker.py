#!/usr/bin/env python
import rospy
import yaml
import os
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseArray, Pose

# 目标点存储文件
WAYPOINTS_FILE = os.path.expanduser("~/catkin_ws/src/my_navigation/config/waypoints.yaml")

# 初始化 ROS 节点
rospy.init_node("interactive_goal_marker")

# 创建 Interactive Marker 服务器
server = InteractiveMarkerServer("interactive_marker_server")

# 创建菜单处理器（用于右键菜单）
menu_handler = MenuHandler()

# 目标点存储（发布用）
waypoints = {}

# **发布目标点**
waypoint_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=10)

def load_waypoints():
    """ 读取上次保存的目标点 """
    if os.path.exists(WAYPOINTS_FILE):
        with open(WAYPOINTS_FILE, "r") as file:
            return yaml.safe_load(file) or {}
    return {}

def save_waypoints():
    """ 保存所有目标点到 YAML 文件 """
    with open(WAYPOINTS_FILE, "w") as file:
        yaml.safe_dump(waypoints, file)
    rospy.loginfo("✅ 目标点已保存")

def publish_waypoints():
    """ 发布所有目标点到 /waypoints 话题 """
    msg = PoseArray()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    for name, coords in waypoints.items():
        pose = Pose()
        pose.position.x = coords["x"]
        pose.position.y = coords["y"]
        quat = tf.transformations.quaternion_from_euler(0, 0, coords["yaw"])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        msg.poses.append(pose)

    waypoint_pub.publish(msg)
    rospy.loginfo("📡 已发布所有目标点到 /waypoints")

def process_feedback(feedback):
    """ 交互回调函数，用户拖动 marker 或旋转后触发 """
    global waypoints
    x = feedback.pose.position.x
    y = feedback.pose.position.y
    yaw = tf.transformations.euler_from_quaternion([
        feedback.pose.orientation.x,
        feedback.pose.orientation.y,
        feedback.pose.orientation.z,
        feedback.pose.orientation.w
    ])[2]

    waypoints[feedback.marker_name] = {"x": x, "y": y, "yaw": yaw}
    rospy.loginfo("🔄 目标点 [%s] 更新: x=%.2f, y=%.2f, yaw=%.2f", feedback.marker_name, x, y, yaw)

    save_waypoints()
    publish_waypoints()

def create_marker(marker_name="goal_marker", x=0.0, y=0.0, yaw=0.0):
    """ 创建交互式标记点 """
    marker = InteractiveMarker()
    marker.header.frame_id = "map"
    marker.name = marker_name
    marker.scale = 0.3

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.01  

    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.controls = []

    # **XY 平面移动控制**
    move_control = InteractiveMarkerControl()
    move_control.name = "move_xy"
    move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE  
    move_control.orientation.w = 1.0
    move_control.orientation.y = 1.0
    move_control.always_visible = True  
    marker.controls.append(move_control)

    # **Z 轴旋转控制**
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_z"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS  
    rotate_control.orientation.w = 1.0
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.y = 1.0
    rotate_control.orientation.z = 0.0
    marker.controls.append(rotate_control)

    # **添加方向箭头**
    arrow_marker = Marker()
    arrow_marker.type = Marker.ARROW  
    arrow_marker.scale.x = 0.1  # **箭头长度**
    arrow_marker.scale.y = 0.02  
    arrow_marker.scale.z = 0.02
    arrow_marker.color.r = 0.0
    arrow_marker.color.g = 1.0  
    arrow_marker.color.b = 0.0
    arrow_marker.color.a = 1.0
    arrow_marker.header.frame_id = "map"
    arrow_marker.pose.position.x = x
    arrow_marker.pose.position.y = y
    arrow_marker.pose.position.z = 0.0  # **让箭头稍微浮起**

    # **确保箭头方向和目标方向一致**
    arrow_marker.pose.orientation.x = quat[0]
    arrow_marker.pose.orientation.y = quat[1]
    arrow_marker.pose.orientation.z = quat[2]
    arrow_marker.pose.orientation.w = quat[3]

    move_control.markers.append(arrow_marker)  

    # **创建球形视觉 marker**
    sphere_marker = Marker()
    sphere_marker.type = Marker.SPHERE
    sphere_marker.scale.x = 0.1
    sphere_marker.scale.y = 0.1
    sphere_marker.scale.z = 0.1
    sphere_marker.color.r = 1.0
    sphere_marker.color.g = 0.0
    sphere_marker.color.b = 0.0
    sphere_marker.color.a = 1.0
    sphere_marker.header.frame_id = "map"
    sphere_marker.pose.position.x = x
    sphere_marker.pose.position.y = y
    sphere_marker.pose.position.z = 0.0  # **确保球体位置正确**

    move_control.markers.append(sphere_marker)  # **添加球形 Marker**

    # **添加文本 Marker（显示目标点名字）**
    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING  # **让文字面向相机**
    text_marker.text = marker_name  # **设置目标点名字**
    text_marker.scale.z = 0.2  # **字体大小**
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0
    text_marker.header.frame_id = "map"
    text_marker.pose.position.x = x  # **名字的位置**
    text_marker.pose.position.y = y
    text_marker.pose.position.z = 0.1  # **降低名字高度**
    
    move_control.markers.append(text_marker)  # **把名字 Marker 添加到控件**

    server.insert(marker, process_feedback)
    server.applyChanges()

def create_menu_button():
    """ 创建 RViz 右键菜单按钮 """
    button_marker = InteractiveMarker()
    button_marker.header.frame_id = "map"
    button_marker.name = "add_goal_menu"
    button_marker.description = "右键 → 添加目标点"
    button_marker.scale = 0.5
    button_marker.pose.position.x = 1.0
    button_marker.pose.position.y = 0.0
    button_marker.pose.position.z = 0.5  

    menu_control = InteractiveMarkerControl()
    menu_control.interaction_mode = InteractiveMarkerControl.MENU
    menu_control.always_visible = True

    button_visual = Marker()
    button_visual.type = Marker.CUBE  
    button_visual.scale.x = 0.3
    button_visual.scale.y = 0.3
    button_visual.scale.z = 0.1
    button_visual.color.r = 0.0  
    button_visual.color.g = 1.0
    button_visual.color.b = 0.0
    button_visual.color.a = 1.0  

    menu_control.markers.append(button_visual)
    button_marker.controls.append(menu_control)

    global menu_handler
    menu_handler.insert("添加新目标点", callback=menu_feedback)

    server.insert(button_marker)
    menu_handler.apply(server, button_marker.name)  
    server.applyChanges()

    rospy.loginfo("✅ 右键菜单按钮已创建")

def menu_feedback(feedback):
    """ 处理菜单点击事件，创建新目标点 """
    global waypoints
    goal_name = "{}".format(len(waypoints)+1)

    rospy.loginfo("✨ 创建新目标点: %s", goal_name)
    create_marker(marker_name=goal_name, x=0.0, y=0.0, yaw=0.0)

    waypoints[goal_name] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    save_waypoints()
    publish_waypoints()

# **🚀 启动流程**
rospy.loginfo("Interactive Markers 服务器已启动")

# **加载并恢复所有目标点**
waypoints = load_waypoints()
for name, coords in waypoints.items():
    create_marker(marker_name=name, x=coords["x"], y=coords["y"], yaw=coords["yaw"])

# **创建交互菜单**
create_menu_button()

# **定期发布目标点**
rate = rospy.Rate(1)  # 1 Hz
while not rospy.is_shutdown():
    publish_waypoints()
    rate.sleep()

rospy.spin()
