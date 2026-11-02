#!/usr/bin/env python
import rospy
import yaml
import os
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseArray, Pose

# Goal point storage file
WAYPOINTS_FILE = os.path.expanduser("~/catkin_ws/src/my_navigation/config/waypoints.yaml")

# Initialize the ROS node
rospy.init_node("interactive_goal_marker")

# Create the Interactive Marker server
server = InteractiveMarkerServer("interactive_marker_server")

# Create the menu handler (for the right-click menu)
menu_handler = MenuHandler()

# Goal point storage (for publishing)
waypoints = {}

# **Publish goal points**
waypoint_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=10)

def load_waypoints():
    """ Read the goal points saved last time """
    if os.path.exists(WAYPOINTS_FILE):
        with open(WAYPOINTS_FILE, "r") as file:
            return yaml.safe_load(file) or {}
    return {}

def save_waypoints():
    """ Save all goal points to a YAML file """
    with open(WAYPOINTS_FILE, "w") as file:
        yaml.safe_dump(waypoints, file)
    rospy.loginfo("✅ Goal points saved")

def publish_waypoints():
    """ Publish all goal points to the /waypoints topic """
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
    rospy.loginfo("📡 Published all goal points to /waypoints")

def process_feedback(feedback):
    """ Interaction callback, triggered after the user drags or rotates the marker """
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
    rospy.loginfo("🔄 Goal point [%s] updated: x=%.2f, y=%.2f, yaw=%.2f", feedback.marker_name, x, y, yaw)

    save_waypoints()
    publish_waypoints()

def create_marker(marker_name="goal_marker", x=0.0, y=0.0, yaw=0.0):
    """ Create an interactive marker """
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

    # **XY plane move control**
    move_control = InteractiveMarkerControl()
    move_control.name = "move_xy"
    move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE  
    move_control.orientation.w = 1.0
    move_control.orientation.y = 1.0
    move_control.always_visible = True  
    marker.controls.append(move_control)

    # **Z axis rotation control**
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_z"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS  
    rotate_control.orientation.w = 1.0
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.y = 1.0
    rotate_control.orientation.z = 0.0
    marker.controls.append(rotate_control)

    # **Add a direction arrow**
    arrow_marker = Marker()
    arrow_marker.type = Marker.ARROW  
    arrow_marker.scale.x = 0.1  # **Arrow length**
    arrow_marker.scale.y = 0.02  
    arrow_marker.scale.z = 0.02
    arrow_marker.color.r = 0.0
    arrow_marker.color.g = 1.0  
    arrow_marker.color.b = 0.0
    arrow_marker.color.a = 1.0
    arrow_marker.header.frame_id = "map"
    arrow_marker.pose.position.x = x
    arrow_marker.pose.position.y = y
    arrow_marker.pose.position.z = 0.0  # **Lift the arrow slightly**

    # **Make sure the arrow points the same way as the goal**
    arrow_marker.pose.orientation.x = quat[0]
    arrow_marker.pose.orientation.y = quat[1]
    arrow_marker.pose.orientation.z = quat[2]
    arrow_marker.pose.orientation.w = quat[3]

    move_control.markers.append(arrow_marker)  

    # **Create a sphere visual marker**
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
    sphere_marker.pose.position.z = 0.0  # **Make sure the sphere position is correct**

    move_control.markers.append(sphere_marker)  # **Add the sphere Marker**

    # **Add a text Marker (shows the goal point name)**
    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING  # **Make the text face the camera**
    text_marker.text = marker_name  # **Set the goal point name**
    text_marker.scale.z = 0.2  # **Font size**
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0
    text_marker.header.frame_id = "map"
    text_marker.pose.position.x = x  # **Name position**
    text_marker.pose.position.y = y
    text_marker.pose.position.z = 0.1  # **Lower the name height**
    
    move_control.markers.append(text_marker)  # **Add the name Marker to the control**

    server.insert(marker, process_feedback)
    server.applyChanges()

def create_menu_button():
    """ Create the RViz right-click menu button """
    button_marker = InteractiveMarker()
    button_marker.header.frame_id = "map"
    button_marker.name = "add_goal_menu"
    button_marker.description = "Right-click → Add goal point"
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
    menu_handler.insert("Add new goal point", callback=menu_feedback)

    server.insert(button_marker)
    menu_handler.apply(server, button_marker.name)  
    server.applyChanges()

    rospy.loginfo("✅ Right-click menu button created")

def menu_feedback(feedback):
    """ Handle the menu click event: create a new goal point """
    global waypoints
    goal_name = "{}".format(len(waypoints)+1)

    rospy.loginfo("✨ Creating new goal point: %s", goal_name)
    create_marker(marker_name=goal_name, x=0.0, y=0.0, yaw=0.0)

    waypoints[goal_name] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    save_waypoints()
    publish_waypoints()

# **🚀 Startup**
rospy.loginfo("Interactive Markers server started")

# **Load and restore all goal points**
waypoints = load_waypoints()
for name, coords in waypoints.items():
    create_marker(marker_name=name, x=coords["x"], y=coords["y"], yaw=coords["yaw"])

# **Create the interactive menu**
create_menu_button()

# **Publish goal points periodically**
rate = rospy.Rate(1)  # 1 Hz
while not rospy.is_shutdown():
    publish_waypoints()
    rate.sleep()

rospy.spin()
