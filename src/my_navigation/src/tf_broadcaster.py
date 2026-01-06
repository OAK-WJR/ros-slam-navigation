#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def broadcast_transforms():
    rospy.init_node('tf_broadcaster', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Example transform from odom -> base_link
        br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(), "base_link", "odom")
        # Example transform from base_link -> laser
        br.sendTransform((0.1, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(), "laser", "base_link")
        rate.sleep()

if __name__ == "__main__":
    try:
        broadcast_transforms()
    except rospy.ROSInterruptException:
        pass
