#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray

def simple_pwm_test():
    rospy.init_node("simple_pwm_test", anonymous=True)
    
    pwm_pub = rospy.Publisher("/raw_pwm", Int16MultiArray, queue_size=10)
    
    rospy.sleep(1)  # Wait for the publisher to initialize
    
    pwm_msg = Int16MultiArray()
    pwm_msg.data = [1600, 1600]
    rospy.loginfo("Publishing [2000, 2000] to /raw_pwm...") 
    pwm_pub.publish(pwm_msg)

    rospy.loginfo("PWM message sent. Node will keep running to ensure /raw_pwm is active.")
    rospy.spin()  # Block and keep the publisher alive

if __name__ == "__main__":
    try:
        simple_pwm_test()
    except rospy.ROSInterruptException:
        pass
