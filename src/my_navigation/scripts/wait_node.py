#!/usr/bin/env python

import rospy
import sys

if __name__ == '__main__':
    rospy.init_node('wait_node')
    
    # Get the delay parameter
    wait_time = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    
    rospy.sleep(wait_time)