#!/usr/bin/env python3
import rospy
import os
from datetime import datetime
import time


configuration_number = "4xT98765"

def main():
    rospy.init_node('service_configuration', anonymous=True)

    rospy.loginfo("Service package 2: Starting configuration...")
    rospy.sleep(1)


    system_info = os.popen("uname -a").read()
    rospy.loginfo("System Info: " + system_info.strip())
    rospy.sleep(1)


    rospy.loginfo("Displaying current date and time for 10 seconds:")
    for _ in range(10):  
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rospy.loginfo("Current Time: " + current_time)
        time.sleep(1)


    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))
    rospy.spin()

if __name__ == '__main__':
    main()
