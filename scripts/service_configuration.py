#!/usr/bin/env python3
import rospy
import os

configuration_number = "1xM18652"

def main():
    rospy.init_node('service_configuration', anonymous=True)

    rospy.loginfo("Service package 2: Starting configuration...")
    rospy.sleep(1)

    system_info = os.popen("uname -a").read()
    rospy.loginfo("System Info: " + system_info.strip())

    rospy.sleep(1)
    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))

    rospy.spin()

if __name__ == '__main__':
    main()
