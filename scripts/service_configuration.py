#!/usr/bin/env python3
import rospy

# Инициализация переменных
configuration_number = "1xM00001"

def main():
    rospy.init_node('service_configuration', anonymous=True)
    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))
    rospy.spin()

if __name__ == '__main__':
    main()
