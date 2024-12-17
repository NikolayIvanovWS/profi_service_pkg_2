#!/usr/bin/env python3
import rospy
import os
from datetime import datetime
import time
from sensor_msgs.msg import BatteryState

configuration_number = "0xP98767"  

def battery_callback(msg):

    rospy.loginfo(f"Battery voltage: {msg.voltage} V")

def main():
    rospy.init_node('service_configuration', anonymous=True)

    rospy.loginfo("Service package 2: Starting configuration...")
    rospy.sleep(1)

    system_info = os.popen("uname -a").read()
    rospy.loginfo("System Info: " + system_info.strip())
    rospy.sleep(1)

    rospy.loginfo("Displaying current date and time for 5 seconds:")
    for _ in range(5):  
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        rospy.loginfo("Current Time: " + current_time)
        time.sleep(1)

    rospy.Subscriber('/bat', BatteryState, battery_callback)
    time.sleep(0.055)

    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))
    
    rospy.signal_shutdown("Script completed successfully")

    rospy.spin()

if __name__ == '__main__':
    main()
