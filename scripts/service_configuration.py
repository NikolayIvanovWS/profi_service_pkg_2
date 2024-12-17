#!/usr/bin/env python3
import rospy
import os
import time
from datetime import datetime
from sensor_msgs.msg import BatteryState, LaserScan

configuration_number = "9xL73628"  

battery_received = False
laser_received = False

def battery_callback(msg):
    global battery_received, battery_sub
    if not battery_received:
        rospy.loginfo(f"Battery voltage: {msg.voltage:.2f} V")
        battery_received = True
        battery_sub.unregister()

def laser_callback(msg):
    global laser_received, laser_sub
    if not laser_received:
        rospy.loginfo(f"Laser scan data received. Range at index 100: {msg.ranges[100]} meters")
        laser_received = True
        laser_sub.unregister()

def main():
    global battery_sub, laser_sub

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

    battery_sub = rospy.Subscriber('/bat', BatteryState, battery_callback)
    laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Wait for all data to be received
    while not (battery_received and laser_received):
        rospy.sleep(0.1)

    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))

    rospy.signal_shutdown("Script completed successfully")
    rospy.spin()

if __name__ == '__main__':
    main()
