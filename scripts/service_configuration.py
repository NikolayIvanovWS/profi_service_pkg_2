#!/usr/bin/env python3
import rospy
import os
from datetime import datetime
import time
from sensor_msgs.msg import BatteryState, LaserScan

configuration_number = "0xT54237"
start_time = time.time()

battery_data_received = False
laser_data_received = False

def battery_callback(msg):
    global battery_data_received
    if not battery_data_received:
        voltage = msg.voltage
        if voltage >= 16.4:
            battery_percent = 100
        elif voltage < 4:
            rospy.loginfo("No battery, powered by a power supply")
            battery_percent = 0
        else:
            battery_percent = (voltage / 16.4) * 100
        rospy.loginfo(f"Battery charge: {battery_percent:.2f}%")
        battery_data_received = True

def laser_callback(msg):
    global laser_data_received
    if not laser_data_received:
        if 0 < len(msg.ranges):
            range_at_0 = msg.ranges[0]
            rospy.loginfo(f"Laser scan data received. Range at index 0: {range_at_0:.2f} meters")
        else:
            rospy.logwarn("Laser scan data is too short, index 0 is not available.")
        laser_data_received = True

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
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    while not rospy.is_shutdown() and not (battery_data_received and laser_data_received):
        rospy.sleep(0.1)

    rospy.loginfo(f"Service package 2: Configuration checksum : {configuration_number}")

    total_time = time.time() - start_time
    rospy.loginfo(f"Service package 2: Total execution time: {total_time:.2f} seconds")

    rospy.signal_shutdown("Script completed successfully")

if __name__ == '__main__':
    main()
