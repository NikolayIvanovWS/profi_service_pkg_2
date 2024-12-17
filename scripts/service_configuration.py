#!/usr/bin/env python3
import rospy
import os
import psutil
from datetime import datetime
import time
from sensor_msgs.msg import BatteryState

configuration_number = "0xP98767"

def battery_callback(msg):
    if msg.voltage < 4.0:
        rospy.loginfo("No battery, powered by a power supply.")
    else:
        battery_percentage = (msg.voltage - 12.0) / 4.4 * 100  # Assuming 16.4V is 100%
        rospy.loginfo(f"Battery charge: {battery_percentage:.2f}%")

def get_disk_space():
    disk = psutil.disk_usage('/')
    percent_used = disk.percent
    rospy.loginfo(f"Disk Usage: {percent_used}% used out of {disk.total / (1024**3):.2f} GB")
    
def get_system_health():
    rospy.loginfo("Starting system health check...")
    # Checking memory usage
    try:
        memory = psutil.virtual_memory()
        memory_used_percentage = memory.percent
        rospy.loginfo(f"Memory usage: {memory_used_percentage}% of total memory")
    except Exception as e:
        rospy.logwarn(f"Failed to retrieve memory info: {str(e)}")

    # Checking CPU temperature
    try:
        cpu_temp = psutil.sensors_temperatures()['cpu_thermal'][0].current
        rospy.loginfo(f"CPU Temperature: {cpu_temp}°C")
    except Exception as e:
        rospy.logwarn(f"Failed to retrieve CPU temperature: {str(e)}")
    
    # Disk space
    get_disk_space()
    
    rospy.loginfo("System health check completed successfully.")

def main():
    rospy.init_node('service_configuration', anonymous=True)
    rospy.loginfo("Service package 2: Starting configuration...")

    system_info = os.popen("uname -a").read()
    rospy.loginfo("System Info: " + system_info.strip())
    rospy.sleep(1)

    rospy.loginfo("Starting system health check...")
    get_system_health()

    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))

    rospy.signal_shutdown("Script completed successfully")
    rospy.spin()

if __name__ == '__main__':
    main()
