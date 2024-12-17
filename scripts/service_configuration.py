#!/usr/bin/env python3
import rospy
import os
import time
from sensor_msgs.msg import BatteryState

configuration_number = "0xP98767"
battery_received = False  

def battery_callback(msg):
    global battery_received

    if not battery_received:
        battery_received = True
        if msg.voltage < 4.0:
            rospy.loginfo("No battery, powered by a power supply")
        else:
            battery_percentage = ((msg.voltage - 12.0) / 4.4) * 100  
            rospy.loginfo(f"Battery charge: {battery_percentage:.2f}%")

        rospy.signal_shutdown("Battery information received")

def check_cpu_temp():
    try:
        cpu_temp = os.popen("vcgencmd measure_temp").read()
        rospy.loginfo(f"CPU Temperature: {cpu_temp.strip()}")
    except Exception as e:
        rospy.logerr(f"Failed to retrieve CPU temperature: {e}")

def check_memory():
    try:
        memory_info = os.popen("free -h").read().splitlines()[1] 
        total, used, free = memory_info.split()[1:4]  
        
        total = float(total.rstrip('GiMi'))
        used = float(used.rstrip('GiMi'))
        
        used_percentage = (used / total) * 100  
        rospy.loginfo(f"Memory usage: {used_percentage:.2f}%")
    except Exception as e:
        rospy.logerr(f"Failed to retrieve memory info: {e}")

def check_system_health():
    check_cpu_temp()
    check_memory()

def main():
    rospy.init_node('service_configuration', anonymous=True)

    rospy.loginfo("Service package 2: Starting configuration...")
    rospy.sleep(1)

    system_info = os.popen("uname -a").read()
    rospy.loginfo("System Info: " + system_info.strip())
    rospy.sleep(1)

    rospy.Subscriber('/bat', BatteryState, battery_callback)
    time.sleep(0.05)  

    check_system_health()

    rospy.loginfo("Service package 2: Configuration checksum : {}".format(configuration_number))

    rospy.signal_shutdown("Script completed successfully")

    rospy.spin()

if __name__ == '__main__':
    main()
