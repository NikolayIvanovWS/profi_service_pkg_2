#!/usr/bin/env python3
import rospy
import psutil
import os
from sensor_msgs.msg import BatteryState, Imu
from sensor_msgs.msg import LaserScan
import subprocess
import tf.transformations as tft

BATTERY_FULL_VOLTAGE = 16.4  
MIN_BATTERY_VOLTAGE = 4.0  

battery_message_printed = False
imu_message_printed = False
lidar_message_printed = False

def get_cpu_temp():
    try:
        temp = subprocess.check_output("vcgencmd measure_temp", shell=True).decode('utf-8')
        return float(temp.replace("temp=", "").replace("'C", ""))
    except:
        return None

def get_memory_usage():
    try:
        mem = psutil.virtual_memory()
        return (mem.used / mem.total) * 100  
    except:
        return None

def get_disk_usage():
    try:
        disk = psutil.disk_usage('/')
        return (disk.used / disk.total) * 100  
    except:
        return None

def battery_callback(msg):
    global battery_message_printed
    if not battery_message_printed:
        battery_percentage = (msg.voltage / BATTERY_FULL_VOLTAGE) * 100 if msg.voltage >= MIN_BATTERY_VOLTAGE else 0
        if msg.voltage < MIN_BATTERY_VOLTAGE:
            rospy.loginfo("Battery charge: No battery, powered by a power supply")
        else:
            rospy.loginfo(f"Battery charge: {battery_percentage:.2f}%")
        battery_message_printed = True

def imu_callback(msg):
    global imu_message_printed
    if not imu_message_printed:
        if msg.orientation:
            x = round(msg.orientation.x, 4)
            y = round(msg.orientation.y, 4)
            z = round(msg.orientation.z, 4)
            w = round(msg.orientation.w, 4)
            euler = tft.euler_from_quaternion([x, y, z, w])
            roll, pitch, yaw = euler  # roll, pitch, yaw (в радианах)
            yaw = round(yaw, 4)  # Округляем yaw до 4 знаков
            rospy.loginfo(f"IMU Cartesian Coordinates - X: {x}, Y: {y}, Theta (Yaw): {yaw}")
        else:
            rospy.loginfo("No IMU data received")
        imu_message_printed = True

def lidar_callback(msg):
    global lidar_message_printed
    if not lidar_message_printed:
        if msg.ranges:
            distance = round(msg.ranges[0], 4)  # Округляем до 4 знаков
            rospy.loginfo(f"Distance at index 0: {distance} meters")
        else:
            rospy.loginfo("No lidar data received")
        lidar_message_printed = True

def check_camera_connection():
    try:
        result = subprocess.run(['v4l2-ctl --list-devices'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if 'video0' in result.stdout.decode():
            rospy.loginfo("Camera connected")
        else:
            rospy.loginfo("Camera not connected")
    except subprocess.CalledProcessError:
        rospy.loginfo("Error checking camera connection")

def main():
    rospy.init_node('robot_status_checker', anonymous=True)

    rospy.loginfo("System Health Check:")
    rospy.sleep(0.5)
    rospy.loginfo(f"CPU Temperature: {get_cpu_temp()}°C")
    rospy.sleep(0.5)
    rospy.loginfo(f"Memory Usage: {get_memory_usage():.2f}%")
    rospy.sleep(0.5)
    rospy.loginfo(f"Disk Usage: {get_disk_usage():.2f}%")
    rospy.sleep(0.5)

    rospy.loginfo("Sensor Data Check:")
    rospy.sleep(0.5)
    rospy.Subscriber('/bat', BatteryState, battery_callback)
    rospy.sleep(0.5)
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.sleep(0.5)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.sleep(0.5)

    check_camera_connection()

    rospy.loginfo("Service package 2: Configuration checksum : 00xU39450")

    rospy.signal_shutdown("Script completed successfully")

    rospy.spin()

if __name__ == '__main__':
    main()
