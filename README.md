
# Profi Service Package 2

## Overview
This package is designed to log various system and robot information during the execution of the configuration process. It provides details on the robot's battery status, lidar range, system information, and current time. The package also logs the configuration checksum for consistency.

## Features

1. **Battery Charge Reporting**:
   - The battery charge is reported in percentage, based on the voltage measurement. 
   - 16.4 V corresponds to 100% battery charge.
   - If the voltage is below 4 V, the package will output a message indicating that the robot is powered by a power supply.

2. **Laser Data**:
   - The package logs the distance value from the lidar sensor at index 0 of the laser scan data.

3. **System Info**:
   - The system's general information is logged using the `uname -a` command.

4. **Time Reporting**:
   - The current date and time is logged for 5 seconds during the execution of the package.

5. **Configuration Checksum**:
   - A unique checksum for the configuration is logged as part of the final output. This checksum is defined by the `configuration_number`.

## Execution

To run the package, use the following `roslaunch` command:

```bash
roslaunch profi_service_pkg_2 start_configure_2.launch
```

The package will log the following information:

- System information
- Current date and time for 5 seconds
- Battery status (as percentage)
- Laser scan data at index 0
- Configuration checksum

## Dependencies

This package relies on the following ROS messages:

- `sensor_msgs/BatteryState`
- `sensor_msgs/LaserScan`

Ensure that the robot is properly publishing battery and laser scan data.

