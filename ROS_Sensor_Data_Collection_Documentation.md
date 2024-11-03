
# Detailed Documentation: ROS-Based Multi-Sensor Data Collection Program

## Overview

This program collects data from a camera, LiDAR, radar, and disdrometer. Using the Robot Operating System (ROS), the program captures and stores sensor data, organized by timestamp, as CSV files and ROS bag files. A graphical user interface (GUI) built with `tkinter` allows the user to control data collection settings, including sampling intervals, data saving conditions, and system status.

### Key Features:
- **Sensor Data Collection**: Collects data from multiple sensors (camera, LiDAR, radar, and disdrometer).
- **Real-Time Monitoring**: Displays real-time system status and updates.
- **Flexible Storage**: Saves data as CSV files and ROS bag files with customized intervals.
- **Rain Condition Control**: Adjusts data collection based on rain status detected via disdrometer input.

## Dependencies

To run this program, ensure you have the following dependencies installed:

- **ROS**: Required for interfacing with sensor topics.
- **Python Libraries**:
  - `rospy`: ROS Python client library.
  - `sensor_msgs` and `std_msgs`: For handling message types from sensors.
  - `cv_bridge`: For converting ROS Image messages to OpenCV images.
  - `cv2`: OpenCV library for image handling.
  - `rosbag`: To save sensor data as ROS bag files.
  - `numpy`: For data processing.
  - `tkinter`: For GUI components.

### Setup

1. **Ensure ROS is installed and configured**.
2. **Install necessary packages** via `pip`:
   ```bash
   pip install opencv-python numpy
   ```
3. **Install ROS-specific Python libraries**:
   ```bash
   sudo apt-get install ros-<distro>-cv-bridge ros-<distro>-sensor-msgs ros-<distro>-std-msgs
   ```

Replace `<distro>` with your ROS distribution name, e.g., `melodic` or `noetic`.
