
# ROS Data Collection Setup Documentation

This documentation provides detailed instructions on setting up a ROS-based data collection system on Ubuntu. 
The system integrates with various sensors like Camera, LiDAR, Radar, and Disdrometer, using ROS and Tkinter for GUI control.

---

## Contents
1. [Dependencies](#dependencies)
2. [Setup ROS Environment](#setup-ros-environment)
3. [File Placement and Permissions](#file-placement-and-permissions)
4. [ROS Package Configuration](#ros-package-configuration)
5. [Running the System](#running-the-system)
6. [Key Functionalities](#key-functionalities)
7. [ROS Topics](#ros-topics)

---

### 1. Dependencies

Make sure you have the following dependencies installed on your Ubuntu system:

- **ROS**: Install your ROS distribution (e.g., ROS Noetic) and source it.
- **Python Libraries**:
  - `rospy`: ROS Python client library
  - `sensor_msgs` and `std_msgs`: ROS message types
  - `cv_bridge`, `opencv-python`: For handling images with OpenCV
  - `numpy`: For data processing
  - `tkinter`: For GUI (included with Python)

Install any missing dependencies using:

```bash
pip install rospy opencv-python numpy
sudo apt-get install ros-noetic-cv-bridge  # For cv_bridge in ROS Noetic
```

### 2. Setup ROS Environment

To integrate this project with ROS:

1. **Create a ROS Package**: Create a package in your ROS workspace to contain these scripts.

    ```bash
    cd ~/catkin_ws/src
    catkin_create_pkg data_collection rospy std_msgs sensor_msgs
    ```

2. **Place Files in the Package**:
   Copy all the project files (`camera.py`, `collection.py`, `disdrometer.py`, `gui.py`, `lidar.py`, `main.py`, and `radar.py`) into the `scripts/` folder within the `data_collection` package.

    ```bash
    mkdir ~/catkin_ws/src/data_collection/scripts
    cp /path/to/your/files/*.py ~/catkin_ws/src/data_collection/scripts/
    ```

3. **Make Scripts Executable**:
   Make sure each script is executable.

    ```bash
    chmod +x ~/catkin_ws/src/data_collection/scripts/*.py
    ```

### 3. ROS Package Configuration

#### CMakeLists.txt

In `CMakeLists.txt`, add:

```cmake
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/camera.py
  scripts/collection.py
  scripts/disdrometer.py
  scripts/gui.py
  scripts/lidar.py
  scripts/radar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### package.xml

In `package.xml`, ensure the following dependencies are listed:

```xml
<depend>rospy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
```

### 4. Build the Package

Navigate to your workspace root and build the package:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 5. Running the System

Run the main script using `rosrun`:

```bash
rosrun data_collection main.py
```

This launches the ROS node `data_collection_node` and opens the Tkinter GUI for controlling data collection.

### 6. Key Functionalities

- **Data Collection Control**: The GUI allows you to start/stop data collection, adjust sampling intervals, and toggle rain status.
- **Conditional Data Saving**: Each sensor (Camera, LiDAR, Radar) checks if data should be saved based on `should_save_data()` in `collection.py`.
- **Force Saving and Rain Check**: A force-save timer and Disdrometer rain status control selective data collection.
- **Data Storage**: Each sensor class saves data to CSV files within `sensor_data/`, creating subfolders for `camera/`, `lidar/`, `radar/`, and `disdrometer/`.

### 7. ROS Topics

The following ROS topics are used for communication between nodes:

- **Published Topics**:
  - `/camera/status`: Status updates from the camera.
  - `/lidar/status`: Status updates from LiDAR.
  - `/radar/status`: Status updates from radar.
  - `/disdrometer/status`: Rain status updates.

- **Subscribed Topics**:
  - `/camera/image_raw`: For receiving raw camera images.
  - `/scan`: LiDAR scan data.
  - `/radar_scan`: Radar point cloud data.
  - `/disdrometer/data`: Data from the disdrometer (precipitation sensor).

---

### Error Handling and Logging

Each class logs data-saving activities and errors using `rospy.loginfo()` for real-time monitoring and debugging.

---

This setup allows ROS to manage and coordinate data collection from multiple sensors, using a GUI to control parameters and states.
