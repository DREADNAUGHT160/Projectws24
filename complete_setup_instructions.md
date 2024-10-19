
# New Sensor Data Collection System Setup

This document provides step-by-step instructions on how to set up a new ROS workspace, adapt the existing `data_collection` scripts, integrate the radar sensor, and implement a GUI for managing sensors and visualizing data using RViz.

## Prerequisites

Before starting, ensure you have:
- ROS Noetic installed.
- LiDAR (e.g., Hokuyo UTM-30LX), Camera (e.g., Logitech C920), and Radar.
- A properly configured Ubuntu environment.

## Step 1: Create a New ROS Workspace

1. **Create the workspace**:

   ```bash
   mkdir -p ~/new_project_ws/src
   cd ~/new_project_ws
   catkin_make
   ```

2. **Source the workspace**:

   ```bash
   source ~/new_project_ws/devel/setup.bash
   ```

## Step 2: Migrate or Rebuild Required Packages

If you need to migrate packages like `data_collection` from the previous workspace, follow these steps:

1. **Copy the package from the old workspace**:

   ```bash
   cp -r /home/carissma/project_ws/src/data_collection ~/new_project_ws/src/
   ```

2. **Modify the package for new sensors**:

   If new sensors (like radar) are introduced, update the `CMakeLists.txt` to include the new dependencies:

   ```cmake
   find_package(catkin REQUIRED COMPONENTS
     rospy
     std_msgs
     sensor_msgs
     geometry_msgs
     message_generation
     radar_msgs  # Add this line for radar dependencies
   )
   ```

3. **Rebuild the workspace**:

   ```bash
   cd ~/new_project_ws
   catkin_make
   ```

## Step 3: Modify and Adapt Existing Scripts

1. **Edit the `Final_datacollection.py` script** to include data collection from radar:

   ```bash
   cd ~/new_project_ws/src/data_collection/src
   nano Final_datacollection.py
   ```

   Example code:

   ```python
   #!/usr/bin/env python
   import rospy
   from sensor_msgs.msg import Image, LaserScan, PointCloud2

   def camera_callback(data):
       rospy.loginfo("Camera data received")

   def lidar_callback(data):
       rospy.loginfo("LiDAR data received")

   def radar_callback(data):
       rospy.loginfo("Radar data received")

   def final_data_collection():
       rospy.init_node('final_data_collection', anonymous=True)

       # Subscribe to camera topic
       rospy.Subscriber('/usb_cam/image_raw', Image, camera_callback)

       # Subscribe to LiDAR topic
       rospy.Subscriber('/scan', LaserScan, lidar_callback)

       # Subscribe to radar topic
       rospy.Subscriber('/radar/points', PointCloud2, radar_callback)

       rospy.spin()

   if __name__ == '__main__':
       final_data_collection()
   ```

2. **Make the script executable**:

   ```bash
   chmod +x Final_datacollection.py
   ```

## Step 4: Set Up GUI for Managing Sensors

You can create a GUI to manage all sensors and start RViz. We’ll use `rqt` for this.

1. **Install `rqt` and `rqt_launch`**:

   ```bash
   sudo apt-get install ros-noetic-rqt ros-noetic-rqt-launch
   ```

2. **Create a Python GUI script**:

   ```bash
   cd ~/new_project_ws/src/data_collection/src
   nano sensor_control_gui.py
   ```

   Add the following code:

   ```python
   import os
   import rospy
   from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout
   from rqt_gui_py.plugin import Plugin

   class SensorControl(Plugin):
       def __init__(self, context):
           super(SensorControl, self).__init__(context)
           self.setObjectName('SensorControl')

           # Create a QWidget for the GUI
           self._widget = QWidget()
           self._widget.setWindowTitle('Sensor Control')

           # Create buttons for each sensor's data collection
           self.camera_button = QPushButton('Start Camera')
           self.lidar_button = QPushButton('Start LiDAR')
           self.radar_button = QPushButton('Start Radar')
           self.rviz_button = QPushButton('Start RViz')
           self.final_button = QPushButton('Start Final Data Collection')

           # Connect buttons to methods
           self.camera_button.clicked.connect(self.start_camera)
           self.lidar_button.clicked.connect(self.start_lidar)
           self.radar_button.clicked.connect(self.start_radar)
           self.rviz_button.clicked.connect(self.start_rviz)
           self.final_button.clicked.connect(self.start_final_collection)

           # Layout for the buttons
           layout = QVBoxLayout()
           layout.addWidget(self.camera_button)
           layout.addWidget(self.lidar_button)
           layout.addWidget(self.radar_button)
           layout.addWidget(self.rviz_button)
           layout.addWidget(self.final_button)
           self._widget.setLayout(layout)

           # Add the widget to the user interface
           context.add_widget(self._widget)

       def start_camera(self):
           os.system("rosrun usb_cam usb_cam_node _video_device:=/dev/video0")

       def start_lidar(self):
           os.system("rosrun urg_node urg_node")

       def start_radar(self):
           os.system("rosrun radar_pkg radar_node")

       def start_rviz(self):
           os.system("rosrun rviz rviz")

       def start_final_collection(self):
           os.system("rosrun data_collection Final_datacollection.py")
   ```

3. **Make the script executable**:

   ```bash
   chmod +x sensor_control_gui.py
   ```

4. **Build the workspace**:

   ```bash
   cd ~/new_project_ws
   catkin_make
   source devel/setup.bash
   ```

5. **Run the GUI**:

   ```bash
   rosrun rqt_gui rqt_gui
   ```

   You should now see the buttons to control the sensors and RViz.

## Step 5: Visualize Data in RViz

1. **Run `roscore`**:

   ```bash
   roscore
   ```

2. **Use the GUI to start the sensors** and RViz.
   - Click **Start Camera** to start the camera node.
   - Click **Start LiDAR** to start the LiDAR node.
   - Click **Start Radar** to start the radar node.
   - Click **Start RViz** to visualize the data.
   - Click **Start Final Data Collection** to run the `Final_datacollection.py` script.

3. **Add topics to RViz** to visualize the sensor data:
   - **Camera**: Add the `Image` display and set the topic to `/usb_cam/image_raw`.
   - **LiDAR**: Add the `LaserScan` display and set the topic to `/scan`.
   - **Radar**: Add the `PointCloud2` display and set the topic to `/radar/points`.

## Step 6: Verify Data Collection

The data collected from the sensors will be saved in directories organized by Unix timestamps in the `~/new_project_ws/` directory.

## Troubleshooting

- **Launch File Errors**: Ensure that all XML tags are properly formatted in your `sensors_visualization.launch` file.
- **Missing Packages**: If you encounter errors about missing packages like `radar_pkg`, ensure the package is installed, or comment out the radar node.

## Conclusion

This guide provides the steps necessary to set up a new ROS workspace, adapt the existing `data_collection` scripts, and implement a GUI for managing sensors and visualizing data in RViz. Follow these steps carefully, and your system should function as expected.
