
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

## Directory Structure

Data collected by this program is saved in directories under a base folder called `sensor_data`. Each run creates a subfolder within `sensor_data` organized by timestamp.

**Directory Tree:**
```
sensor_data/
│
├── camera/                  # Camera image files
│   └── run_<timestamp>/
│       ├── camera_<timestamp>.jpg
│
├── csv_data/                # CSV files for each sensor data type
│   └── run_<timestamp>/
│       ├── lidar/           # LiDAR data as CSV
│       ├── radar/           # Radar data as CSV
│       └── disdrometer/     # Disdrometer data as CSV
│
└── bag_data/                # ROS bag files
    ├── lidar/               # LiDAR data as bag files
    └── radar/               # Radar data as bag files
```

Each run is uniquely identified by a timestamp, ensuring a structured and organized collection of data.

## Code Structure and Documentation

The program contains two main classes:
- **DataCollection**: Manages sensor data collection and saving logic.
- **DataCollectionGUI**: Provides a user interface for setting parameters and starting/stopping data collection.

### Class: `DataCollection`

The `DataCollection` class encapsulates the core logic for data collection, data handling, and ROS interaction.

#### Constructor: `__init__(self, gui)`

Initializes necessary variables, creates directories, and sets up ROS subscriptions for each sensor.

**Parameters**:
- `gui`: Reference to the GUI instance, allowing the class to update GUI elements based on program state.

**Attributes**:
- `self.base_data_dir`: Root directory for storing sensor data.
- `self.sensor_dirs`: Dictionary of directories specific to each sensor type.
- `self.csv_files`: Paths to CSV files for each sensor.
- `self.bag_files`: Paths to ROS bag files for LiDAR and radar.
- `self.bridge`: OpenCV bridge for converting ROS Image messages to OpenCV format.
- `self.is_raining`: Boolean to track rain status based on disdrometer data.
- `self.initialized`: Indicates if data collection has started after the first disdrometer reading.
- `self.is_running`: Controls whether data collection is active.

#### Method: `initialize_csv_files()`

Creates CSV files for LiDAR, radar, and disdrometer data, writing column headers if the files are empty.

**Headers**:
- **LiDAR**: Time, Header Stamp, Header Frame ID, Angle Min, Angle Max, etc.
- **Radar**: Time, Header Stamp, Header Frame ID, Average X, Average Y, etc.
- **Disdrometer**: Time, Precipitation Type, Intensity

#### Method: `append_to_csv(sensor, data)`

Appends a row of data to the CSV file corresponding to the specified sensor.

**Parameters**:
- `sensor`: The sensor type (e.g., `lidar`, `radar`, or `disdrometer`).
- `data`: A list of values to be appended as a row in the CSV file.

#### Method: `start_collection()`

Sets `is_running` to `True`, allowing sensor callbacks to start saving data. It updates the GUI to reflect the collection status.

#### Method: `stop_collection()`

Sets `is_running` to `False`, halting data collection and updating the GUI accordingly.

#### Method: `toggle_rain_status()`

Toggles the `is_raining` flag and updates the GUI based on the rain status.

#### Callback Methods for Each Sensor

- **`camera_callback(msg)`**: Called when a camera image is received. Saves the image at intervals defined by `camera_interval`.
  
  **Parameters**:
    - `msg`: ROS Image message.
  
  **Process**:
    1. Converts the ROS image message to an OpenCV format using `self.bridge.imgmsg_to_cv2`.
    2. Saves the image to `self.sensor_dirs["camera"]` as a `.jpg` file.

- **`lidar_callback(msg)`**: Called when LiDAR data is received. Processes and saves data to the CSV file.
  
  **Parameters**:
    - `msg`: ROS LaserScan message.
  
  **Process**:
    1. Aggregates relevant LiDAR scan details (average distance and intensity).
    2. Appends this information as a row in the LiDAR CSV file.

- **`radar_callback(msg)`**: Called when radar data is received. Processes radar points and stores them in CSV.
  
  **Parameters**:
    - `msg`: ROS PointCloud2 message.
  
  **Process**:
    1. Reads radar points (x, y, z, intensity).
    2. Computes average values and appends them to the radar CSV file.

- **`callback_disdrometer(msg)`**: Called when disdrometer data is received, parsing rain intensity.

  **Parameters**:
    - `msg`: ROS String message containing precipitation data.
  
  **Process**:
    1. Parses precipitation type and intensity.
    2. Sets `is_raining` to `True` if intensity is above 0.
    3. Appends the disdrometer data to its respective CSV file.

#### Method: `should_save_data()`

Evaluates if data should be saved based on rain conditions, initialization, and the force-save interval.

### Class: `DataCollectionGUI`

The `DataCollectionGUI` class provides a `tkinter` interface for configuring data collection parameters and displaying status updates.

#### Constructor: `__init__(self, root)`

Initializes the GUI components, creates GUI widgets, and starts ROS in a separate thread.

**Attributes**:
- `self.lidar_sample_interval`: Interval for LiDAR data sampling.
- `self.radar_sample_interval`: Interval for radar data sampling.
- `self.camera_interval`: Interval for capturing camera images.
- `self.force_save_interval`: Interval for force-saving data regardless of rain.
- `self.use_disdrometer`: Enables/disables using disdrometer data to control data saving.

#### Method: `create_widgets()`

Sets up GUI layout, input fields, and status indicators. Configures controls for starting, stopping, and resetting data collection.

#### GUI Control Methods

These methods provide specific controls for data collection, accessible via the GUI:
- **`start_data_collection()`**: Activates data collection.
- **`stop_data_collection()`**: Stops data collection.
- **`restart_data_collection()`**: Stops and restarts data collection.
- **`toggle_rain_status()`**: Toggles rain status, allowing manual control.

#### Status Update Methods

- **`update_status(message, color)`**: Updates the main status label.
- **`update_disdrometer_status(message)`**: Displays the current rain status.
- **`update_saving_status(sensor, status)`**: Shows the save status of each sensor (e.g., Saving or Idle).

## Running the Program

1. **Start the ROS Master**: Run the ROS master with:
   ```bash
   roscore
   ```
2. **Execute the Script**: Run the program from the terminal:
   ```bash
   python3 <script_name>.py
   ```
3. **Configure Parameters in GUI**: Adjust sampling intervals and other parameters as desired.
4. **Start Data Collection**: Click **Start** to begin collecting data, and view real-time status in the GUI.

## Data Flow

- **Incoming Sensor Data**: Data from each sensor is received via ROS subscribers.
- **Processing and Saving**: Each sensor’s callback processes data, calculates relevant averages, and saves to CSV or bag files.
- **Condition-Based Control**: Data saving is conditioned on sampling intervals, rain status, and a force-save interval.

---

## Complete Code

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import os
import time
import csv
import cv2
import sensor_msgs.point_cloud2 as pc2
import rosbag
from datetime import datetime, timedelta
import numpy as np
import tkinter as tk
from tkinter import messagebox
import threading

class DataCollection:
    def __init__(self, gui):
        self.gui = gui  # Reference to GUI to get real-time updates on parameters
        self.base_data_dir = os.path.join(os.getcwd(), "sensor_data")
        os.makedirs(self.base_data_dir, exist_ok=True)

        # Timestamp for this data collection session
        self.start_time = int(time.time())
        
        # Set up folders for CSV and bag files
        self.csv_base_dir = os.path.join(self.base_data_dir, "csv_data", f"run_{self.start_time}")
        self.bag_base_dir = os.path.join(self.base_data_dir, "bag_data")

        self.sensor_dirs = {
            "camera": os.path.join(self.base_data_dir, "camera", f"run_{self.start_time}"),
            "lidar_csv": os.path.join(self.csv_base_dir, "lidar"),
            "radar_csv": os.path.join(self.csv_base_dir, "radar"),
            "disdrometer_csv": os.path.join(self.csv_base_dir, "disdrometer"),
            "lidar_bag": os.path.join(self.bag_base_dir, "lidar"),
            "radar_bag": os.path.join(self.bag_base_dir, "radar")
        }

        for path in self.sensor_dirs.values():
            os.makedirs(path, exist_ok=True)

        # Initialize paths for CSV files
        self.csv_files = {
            "lidar": os.path.join(self.sensor_dirs["lidar_csv"], "lidar_data.csv"),
            "radar": os.path.join(self.sensor_dirs["radar_csv"], "radar_data.csv"),
            "disdrometer": os.path.join(self.sensor_dirs["disdrometer_csv"], "disdrometer_data.csv")
        }

        # Initialize paths for bag files with timestamped filenames
        self.bag_files = {
            "lidar": rosbag.Bag(os.path.join(self.sensor_dirs["lidar_bag"], f"lidar_data_{self.start_time}.bag"), 'w'),
            "radar": rosbag.Bag(os.path.join(self.sensor_dirs["radar_bag"], f"radar_data_{self.start_time}.bag"), 'w')
        }

        # Initialize CSV files and write headers if empty
        self.initialize_csv_files()

        # Initialize ROS node
        rospy.init_node('data_collection_node', anonymous=True)

        # Initialize bridge for image data
        self.bridge = CvBridge()

        # Buffers to aggregate data within each interval
        self.lidar_buffer = []
        self.radar_buffer = []

        # Control variables for conditional saving
        self.is_raining = False  # Status flag based on disdrometer input
        self.initialized = False  # Flag to start saving data after first disdrometer reading
        self.last_force_save = datetime.now()
        self.is_running = False  # Control for starting and stopping data collection

        # Define subscribers for each sensor
        rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/radar_scan', PointCloud2, self.radar_callback)
        rospy.Subscriber('/disdrometer/data', String, self.callback_disdrometer)

        # Timestamps for controlling sampling frequency
        self.next_camera_time = time.time()
        self.next_lidar_time = time.time()
        self.next_radar_time = time.time()

    def initialize_csv_files(self):
        headers = {
            "lidar": ["Time", "Header Stamp", "Header Frame ID", "Angle Min", "Angle Max",
                      "Angle Increment", "Time Increment", "Scan Time", "Range Min", "Range Max",
                      "Average Distance", "Average Intensity"],
            "radar": ["Time", "Header Stamp", "Header Frame ID", "Average X", "Average Y", "Average Z", "Average Intensity"],
            "disdrometer": ["Time", "Precipitation Type", "Intensity"]
        }

        for sensor, filepath in self.csv_files.items():
            if not os.path.exists(filepath) or os.path.getsize(filepath) == 0:
                with open(filepath, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(headers[sensor])

    def append_to_csv(self, sensor, data):
        """Appends data to the CSV file of the specified sensor."""
        if sensor in self.csv_files:
            with open(self.csv_files[sensor], 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)

    def start_collection(self):
        self.is_running = True
        self.gui.update_status("Collecting Data", "green")

    def stop_collection(self):
        self.is_running = False
        self.gui.update_status("Stopped", "red")

    def toggle_rain_status(self):
        self.is_raining = not self.is_raining
        rain_status = "Raining" if self.is_raining else "No rain"
        self.gui.update_disdrometer_status(rain_status)
        return rain_status

    def camera_callback(self, msg):
        if self.is_running and time.time() >= self.next_camera_time and self.should_save_data():
            try:
                interval = self.gui.camera_interval.get()
                self.next_camera_time = time.time() + interval
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                timestamp = int(time.time())
                file_path = os.path.join(self.sensor_dirs["camera"], f"camera_{timestamp}.jpg")
                cv2.imwrite(file_path, image)
                rospy.loginfo(f"Camera data saved at {file_path}")
                self.gui.update_saving_status("Camera", "Saving")
            except Exception as e:
                rospy.logerr(f"Failed to save camera image: {e}")
        else:
            self.gui.update_saving_status("Camera", "Idle")

    def lidar_callback(self, msg):
        if self.is_running and time.time() >= self.next_lidar_time and self.should_save_data():
            self.next_lidar_time = time.time() + self.gui.lidar_sample_interval.get()
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            data = [timestamp, msg.header.stamp.to_sec(), msg.header.frame_id, msg.angle_min, msg.angle_max,
                    msg.angle_increment, msg.time_increment, msg.scan_time, msg.range_min, msg.range_max,
                    np.mean(msg.ranges), np.mean(msg.intensities) if msg.intensities else "N/A"]
            self.append_to_csv("lidar", data)
            self.gui.update_saving_status("LiDAR", "Saving")
        else:
            self.gui.update_saving_status("LiDAR", "Idle")

    def radar_callback(self, msg):
        if self.is_running and time.time() >= self.next_radar_time and self.should_save_data():
            self.next_radar_time = time.time() + self.gui.radar_sample_interval.get()
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
            if points:
                # Check if each point has an intensity field
                if len(points[0]) == 4:
                    avg_x, avg_y, avg_z, avg_intensity = map(np.mean, zip(*points))
                else:
                    avg_x, avg_y, avg_z = map(np.mean, zip(*points))
                    avg_intensity = "N/A"
            else:
                avg_x = avg_y = avg_z = avg_intensity = "N/A"
            data = [timestamp, msg.header.stamp.to_sec(), msg.header.frame_id, avg_x, avg_y, avg_z, avg_intensity]
            self.append_to_csv("radar", data)
            self.gui.update_saving_status("Radar", "Saving")
        else:
            self.gui.update_saving_status("Radar", "Idle")

    def callback_disdrometer(self, msg):
        if not self.initialized:
            self.initialized = True
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        data = msg.data.split(', ')
        intensity = data[1].split(":")[1].strip(" mm/hr")
        self.is_raining = float(intensity) > 0
        self.append_to_csv("disdrometer", [timestamp, data[0].split(":")[1].strip(), intensity])
        self.gui.update_disdrometer_status("Raining" if self.is_raining else "No rain")

    def should_save_data(self):
        current_time = datetime.now()
        force_interval = timedelta(minutes=self.gui.force_save_interval.get())
        if self.initialized and (self.is_raining or current_time - self.last_force_save >= force_interval):
            if current_time - self.last_force_save >= force_interval:
                self.last_force_save = current_time
            return True
        return False

class DataCollectionGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Data Collection Control Panel")

        # GUI Variables
        self.lidar_sample_interval = tk.DoubleVar(value=10)
        self.radar_sample_interval = tk.DoubleVar(value=10)
        self.camera_interval = tk.DoubleVar(value=20)
        self.force_save_interval = tk.DoubleVar(value=59)
        self.use_disdrometer = tk.BooleanVar(value=True)

        # Traces for parameter change detection
        self.lidar_sample_interval.trace("w", lambda *args: self.show_param_change("LiDAR Sample Interval"))
        self.radar_sample_interval.trace("w", lambda *args: self.show_param_change("Radar Sample Interval"))
        self.camera_interval.trace("w", lambda *args: self.show_param_change("Camera Interval"))
        self.force_save_interval.trace("w", lambda *args: self.show_param_change("Force Save Interval"))

        # GUI layout
        self.create_widgets()
        self.data_collection = DataCollection(self)

        # Start ROS in a separate thread
        threading.Thread(target=rospy.spin, daemon=True).start()

    def create_widgets(self):
        tk.Label(self.root, text="LiDAR Sample Interval (s)").grid(row=0, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.lidar_sample_interval).grid(row=0, column=1)

        tk.Label(self.root, text="Radar Sample Interval (s)").grid(row=1, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.radar_sample_interval).grid(row=1, column=1)

        tk.Label(self.root, text="Camera Interval (s)").grid(row=2, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.camera_interval).grid(row=2, column=1)

        tk.Label(self.root, text="Force Save Interval (min)").grid(row=3, column=0, sticky="e")
        tk.Entry(self.root, textvariable=self.force_save_interval).grid(row=3, column=1)

        tk.Checkbutton(self.root, text="Use Disdrometer for Data Collection", variable=self.use_disdrometer, command=self.display_update_message).grid(row=4, columnspan=2)

        # Control Buttons
        tk.Button(self.root, text="Start", command=self.start_data_collection).grid(row=5, column=0, pady=10)
        tk.Button(self.root, text="Stop", command=self.stop_data_collection).grid(row=5, column=1, pady=10)
        tk.Button(self.root, text="Restart", command=self.restart_data_collection).grid(row=6, columnspan=2, pady=10)
        tk.Button(self.root, text="Test Data Collection (Rain Toggle)", command=self.toggle_rain_status).grid(row=7, columnspan=2, pady=10)

        # Status Display
        self.status_label = tk.Label(self.root, text="Status: Stopped", fg="red")
        self.status_label.grid(row=8, columnspan=2)

        self.disdrometer_label = tk.Label(self.root, text="Disdrometer Status: No data")
        self.disdrometer_label.grid(row=9, columnspan=2)

        # Sensor Saving Status Labels
        self.saving_status_labels = {
            "Camera": tk.Label(self.root, text="Camera: Idle", fg="blue"),
            "LiDAR": tk.Label(self.root, text="LiDAR: Idle", fg="blue"),
            "Radar": tk.Label(self.root, text="Radar: Idle", fg="blue")
        }
        self.saving_status_labels["Camera"].grid(row=10, columnspan=2)
        self.saving_status_labels["LiDAR"].grid(row=11, columnspan=2)
        self.saving_status_labels["Radar"].grid(row=12, columnspan=2)

        # Real-time clock
        self.clock_label = tk.Label(self.root, text="", font=("Helvetica", 10))
        self.clock_label.grid(row=13, columnspan=2)
        self.update_clock()  # Start the clock update

    def show_param_change(self, parameter_name):
        messagebox.showinfo("Parameter Changed", f"{parameter_name} has been updated.")

    def update_clock(self):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.clock_label.config(text=f"Current Time: {current_time}")
        self.root.after(1000, self.update_clock)  # Update clock every second

    def start_data_collection(self):
        self.data_collection.start_collection()
        self.update_status("Collecting Data", "green")

    def stop_data_collection(self):
        self.data_collection.stop_collection()
        self.update_status("Stopped", "red")

    def restart_data_collection(self):
        self.stop_data_collection()
        time.sleep(1)  # Pause to ensure stop completes
        self.start_data_collection()
        messagebox.showinfo("Restarted", "Data collection has been restarted.")

    def toggle_rain_status(self):
        rain_status = self.data_collection.toggle_rain_status()
        messagebox.showinfo("Rain Status", f"Rain status set to: {rain_status}")

    def update_status(self, message, color="black"):
        self.status_label.config(text=f"Status: {message}", fg=color)

    def update_disdrometer_status(self, message):
        self.disdrometer_label.config(text=f"Disdrometer Status: {message}")

    def update_saving_status(self, sensor, status):
        self.saving_status_labels[sensor].config(text=f"{sensor}: {status}", fg="green" if status == "Saving" else "blue")

    def display_update_message(self):
        messagebox.showinfo("Settings Updated", "Settings have been updated.")
        
if __name__ == "__main__":
    root = tk.Tk()
    app = DataCollectionGUI(root)
    root.mainloop()
```