
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
