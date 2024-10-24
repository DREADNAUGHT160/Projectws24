
# Sensor Data Collection Setup and Usage Guide

This guide provides instructions on setting up and running the **sensor data collection** system, which collects data from a camera, LiDAR, radar, and disdrometer. The data is stored in CSV files for each sensor in a folder named with a Unix timestamp.

## Prerequisites

Ensure the following are installed and set up on the system:
1. **Python 3**.
2. **ROS (Robot Operating System)**.
3. Necessary sensor drivers for the camera, LiDAR, radar, and disdrometer, publishing data to ROS topics.

## Step 1: Clone the Repository

First, clone or copy the sensor data collection code into the desired directory.

```bash
git clone <repository-url>
cd <repository-directory>
```

## Step 2: Install Required Python Libraries

Make sure to install the following Python libraries:

```bash
pip install rospy subprocess csv datetime
```

## Step 3: Verify ROS Sensor Topics

Ensure that the following ROS topics are active:
- `/usb_cam/image_raw` (for the camera)
- `/scan` (for LiDAR)
- `/ti_mmwave/radar_scan_pcl_0` (for radar)

Check by running:

```bash
rostopic list
```

## Step 4: Running the Data Collection Script

1. Open a terminal in the directory where the `data_collection_final.py` script is located.
2. Run the script:

   ```bash
   python3 data_collection_final.py
   ```

3. When prompted, specify the **path to the data server** where the CSV files will be saved. For example:

   ```
   Enter the main data server path (e.g., /path/to/data_server_1): /home/user/data_server_1
   ```

   A folder will be created in `/home/user/data_server_1` named with the current Unix timestamp (e.g., `1681847483`). The folder will contain the following CSV files:
   
   - `camera_data_UNIX_TIMESTAMP.csv`
   - `lidar_data_UNIX_TIMESTAMP.csv`
   - `radar_data_UNIX_TIMESTAMP.csv`
   - `disdrometer_data_UNIX_TIMESTAMP.csv`

## Step 5: Data Monitoring and Collection

The script collects real-time data from the camera, LiDAR, radar, and disdrometer. Each sensor's data is saved in separate CSV files with the following columns:

- **Camera**:
  - Timestamp (human-readable and Unix)
  - Number of rays sent
  - Detections (always 0)

- **LiDAR**:
  - Timestamp (human-readable and Unix)
  - Number of rays sent
  - Number of reflections

- **Radar**:
  - Timestamp (human-readable and Unix)
  - Number of rays sent
  - Number of detections

- **Disdrometer**:
  - Timestamp (human-readable and Unix)
  - Rain intensity (mm/h)
  - Average droplet size (mm)
  - Droplet concentration (drops/m³)
  - Average droplet velocity (m/s)

## Step 6: Ending the Program

Once the data collection is complete, stop the program using `Ctrl + C` in the terminal.

## Troubleshooting

- **Missing Data**: If any sensor data is missing, check the corresponding ROS topics to ensure they are publishing.
- **Folder/CSV Issues**: Verify that the specified path in Step 4 is correct and accessible.

---

This guide ensures proper setup and running of the sensor data collection program, organizing data efficiently and synchronizing sensor data with human-readable timestamps.
