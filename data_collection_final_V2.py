
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import os
import csv
import time
import subprocess
from datetime import datetime

class Data_Collection:
    def __init__(self, data_server_path) -> None:
        # Main data server folder (root directory where all data is stored)
        self.data_server_path = data_server_path
        
        # Create a subfolder with the current Unix timestamp
        self.unix_timestamp = int(time.time())
        self.session_folder = os.path.join(self.data_server_path, str(self.unix_timestamp))
        
        if not os.path.exists(self.session_folder):
            os.makedirs(self.session_folder)

        # Initialize CSV files for each sensor and disdrometer
        self.init_csv_files()

        # Subscribe to the camera, LiDAR, and radar topics
        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_camera)
        rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.Subscriber('/ti_mmwave/radar_scan_pcl_0', PointCloud2, self.callback_radar)

    def init_csv_files(self):
        # CSV file paths for each sensor and disdrometer, with Unix timestamp in filenames
        self.camera_csv = os.path.join(self.session_folder, f'camera_data_{self.unix_timestamp}.csv')
        self.lidar_csv = os.path.join(self.session_folder, f'lidar_data_{self.unix_timestamp}.csv')
        self.radar_csv = os.path.join(self.session_folder, f'radar_data_{self.unix_timestamp}.csv')
        self.disdro_csv = os.path.join(self.session_folder, f'disdrometer_data_{self.unix_timestamp}.csv')

        # Initialize CSV files with headers
        self.create_csv(self.camera_csv, ['Timestamp (Human-Readable)', 'Unix Timestamp', 'Rays Sent', 'Detections'])
        self.create_csv(self.lidar_csv, ['Timestamp (Human-Readable)', 'Unix Timestamp', 'Rays Sent', 'Reflections'])
        self.create_csv(self.radar_csv, ['Timestamp (Human-Readable)', 'Unix Timestamp', 'Rays Sent', 'Detections'])
        self.create_csv(self.disdro_csv, ['Timestamp (Human-Readable)', 'Unix Timestamp', 'Rain Intensity (mm/h)', 
                                          'Average Droplet Size (mm)', 'Droplet Concentration (drops/m³)', 
                                          'Average Droplet Velocity (m/s)'])

    def create_csv(self, file_path, headers):
        # Create and initialize a CSV file with headers
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)

    def write_to_csv(self, file_path, data):
        # Write data to the specified CSV file
        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def get_human_readable_time(self, unix_timestamp):
        # Convert Unix timestamp to human-readable format
        return datetime.utcfromtimestamp(unix_timestamp).strftime('%Y-%m-%d %H:%M:%S')

    def callback_camera(self, image_raw):
        # Process camera data
        timestamp = rospy.Time.now().to_sec()
        human_time = self.get_human_readable_time(timestamp)
        rays_sent = 1  # Camera sends one image at a time
        detections = 0  # No detections for the camera

        # Save camera data to CSV with human-readable and Unix timestamps
        self.write_to_csv(self.camera_csv, [human_time, timestamp, rays_sent, detections])

    def callback_lidar(self, scan):
        # Process LiDAR data
        timestamp = rospy.Time.now().to_sec()
        human_time = self.get_human_readable_time(timestamp)
        rays_sent = len(scan.ranges)  # Total number of rays sent
        reflections = sum(1 for distance in scan.ranges if distance < scan.range_max)  # Count reflections

        # Save LiDAR data to CSV with human-readable and Unix timestamps
        self.write_to_csv(self.lidar_csv, [human_time, timestamp, rays_sent, reflections])

    def callback_radar(self, radar_scan):
        # Process radar data
        timestamp = rospy.Time.now().to_sec()
        human_time = self.get_human_readable_time(timestamp)
        rays_sent = len(radar_scan.data)  # Assuming radar sends rays as point cloud data
        detections = rays_sent  # Assuming detections are equal to rays for radar

        # Save radar data to CSV with human-readable and Unix timestamps
        self.write_to_csv(self.radar_csv, [human_time, timestamp, rays_sent, detections])

    def start_disdrometer(self):
        # Simulate the disdrometer data process (replace with actual subprocess)
        print("Starting disdrometer process...")
        for _ in range(10):  # Simulated data points
            timestamp = rospy.Time.now().to_sec()
            human_time = self.get_human_readable_time(timestamp)

            # Simulate disdrometer data
            rain_intensity = round(2.5 + 0.5 * _, 2)  # Simulated rain intensity (in mm/h)
            avg_droplet_size = round(1.2 + 0.1 * _, 2)  # Simulated average droplet size (in mm)
            droplet_concentration = round(100 + 10 * _, 2)  # Simulated droplet concentration (drops/m³)
            avg_droplet_velocity = round(4.5 + 0.2 * _, 2)  # Simulated droplet velocity (in m/s)

            # Save disdrometer data to CSV with human-readable and Unix timestamps
            self.write_to_csv(self.disdro_csv, [human_time, timestamp, rain_intensity, avg_droplet_size, 
                                                droplet_concentration, avg_droplet_velocity])
            rospy.sleep(1)  # Simulate delay between disdrometer readings

    def run(self):
        # Start disdrometer data collection
        disdro_process = subprocess.Popen(["python3 disdrometer_both.py {} {}".format(self.session_folder, str(time.time()))], shell=True)

        # Run the disdrometer in parallel
        self.start_disdrometer()

        # Keep collecting data from camera, LiDAR, and radar
        rospy.spin()

        # Wait for the disdrometer process to finish
        disdro_process.wait()

if __name__ == '__main__':
    # Get the main data server directory from the user at runtime
    data_server_path = input("Enter the main data server path (e.g., /path/to/data_server_1): ")

    # Initialize ROS node and start data collection
    rospy.init_node('Data_Collection')
    data_collector = Data_Collection(data_server_path)
    data_collector.run()
