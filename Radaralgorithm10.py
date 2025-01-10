import os
import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import imageio.v2 as imageio

# Input and output file paths
input_file = '/home/carissma/new_ros-workspace/src/my_package/src/sensor_data/radar/radar_data.csv'
output_file = '/home/carissma/new_ros-workspace/src/Scheduling/radar/output/divided_data/radar_data_divided_by_timestamp.xlsx'
output_folder = '/home/carissma/new_ros-workspace/src/Scheduling/radar/output/NewResults'

# Ensure the output folder exists
os.makedirs(output_folder, exist_ok=True)

# DBSCAN parameters
DBSCAN_EPS = 0.01
DBSCAN_MIN_SAMPLES = 1
INTENSITY_THRESHOLD = 22

# InfluxDB configuration
INFLUXDB_URL = "http://localhost:8086"
INFLUXDB_TOKEN = "knhCGxvIcQbFtNpQJli5i7XHsdPd9fJnSuSH0pbaMRC2na2Y-ujQq5FwViBg8_oKZIxZWz3yfBHL4wp6o8aIlw=="
INFLUXDB_ORG = "winterproject24"
INFLUXDB_BUCKET = "radar_metrics"

# Initialize InfluxDB client
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)

def calculate_range_azimuth_elevation(df):
    df['range'] = np.sqrt(df['X']**2 + df['Y']**2 + df['Z']**2)
    df['azimuth'] = np.degrees(np.arctan2(df['Y'], df['X']))
    df['elevation'] = np.degrees(np.arctan2(df['Z'], np.sqrt(df['X']**2 + df['Y']**2)))
    return df

def convert_unix_to_actual_time(df):
    df['actualtime'] = pd.to_datetime(df['Unix_Timestamp'], unit='s')
    return df

def process_sheet_with_dbscan(df, eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES, intensity_threshold=INTENSITY_THRESHOLD):
    clustering_features = df[['range', 'Intensity']].values
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(clustering_features)
    df['cluster'] = db.labels_
    reflectors = df[(df['cluster'] != -1) & (df['Intensity'] >= intensity_threshold)]
    reflector_info = reflectors.groupby('cluster').agg({
        'range': 'mean', 'Intensity': 'mean', 'X': 'mean', 'Y': 'mean', 'Z': 'mean',
        'azimuth': 'mean', 'elevation': 'mean'
    }).reset_index()
    return df, reflectors, reflector_info

def visualize_clusters(reflectors, reflector_info, sheet_name):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    scatter = ax.scatter(reflectors['X'], reflectors['Y'], reflectors['Z'], 
                         c=reflectors['Intensity'], cmap='viridis', s=10)
    for _, reflector in reflector_info.iterrows():
        ax.text(reflector['X'], reflector['Y'], reflector['Z'], f"r{int(reflector['cluster'])}", 
                color='red', fontsize=10)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Reflector Clusters for {sheet_name}')
    plt.colorbar(scatter, label='Intensity')

    return fig

def save_results_to_csv(reflector_info, sheet_name):
    csv_path = os.path.join(output_folder, f"{sheet_name}_reflectors.csv")
    reflector_info.to_csv(csv_path, index=False)
    print(f"Reflector data saved for {sheet_name} to {csv_path}.")

def save_to_influxdb(reflector_info, sheet_name):
    try:
        ns_timestamp = int(sheet_name) * int(1e9)
        points = []
        for _, row in reflector_info.iterrows():
            point = Point("reflector_summary") \
                .time(ns_timestamp) \
                .tag("cluster_id", row['cluster']) \
                .field("range", row['range']) \
                .field("intensity", row['Intensity']) \
                .field("x", row['X']) \
                .field("y", row['Y']) \
                .field("z", row['Z']) \
                .field("azimuth", row['azimuth']) \
                .field("elevation", row['elevation'])
            points.append(point)

        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=points)
        print(f"Reflector data from {sheet_name} successfully uploaded to InfluxDB.")
    except Exception as e:
        print(f"Error uploading data from {sheet_name} to InfluxDB: {e}")

def process_all_sheets(file_path, eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES, intensity_threshold=INTENSITY_THRESHOLD):
    xls = pd.ExcelFile(file_path)
    sheet_names = xls.sheet_names
    combined_reflector_info = []
    frames = []
    processed_timestamps = []

    # Check if the files already exist in the output folder
    existing_files = os.listdir(output_folder)
    processed_timestamps_set = set()

    for file in existing_files:
        if file.endswith("_reflectors.csv"):
            timestamp = file.split('_')[0]
            processed_timestamps_set.add(timestamp)

    print(f"Processed timestamps already: {processed_timestamps_set}")

    for sheet_name in sheet_names:
        print(f"Processing sheet: {sheet_name}")
        df = xls.parse(sheet_name)
        required_columns = ['X', 'Y', 'Z', 'Intensity', 'range', 'azimuth', 'elevation']

        # Skip already processed timestamps
        if sheet_name in processed_timestamps_set:
            print(f"Skipping already processed timestamp: {sheet_name}")
            continue

        if not all(col in df.columns for col in required_columns):
            print(f"Skipping {sheet_name}: Missing required columns.")
            continue

        df, reflectors, reflector_info = process_sheet_with_dbscan(df, eps, min_samples, intensity_threshold)

        fig = visualize_clusters(reflectors, reflector_info, sheet_name)
        frame_path = os.path.join(output_folder, f"{sheet_name}_clusters.png")
        plt.savefig(frame_path)
        frames.append(imageio.imread(frame_path))
        plt.close(fig)

        save_results_to_csv(reflector_info, sheet_name)
        # Add Unix Timestamp and corresponding actual time to the reflector info
        save_to_influxdb(reflector_info, sheet_name)

        reflector_info['Timestamp'] = int(sheet_name)
        reflector_info['Actual_Time'] = pd.to_datetime(reflector_info['Timestamp'], unit='s')  # Convert to actual time
        combined_reflector_info.append(reflector_info)

        # Add the processed timestamp to the list
        processed_timestamps.append(sheet_name)

        # Check if no new timestamps were processed
        if not processed_timestamps:
            print("No new timestamps to process.")
        else:
            print(f"Processed the following timestamps: {processed_timestamps}")

        # Save the processed timestamps to a text file
        if processed_timestamps:
            timestamp_log_path = os.path.join(output_folder, "processed_timestamps.txt")
            with open(timestamp_log_path, 'w') as f:
                for timestamp in processed_timestamps:
                    f.write(f"{timestamp}\n")
            print(f"Processed timestamps saved to {timestamp_log_path}")    
    
    # Save the combined reflector info to a CSV file
    if combined_reflector_info:
        combined_df = pd.concat(combined_reflector_info, ignore_index=True)
        combined_csv_path = os.path.join(output_folder, "all_frames_reflector_summary.csv")
        combined_df.to_csv(combined_csv_path, index=False)
        print(f"Combined reflector summary saved to {combined_csv_path}")

    # Generate and save GIF if frames were generated
    if frames:
        gif_path = os.path.join(output_folder, "reflector_clusters_animation.gif")
        imageio.mimsave(gif_path, frames, duration=0.5)
        print(f"GIF animation saved to {gif_path}")

# Main execution
if __name__ == "__main__":
    # Load the CSV file into a Pandas DataFrame 
    df = pd.read_csv(input_file)
    # Convert the 'Unix_Timestamp' column to integers
    df['Unix_Timestamp'] = df['Unix_Timestamp'].astype(float).astype(int)
    
    # Perform calculations
    df = convert_unix_to_actual_time(df)
    df = calculate_range_azimuth_elevation(df)
    
    # Group the data by the 'Unix_Timestamp' column
    grouped = df.groupby('Unix_Timestamp')
    
    # Create an Excel writer to save the output
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        for timestamp, group in grouped:
            sheet_name = f'{timestamp}'
            if len(sheet_name) > 31:
                sheet_name = f'UTS_{timestamp}'
            group.to_excel(writer, sheet_name=sheet_name, index=False)

    print(f"Data has been divided by Unix_Timestamp, and processed data is saved to '{output_file}'.")

    # Run the DBSCAN processing for all sheets and create the GIF animation
    process_all_sheets(output_file)
