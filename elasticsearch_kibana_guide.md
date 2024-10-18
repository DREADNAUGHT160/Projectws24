
# Elasticsearch and Kibana Setup and Usage on Ubuntu

## 1. Installing Elasticsearch and Kibana on Ubuntu

### 1.1 Add the GPG Key and APT Repository

1. **Add the Elasticsearch GPG key**:
   ```bash
   wget -qO - https://artifacts.elastic.co/GPG-KEY-elasticsearch | sudo apt-key add -
   ```

2. **Install the APT-transport package**:
   ```bash
   sudo apt-get install apt-transport-https
   ```

3. **Add the Elasticsearch repository**:
   ```bash
   echo "deb https://artifacts.elastic.co/packages/7.x/apt stable main" | sudo tee -a /etc/apt/sources.list.d/elastic-7.x.list
   ```

### 1.2 Install Elasticsearch and Kibana

1. **Update the system and install Elasticsearch**:
   ```bash
   sudo apt-get update && sudo apt-get install elasticsearch
   ```

2. **Install Kibana**:
   ```bash
   sudo apt-get install kibana
   ```

### 1.3 Configure Elasticsearch and Kibana to Run on System Startup

To have Elasticsearch and Kibana start on system startup:

1. **Enable Elasticsearch service**:
   ```bash
   sudo systemctl enable elasticsearch
   ```

2. **Enable Kibana service**:
   ```bash
   sudo systemctl enable kibana
   ```

## 2. Starting Elasticsearch and Kibana Manually

### 2.1 Start Elasticsearch

To start Elasticsearch each time:

```bash
sudo systemctl start elasticsearch
```

### 2.2 Start Kibana

To start Kibana each time:

```bash
sudo systemctl start kibana
```

### 2.3 Verify Elasticsearch and Kibana are Running

- To **verify Elasticsearch**, open a browser and visit:

  ```bash
  http://localhost:9200
  ```

- To **verify Kibana**, open a browser and visit:

  ```bash
  http://localhost:5601
  ```

## 3. Python Program to Convert CSV to JSON and Upload to Elasticsearch

Here’s a Python script that will help you convert a CSV file into JSON and upload it to Elasticsearch. 

### Prerequisites

Ensure you have the following Python packages installed:

```bash
pip install pandas elasticsearch
```

### Python Script

Save the following script in a file, e.g., `upload_to_elasticsearch.py`:

```python
import pandas as pd
import json
from elasticsearch import Elasticsearch

# Initialize Elasticsearch client
es = Elasticsearch([{'host': 'localhost', 'port': 9200}])

def csv_to_json(csv_file_path):
    # Read the CSV file
    df = pd.read_csv(csv_file_path)
    
    # Convert dataframe to JSON format
    json_data = df.to_dict(orient='records')
    
    return json_data

def upload_to_elasticsearch(json_data, index_name):
    for i, record in enumerate(json_data):
        # Upload each record to Elasticsearch
        es.index(index=index_name, id=i, body=record)

def main(csv_file_path, index_name):
    # Convert CSV to JSON
    json_data = csv_to_json(csv_file_path)
    
    # Upload JSON data to Elasticsearch
    upload_to_elasticsearch(json_data, index_name)
    print(f"Data from {csv_file_path} uploaded to Elasticsearch index '{index_name}'.")

if __name__ == "__main__":
    # Path to your CSV file
    csv_file_path = 'data.csv'
    
    # Elasticsearch index name
    index_name = 'my_index'
    
    main(csv_file_path, index_name)
```

### Script Explanation

- **csv_to_json function**: Converts a CSV file into JSON format using `pandas`.
- **upload_to_elasticsearch function**: Iterates over the JSON data and uploads it to Elasticsearch.
- **Main Function**: Takes the CSV file path and Elasticsearch index name, converts the CSV to JSON, and uploads it.

### Running the Script

To run the script:

```bash
python3 upload_to_elasticsearch.py
```

- **Modify `csv_file_path`** to the path of your CSV file.
- **Change `index_name`** to the desired Elasticsearch index name.

## 4. Routine Summary: Starting Elasticsearch, Kibana, and Uploading Data

### Each Time You Want to Start Elasticsearch and Kibana:

1. **Start Elasticsearch**:
   ```bash
   sudo systemctl start elasticsearch
   ```

2. **Start Kibana**:
   ```bash
   sudo systemctl start kibana
   ```

3. **Verify Elasticsearch and Kibana** are running by visiting `http://localhost:9200` (for Elasticsearch) and `http://localhost:5601` (for Kibana).

4. **Run the Python Script** to upload CSV data to Elasticsearch:
   ```bash
   python3 upload_to_elasticsearch.py
   ```

### Notes:
- Make sure Elasticsearch and Kibana services are up and running before uploading data.
- To check the status of Elasticsearch and Kibana:
  ```bash
  sudo systemctl status elasticsearch
  sudo systemctl status kibana
  ```

This guide should help you set up Elasticsearch and Kibana on Ubuntu, as well as how to start them and upload data to Elasticsearch using Python.
