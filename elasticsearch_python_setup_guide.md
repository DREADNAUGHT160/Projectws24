
# **Elasticsearch and Python Setup Guide**

## **Introduction**

This guide will walk you through setting up Elasticsearch, installing the necessary dependencies, and running a Python script that uploads CSV data into an Elasticsearch index.

### **Prerequisites**
1. Ubuntu operating system or similar Linux distribution.
2. Python 3 installed.
3. Sufficient privileges (sudo) to install software and run services.

---

## **1. Installing Elasticsearch on Ubuntu**

### Step 1: Add Elasticsearch GPG Key
First, you need to import the GPG key for Elasticsearch:

```bash
wget -qO - https://artifacts.elastic.co/GPG-KEY-elasticsearch | sudo apt-key add -
```

### Step 2: Add Elasticsearch Repository
Next, add the Elasticsearch APT repository to your system:

```bash
sudo sh -c 'echo "deb https://artifacts.elastic.co/packages/7.x/apt stable main" > /etc/apt/sources.list.d/elastic-7.x.list'
```

### Step 3: Update and Install Elasticsearch
Run the following commands to update your package list and install Elasticsearch:

```bash
sudo apt-get update
sudo apt-get install elasticsearch
```

### Step 4: Start and Enable Elasticsearch
After installation, start the Elasticsearch service and ensure it runs on boot:

```bash
sudo systemctl start elasticsearch
sudo systemctl enable elasticsearch
```

### Step 5: Verify Elasticsearch is Running
Verify that Elasticsearch is running by making a request to `localhost:9200`:

```bash
curl -X GET "localhost:9200/"
```

You should see a response with information about your Elasticsearch cluster.

---

## **2. Installing Kibana (Optional)**

Kibana is a graphical tool that lets you visualize the data stored in Elasticsearch.

### Step 1: Install Kibana

```bash
sudo apt-get install kibana
```

### Step 2: Start and Enable Kibana

```bash
sudo systemctl start kibana
sudo systemctl enable kibana
```

### Step 3: Access Kibana
Kibana can be accessed in your web browser at `http://localhost:5601`.

---

## **3. Setting Up the Python Environment**

### Step 1: Install Python3 and Pip
Ensure Python3 and `pip` (Python package manager) are installed:

```bash
sudo apt-get install python3 python3-pip
```

### Step 2: Install Required Python Libraries
You'll need the following Python libraries to interact with Elasticsearch and handle CSV files:

```bash
pip3 install pandas elasticsearch
```

### Step 3: Verify Python and Library Installation
To verify the Python installation and libraries, run the following commands in a terminal:

```bash
python3 --version
pip3 show pandas elasticsearch
```

---

## **4. Running the Python Script**

### Step 1: Prepare Your CSV File
Make sure you have the CSV file you want to upload (e.g., `username.csv`). The script assumes the file is located in `/home/your_username/Downloads/`.

### Step 2: Python Script

Here is the Python script that uploads CSV data into an Elasticsearch index:

```python
import pandas as pd
import os
from elasticsearch import Elasticsearch
from elasticsearch.helpers import bulk

# Initialize Elasticsearch client with error handling
try:
    es = Elasticsearch([{'host': 'localhost', 'port': 9200, 'scheme': 'http'}])
    # Check if Elasticsearch is running
    if not es.ping():
        raise ValueError("Elasticsearch connection failed")
except Exception as e:
    print(f"Error connecting to Elasticsearch: {e}")
    exit(1)  # Exit if Elasticsearch connection fails

def csv_to_json(csv_file_path):
    # Validate if the file exists
    if not os.path.exists(csv_file_path):
        raise FileNotFoundError(f"The file {csv_file_path} does not exist.")
    
    # Read the CSV file
    df = pd.read_csv(csv_file_path)
    
    # Convert dataframe to JSON format
    json_data = df.to_dict(orient='records')
    
    return json_data

def upload_to_elasticsearch(json_data, index_name):
    actions = [
        {
            "_index": index_name,
            "_id": i,
            "_source": record
        }
        for i, record in enumerate(json_data)
    ]
    
    # Use bulk upload with error handling
    try:
        success, failed = bulk(es, actions)
        print(f"Successfully uploaded {success} documents.")
        
        if failed:
            print(f"Failed to upload {len(failed)} documents.")
    except Exception as e:
        print(f"Error during bulk upload: {e}")

def main(csv_file_path, index_name):
    try:
        # Convert CSV to JSON
        json_data = csv_to_json(csv_file_path)
        
        # Upload JSON data to Elasticsearch
        upload_to_elasticsearch(json_data, index_name)
        print(f"Data from {csv_file_path} uploaded to Elasticsearch index '{index_name}'.")
    
    except FileNotFoundError as fnf_error:
        print(fnf_error)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Path to your CSV file
    csv_file_path = '/home/your_username/Downloads/username.csv'
    
    # Elasticsearch index name
    index_name = 'research'
    
    main(csv_file_path, index_name)
```

### Step 3: Running the Script

1. Save the script as `upload_to_elasticsearch.py`.
2. Ensure your CSV file is correctly located in `/home/your_username/Downloads/`.
3. Run the Python script:

```bash
python3 upload_to_elasticsearch.py
```

This will upload your CSV data into the Elasticsearch index called `research`.

---

## **5. Troubleshooting**

### Issue 1: Elasticsearch Fails to Start
If Elasticsearch fails to start, check the logs for errors:

```bash
sudo journalctl -u elasticsearch
```

### Issue 2: Kibana Shows "Kibana server is not ready yet"
Kibana can take a few minutes to initialize. If it doesn't start, check the Kibana logs for errors:

```bash
sudo journalctl -u kibana
```

### Issue 3: Python Script Errors
If the script fails to upload data, make sure Elasticsearch is running and accessible on port 9200. You can test the connection with:

```bash
curl -X GET "localhost:9200/"
```

---

## **Conclusion**

By following this guide, you should be able to set up Elasticsearch, Kibana (optional), and run the Python script to upload CSV data into Elasticsearch. If you encounter any issues or need further assistance, feel free to reach out for help.

