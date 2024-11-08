
# Installing InfluxDB on Ubuntu 20.04

InfluxDB is an open-source time-series database that's ideal for storing metrics and events. Below is a step-by-step guide for installing InfluxDB v2 on Ubuntu 20.04.

## Step-by-Step Installation Guide

### 1. Update System Packages

Open a terminal and update the package list to ensure your system has the latest package information:

```sh
sudo apt update
sudo apt upgrade -y
```

### 2. Add InfluxDB Repository

To get the latest version of InfluxDB, add the InfluxData repository by importing the GPG key and adding the repository:

```sh
curl -sL https://repos.influxdata.com/influxdb.key | sudo apt-key add -
```

Add the repository for Ubuntu 20.04:

```sh
source /etc/lsb-release
echo "deb https://repos.influxdata.com/ubuntu ${DISTRIB_CODENAME} stable" | sudo tee /etc/apt/sources.list.d/influxdb.list
```

### 3. Install InfluxDB

Now that the repository has been added, install InfluxDB:

```sh
sudo apt update
sudo apt install influxdb -y
```

### 4. Start and Enable InfluxDB Service

Start the InfluxDB service and enable it to start on boot:

```sh
sudo systemctl start influxdb
sudo systemctl enable influxdb
```

Verify that the service is running properly:

```sh
sudo systemctl status influxdb
```

### 5. Configure Firewall (Optional)

If you have a firewall enabled, allow InfluxDB's default port (`8086`):

```sh
sudo ufw allow 8086/tcp
```

### 6. Verify Installation

To check if InfluxDB is installed and running, you can check the version:

```sh
influx --version
```

Or, send a request to the local server:

```sh
curl http://localhost:8086/health
```

This should return a JSON object with `"status": "pass"` if InfluxDB is running correctly.

## Setting Up InfluxDB v2

### 1. Access Web Interface

InfluxDB v2 uses a web-based setup. Open your browser and navigate to:

```
http://<your-server-ip>:8086
```

You will be prompted to create:

- **Initial User Account**: Username, password, and organization name.
- **Initial Bucket**: Your data storage location.

### 2. Accessing the CLI

InfluxDB provides a command-line interface (`influx`) to interact with your database. You can use this to add data, create dashboards, and query metrics.

```sh
influx setup
```

This command will guide you to set up initial credentials and configurations if not done via the web UI.

## Basic Commands to Interact with InfluxDB

- **Log in to InfluxDB**:

  ```sh
  influx
  ```

- **Write Data**:

  ```sh
  influx write --bucket my-bucket --org my-org --token my-token \
  'sensor_data temperature=23.5 1640995200000000000'
  ```

- **Query Data**:

  ```sh
  influx query 'from(bucket: "my-bucket") |> range(start: -1h)'
  ```

## Additional Considerations

### Configuration File

InfluxDB’s configuration file is usually located at `/etc/influxdb/influxdb.conf` for v1.x or `/etc/influxdb/config.yml` for v2. Adjust settings as per your needs.

### Service Commands

Some useful service commands include:

- Restart the service:

  ```sh
  sudo systemctl restart influxdb
  ```

- Stop the service:

  ```sh
  sudo systemctl stop influxdb
  ```

### Data Retention

Set up retention policies to automatically remove old data:

```sh
influx bucket update -n my-bucket --retention 30d
```

This command sets the retention period to 30 days.

## Wrapping Up

InfluxDB is now successfully installed and set up on your Ubuntu 20.04 server. You can use the web UI or command-line interface to interact with it, ingesting and querying time-series data. The flexibility of InfluxDB makes it suitable for various use cases, including monitoring, metrics collection, and IoT applications.
