
# Installation Guide: InfluxDB and Grafana on Ubuntu

This guide provides step-by-step instructions to install InfluxDB and Grafana on an Ubuntu system.

## Prerequisites

- **Operating System:** Ubuntu 20.04 or later
- **User Privileges:** A user account with `sudo` privileges
- **Network Access:** Internet connectivity to download packages

---

## 1. Install InfluxDB

InfluxDB is an open-source time-series database optimized for high-availability storage and retrieval of time-stamped data.

### 1.1 Add the InfluxData Repository

Run the following commands to add the InfluxData GPG key and repository:

```bash
sudo apt update
sudo apt install -y gnupg2 curl
curl -sL https://repos.influxdata.com/influxdata-archive_compat.key | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/influxdata.gpg
echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list
```

### 1.2 Install InfluxDB

Update the package list and install InfluxDB:

```bash
sudo apt update
sudo apt install -y influxdb2
```

### 1.3 Start and Enable InfluxDB Service

Start the InfluxDB service and enable it to start on boot:

```bash
sudo systemctl enable --now influxdb
```

### 1.4 Verify the Service Status

Check if the service is running:

```bash
sudo systemctl status influxdb
```

---

## 2. Install Grafana

Grafana is an open-source platform for monitoring and observability, providing dashboards and visualization capabilities.

### 2.1 Add the Grafana Repository

Add the Grafana GPG key and repository:

```bash
sudo apt update
sudo apt install -y gnupg2 curl software-properties-common
curl -fsSL https://apt.grafana.com/gpg.key | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/grafana.gpg
echo 'deb [signed-by=/etc/apt/trusted.gpg.d/grafana.gpg] https://apt.grafana.com stable main' | sudo tee /etc/apt/sources.list.d/grafana.list
```

### 2.2 Install Grafana

Update the package list and install Grafana:

```bash
sudo apt update
sudo apt install -y grafana
```

### 2.3 Start and Enable Grafana Service

Start the Grafana service and enable it to start on boot:

```bash
sudo systemctl enable --now grafana-server
```

### 2.4 Verify the Service Status

Check if the service is running:

```bash
sudo systemctl status grafana-server
```

---

## 3. Access Grafana

Grafana's web interface is accessible via your web browser:

1. Navigate to `http://<your_server_ip>:3000/`. Replace `<your_server_ip>` with your server's IP address. If accessing locally, use `http://localhost:3000`.
2. Log in with the default credentials:
   - **Username:** `admin`
   - **Password:** `admin`
3. You will be prompted to change the default password upon first login.

---

## 4. Configure InfluxDB as a Data Source in Grafana

To visualize data from InfluxDB in Grafana:

1. In Grafana, click on the gear icon (⚙️) to access the **Configuration** menu, then select **Data Sources**.
2. Click **Add data source** and choose **InfluxDB**.
3. Fill in the following details:
   - **HTTP URL:** `http://localhost:8086`
   - **Access:** `Server`
   - **Database:** (Specify your InfluxDB database name)
   - **User:** (Your InfluxDB username)
   - **Password:** (Your InfluxDB password)
4. Click **Save & Test** to verify the connection.

---

By following these steps, you have successfully installed and configured InfluxDB and Grafana on your Ubuntu system, enabling you to collect and visualize time-series data effectively.

For further information, refer to:
- [InfluxDB Documentation](https://docs.influxdata.com/influxdb/latest/)
- [Grafana Documentation](https://grafana.com/docs/)
