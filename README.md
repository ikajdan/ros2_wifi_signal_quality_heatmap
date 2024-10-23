# ROS2 Wi-Fi Signal Quality Logger

<br>
<div align="center">
  <img src="https://github.com/user-attachments/assets/7e39de8b-a83c-4c5c-95f9-fddfc8dc1ea1" width="800"/>
</div>
<br>

This project is a ROS2 package for logging Wi-Fi signal quality and generating a heatmap of the Wi-Fi signal quality in a given area. It consists of two parts: a ROS2 package for logging the Wi-Fi signal quality and a Python script for generating a heatmap of the Wi-Fi signal quality.

## ROS2 Package

This package logs Wi-Fi signal quality and other related information. It uses the `iw` utility to get the Wi-Fi signal parameters and publishes them to ROS2 topics.

The following data is published:

- SSID - The name of the Wi-Fi network.
- Frequency - The frequency (in MHz).
- Link Quality - The link quality (unitless, 0-70).
- Signal Level - The signal strength (in dBm).
- Noise Level - The noise level (in dBm).
- RX Bitrate - The receive bitrate (in MBit/s).
- TX Bitrate - The transmit bitrate (in MBit/s).

### Usage

1. Install the `iw` utility:

```bash
apt install iw
```

2. Adjust the interface name if needed:

```python
self.interface = "wlp1s0"
```

3. Build the package:

```bash
colcon build --packages-select wifi_signal_quality_logger
```

4. Start the logger:

```bash
ros2 run wifi_signal_quality_logger wifi_signal_quality_logger
```

5. Check the published topics:

```bash
ros2 topic list | grep wifi
```

6. Record the data:

```bash
ros2 bag record /tf /wifi_link_quality /wifi_noise_level /wifi_signal_level /wifi_rx_bitrate /wifi_tx_bitrate
```

## Map Generator

The heatmap can be visualized using the `map_generator`. The script visualizes the signal parameters of a Wi-Fi network in a given area as a heatmap.

### Usage

1. Make sure to set the correct MGRS zone in the `main.py` file:

```python
MGRS_ZONE = "33UXU"
```

2. Install the required packages:

```bash
pip install -r requirements.txt
```

3. Run the script:

```bash
python main.py <rosbag_path>
```

4. Open the generated `heatmap.html` file in your browser to view the heatmap. You can choose between map layers and select displayed data in the menu.

<br>
<div align="center">
  <img src="https://github.com/user-attachments/assets/a7114cc3-9dea-45ce-ad2f-d9d72b1c3867"/>
</div>
<br>

## License

This project is licensed under the [MIT License](LICENSE.md), which means you are free to use, modify, and distribute the project's source code and documentation for both commercial and non-commercial purposes.
