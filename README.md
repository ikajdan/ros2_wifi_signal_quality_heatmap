# ROS2 Wi-Fi Signal Quality Logger

This project is a ROS2 package for logging Wi-Fi signal quality and generating a heatmap of the Wi-Fi signal quality in a given area. It consists of two parts: a ROS2 package for logging the Wi-Fi signal quality and a Python script for generating a heatmap of the Wi-Fi signal quality.

<br>
<div align="center">
  <img src="https://github.com/user-attachments/assets/d7a4dbf1-d5a7-4e95-9b54-553d81b7dfaf" width="700" height="auto"/>
  <br><br>
  <em>Heatmap of Wi-Fi signal quality in a given area.</em>
</div>
<br>

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
  <br><br>
  <em>Menu for selecting displayed data.</em>
</div>
<br>

## License

This project is licensed under the MIT License. See the [LICENSE.md](LICENSE.md) file for more information.
