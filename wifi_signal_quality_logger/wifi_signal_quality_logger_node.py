import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String


class WifiSignalQualityLoggerNode(Node):
    def __init__(self):
        super().__init__("wifi_signal_quality_logger")
        self.logger = self.get_logger()

        self.script_path = os.path.join(
            get_package_share_directory("wifi_signal_quality_logger"),
            "get_wifi_info.sh",
        )
        self.interface = "wlp9s0"

        self.ssid_publisher = self.create_publisher(String, "wifi_ssid", 10)
        self.frequency_publisher = self.create_publisher(Float32, "wifi_frequency", 10)
        self.link_quality_publisher = self.create_publisher(
            Int32, "wifi_link_quality", 10
        )
        self.signal_level_publisher = self.create_publisher(
            Int32, "wifi_signal_level", 10
        )
        self.noise_level_publisher = self.create_publisher(
            Int32, "wifi_noise_level", 10
        )
        self.rx_bitrate_publisher = self.create_publisher(
            Float32, "wifi_rx_bitrate", 10
        )
        self.tx_bitrate_publisher = self.create_publisher(
            Float32, "wifi_tx_bitrate", 10
        )

        self.timer = self.create_timer(1.0, self.log_wifi_info)

    def log_wifi_info(self):
        try:
            result = subprocess.run(
                [self.script_path, self.interface],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.returncode != 0:
                self.logger.error(f"Failed to get Wi-Fi info: {result.stderr}")
                return

            output_lines = result.stdout.strip().split("\n")
            wifi_info = {
                line.split(":")[0].strip(): line.split(":")[1].strip()
                for line in output_lines
            }

            self.publish_info(wifi_info)

        except Exception as e:
            self.logger.error(f"An error occurred: {str(e)}")

    def publish_info(self, wifi_info):
        try:
            ssid = wifi_info.get("SSID", "")
            self.ssid_publisher.publish(String(data=ssid))

            frequency = wifi_info.get("Frequency", "0.0")
            self.frequency_publisher.publish(
                Float32(data=float(frequency) if frequency else 0.0)
            )

            link_quality = wifi_info.get("Link Quality", "0")
            self.link_quality_publisher.publish(
                Int32(data=int(link_quality) if link_quality else 0)
            )

            signal_level = wifi_info.get("Signal Level", "0")
            self.signal_level_publisher.publish(
                Int32(data=int(signal_level) if signal_level else 0)
            )

            noise_level = wifi_info.get("Noise Level", "0")
            self.noise_level_publisher.publish(
                Int32(data=int(noise_level) if noise_level else 0)
            )

            rx_bitrate = wifi_info.get("RX Bitrate", "0.0")
            self.rx_bitrate_publisher.publish(
                Float32(data=float(rx_bitrate) if rx_bitrate else 0.0)
            )

            tx_bitrate = wifi_info.get("TX Bitrate", "0.0")
            self.tx_bitrate_publisher.publish(
                Float32(data=float(tx_bitrate) if tx_bitrate else 0.0)
            )

        except ValueError as e:
            self.logger.error(f"Error converting Wi-Fi info: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = WifiSignalQualityLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
