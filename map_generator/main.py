import sys
from pathlib import Path

import folium
from folium.plugins import HeatMap
from mgrs import MGRS
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# MGRS zone
MGRS_ZONE = "12UUA"


def mgrs_to_ll(easting, northing):
    """
    Converts MGRS coordinates to latitude and longitude.

    Parameters:
    easting (float): The easting value of the MGRS coordinate.
    northing (float): The northing value of the MGRS coordinate.

    Returns:
    tuple: A tuple containing the latitude and longitude values.
    """

    easting_str = f"{int(easting):05d}"
    northing_str = f"{int(northing):05d}"
    mgrs_code = f"{MGRS_ZONE}{easting_str}{northing_str}"
    m = MGRS()
    lat, lon = m.toLatLon(mgrs_code)

    return lat, lon


def extract_data_from_bag(bagpath):
    """
    Extracts Wi-Fi data from a ROS 2 bag file.

    Parameters:
    bagpath (str): The path to the ROS 2 bag file.

    Returns:
    dict: A dictionary containing the extracted Wi-Fi data
    """

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    wifi_data = {
        "link_quality": [],
        "signal_level": [],
        "noise_level": [],
        "rx_bitrate": [],
        "tx_bitrate": [],
    }
    positions = {}
    allowed_difference = 5 * 10**9

    with AnyReader([bagpath], default_typestore=typestore) as reader:
        tf_connections = [x for x in reader.connections if x.topic == "/tf"]
        wifi_connections = {
            "link_quality": [
                x for x in reader.connections if x.topic == "/wifi_link_quality"
            ],
            "signal_level": [
                x for x in reader.connections if x.topic == "/wifi_signal_level"
            ],
            "noise_level": [
                x for x in reader.connections if x.topic == "/wifi_noise_level"
            ],
            "rx_bitrate": [
                x for x in reader.connections if x.topic == "/wifi_rx_bitrate"
            ],
            "tx_bitrate": [
                x for x in reader.connections if x.topic == "/wifi_tx_bitrate"
            ],
        }

        for connection, timestamp, rawdata in reader.messages(
            connections=tf_connections
        ):
            msg = reader.deserialize(rawdata, connection.msgtype)
            for transform in msg.transforms:
                if transform.child_frame_id == "base_link":
                    pos = transform.transform.translation
                    position = (pos.x, pos.y)
                    positions[timestamp] = position

        for key, connections in wifi_connections.items():
            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                msg = reader.deserialize(rawdata, connection.msgtype)
                if not hasattr(msg, "data"):
                    continue
                closest_timestamp = min(
                    positions.keys(), key=lambda t: abs(t - timestamp)
                )
                if abs(closest_timestamp - timestamp) <= allowed_difference:
                    pos = positions[closest_timestamp]
                    geo_pos = mgrs_to_ll(pos[0], pos[1])
                    wifi_data[key].append((geo_pos[0], geo_pos[1], msg.data))

    for key in wifi_data:
        print(f"Extracted {len(wifi_data[key])} data points for {key} from the bag.")

    return wifi_data


def create_heatmap(data):
    """
    Creates a heatmap from the extracted Wi-Fi data.

    Parameters:
    data (dict): A dictionary containing the extracted Wi-Fi data.

    Returns:
    folium.Map: A folium map object containing the heatmap
    """

    if not any(data.values()):
        raise ValueError("No data to display on the map.")

    first_point = (
        next(iter(data.values()))[0] if any(data.values()) else (52.40173, 16.95165)
    )
    heatmap = folium.Map(
        location=[first_point[0], first_point[1]], zoom_start=22, tiles=None
    )

    # Add tile layers
    folium.TileLayer("cartodb positron", name="CartoDB Positron").add_to(heatmap)
    folium.TileLayer("openstreetmap", name="OpenStreetMap").add_to(heatmap)

    for key, values in data.items():
        if values:
            layer = folium.FeatureGroup(
                name=f'Wi-Fi {key.replace("_", " ").title()} Heatmap'
            ).add_to(heatmap)
            HeatMap(values, radius=15, blur=15, min_opacity=0.5, max_zoom=22).add_to(
                layer
            )

    # Add layer control
    folium.LayerControl().add_to(heatmap)

    heatmap.save("heatmap.html")

    print(
        "\nHeatmap created successfully. Open heatmap.html in a web browser to view it."
    )

    return heatmap


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python main.py <rosbag_path>")
        sys.exit(1)

    rosbag_path = Path(sys.argv[1])
    wifi_data = extract_data_from_bag(rosbag_path)
    if any(wifi_data.values()):
        create_heatmap(wifi_data)
    else:
        print("No data available to create heatmap.")
