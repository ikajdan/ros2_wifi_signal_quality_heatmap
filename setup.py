from setuptools import find_packages, setup

package_name = "wifi_signal_quality_logger"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["get_wifi_info.sh"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ignacy Kajdan",
    maintainer_email="ignacy.kajdan@gmail.com",
    description="Wi-Fi signal quality logger",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wifi_signal_quality_logger = wifi_signal_quality_logger.wifi_signal_quality_logger_node:main"
        ],
    },
)
