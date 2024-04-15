# nanoLambda_ros2_driver

This ROS 2 driver integrates the nanoLambda NSP32 spectrometer for spectral sampling, utilizing the nanoLambda API. It is designed to help developers and researchers in robotics and automation to easily incorporate spectral data collection into their ROS 2 applications.

Please refer to for more information. https://nanolambda.myshopify.com/pages/resources


## Prerequisites

Before installing and using the nanoLambda_ros2_driver, ensure that you have ROS 2 installed on your system. The driver is tested with ROS 2 Foxy Fitzroy but should be compatible with other ROS 2 releases that support Python packages.

## Installation

### 1. Install ROS 2

Follow the instructions on the ROS 2 website to install ROS 2:

[ROS 2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html)

Ensure that you select the correct distribution according to your needs and operating system.

### 2. Setup Your Workspace

Clone this repository into your ROS 2 workspace's `src` directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yourusername/nanoLambda_ros2_driver.git

cd ~/ros2_ws

colcon build --symlink-install

source ~/ros2_ws/install/setup.bash

ros2 service call /get_spectrum custom_interfaces/srv/GetSpectrum "{}"

