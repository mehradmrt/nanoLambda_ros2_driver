# nanoLambda_ros2_driver

This ROS 2 driver integrates the nanoLambda NSP32 spectrometer for spectral sampling, utilizing the nanoLambda API.

Please refer to https://nanolambda.myshopify.com/pages/resources for more information.


## Prerequisites

Before installing and using the nanoLambda_ros2_driver, ensure that you have ROS 2 installed on your system. The driver is tested with ROS 2 Foxy Fitzroy but should be compatible with other ROS 2 releases such as ROS2 Humble. 

Make sure you add the following reules to your usb rules:

'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A5XK3RJT", KERNEL=="ttyUSB*", SYMLINK+="nanospec"'

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
git clone https://github.com/mehradmrt/nanoLambda_ros2_driver

cd ~/ros2_ws

colcon build 

```

### 3. call the service to get the spectra from NSP32:
 
In the same terminal:
```bash
source install/setup.bash

ros2 run nanospec NSP32_service_node
```

open a separate terminal and:
```bash
source install/setup.bash

ros2 service call /get_spectrum custom_interfaces/srv/GetSpectrum "{}"
```

As a result the ros2 service provides the spectral data at one instance in the format:

```bash
uint16[] wavelengths
float64[] spectrum
```
 

