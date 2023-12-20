# LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2 User Guide

## 1. Project Introduction
LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2 is a ROS (Robot Operating System) driver for LIDAR sensors in a Linux environment. It is compatible with ls128, ls25d, and ls180 LIDAR devices. The program has been tested on Ubuntu 20.04 with ROS Foxy and Ubuntu 20.04 with ROS Galactic.

## 2. Dependencies

1. **Ubuntu 20.04 with ROS Foxy/Ubuntu 20.04 with ROS Galactic**
2. **ROS Dependencies**

```bash
# Installation
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib ros-$ROS_DISTRO-pcl-conversions
```

3. Other dependencies

```bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
```



## 3. Compilation and Execution
   
### 3.1 Compilation

~~~bash
mkdir -p ~/lidar_ws/src
#将LSLIDAR_LS128_ROS_V1.0.0_221012_ROS2压缩包解压缩放到~/lidar_ws/src 目录下
cd ~/lidar_ws
colcon build
source install/setup.bash
~~~

### 3.2 Execution

Run a single LIDAR:

~~~bash
ros2 launch lslidar_driver lslidar_ls128.launch
~~~

Run multiple LIDARs:

~~~bash
ros2 launch lslidar_driver lslidar_ls128_double.launch
~~~

## 4. Parameter Description

The content of the lslidar_ls128.yaml file is as follows, with each parameter explained in the comments.

~~~bash
/ls128/lslidar_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200          # LIDAR IP
    msop_port: 2368                   # Data packet destination port
    difop_port: 2369                  # Device packet destination port
    frame_id: laser_link              # Coordinate frame ID
    add_multicast: false              # Enable multicast mode
    group_ip: 224.1.1.2               # Multicast IP address
    time_synchronization: false       # Whether the LIDAR uses GPS or PTP time synchronization, set to true if yes
    min_range: 0.3                    # Unit: meters. Minimum value of LIDAR blind zone, points below this value are filtered
    max_range: 500.0                  # Unit: meters. Maximum value of LIDAR blind zone, points above this value are filtered
    scan_start_angle: -60              # Starting angle, range: [-60,60]
    scan_end_angle: 60                # Ending angle, range: [-60,60]
    scan_num: 15                       # Laser scan lines
    publish_scan: false               # Whether to publish scan data
    topic_name: lslidar_point_cloud   # Point cloud topic name, can be modified
    # pcap: /home/ls/work/fuhong/data/ls128/20221012-ntp.pcap                        # Path to pcap file, uncomment this line when loading pcap file
~~~

## Multicast Mode:

* Set the LIDAR to enable multicast mode on the host computer.
* Modify the launch file with the following parameters:

~~~bash
add_multicast: true                    # Enable multicast mode.
group_ip: 224.1.1.2                    # Multicast IP address
~~~

* Run the following command to add the computer to the group (replace enp2s0 with the user's computer network card name, check the network card name with ifconfig):
  
~~~shell
ifconfig
sudo route add -net 224.0.0.0/4 dev enp2s0
~~~


## Offline PCAP Mode:

Modify the launch file with the following parameters:

  ~~~shell
  #取消注释
  pcap: /home/ls/work/fuhong/data/ls128/20221012-ntp.pcap                         #pcap包路径，加载pcap包时打开此注释
  ~~~


## FAQ

Bug Report

Original version: LSLIDAR_LS128_ROS_V1.0.1_221012_ROS2

Modification: Original version

Date: 2022-10-12

----------------

Original version: LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2

Modifications: 
1. FPGA version upgrade, modification of point cloud calculation formula
​2. Added support for ROS2 Humble, ROS2 Dashing, and ROS2 Eloquent

Date: 2022-11-28

