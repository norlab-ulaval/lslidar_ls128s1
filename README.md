# LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2使用说明

## 1.工程介绍
​		LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2为linux环境下雷达ros驱动，适用于ls128,ls25d,ls180雷达，程序在ubuntu 20.04 ros foxy和ubuntu 20.04 ros galactic下测试通过。

## 2.依赖

1.ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~



## 3.编译运行

### 3.1 编译

~~~bash
mkdir -p ~/lidar_ws/src
#将LSLIDAR_LS128_ROS_V1.0.0_221012_ROS2压缩包解压缩放到~/lidar_ws/src 目录下
cd ~/lidar_ws
colcon build
source install/setup.bash
~~~

### 3.2 运行

运行单个雷达:

~~~bash
ros2 launch lslidar_driver lslidar_ls128.launch
~~~

运行多个雷达：

~~~bash
ros2 launch lslidar_driver lslidar_ls128_double.launch
~~~

## 4.参数介绍

lslidar_ls128.yaml文件内容如下，每个参数含义见注释说明。

~~~bash
/ls128/lslidar_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200          #雷达ip
    msop_port: 2368                   #数据包目的端口
    difop_port: 2369                  #设备包目的端口
    frame_id: laser_link              #坐标系id
    add_multicast: false              #是否开启组播模式
    group_ip: 224.1.1.2               #组播ip地址
    time_synchronization: false       #雷达是否使用gps或ptp授时，使用改为true
    min_range: 0.3                    #单位，米。雷达盲区最小值，小于此值的点被过滤
    max_range: 500.0                  #单位，米。雷达盲区最大值 ，大于此值的点被过滤
    scan_start_angle: -60              # 起始角度，范围[-60,60]
    scan_end_angle: 60                # 结束角度，范围[-60,60]
    scan_num: 15                       #laserscan线号
    publish_scan: false               #是否发布scan
    topic_name: lslidar_point_cloud   #点云话题名称，可修改
    #pcap: /home/ls/work/fuhong/data/ls128/20221012-ntp.pcap                        #pcap包路径，加载pcap包时打开此注释
~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  add_multicast: true                    #是否开启组播模式。
  group_ip: 224.1.1.2                    #组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 修改launch文件以下参数

  ~~~shell
  #取消注释
  pcap: /home/ls/work/fuhong/data/ls128/20221012-ntp.pcap                         #pcap包路径，加载pcap包时打开此注释
  ~~~


## FAQ

Bug Report

Original version : LSLIDAR_LS128_ROS_V1.0.1_221012_ROS2

Modify:  original version

Date    : 2022-10-12

----------------

Original version : LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2

Modify:  1.fpga版本升级，修改点云计算公式

​                2.新增对ros2 humble, ros2 dashing, ros2 eloquent的支持

Date    : 2022-11-28

