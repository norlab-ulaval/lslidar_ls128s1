/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/


#ifndef _LS_C16_DRIVER_H_
#define _LS_C16_DRIVER_H_

#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.29577951

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "input.h"
#include <std_msgs/msg/string.h>
#include <lslidar_msgs/msg/lslidar_packet.hpp>
#include <lslidar_msgs/msg/lslidar_point.hpp>
#include <lslidar_msgs/msg/lslidar_scan.hpp>
#include <lslidar_msgs/srv/lslidarcontrol.hpp>
#include <chrono>


namespace lslidar_driver {
    /** Special Defines for LSCh support **/
    static const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    static const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    static float g_fDistanceAcc = 0.1 * 0.01;

    struct PointXYZIRT {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        uint16_t ring;
        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    struct Firing {
        double vertical_angle;
//        int vertical_line;
        double azimuth;
        double distance;
        float intensity;
        double time;
        int channel_number;
        double prism_angle[4];
    };

    class lslidarDriver : public rclcpp::Node {
    private:
        void difopPoll(void);

        void initTimeStamp(void);

        bool isPointInRange(const double &distance) const {
            return (distance >= min_range && distance <= max_range);
        }

        int convertCoordinate(struct Firing lidardata);

        // Publish data
        void publishPointCloud();

        void publishLaserScan();

        void lslidarChPacketProcess(
                const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg);

        bool loadParameters();
        bool createRosIO();
        //socket Parameters
        int msop_udp_port;
        int difop_udp_port;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

        union two_bytes {
            int16_t value;
            char bytes[2];
        };
        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;

        in_addr lidar_ip;
        int socket_id;
        bool add_multicast;
        double prism_angle[4];

        // ROS related variables
        bool time_synchronization;
        uint64_t pointcloudTimeStamp;
        unsigned char packetTimeStamp[10];
        rclcpp::Time timeStamp;

        // Configuration parameters
        double min_range;
        double max_range;
        bool publish_point_cloud;
        bool publish_laserscan;
        rclcpp::Time packet_timeStamp;
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;
        int channel_num;
        double horizontal_angle_resolution;

        std::string pointcloud_topic;
        lslidar_msgs::msg::LslidarScan::UniquePtr sweep_data;
        lslidar_msgs::msg::LslidarScan::UniquePtr sweep_data_bak;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        int return_mode;
        int scan_start_angle;
        int scan_end_angle;
        double point_time;
        double g_fAngleAcc_V;
        std::mutex pointcloud_lock;

        typedef boost::shared_ptr<lslidarDriver> lslidarDriverPtr;
        typedef boost::shared_ptr<const lslidarDriver> lslidarDriverConstPtr;

    public:
        lslidarDriver();
        lslidarDriver(const rclcpp::NodeOptions &options);
        ~lslidarDriver();
        bool initialize();
        bool polling();
    };
    typedef PointXYZIRT VPoint;
}  // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIRT,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (double, timestamp, timestamp))

#endif
