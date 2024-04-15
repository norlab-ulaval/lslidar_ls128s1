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

#include "lslidar_driver/lslidar_driver.h"

#include <memory>

namespace lslidar_driver {
    using namespace std::chrono;
    lslidarDriver::lslidarDriver() : lslidarDriver(rclcpp::NodeOptions()) {
        return;
    }

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions &options) : Node("lslidar_node", options), socket_id(-1),
                                                                       time_synchronization(false),
                                                                       min_range(0.15),
                                                                       max_range(200),
                                                                       publish_point_cloud(true),
                                                                       publish_laserscan(false),
                                                                       packet_end_time(0.0),
                                                                       current_packet_time(0.0),
                                                                       last_packet_time(0.0),
                                                                       channel_num(0),
                                                                       horizontal_angle_resolution(0.2),
                                                                       sweep_data(
                                                                               new lslidar_msgs::msg::LslidarScan()),
                                                                       sweep_data_bak(
                                                                               new lslidar_msgs::msg::LslidarScan()),
                                                                       return_mode(1),
                                                                       point_time(0.0),
                                                                       g_fAngleAcc_V(0.01){

        return;
    }

    lslidarDriver::~lslidarDriver() {
        if (nullptr == difop_thread_) {
            difop_thread_->join();
        }
        (void) close(socket_id);
        return;
    }


    bool lslidarDriver::loadParameters() {
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<std::string>("device_ip", "192.168.1.201");
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<int>("scan_num", 8);
        this->declare_parameter<bool>("time_synchronization", false);
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<int>("scan_start_angle", -60);
        this->declare_parameter<int>("scan_end_angle", 60);
        this->declare_parameter<bool>("coordinate_opt", false);

        msop_udp_port = 0;
        difop_udp_port = 0;
        this->get_parameter("pcap", dump_file);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("scan_num", channel_num);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("scan_start_angle", scan_start_angle);
        this->get_parameter("scan_end_angle", scan_end_angle);
        this->get_parameter("time_synchronization", time_synchronization);
        this->get_parameter("publish_laserscan", publish_laserscan);
        this->get_parameter("topic_name", pointcloud_topic);

        RCLCPP_INFO(this->get_logger(), "dump_file: %s", dump_file.c_str());
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "group_ip_string: %s", group_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "add_multicast: %d", add_multicast);
        RCLCPP_INFO(this->get_logger(), "device_ip: %s", lidar_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "msop_port: %d", msop_udp_port);
        RCLCPP_INFO(this->get_logger(), "difop_udp_port: %d", difop_udp_port);
        RCLCPP_INFO(this->get_logger(), "min_range: %f", min_range);
        RCLCPP_INFO(this->get_logger(), "max_range: %f", max_range);
        RCLCPP_INFO(this->get_logger(), "scan_start_angle: %d", scan_start_angle);
        RCLCPP_INFO(this->get_logger(), "scan_end_angle: %d", scan_end_angle);
        RCLCPP_INFO(this->get_logger(), "time_synchronization: %d", time_synchronization);
        RCLCPP_INFO(this->get_logger(), "topic_name: %s", pointcloud_topic.c_str());

        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        if (add_multicast)
            RCLCPP_INFO(this->get_logger(), "opening UDP socket: group_address %s", group_ip_string.c_str());
        return true;
    }

    bool lslidarDriver::createRosIO() {
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        double packet_rate = 11228;
        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(this, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(this, difop_udp_port, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(this, msop_udp_port));
            difop_input_.reset(new lslidar_driver::InputSocket(this, difop_udp_port));
        }

        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&lslidarDriver::difopPoll, this)));
        return true;
    }

    bool lslidarDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot load all required ROS parameters...");
            return false;
        }

        if (!createRosIO()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot create all ROS IO...");
            return false;
        }
        return true;
    }

    bool lslidarDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(
                new lslidar_msgs::msg::LslidarPacket());

        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            auto startTime = std::chrono::system_clock::now();
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) {
                auto endTime = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
                auto spentTime = double(duration.count()) * std::chrono::microseconds::period::num;   //s
                RCLCPP_DEBUG(this->get_logger(), "polling time: %f microseconds",spentTime);
                break;       // got a full packet?
            }
            if (rc < 0) return false; // end of file reached?
        }

        // publish message using time of last packet read
        RCLCPP_DEBUG(this->get_logger(), "Publishing a full lslidar scan");
        if (time_synchronization) {    // todo 可能要改
            // it is already the msop msg
            // use the first packets
            lslidar_msgs::msg::LslidarPacket pkt = *packet;
            if (0xff == pkt.data[1194]) {    //ptp授时
                //std::cout << "ptp";
                uint64_t timestamp_s = (pkt.data[1195] * pow(2, 32) + pkt.data[1196] * pow(2, 24) +
                                        pkt.data[1197] * pow(2, 16) +
                                        pkt.data[1198] * pow(2, 8) + pkt.data[1199] * pow(2, 0));
                uint64_t timestamp_nsce = (pkt.data[1200] * pow(2, 24) + pkt.data[1201] * pow(2, 16) +
                                           pkt.data[1202] * pow(2, 8) +
                                           pkt.data[1203] * pow(2, 0));
                timeStamp = rclcpp::Time(timestamp_s, timestamp_nsce);  // s,ns
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            } else {          //gps授时
                this->packetTimeStamp[4] = pkt.data[1199];
                this->packetTimeStamp[5] = pkt.data[1198];
                this->packetTimeStamp[6] = pkt.data[1197];
                this->packetTimeStamp[7] = pkt.data[1196];
                this->packetTimeStamp[8] = pkt.data[1195];
                this->packetTimeStamp[9] = pkt.data[1194];
                struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time)); //s
                uint64_t packet_timestamp;
                packet_timestamp = (pkt.data[1203] +
                                    pkt.data[1202] * pow(2, 8) +
                                    pkt.data[1201] * pow(2, 16) +
                                    pkt.data[1200] * pow(2, 24)); //ns

                timeStamp = rclcpp::Time(pointcloudTimeStamp, packet_timestamp);  // s,ns
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }
        } else {
            packet->stamp = get_clock()->now();
            current_packet_time = rclcpp::Time(packet->stamp).seconds();
        }
        packet->prism_angle[0] = this->prism_angle[0];
        packet->prism_angle[1] = this->prism_angle[1];
        packet->prism_angle[2] = this->prism_angle[2];
        packet->prism_angle[3] = this->prism_angle[3];

        auto startTime = std::chrono::system_clock::now();
        lslidarChPacketProcess(packet);

        auto endTime = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        auto spentTime =
                double(duration.count()) * std::chrono::microseconds::period::num;   //s
        RCLCPP_DEBUG(this->get_logger(), "lslidarChPacketProcess time: %f microseconds",spentTime);
        return true;
    }

    void lslidarDriver::initTimeStamp(void) {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
        this->pointcloudTimeStamp = 0;
        this->timeStamp = rclcpp::Time(0.0);
    }

    void lslidarDriver::difopPoll(void) {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(
                new lslidar_msgs::msg::LslidarPacket());
        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                if (difop_packet->data[0] == 0x00 ||  difop_packet->data[0] == 0xa5)
                {
                    if(difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                       difop_packet->data[3] == 0x5a) {
                        //getFPGA_GPSTimeStamp(difop_packet);
                        int majorVersion = difop_packet->data[1202];
                        int minorVersion1 = difop_packet->data[1203] / 16;
                        int minorVersion2 = difop_packet->data[1203] % 16;

                        //v1.1 :0.01   //v1.2以后  ： 0.0025
                        if (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) {
                            g_fAngleAcc_V = 0.0025;
                        }else {
                            g_fAngleAcc_V = 0.01;
                        }

                        //getFPGA_GPSTimeStamp(difop_packet);
                        float fInitAngle_V = difop_packet->data[188] * 256 + difop_packet->data[189];
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[0] = fInitAngle_V * g_fAngleAcc_V;

                        fInitAngle_V = difop_packet->data[190] * 256 + difop_packet->data[191];
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[1] = fInitAngle_V * g_fAngleAcc_V;

                        fInitAngle_V = difop_packet->data[192] * 256 + difop_packet->data[193];
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[2] = fInitAngle_V * g_fAngleAcc_V;

                        fInitAngle_V = difop_packet->data[194] * 256 + difop_packet->data[195];
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[3] = fInitAngle_V * g_fAngleAcc_V;
                    }
                }

            } else if (rc < 0) {
                return;
            }
        }
    }

    void lslidarDriver::publishPointCloud() {
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        pcl::PointCloud<VPoint>::Ptr point_cloud(new pcl::PointCloud<VPoint>);
        point_cloud->header.frame_id = frame_id;
        point_cloud->height = 1;

        // The first and last point in each scan is ignored, which
        // seems to be corrupted based on the received data.
        // TODO: The two end points should be removed directly
        //    in the scans.
        VPoint point;
        auto startTime = system_clock::now();
        if (!sweep_data_bak->points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "pointcloud size %zu .",sweep_data_bak->points.size());
            point_cloud->header.stamp = static_cast<uint64_t>(point_time * 1e6);
            for (size_t j = 1; j < sweep_data_bak->points.size() - 1; ++j) {
                if ((sweep_data_bak->points[j].azimuth < scan_start_angle) or
                    (sweep_data_bak->points[j].azimuth > scan_end_angle)) {
                    continue;
                }

                point.timestamp = sweep_data_bak->points[j].time;
                point.x = sweep_data_bak->points[j].x;
                point.y = sweep_data_bak->points[j].y;
                point.z = sweep_data_bak->points[j].z;
                point.intensity = sweep_data_bak->points[j].intensity;
                point.ring = sweep_data_bak->points[j].line;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }

            auto startTime2 = system_clock::now();
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pointcloud_pub->publish(pc_msg);
            //RCLCPP_INFO(this->get_logger(), "publish pointcloud..");
            auto endTime = system_clock::now();
            auto duration = duration_cast<microseconds>(endTime - startTime2);
            auto spentTime = double(duration.count()) * microseconds::period::num;   //s
            RCLCPP_DEBUG(this->get_logger(), "pointcloud convert time: %f microseconds",spentTime);
        }
        auto endTime = system_clock::now();
        auto duration = duration_cast<microseconds>(endTime - startTime);
        auto spentTime =  double(duration.count()) * microseconds::period::num;   //s
        RCLCPP_DEBUG(this->get_logger(), "puhlish pointcloud time: %f microseconds",spentTime);
    }

    void lslidarDriver::publishLaserScan() {
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg(new sensor_msgs::msg::LaserScan());
        scan_msg->header.frame_id = frame_id;
        if (time_synchronization) {
            scan_msg->header.stamp = packet_timeStamp;
        } else {
            scan_msg->header.stamp = rclcpp::Time();
        }
        scan_msg->angle_min = DEG2RAD(30);
        scan_msg->angle_max = DEG2RAD(150);
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
        scan_msg->angle_increment = horizontal_angle_resolution;  //todo 角度分辨率要改
        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        if (sweep_data_bak->points.size() > 0) {
            for (size_t j = 0; j < sweep_data_bak->points.size() - 1; ++j) {
                if (channel_num == sweep_data_bak->points[j].line) {
                    float horizontal_angle = sweep_data_bak->points[j].azimuth;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index <= point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_index] = sweep_data_bak->points[j].distance * g_fDistanceAcc;
                    scan_msg->intensities[point_index] = sweep_data_bak->points[j].intensity;
                }
            }
            scan_pub->publish(std::move(scan_msg));
        }
    }

    int lslidarDriver::convertCoordinate(struct Firing lidardata) {
        if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) {
            return -1;
        }
        double fAngle_H = 0.0;         //水平角度
        double fAngle_V = 0.0;        // 垂直角度
        fAngle_H = lidardata.azimuth;
        fAngle_V = lidardata.vertical_angle;
        //不加畸变的
//        根据通道不同偏移角度不同
//        fAngle_V = lidardata.vertical_angle + lidardata.prism_angle[lidardata.channel_number];
//
//        fAngle_H = lidardata.azimuth;
//        ROS_INFO_ONCE("fAngle_H = %f, fAngle_V = %f", fAngle_H, fAngle_V);
//        double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
//        x_coord = (lidardata.distance * cos(DEG2RAD(fAngle_V)) * sin(DEG2RAD(fAngle_H))) * g_fDistanceAcc;
//        y_coord = (lidardata.distance * cos(DEG2RAD(fAngle_V)) * cos(DEG2RAD(fAngle_H))) * g_fDistanceAcc;
//        z_coord = (lidardata.distance * sin(DEG2RAD(fAngle_V))) * g_fDistanceAcc;

        //加畸变
        double fSinV_angle = 0;
        double fCosV_angle = 0;
        //摆镜角度
        float fPutTheMirrorOffAngle = 0;
        //根据通道不同偏移角度不同
        switch (lidardata.channel_number) {
            case 0:
                fPutTheMirrorOffAngle = 0;   //todo 1.6
                break;
            case 1:
                fPutTheMirrorOffAngle = -2;  //todo -0.4
                break;
            case 2:
                fPutTheMirrorOffAngle = -1;  //todo 0.6
                break;
            case 3:
                fPutTheMirrorOffAngle = -3;  //todo -1.4
                break;
            default:
                fPutTheMirrorOffAngle = -1.4;
                break;
        }

        //振镜偏移角度 = 实际垂直角度 / 2  - 偏移值
        double fGalvanometrtAngle = 0;
        //fGalvanometrtAngle = (((fAngle_V + 0.05) / 0.8) + 1) * 0.46 + 6.72;
        fGalvanometrtAngle = fAngle_V + 7.26;

        double fAngle_R0 = cos(DEG2RAD(30)) * cos(DEG2RAD(fPutTheMirrorOffAngle)) * cos(DEG2RAD(fGalvanometrtAngle)) -
                           sin(DEG2RAD(fGalvanometrtAngle)) * sin(DEG2RAD(fPutTheMirrorOffAngle));

        fSinV_angle = 2 * fAngle_R0 * sin(DEG2RAD(fGalvanometrtAngle)) + sin(DEG2RAD(fPutTheMirrorOffAngle));
        fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

        double fSinCite = (2 * fAngle_R0 * cos(DEG2RAD(fGalvanometrtAngle)) * sin(DEG2RAD(30)) -
                           cos(DEG2RAD(fPutTheMirrorOffAngle)) * sin(DEG2RAD(60))) / fCosV_angle;
        double fCosCite = sqrt(1 - pow(fSinCite, 2));

        double fSinCite_H = sin(DEG2RAD(fAngle_H)) * fCosCite + cos(DEG2RAD(fAngle_H)) * fSinCite;
        double fCosCite_H = cos(DEG2RAD(fAngle_H)) * fCosCite - sin(DEG2RAD(fAngle_H)) * fSinCite;

        double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
        x_coord = (lidardata.distance * fCosV_angle * fSinCite_H) * g_fDistanceAcc;
        y_coord = (lidardata.distance * fCosV_angle * fCosCite_H) * g_fDistanceAcc;
        z_coord = (lidardata.distance * fSinV_angle) * g_fDistanceAcc;

        sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
        lslidar_msgs::msg::LslidarPoint &new_point =        // new_point 为push_back最后一个的引用
                sweep_data->points[sweep_data->points.size() - 1];
        new_point.x = x_coord;
        new_point.y = y_coord;
        new_point.z = z_coord;
        new_point.vertical_angle = fAngle_V;
        new_point.azimuth = lidardata.azimuth;
//        ROS_INFO("azimuth: %f", new_point.azimuth);
        new_point.distance = lidardata.distance;
        new_point.intensity = lidardata.intensity;
        new_point.line = lidardata.channel_number;   // todo 需要加上线号
        new_point.time = lidardata.time;
        return 0;
    }

    void lslidarDriver::lslidarChPacketProcess(
            const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg) {
        struct Firing lidardata{};
        // Convert the msg to the raw packet type.

        packet_timeStamp = msg->stamp;
        packet_end_time = packet_timeStamp.seconds();
        bool packetType = false;
        if (msg->data[1205] == 0x02) {    //双回波
            return_mode = 2;
        }

        if (return_mode == 1) {
            auto startTime = system_clock::now();
            double packet_interval_time =
                    (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_SINGLE_ECHO / 8.0);

            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8) {
                if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                    (msg->data[point_idx + 2] == 0xbb) && (msg->data[point_idx + 3] == 0xcc) &&
                    (msg->data[point_idx + 4] == 0xdd)) {
                    packetType = true;
                } else {
                    // Compute the time of the point
                    if (last_packet_time > 1e-6) {
                        point_time =
                                packet_end_time -
                                packet_interval_time * ((POINTS_PER_PACKET_SINGLE_ECHO - point_idx) / 8 - 1);
                        point_time = point_time < 0.0 ? 0.0 : point_time;
                        if (point_time <0.0) {
                            RCLCPP_INFO(this->get_logger(), "point_time: %.9f",point_time);
                        }
                    } else {
                        point_time = current_packet_time;
                    }

//                if (msg->data[point_idx] < 255) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    //水平角度
                    double fAngle_H = msg->data[point_idx + 1] + msg->data[point_idx] * 256;
                    if (fAngle_H > 32767) {
                        fAngle_H = (fAngle_H - 65536);
                    }
                    lidardata.azimuth = fAngle_H * 0.01;
                    //垂直角度+通道号
                    int iTempAngle = msg->data[point_idx + 2];
                    int iChannelNumber = iTempAngle >> 6; //左移六位 通道号
                    int iSymmbol = (iTempAngle >> 5) & 0x01; //左移五位 符号位
                    double fAngle_V = 0.0;
                    if (1 == iSymmbol) // 符号位 0：正数 1：负数
                    {
                        int iAngle_V = msg->data[point_idx + 3] + msg->data[point_idx + 2] * 256;

                        fAngle_V = iAngle_V | 0xc000;
                        if (fAngle_V > 32767) {
                            fAngle_V = (fAngle_V - 65536);
                        }
                    } else {
                        int iAngle_Hight = iTempAngle & 0x3f;
                        fAngle_V = msg->data[point_idx + 3] + iAngle_Hight * 256;
                    }
                    // 棱镜角度
                    for (int i = 0; i < 4; ++i) {
                        lidardata.prism_angle[i] = msg->prism_angle[i];
                    }

                    lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;  // todo 改变
                    lidardata.channel_number = iChannelNumber;
                    lidardata.distance = (msg->data[point_idx + 4] * 65536 + msg->data[point_idx + 5] * 256 +
                                          msg->data[point_idx + 6]);
                    lidardata.intensity = msg->data[point_idx + 7];
                    lidardata.time = point_time;
                    lidardata.azimuth = fAngle_H * 0.01;
                    convertCoordinate(lidardata);
//                }
                }

                if (packetType) {
//                    //("---------------onesweep--------------------------\n");
                    if (publish_point_cloud) {
                        auto startTime1 = system_clock::now();
                        {
                            std::unique_lock<std::mutex> lock(pointcloud_lock);
                            sweep_data_bak = std::move(sweep_data);
                        }

                        auto endTime1 = system_clock::now();
                        auto duration = duration_cast<microseconds>(endTime1 - startTime1);
                        auto spentTime =
                                double(duration.count()) * microseconds::period::num;   //s
//                        std::cout << "cost time: " << spentTime << std::endl;
                        std::thread pointcloud_pub_thread([this] { publishPointCloud(); });   //todo
                        pointcloud_pub_thread.detach();
                    }
                    packetType = false;
                    sweep_data = std::make_unique<lslidar_msgs::msg::LslidarScan>();
//                    std::cout << "new sweep_data: " <<sweep_data <<std::endl;
                }
            }
            auto endTime = system_clock::now();
            auto duration = duration_cast<microseconds>(endTime - startTime);
            auto spentTime =
                    double(duration.count()) * microseconds::period::num;   //s
            RCLCPP_DEBUG(this->get_logger(), "for loop time: %f microseconds",spentTime);
        } else {
            double packet_interval_time =
                    (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_DOUBLE_ECHO / 12.0);
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12) {
                if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                    (msg->data[point_idx + 2] == 0xbb) && (msg->data[point_idx + 3] == 0xcc) &&
                    (msg->data[point_idx + 4] == 0xdd)) {
                    packetType = true;
                } else {
                    // Compute the time of the point
                    if (last_packet_time > 1e-6) {
                        point_time =
                                packet_end_time -
                                packet_interval_time * ((POINTS_PER_PACKET_DOUBLE_ECHO - point_idx) / 12 - 1);
                    } else {
                        point_time = current_packet_time;
                    }
//                if (msg->data[point_idx] < 255) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    //水平角度
                    double fAngle_H = msg->data[point_idx + 1] + msg->data[point_idx] * 256;
                    if (fAngle_H > 32767) {
                        fAngle_H = (fAngle_H - 65536);
                    }
                    lidardata.azimuth = fAngle_H * 0.01;

                    //垂直角度+通道号
                    int iTempAngle = msg->data[point_idx + 2];
                    int iChannelNumber = iTempAngle >> 6; //左移六位 通道号
                    int iSymmbol = (iTempAngle >> 5) & 0x01; //左移五位 符号位
                    double fAngle_V = 0.0;
                    if (1 == iSymmbol) // 符号位 0：正数 1：负数
                    {
                        int iAngle_V = msg->data[point_idx + 3] + msg->data[point_idx + 2] * 256;

                        fAngle_V = iAngle_V | 0xc000;
                        if (fAngle_V > 32767) {
                            fAngle_V = (fAngle_V - 65536);
                        }
                    } else {
                        int iAngle_Hight = iTempAngle & 0x3f;
                        fAngle_V = msg->data[point_idx + 3] + iAngle_Hight * 256;
                    }

                    // 棱镜角度
                    for (int i = 0; i < 4; ++i) {
                        lidardata.prism_angle[i] = msg->prism_angle[i];
                    }

                    lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;   // todo 改变
                    lidardata.channel_number = iChannelNumber;
                    lidardata.distance = (msg->data[point_idx + 4] * 65536 + msg->data[point_idx + 5] * 256 +
                                          msg->data[point_idx + 6]);
                    lidardata.intensity = msg->data[point_idx + 7];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);  // 第一个点

                    lidardata.distance = (msg->data[point_idx + 8] * 65536 + msg->data[point_idx + 9] * 256 +
                                          msg->data[point_idx + 10]);
                    lidardata.intensity = msg->data[point_idx + 11];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);  // 第二个点
//                }
                }

                if (packetType) {
                    //("---------------onesweep--------------------------\n");
                    if (publish_point_cloud) {
                        {
                            std::unique_lock<std::mutex> lock(pointcloud_lock);
                            sweep_data_bak = std::move(sweep_data);
                        }
                        std::thread pointcloud_pub_thread([this] { publishPointCloud(); });
                        pointcloud_pub_thread.detach();
                    }
                    packetType = false;
                    sweep_data = std::make_unique<lslidar_msgs::msg::LslidarScan>();
                }
            }
        }
        last_packet_time = current_packet_time;
    }
}  // namespace lslidar_driver
