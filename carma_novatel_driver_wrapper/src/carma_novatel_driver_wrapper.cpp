/*
 * Copyright (C) 2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include <memory>

#include <unordered_set>
#include "carma_novatel_driver_wrapper/carma_novatel_driver_wrapper.hpp"
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uncertainty_tools/uncertainty_tools.h>

using std_msec = std::chrono::milliseconds;

namespace carma_novatel_driver_wrapper
{   
    CarmaNovatelDriverWrapper::CarmaNovatelDriverWrapper(const rclcpp::NodeOptions &options)
            : CarmaLifecycleNode(options),
            lifecycle_mgr_(get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(), get_node_services_interface())
    {
        
    }
    
    void CarmaNovatelDriverWrapper::inspvax_callback(const novatel_oem7_msgs::msg::INSPVAX::UniquePtr msg)
    {
        last_imu_msg_ = this->now();

        gps_msgs::msg::GPSFix fix_msg;
        // NOTE: At the moment CARMA's hardware interfaces do not use GPS status information
        // Therefore that information is not currently populated in this message
        // Convert position
        fix_msg.header = msg->header;
        fix_msg.latitude = msg->latitude;
        fix_msg.longitude = msg->longitude;
        fix_msg.altitude = msg->height;
        //Set position covariance
        fix_msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        fix_msg.position_covariance[0] = msg->longitude_stdev * msg->longitude_stdev;
        fix_msg.position_covariance[4] = msg->latitude_stdev * msg->latitude_stdev;
        fix_msg.position_covariance[8] = msg->height_stdev * msg->height_stdev;

        //Convert orientation
        // NOTE: It is unclear in the message spec what dip represents so it will be ignored
        fix_msg.track = msg->azimuth;
        fix_msg.pitch = msg->pitch;
        fix_msg.roll = msg->roll;

        // GPSFix messages request uncertainty reported with 95% confidence interval. That is 2 times the standard deviation
        fix_msg.err_track = 2.0 * msg->azimuth_stdev;
        fix_msg.err_pitch = 2.0 * msg->pitch_stdev;
        fix_msg.err_roll = 2.0 * msg->roll_stdev;

        // Convert Velocity
        std::vector<double> components, variances;
        components.push_back(msg->north_velocity);
        variances.push_back(2.0 * msg->north_velocity_stdev);

        components.push_back(msg->east_velocity);
        variances.push_back(2.0 * msg->east_velocity_stdev);

        std::tuple<double, double> speed_tuple = uncertainty_tools::computeVectorMagnitudeAndUncertainty(components, variances);
        fix_msg.speed = std::get<0>(speed_tuple);
        fix_msg.climb = msg->up_velocity;

        fix_msg.err_climb = 2.0 * msg->up_velocity_stdev;
        fix_msg.err_speed = std::get<1>(speed_tuple);

        fix_fused_pub_->publish(fix_msg);

    }

    void CarmaNovatelDriverWrapper::imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        last_gnss_msg_ = this->now();
    }

    
    carma_ros2_utils::CallbackReturn CarmaNovatelDriverWrapper::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Novatel Driver wrapper trying to configure");

        //Load Parameters
        config_.imu_timeout = this->declare_parameter<int64_t>("imu_timout", config_.imu_timeout);
        config_.gnss_timeout = this->declare_parameter<int64_t>("gnss_timout", config_.gnss_timeout);
        config_.subsystem_namespace = this->declare_parameter<std::string>("subsystem_namespace", config_.subsystem_namespace);

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);
        
        //Add subscribers and publishers
        inspvax_sub_ = create_subscription<novatel_oem7_msgs::msg::INSPVAX>("INSPVAX", 5,
            std::bind(&CarmaNovatelDriverWrapper::inspvax_callback, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_raw", 5,
            std::bind(&CarmaNovatelDriverWrapper::imu_callback, this, std::placeholders::_1));

        fix_fused_pub_ = create_publisher<gps_msgs::msg::GPSFix>("gnss_fix_fused", 10.0);
        alert_pub_ = create_publisher<carma_msgs::msg::SystemAlert>("system_alert",10);

        //Initialize lifecycle manager
        std::vector <std::string> managed_nodes;
        managed_nodes.push_back(config_.subsystem_namespace);
        lifecycle_mgr_.set_managed_nodes(managed_nodes);

        return CallbackReturn::SUCCESS;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_novatel_driver_wrapper::CarmaNovatelDriverWrapper)