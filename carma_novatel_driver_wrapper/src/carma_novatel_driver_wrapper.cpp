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
            : CarmaLifecycleNode(options)
    {
        config_ = CarmaNovatelDriverWrapperConfig();
        config_.imu_timeout = this->declare_parameter<double>("imu_timeout", config_.imu_timeout);
        config_.gnss_timeout = this->declare_parameter<double>("gnss_timeout", config_.gnss_timeout);
        config_.timer_callback = this->declare_parameter<int>("timer_callback", config_.timer_callback);
    }
    
    void CarmaNovatelDriverWrapper::inspvax_callback(const novatel_oem7_msgs::msg::INSPVAX::UniquePtr msg)
    {
        last_gnss_msg_ = this->now();

        // NOTE: At the moment CARMA's hardware interfaces do not use GPS status information
        // Therefore that information is not currently populated in this message
        // Convert position
        gnss_fix_fused_msg_.header = msg->header;
        gnss_fix_fused_msg_.latitude = msg->latitude;
        gnss_fix_fused_msg_.longitude = msg->longitude;
        gnss_fix_fused_msg_.altitude = msg->height;
        //Set position covariance
        gnss_fix_fused_msg_.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gnss_fix_fused_msg_.position_covariance[0] = msg->longitude_stdev * msg->longitude_stdev;
        gnss_fix_fused_msg_.position_covariance[4] = msg->latitude_stdev * msg->latitude_stdev;
        gnss_fix_fused_msg_.position_covariance[8] = msg->height_stdev * msg->height_stdev;

        //Convert orientation
        // NOTE: It is unclear in the message spec what dip represents so it will be ignored
        gnss_fix_fused_msg_.track = msg->azimuth;
        gnss_fix_fused_msg_.pitch = msg->pitch;
        gnss_fix_fused_msg_.roll = msg->roll;

        // GPSFix messages request uncertainty reported with 95% confidence interval. That is 2 times the standard deviation
        gnss_fix_fused_msg_.err_track = 2.0 * msg->azimuth_stdev;
        gnss_fix_fused_msg_.err_pitch = 2.0 * msg->pitch_stdev;
        gnss_fix_fused_msg_.err_roll = 2.0 * msg->roll_stdev;

        // Convert Velocity
        std::vector<double> components, variances;
        components.push_back(msg->north_velocity);
        variances.push_back(2.0 * msg->north_velocity_stdev);

        components.push_back(msg->east_velocity);
        variances.push_back(2.0 * msg->east_velocity_stdev);

        std::tuple<double, double> speed_tuple = uncertainty_tools::computeVectorMagnitudeAndUncertainty(components, variances);
        gnss_fix_fused_msg_.speed = std::get<0>(speed_tuple);
        gnss_fix_fused_msg_.climb = msg->up_velocity;

        gnss_fix_fused_msg_.err_climb = 2.0 * msg->up_velocity_stdev;
        gnss_fix_fused_msg_.err_speed = std::get<1>(speed_tuple);

        fix_fused_pub_->publish(gnss_fix_fused_msg_);
    }

    void CarmaNovatelDriverWrapper::imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        last_imu_msg_ = this->now();
    }

    
    carma_ros2_utils::CallbackReturn CarmaNovatelDriverWrapper::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Novatel Driver wrapper trying to configure");

        //Create initial config
        config_ = CarmaNovatelDriverWrapperConfig();

        //Load Parameters
        this->get_parameter<double>("imu_timeout", config_.imu_timeout);
        this->get_parameter<double>("gnss_timeout", config_.gnss_timeout);
        this->get_parameter<int>("timer_callback", config_.timer_callback);

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);
        
        //Add subscribers and publishers
        inspvax_sub_ = create_subscription<novatel_oem7_msgs::msg::INSPVAX>("novatel/oem7/inspvax", 5,
            std::bind(&CarmaNovatelDriverWrapper::inspvax_callback, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_raw", 5,
            std::bind(&CarmaNovatelDriverWrapper::imu_callback, this, std::placeholders::_1));

        fix_fused_pub_ = create_publisher<gps_msgs::msg::GPSFix>("gnss_fix_fused", 10.0);
        
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn CarmaNovatelDriverWrapper::handle_on_activate(const rclcpp_lifecycle::State &){

        timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.timer_callback), 
        std::bind(&CarmaNovatelDriverWrapper::timerCallback, this));

        last_gnss_msg_ = this->now();
        last_imu_msg_ = this->now();

        return CallbackReturn::SUCCESS;
    }

    void CarmaNovatelDriverWrapper::timerCallback(){
        rclcpp::Time now = this->now();
        rclcpp::Duration duration_gnss = now - last_gnss_msg_;
        rclcpp::Duration duration_imu = now - last_imu_msg_;

        if(duration_gnss.seconds() > config_.gnss_timeout){
            throw std::invalid_argument("GPS message wait timed out");
        }

        if(duration_imu.seconds() > config_.imu_timeout){
            throw std::invalid_argument("IMU message wait timed out");
        }
   
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_novatel_driver_wrapper::CarmaNovatelDriverWrapper)