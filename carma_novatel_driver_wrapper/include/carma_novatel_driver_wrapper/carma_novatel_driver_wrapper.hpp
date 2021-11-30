#pragma once

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

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include "carma_novatel_driver_wrapper/carma_novatel_driver_wrapper_config.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include <sensor_msgs/msg/imu.hpp>

namespace carma_novatel_driver_wrapper
{

  class CarmaNovatelDriverWrapper : public carma_ros2_utils::CarmaLifecycleNode
  {
    public:

    CarmaNovatelDriverWrapper() = delete;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit CarmaNovatelDriverWrapper(const rclcpp::NodeOptions &options);

    ~CarmaNovatelDriverWrapper() = default;
    

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    carma_ros2_utils::CallbackReturn on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg);
    
    std::string get_qualified_name();

    //! Lifecycle Manager which will track the managed nodes and call their lifecycle services on request
    ros2_lifecycle_manager::Ros2LifecycleManager lifecycle_mgr_;

    //Add Publishers
    std::shared_ptr<rclcpp::Publisher<gps_msgs::msg::GPSFix>> fix_fused_pub_;
    std::shared_ptr<rclcpp::Publisher<carma_msgs::msg::SystemAlert>> alert_pub_;
    
    
    //Add Subscribers
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr inspvax_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::string name_;

    CarmaNovatelDriverWrapperConfig config_;

    //Callbacks
    void inspvax_callback(const novatel_oem7_msgs::msg::INSPVAX::UniquePtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg);

    rclcpp::Time last_gnss_msg_;
    rclcpp::Time last_imu_msg_;

  };

} //namespace carma_novatel_driver_wrapper