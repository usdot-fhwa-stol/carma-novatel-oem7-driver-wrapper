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

// #include "carma_novatel_driver_wrapper/carma_novatel_driver_wrapper_worker.hpp"

using std_msec = std::chrono::milliseconds;

namespace carma_novatel_driver_wrapper
{   
    CarmaNovatelDriverWrapper::CarmaNovatelDriverWrapper(const rclcpp::NodeOptions &options)
            : CarmaLifecycleNode(options),
            lifecycle_mgr_(get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(), get_node_services_interface())
    {
        
    }
    
    void CarmaNovatelDriverWrapper::initialize(){

        //Add subscribers and publishers
        // inspvax_sub_ = create_subscription<novatel_oem7_msgs::msg::INSPVAX>("INSPVAX", 5,
        //     std::bind(&CarmaNovatelDriverWrapperWorker::inspvax_callback, this, std::placeholders::_1));
        // imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu_raw", 5,
        //     std::bind(&CarmaNovatelDriverWrapperWorker::imu_callback, this, std::placeholders::_1));

        fix_fused_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("gnss_fix_fused", 10.0);
        status_pub_= node_->create_publisher<carma_driver_msgs::msg::DriverStatus>("driver_discovery",10);
        alert_pub_ = node_->create_publisher<carma_msgs::msg::SystemAlert>("system_alert",10);


    }

    carma_ros2_utils::CallbackReturn CarmaNovatelDriverWrapper::on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg)
    {
        RCLCPP_INFO_STREAM(
          node_->get_logger(), "Received SystemAlert message of type: " << static_cast<int>(msg->type) << " with message: " << msg->description);

        
        // NOTE: Here we check the required nodes not the full managed node set
        if (msg->type == carma_msgs::msg::SystemAlert::FATAL)
        {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(), "Failure in required node: " << msg->source_node);

            carma_msgs::msg::SystemAlert alert;
            alert.type = carma_msgs::msg::SystemAlert::FATAL;
            alert.description = config_.subsystem_namespace + " subsytem has failed with error: " + msg->description;
            alert.source_node = get_node_base_interface()->get_fully_qualified_name();
            publish_system_alert(alert);
        }
        else
        {// Optional node has failed
            RCLCPP_WARN_STREAM(
                node_->get_logger(), "Failure in optional node: " << msg->source_node);
        }
    }

    carma_ros2_utils::CallbackReturn CarmaNovatelDriverWrapper::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Novatel Driver wrapper trying to configure");

        //Load Parameters
        config_.imu_timeout = node_->declare_parameter<int64_t>("imu_timout", config_.imu_timeout);
        config_.gnss_timeout = node_->declare_parameter<int64_t>("gnss_timout", config_.gnss_timeout);
        config_.subsystem_namespace = node_->declare_parameter<std::string>("subsystem_namespace", config_.subsystem_namespace);

        RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded config: " << config_);

        // Create subscriptions
        system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(
        system_alert_topic_, 100,
        std::bind(&CarmaNovatelDriverWrapper::on_system_alert, this, std::placeholders::_1));
        
        //Initialize lifecycle manager
        std::vector <std::string> managed_nodes;
        managed_nodes.push_back(config_.subsystem_namespace);
        lifecycle_mgr_.set_managed_nodes(managed_nodes);

        // With all of our managed nodes now being tracked we can execute their configure operations
        bool success = lifecycle_mgr_.configure(std_msec(config_.service_timeout_ms), std_msec(config_.call_timeout_ms)).empty();

        if(success)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Subsystem able to configure");
            return CallbackReturn::SUCCESS;
        }
        else
        {

            RCLCPP_INFO_STREAM(node_->get_logger(), "Subsystem unable to configure");
            return CallbackReturn::FAILURE;
        }
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_novatel_driver_wrapper::CarmaNovatelDriverWrapper)