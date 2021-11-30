#include "gtest/gtest.h"
#include "carma_novatel_driver_wrapper/carma_novatel_driver_wrapper.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include <rclcpp/executor.hpp>

using std_msec = std::chrono::milliseconds;

TEST(TestCarmaNovatelDriverWrapper, Test_gps_callback){

    auto node = rclcpp::Node::make_shared("test_node");
    std::string topic = "test_subscription";

    auto publisher = node->create_publisher<novatel_oem7_msgs::msg::INSPVAX>("INSPVAX", 10);
    novatel_oem7_msgs::msg::INSPVAX inspvax_msg;
    inspvax_msg.longitude = 34.0;
    
    gps_msgs::msg::GPSFix gps_msg_cb;

    std::promise<void> sub_called;
    std::shared_future<void> sub_called_future(sub_called.get_future());

    auto callback =
    [&gps_msg_cb, &sub_called](const gps_msgs::msg::GPSFix::SharedPtr msg) -> void
    {
        gps_msg_cb.longitude = msg->longitude;
        sub_called.set_value();
    };

    novatel_oem7_msgs::msg::INSPVAX gps_msg_pub;
    gps_msg_pub.longitude = 34.5;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    publisher->publish(gps_msg_pub);

    auto worker_node = std::make_shared<carma_novatel_driver_wrapper::CarmaNovatelDriverWrapper>(rclcpp::NodeOptions());
    rclcpp_lifecycle::State state_now;
    worker_node->on_configure(state_now);
    executor.add_node(worker_node->get_node_base_interface());

    worker_node->on_activate(state_now);

    rclcpp::FutureReturnCode future_ret;
    auto timeout = std::chrono::milliseconds(100);
    future_ret = executor.spin_until_future_complete(sub_called_future, timeout);

    for(int i = 0;i< 5;i++){
        // executor.spin_some();
        timeout = std::chrono::milliseconds(10);
        future_ret = executor.spin_until_future_complete(sub_called_future, timeout);
        publisher->publish(gps_msg_pub);
    }
    
    auto subscriber = node->create_subscription<gps_msgs::msg::GPSFix>("gnss_fix_fused", 10, callback);
    
    timeout = std::chrono::milliseconds(100);
    future_ret = executor.spin_until_future_complete(sub_called_future, timeout);

    EXPECT_TRUE(gps_msg_cb.longitude == gps_msg_pub.longitude);

}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}