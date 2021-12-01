#include "gtest/gtest.h"
#include "carma_novatel_driver_wrapper/carma_novatel_driver_wrapper.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include <rclcpp/executor.hpp>

using std_msec = std::chrono::milliseconds;

TEST(TestCarmaNovatelDriverWrapper, Test_GPScalback){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<carma_novatel_driver_wrapper::CarmaNovatelDriverWrapper>(options);
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition
    std::unique_ptr<novatel_oem7_msgs::msg::INSPVAX> msg = std::make_unique<novatel_oem7_msgs::msg::INSPVAX>();
    msg->longitude = 74.5237892;
    msg->latitude = 38.2367299;
    msg->height = 32.0;
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(worker_node->get_node_base_interface());
    worker_node->inspvax_callback(move(msg));
    
    EXPECT_TRUE(std::abs(worker_node->gnss_fix_fused_msg_.longitude - 74.5237892) < 0.001);

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