#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);



class MinimalPublisher : public rclcpp::Node
{
  public:
  
    MinimalPublisher(): Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::PointCloud();
      //message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Denemeeee");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
    size_t count_;
};










int
main (int argc, char * argv[])
{

  /**************************/
  



  /**************************/


  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../catkin_ws/src/pcl/doc/capture0001.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  /*for (const auto& point: *cloud)
    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;*/
  



  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../catkin_ws/src/pcl/doc/capture0002.pcd", *cloud2) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud2->width * cloud2->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  /*for (const auto& point: *cloud)
    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;*/

  

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return (0);
}