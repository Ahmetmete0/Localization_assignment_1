#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include "std_msgs/msg/float64.hpp"


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>




#include <pcl/visualization/pcl_visualizer.h>


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

using namespace std;

std_msgs::msg::Float64 score;
Eigen::Matrix4f transform_matrix;

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);


  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  


  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (5);
  for (int i = 0; i < 30; ++i)
  {

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

  }

  transform_matrix   = reg.getFinalTransformation();
  score.data = reg.getFitnessScore();

  
	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  
  final_transform = targetToSource;
 }

class MinimalPublisher : public rclcpp::Node
{
  public:

    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_1;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_2;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_align_publisher;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pc_publisher_error;

      point_cloud_publisher_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_1", 10);
      point_cloud_publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_2", 10);
      point_cloud_align_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_align", 10);
      

      sensor_msgs::msg::PointCloud2 ros_cloud1;
      sensor_msgs::msg::PointCloud2 ros_cloud2;
      sensor_msgs::msg::PointCloud2 aligned_ros;

      

      PointCloud::Ptr result (new PointCloud), source, target;
      Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;


      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);


     

      if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../catkin_ws/src/pcl/doc/capture0001.pcd", *cloud_1) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        
      }

      if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../catkin_ws/src/pcl/doc/capture0002.pcd", *cloud_2) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        
      }

      pcl::toROSMsg(*cloud_1, ros_cloud1);
      pcl::toROSMsg(*cloud_2, ros_cloud2);


      source = cloud_1;
      target = cloud_2;


      PointCloud::Ptr temp (new PointCloud);
      pairAlign (source, target, temp, pairTransform, true);

      //transform current pair into the global transform
      pcl::transformPointCloud (*temp, *result, GlobalTransform);

      //update the global transform
      GlobalTransform *= pairTransform;

      pcl::toROSMsg(*result, aligned_ros);

      RCLCPP_INFO_STREAM(this->get_logger(), "transform matrix: " << "\n" << transform_matrix << "\n"
                                            << "fitness score: " << score.data << "\n");


      //save aligned pair, transformed into the first cloud's frame
      std::stringstream ss;
      ss << "output" << ".pcd";
      pcl::io::savePCDFile (ss.str (), *result, true);





      while (rclcpp::ok())
      {
        ros_cloud1.header.frame_id = "map";
        ros_cloud1.header.stamp = this->get_clock()->now();

        ros_cloud2.header.frame_id = "map";
        ros_cloud2.header.stamp = this->get_clock()->now();

        aligned_ros.header.frame_id = "map";
        aligned_ros.header.stamp = this->get_clock()->now();

        point_cloud_publisher_1->publish(ros_cloud1);
        point_cloud_publisher_2->publish(ros_cloud2);
        point_cloud_align_publisher->publish(aligned_ros);


        /*auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);*/
      }



    }

    

    
    size_t count_;
};




////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */



/* ---[ */
int main (int argc, char** argv)
{
  // Load data
  //std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  

	PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
  /*for (std::size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1].cloud;
    target = data[i].cloud;


    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%zu) with %s (%zu).\n", data[i-1].f_name.c_str (), static_cast<std::size_t>(source->size ()), data[i].f_name.c_str (), static_cast<std::size_t>(target->size ()));
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform
    GlobalTransform *= pairTransform;

    //save aligned pair, transformed into the first cloud's frame
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);

  }*/


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
}
/* ]--- */