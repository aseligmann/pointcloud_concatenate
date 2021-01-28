#pragma once           // Only include once per compile
#ifndef POINTCLOUD_CONCATENATE  // Conditional compiling
#define POINTCLOUD_CONCATENATE

// Includes
#include <ros/ros.h>  // ROS header

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

// Macro to warn about unset parameters
#define ROSPARAM_WARN(param_name, default_val)                               \
  std::cout << "\033[33m"                                                    \
            << "[WARN] Param is not set: " << param_name                          \
            << ". Setting to default value: " << default_val << "\033[0m\n"  \
            << std::endl

// Define class
class PointcloudConcatenate {
public:
  // Constructor and destructor
  PointcloudConcatenate(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~PointcloudConcatenate();

  // Public functions
  void handleParams();
  void update();
  double getHz();

  // Public variables and objects
  
private:
  // Private functions
  void subCallbackCloudIn1(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudIn2(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudIn3(sensor_msgs::PointCloud2 msg);
  void subCallbackCloudIn4(sensor_msgs::PointCloud2 msg);
  void publishPointcloud(sensor_msgs::PointCloud2 cloud);

  // Private variables and objects
  ros::NodeHandle nh_;
  std::string node_name_;

  // Parameters
  std::string param_frame_target_;
  int param_clouds_;
  double param_hz_;

  // Publisher and subscribers
  ros::Subscriber sub_cloud_in1 = nh_.subscribe("cloud_in1", 1, &PointcloudConcatenate::subCallbackCloudIn1, this);
  ros::Subscriber sub_cloud_in2 = nh_.subscribe("cloud_in2", 1, &PointcloudConcatenate::subCallbackCloudIn2, this);
  ros::Subscriber sub_cloud_in3 = nh_.subscribe("cloud_in3", 1, &PointcloudConcatenate::subCallbackCloudIn3, this);
  ros::Subscriber sub_cloud_in4 = nh_.subscribe("cloud_in4", 1, &PointcloudConcatenate::subCallbackCloudIn4, this);
  ros::Publisher pub_cloud_out = nh_.advertise<sensor_msgs::PointCloud2>(node_name_ + "/cloud_out", 1);

  // Other

  sensor_msgs::PointCloud2 cloud_in1;
  sensor_msgs::PointCloud2 cloud_in2;
  sensor_msgs::PointCloud2 cloud_in3;
  sensor_msgs::PointCloud2 cloud_in4;
  sensor_msgs::PointCloud2 cloud_out;
  bool cloud_in1_received = false;
  bool cloud_in2_received = false;
  bool cloud_in3_received = false;
  bool cloud_in4_received = false;
  bool cloud_in1_received_recent = false;
  bool cloud_in2_received_recent = false;
  bool cloud_in3_received_recent = false;
  bool cloud_in4_received_recent = false;

  // Initialization tf2 listener
  boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
  boost::shared_ptr<tf2_ros::TransformListener> tfListener;
};

#endif