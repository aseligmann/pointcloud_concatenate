#include "pointcloud_concatenate/pointcloud_concatenate.hpp"

// Constructor
PointcloudConcatenate::PointcloudConcatenate(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  nh_ = nh;  // Set nodehandle
  node_name_ = ros::this_node::getName();

  // Initialise variables / parameters to class variables
  handleParams();

  // Initialization tf2 listener
  tfBuffer.reset(new tf2_ros::Buffer);
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  // Initialise publishers and subscribers
  // Queues size of 1 to only keep the most recent message
  sub_cloud_in1 = nh_.subscribe("cloud_in1", 1, &PointcloudConcatenate::subCallbackCloudIn1, this);
  sub_cloud_in2 = nh_.subscribe("cloud_in2", 1, &PointcloudConcatenate::subCallbackCloudIn2, this);
  sub_cloud_in3 = nh_.subscribe("cloud_in3", 1, &PointcloudConcatenate::subCallbackCloudIn3, this);
  sub_cloud_in4 = nh_.subscribe("cloud_in4", 1, &PointcloudConcatenate::subCallbackCloudIn4, this);
  pub_cloud_out = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
}

// Destructor
PointcloudConcatenate::~PointcloudConcatenate() {
  // Free up allocated memory
  ROS_INFO("Destructing PointcloudConcatenate...");
  // delete pointer_name;
}

void PointcloudConcatenate::subCallbackCloudIn1(sensor_msgs::PointCloud2 msg) {
  cloud_in1 = msg;
  cloud_in1_received = true;
  cloud_in1_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn2(sensor_msgs::PointCloud2 msg) {
  cloud_in2 = msg;
  cloud_in2_received = true;
  cloud_in2_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn3(sensor_msgs::PointCloud2 msg) {
  cloud_in3 = msg;
  cloud_in3_received = true;
  cloud_in3_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn4(sensor_msgs::PointCloud2 msg) {
  cloud_in4 = msg;
  cloud_in4_received = true;
  cloud_in4_received_recent = true;
}

void PointcloudConcatenate::handleParams() {
  // Handle parameters

  // Set parameters
  ROS_INFO("Loading parameters...");
  std::string param_name;

  // Target frame
  std::string parse_str;
  param_name = node_name_ + "/target_frame";
  ros::param::get(param_name, parse_str);
  if (!parse_str.length() > 0) {
    param_frame_target_ = "base_link";
    ROSPARAM_WARN(param_name, param_frame_target_);
  }
  param_frame_target_ = parse_str;

  // Number of pointclouds
  param_name = node_name_ + "/clouds";
  if (!ros::param::get(param_name, param_clouds_)) {
    param_clouds_ = 2;
    ROSPARAM_WARN(param_name, param_clouds_);
  }

  // Frequency to update/publish
  param_name = node_name_ + "/hz";
  if (!ros::param::get(param_name, param_hz_)) {
    param_hz_ = 10;
    ROSPARAM_WARN(param_name, param_hz_);
  }

  ROS_INFO("Parameters loaded.");
}

double PointcloudConcatenate::getHz() {
  return param_hz_;
}

void PointcloudConcatenate::update() {
  // Is run periodically and handles calling the different methods

  if (pub_cloud_out.getNumSubscribers() > 0 && param_clouds_ >= 1) {
    // Initialise pointclouds
    sensor_msgs::PointCloud2 cloud_to_concat;
    cloud_out = cloud_to_concat; // Clear the output pointcloud
    
    // Track success of transforms
    bool success = true;

    // Sleep if no pointclouds have been received yet
    if ((!cloud_in1_received) && (!cloud_in2_received) && (!cloud_in3_received) && (!cloud_in4_received)) {
      ROS_WARN("No pointclouds received yet. Waiting 1 second...");

      // Set end time
      ros::Time end = ros::Time::now();
      end.sec += 1;
      // Sleep
      ros::Time::sleepUntil(end);

      return;
    }

    
    // Concatenate the first pointcloud
    if (param_clouds_ >= 1 && success && cloud_in1_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in1_received_recent) {
        ROS_WARN("Cloud 1 was not received since last update, reusing last received message...");
      }
      cloud_in1_received_recent = false;

      // Transform pointcloud to the target frame
      // Here we just assign the pointcloud directly to the output to ensure the secondary
      // data is inherited correctly.
      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in1, cloud_out, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 1 from %s to %s failed!", cloud_in1.header.frame_id.c_str(), param_frame_target_.c_str());
      }
    }

    // Concatenate the second pointcloud
    if (param_clouds_ >= 2 && success && cloud_in2_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in2_received_recent) {
        ROS_WARN("Cloud 2 was not received since last update, reusing last received message...");
      }
      cloud_in2_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in2, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 2 from %s to %s failed!", cloud_in2.header.frame_id.c_str(), param_frame_target_.c_str());
      }
      
      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Concatenate the third pointcloud
    if (param_clouds_ >= 3 && success && cloud_in3_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in3_received_recent) {
        ROS_WARN("Cloud 3 was not received since last update, reusing last received message...");
      }
      cloud_in3_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in3, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 3 from %s to %s failed!", cloud_in3.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Concatenate the fourth pointcloud
    if (param_clouds_ >= 4 && success && cloud_in4_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in4_received_recent) {
        ROS_WARN("Cloud 4 was not received since last update, reusing last received message...");
      }
      cloud_in4_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in4, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 4 from %s to %s failed!", cloud_in4.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Publish the concatenated pointcloud
    if (success) {
      publishPointcloud(cloud_out);
    }
  }
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::PointCloud2 cloud) {
  // Publishes the combined pointcloud

  // Update the timestamp
  cloud.header.stamp = ros::Time::now();
  // Publish
  pub_cloud_out.publish(cloud);
}