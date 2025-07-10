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
  // Also, get the timestamp
  cloud_in1_timestamp = msg.header.stamp;
}

void PointcloudConcatenate::subCallbackCloudIn2(sensor_msgs::PointCloud2 msg) {
  cloud_in2 = msg;
  cloud_in2_received = true;
  cloud_in2_received_recent = true;
  // Also, get the timestamp
  cloud_in2_timestamp = msg.header.stamp;
}

void PointcloudConcatenate::subCallbackCloudIn3(sensor_msgs::PointCloud2 msg) {
  cloud_in3 = msg;
  cloud_in3_received = true;
  cloud_in3_received_recent = true;
  // Also, get the timestamp
  cloud_in3_timestamp = msg.header.stamp;
}

void PointcloudConcatenate::subCallbackCloudIn4(sensor_msgs::PointCloud2 msg) {
  cloud_in4 = msg;
  cloud_in4_received = true;
  cloud_in4_received_recent = true;
  // Also, get the timestamp
  cloud_in4_timestamp = msg.header.stamp;
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

  // Time threshold for skipping old pointclouds
  param_name = node_name_ + "/time_threshold";
  if (!ros::param::get(param_name, param_time_threshold_)) {
    param_time_threshold_ = 0.1;
    ROSPARAM_WARN(param_name, param_time_threshold_);
  }

  ROS_INFO("Parameters loaded.");
}

bool PointcloudConcatenate::checkTimeThresholdOk(ros::Time& reference_time, ros::Time& current_time, const int& cloud_index) {
  double timestamp_diff = std::abs((current_time - reference_time).toSec());
  if (timestamp_diff > param_time_threshold_) {
    ROS_INFO("Cloud %d is too old [%.3f s], skipping...", cloud_index, timestamp_diff);
    return false;
  }
  return true;
}

void PointcloudConcatenate::concatenate_with_reference_cloud(sensor_msgs::PointCloud2& reference_cloud, const sensor_msgs::PointCloud2 cloud_to_concat, bool& success, bool& cloud_received_recent, ros::Time& reference_time, ros::Time& current_time, const int& cloud_index, const bool& update_reference_time) {
  // ROS_INFO cloud idx
  // ROS_INFO("Concatenating cloud %d", cloud_index);
  if (param_clouds_ >= cloud_index && success && cloud_received_recent) {
      // Warn if cloud was not received since last update
      if (!cloud_received_recent) {
        ROS_WARN("Cloud was not received since last update, reusing last received message...");
      }
      cloud_received_recent = false;
      bool success_time_threshold = false;
      if (update_reference_time) {
        success_time_threshold = true;
        reference_time = cloud_to_concat.header.stamp;
      } else {
        success_time_threshold = this->checkTimeThresholdOk(reference_time, current_time, cloud_index);
      }
      if (success_time_threshold) {
        sensor_msgs::PointCloud2 transformed_cloud;
        // Transform pointcloud to the target frame if needed
        if (cloud_to_concat.header.frame_id != param_frame_target_) {
          // ROS_INFO("Transforming cloud %d from %s to %s", cloud_index, cloud_to_concat.header.frame_id.c_str(), param_frame_target_.c_str());
          success = pcl_ros::transformPointCloud(param_frame_target_, cloud_to_concat, transformed_cloud, *tfBuffer);
          if (!success) {
            ROS_WARN("Transforming cloud %d from %s to %s failed!", cloud_index, cloud_to_concat.header.frame_id.c_str(), param_frame_target_.c_str());
            return;
          }
        } else {
          // ROS_INFO("Cloud %d is already in the target frame", cloud_index);
          transformed_cloud = cloud_to_concat;
        }
        // For the first cloud, set the header and assign directly
        if (update_reference_time) {
          reference_cloud = transformed_cloud;
          // ROS_INFO("Cloud %d is the first cloud. Taken as reference", cloud_index);
        } else {
          // Concatenate the pointclouds (all in the same frame now)
          pcl::concatenatePointCloud(reference_cloud, transformed_cloud, reference_cloud);
          // ROS_INFO("Cloud %d is concatenated with the reference cloud wrt the target frame: %s", cloud_index, param_frame_target_.c_str());
        }
      }
    }
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

    // Set reference time to the current time
    ros::Time reference_time = ros::Time::now();

    // Concatenate the first pointcloud
    this->concatenate_with_reference_cloud(cloud_out, cloud_in1, success, cloud_in1_received_recent, reference_time, cloud_in1_timestamp, 0, true);

    // Concatenate the second pointcloud
    this->concatenate_with_reference_cloud(cloud_out, cloud_in2, success, cloud_in2_received_recent, reference_time, cloud_in2_timestamp, 1, false);

    // Concatenate the third pointcloud
    this->concatenate_with_reference_cloud(cloud_out, cloud_in3, success, cloud_in3_received_recent, reference_time, cloud_in3_timestamp, 2, false);

    // Concatenate the fourth pointcloud
    this->concatenate_with_reference_cloud(cloud_out, cloud_in4, success, cloud_in4_received_recent, reference_time, cloud_in4_timestamp, 3, false);

    // Publish the concatenated pointcloud
    if (success) {
      publishPointcloud(cloud_out);
    }
  }
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::PointCloud2 cloud) {
  // Publishes the combined pointcloud
  pub_cloud_out.publish(cloud);
}