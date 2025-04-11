// vgicp_slam_node.cpp
// FastVGICPを使ったVGICPSLAMノード
// - 初期傾き補正は保存時にのみ適用
// - 初回スキャンでマップ構築＆表示
// - PCD保存時に日時付きファイル名使用
// - サービス呼び出しで任意タイミングで補正マップ保存可能
// - プログラム終了時にも自動で保存

#include "vgicp_slam/vgicp_slam_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <numeric>
#include <chrono>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;//_1 は、bindされた関数の第1引数をそのまま imuCallback に渡すという意味
using std::placeholders::_2;

VGICPSLAMNode::VGICPSLAMNode() : Node("vgicp_slam_node"), collecting_imu_(true), imu_sample_count_(0) {
  declareParameters();
  getParameters();
  printParameters();
  initializeSubscriptions();
  initializePublishers();
  initializeVGICP();
  initializeState();

  rclcpp::on_shutdown([this]() {
    RCLCPP_INFO(this->get_logger(), "Saving global map before shutdown...");
    saveGlobalMapToPCD();
  });
}

void VGICPSLAMNode::declareParameters() {
  declare_parameter("voxel_size_for_vgicp", 0.3);
  declare_parameter("voxel_size_for_map", 0.1);
  declare_parameter("vgicp_resolution", 1.0);
  declare_parameter("min_add_dist", 0.3);
  declare_parameter("window_size", 20);
  declare_parameter("vgicp_num_threads", 4);
  declare_parameter("vgicp_max_iterations", 30);
  declare_parameter("vgicp_correspondence_randomness", 20);
  declare_parameter("vgicp_transformation_epsilon", 1e-5);
  declare_parameter("vgicp_rotation_epsilon", 2e-3);
  
}

void VGICPSLAMNode::getParameters() {
  voxel_size_for_vgicp_ = get_parameter("voxel_size_for_vgicp").as_double();
  voxel_size_for_map_ = get_parameter("voxel_size_for_map").as_double();
  vgicp_resolution_ = get_parameter("vgicp_resolution").as_double();
  min_add_dist_ = get_parameter("min_add_dist").as_double();
  window_size_ = get_parameter("window_size").as_int();
  vgicp_num_threads_ = get_parameter("vgicp_num_threads").as_int();
  vgicp_max_iterations_ = get_parameter("vgicp_max_iterations").as_int();
  vgicp_correspondence_randomness_ = get_parameter("vgicp_correspondence_randomness").as_int();
  vgicp_transformation_epsilon_ = get_parameter("vgicp_transformation_epsilon").as_double();
  vgicp_rotation_epsilon_ = get_parameter("vgicp_rotation_epsilon").as_double();
  
}

void VGICPSLAMNode::printParameters() {
  RCLCPP_INFO(this->get_logger(), "--- VGICP SLAM Parameters ---");
  RCLCPP_INFO(this->get_logger(), "voxel_size_for_vgicp: %.3f", voxel_size_for_vgicp_);
  RCLCPP_INFO(this->get_logger(), "voxel_size_for_map: %.3f", voxel_size_for_map_);
  RCLCPP_INFO(this->get_logger(), "vgicp_resolution: %.3f", vgicp_resolution_);
  RCLCPP_INFO(this->get_logger(), "min_add_dist: %.3f", min_add_dist_);
  RCLCPP_INFO(this->get_logger(), "window_size: %d", window_size_);
  RCLCPP_INFO(this->get_logger(), "vgicp_num_threads: %d", vgicp_num_threads_);
  RCLCPP_INFO(this->get_logger(), "vgicp_max_iterations: %d", vgicp_max_iterations_);
  RCLCPP_INFO(this->get_logger(), "vgicp_correspondence_randomness: %d", vgicp_correspondence_randomness_);
  RCLCPP_INFO(this->get_logger(), "vgicp_transformation_epsilon: %.6f", vgicp_transformation_epsilon_);
  RCLCPP_INFO(this->get_logger(), "vgicp_rotation_epsilon: %.6f", vgicp_rotation_epsilon_);
  
}

void VGICPSLAMNode::initializeSubscriptions() {
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/livox/imu", 100, std::bind(&VGICPSLAMNode::imuCallback, this, _1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", rclcpp::SensorDataQoS(), std::bind(&VGICPSLAMNode::pointCloudCallback, this, _1));
}

void VGICPSLAMNode::initializePublishers() {
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vgicp_pose", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/vgicp_path", 10);
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/vgicp_map_built", 1);
  global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/vgicp_map_global", 1);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "/save_map", std::bind(&VGICPSLAMNode::onSaveMapService, this, _1, _2));
}

void VGICPSLAMNode::initializeVGICP() {
  vgicp_ = std::make_shared<fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>>();
  vgicp_->setResolution(vgicp_resolution_);
  vgicp_->setNumThreads(vgicp_num_threads_);
  vgicp_->setMaximumIterations(vgicp_max_iterations_);
  vgicp_->setCorrespondenceRandomness(vgicp_correspondence_randomness_);
  vgicp_->setTransformationEpsilon(vgicp_transformation_epsilon_);
  vgicp_->setRotationEpsilon(vgicp_rotation_epsilon_);
}

void VGICPSLAMNode::initializeState() {
  path_.header.frame_id = "map";
  is_first_frame_ = true;
  previous_pose_ = Eigen::Matrix4f::Identity();
  last_added_position_ = Eigen::Vector3f::Zero();
  global_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  initial_rotation_ = Eigen::Matrix3f::Identity();
}


VGICPSLAMNode::~VGICPSLAMNode() {
  RCLCPP_INFO(this->get_logger(), "VGICPSLAMNode destructed.");
}

void VGICPSLAMNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!collecting_imu_) return;
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double az = msg->linear_acceleration.z;

  double roll = std::atan2(ay, az);
  double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
  roll_list_.push_back(roll);
  pitch_list_.push_back(pitch);
  imu_sample_count_++;

  if (imu_sample_count_ >= max_imu_samples_) {
    double avg_roll = std::accumulate(roll_list_.begin(), roll_list_.end(), 0.0) / roll_list_.size();
    double avg_pitch = std::accumulate(pitch_list_.begin(), pitch_list_.end(), 0.0) / pitch_list_.size();

    Eigen::Matrix3f rot_roll = Eigen::AngleAxisf(avg_roll, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::Matrix3f rot_pitch = Eigen::AngleAxisf(avg_pitch, Eigen::Vector3f::UnitY()).toRotationMatrix();
    initial_rotation_ = rot_pitch * rot_roll;
    collecting_imu_ = false;

    RCLCPP_INFO(this->get_logger(), "Initial IMU alignment complete: roll=%.2f deg, pitch=%.2f deg", avg_roll * 180 / M_PI, avg_pitch * 180 / M_PI);
  }
}

void VGICPSLAMNode::onSaveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  saveGlobalMapToPCD();
  response->success = true;
  response->message = "Global map saved with tilt correction.";
}

void VGICPSLAMNode::saveGlobalMapToPCD() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "global_map_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".pcd";
  std::string filename = ss.str();

  pcl::PointCloud<pcl::PointXYZI>::Ptr corrected(new pcl::PointCloud<pcl::PointXYZI>(*global_map_));
  for (auto& pt : corrected->points) {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    p = initial_rotation_ * p;
    pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
  }
  pcl::io::savePCDFileBinary(filename, *corrected);
  RCLCPP_INFO(this->get_logger(), "Corrected global map saved to %s", filename.c_str());
}

void VGICPSLAMNode::addToLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  local_map_queue_.push_back(cloud);
  while (static_cast<int>(local_map_queue_.size()) > window_size_) {
    local_map_queue_.pop_front();
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr VGICPSLAMNode::buildLocalMap() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto& c : local_map_queue_) {
    *map += *c;
  }
  return map;
}

void VGICPSLAMNode::publishMap() {
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*buildLocalMap(), map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = now();
  map_pub_->publish(map_msg);
}

void VGICPSLAMNode::publishGlobalMap() {
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*global_map_, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = now();
  global_map_pub_->publish(map_msg);
}

// pointCloudCallback.cpp
void VGICPSLAMNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (collecting_imu_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for IMU alignment...");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *input_cloud);
  if (input_cloud->empty()) return;

  // VGICP用のダウンサンプリング
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_vgicp;
  voxel_vgicp.setInputCloud(input_cloud);
  voxel_vgicp.setLeafSize(voxel_size_for_vgicp_, voxel_size_for_vgicp_, voxel_size_for_vgicp_);
  voxel_vgicp.filter(*filtered_cloud);

  // 初回は地図作成＋表示
  if (is_first_frame_) {
    addToLocalMap(filtered_cloud);
    *global_map_ += *input_cloud;

    // グローバルマップのダウンサンプリング
    pcl::VoxelGrid<pcl::PointXYZI> voxel_map;
    voxel_map.setInputCloud(global_map_);
    voxel_map.setLeafSize(voxel_size_for_map_, voxel_size_for_map_, voxel_size_for_map_);
    voxel_map.filter(*global_map_);

    vgicp_->setInputTarget(buildLocalMap());
    publishMap();
    publishGlobalMap();
    is_first_frame_ = false;
    return;
  }

  // VGICPマッチング
  vgicp_->setInputSource(filtered_cloud);
  pcl::PointCloud<pcl::PointXYZI> aligned;
  Eigen::Matrix4f guess = previous_pose_;
  vgicp_->align(aligned, guess);
  if (!vgicp_->hasConverged()) return;

  Eigen::Matrix4f transformation = vgicp_->getFinalTransformation();




  previous_pose_ = transformation;

  Eigen::Vector3f current_position = transformation.block<3, 1>(0, 3);
  if ((current_position - last_added_position_).norm() >= min_add_dist_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_transformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*filtered_cloud, *transformed, transformation);
    pcl::transformPointCloud(*input_cloud, *input_transformed, transformation);
    addToLocalMap(transformed);
    *global_map_ += *input_transformed;

    // グローバルマップのダウンサンプリング
    pcl::VoxelGrid<pcl::PointXYZI> voxel_map;
    voxel_map.setInputCloud(global_map_);
    voxel_map.setLeafSize(voxel_size_for_map_, voxel_size_for_map_, voxel_size_for_map_);
    voxel_map.filter(*global_map_);

    vgicp_->setInputTarget(buildLocalMap());
    last_added_position_ = current_position;
    publishMap();
    publishGlobalMap();
    RCLCPP_INFO(this->get_logger(), "Local + Global map updated.");
  }

  // 推定姿勢をパブリッシュ
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = "map";
  pose_msg.pose.position.x = transformation(0, 3);
  pose_msg.pose.position.y = transformation(1, 3);
  pose_msg.pose.position.z = transformation(2, 3);
  Eigen::Matrix3f rot = transformation.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub_->publish(pose_msg);
  path_.header.stamp = msg->header.stamp;
  path_.poses.push_back(pose_msg);
  path_pub_->publish(path_);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = msg->header.stamp;
  tf.header.frame_id = "map";
  tf.child_frame_id = "livox_frame";
  tf.transform.translation.x = pose_msg.pose.position.x;
  tf.transform.translation.y = pose_msg.pose.position.y;
  tf.transform.translation.z = pose_msg.pose.position.z;
  tf.transform.rotation = pose_msg.pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VGICPSLAMNode>());
  rclcpp::shutdown();
  return 0;
}