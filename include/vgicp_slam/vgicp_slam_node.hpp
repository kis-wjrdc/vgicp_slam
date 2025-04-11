// vgicp_slam_node.hpp
// ヘッダファイル: FastVGICPを使ったSLAMノードのクラス宣言
// - クラス定義のみ（実装は cpp に分離）
// - IMUとLiDARデータを使って地図生成・自己位置推定を行う
// - 地図保存やTFブロードキャストなども含む

#pragma once

// --- ROS2 関連ヘッダ ---
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

// --- PCL / Eigen 関連 ---
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <deque>
#include <Eigen/Dense>

// --- VGICPSLAMノードクラス定義 ---
class VGICPSLAMNode : public rclcpp::Node {
public:
  VGICPSLAMNode();  // コンストラクタ
  ~VGICPSLAMNode(); // デストラクタ

private:
  // --- 初期化関数 ---
  void declareParameters();      // パラメータ宣言
  void getParameters();          // パラメータ取得
  void printParameters();        // パラメータ表示
  void initializeSubscriptions();// サブスクライバ初期化
  void initializePublishers();   // パブリッシャ初期化
  void initializeVGICP();        // VGICP設定
  void initializeState();        // 内部状態初期化

  // --- コールバック関数 ---
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);              // IMUデータ受信処理
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg); // LiDARデータ受信処理

  // --- 地図保存・サービス ---
  void onSaveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response); // 地図保存サービス
  void saveGlobalMapToPCD(); // 地図のPCD保存

  // --- 地図管理 ---
  void addToLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);      // 局所地図に追加
  pcl::PointCloud<pcl::PointXYZI>::Ptr buildLocalMap();                       // 局所地図構築
  void publishMap();        // 局所地図のパブリッシュ
  void publishGlobalMap(); // 全体地図のパブリッシュ

  // --- IMU補正用 ---
  bool collecting_imu_;                      // IMU補正中かどうか
  int imu_sample_count_;                     // IMUサンプル数カウント
  const int max_imu_samples_ = 200;          // IMU補正に必要なサンプル数
  std::vector<double> roll_list_, pitch_list_; // ロール・ピッチリスト
  Eigen::Matrix3f initial_rotation_;         // 初期傾き補正回転行列

  // --- ROS2 通信関連 ---
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

  // --- VGICP / 地図データ ---
  std::shared_ptr<fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>> vgicp_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> local_map_queue_; // 局所地図バッファ
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;                   // 全体地図

  // --- SLAM状態保持 ---
  bool is_first_frame_;                     // 初回スキャンフラグ
  Eigen::Matrix4f previous_pose_;          // 前回の姿勢
  Eigen::Vector3f last_added_position_;    // 最後に追加した位置
  nav_msgs::msg::Path path_;               // 経路情報

  // --- パラメータ ---
  double voxel_size_for_vgicp_;
  double voxel_size_for_map_;
  double vgicp_resolution_;
  double min_add_dist_;
  int window_size_;
  int vgicp_num_threads_;
  int vgicp_max_iterations_;
  int vgicp_correspondence_randomness_;
  double vgicp_transformation_epsilon_;
  double vgicp_rotation_epsilon_;
  

};
