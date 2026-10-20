// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("max_iterations", 30);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  // 地面机器人只需修正 yaw；开启后强制 map->odom 的 roll/pitch/z 为零，
  // 防止 GICP 在退化场景下收敛到带俯仰的错误解
  this->declare_parameter("constrain_planar", true);
  this->declare_parameter("min_accumulated_points", 3000);
  this->declare_parameter("correction_alpha", 0.4);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  // 跟踪健康度发布：安全层靠它发现"map->odom 还在发、但位姿已经不可信"这种
  // 看门狗用 TF 新鲜度发现不了的失效。默认开启。
  this->declare_parameter("publish_tracking_status", true);
  this->declare_parameter("tracking_ok_topic", "/localization/tracking_ok");
  this->declare_parameter("tracking_error_topic", "/localization/tracking_error");
  // GICP 报 converged 但 error 超过该值时，同样判为不可信。0 = 只信 converged。
  // 建议先观察一段时间的实际 error 分布再启用，别凭空定阈值。
  this->declare_parameter("tracking_error_warn", 0.0);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("constrain_planar", constrain_planar_);
  this->get_parameter("min_accumulated_points", min_accumulated_points_);
  this->get_parameter("correction_alpha", correction_alpha_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // 初始化位姿
  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
    result_t_ = constrainPlanar(result_t_);
  }

  // 预定初始位姿
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // 加载地图并对齐的机器的 odom 下
  loadGlobalMap(prior_pcd_file_);

  // pcd点云降采样，转为 0.25*0.25*0.25 体素盒子，
  // 然后每个体素盒子对应一个协方差矩阵(用于还原体素盒子内部点云原本的样子)
  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // 计算每个点附近 num_neighbors_ 个点的协方差矩阵
  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // 点云进入 KdTree 以加速最近邻搜索
  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  auto prior_map_qos = rclcpp::QoS(1).transient_local();
  prior_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/prior_pcd_map", prior_map_qos);
  aligned_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/gicp_aligned_scan", rclcpp::SensorDataQoS());
  sensor_msgs::msg::PointCloud2 prior_map_msg;
  pcl::toROSMsg(*global_map_, prior_map_msg);
  prior_map_msg.header.frame_id = map_frame_;
  prior_map_msg.header.stamp = this->now();
  prior_map_pub_->publish(prior_map_msg);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  bool publish_status = true;
  this->get_parameter("publish_tracking_status", publish_status);
  this->get_parameter("tracking_error_warn", tracking_error_warn_);
  if (publish_status) {
    std::string ok_topic, error_topic;
    this->get_parameter("tracking_ok_topic", ok_topic);
    this->get_parameter("tracking_error_topic", error_topic);
    // 用 transient_local：安全层后启动也能立刻拿到最近一次判定
    auto status_qos = rclcpp::QoS(1).transient_local();
    tracking_ok_pub_ = this->create_publisher<std_msgs::msg::Bool>(ok_topic, status_qos);
    tracking_error_pub_ = this->create_publisher<std_msgs::msg::Float32>(error_topic, status_qos);
    RCLCPP_INFO(
      this->get_logger(), "Tracking status published on %s / %s (error_warn=%.3f)",
      ok_topic.c_str(), error_topic.c_str(), tracking_error_warn_);
  }

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  // 累积几帧点云，提高点云密度，增强匹配效果
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  // 上游点云是突发到达的, 单个周期可能只有 0~1 帧。
  // 点太少时不清空累积、推迟到下一周期, 避免用稀疏点解出抖动的修正量。
  if (accumulated_cloud_->size() < static_cast<size_t>(min_accumulated_points_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Accumulated points (%zu) below threshold (%d), waiting for more scans.",
      accumulated_cloud_->size(), min_accumulated_points_);
    return;
  }

  // 实时点云降采样，转为 0.25*0.25*0.25 体素盒子，
  // 然后每个体素盒子对应一个协方差矩阵(用于还原体素盒子内部点云原本的样子)
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  // 计算每个点附近 num_neighbors_ 个点的协方差矩阵
  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  // 点云进入 KdTree 以加速最近邻搜索
  // Create KdTree for source
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  // 设置 GICP 参数
  register_->reduction.num_threads = num_threads_;  // 线程数
  register_->rejector.max_dist_sq = max_dist_sq_;   // 最大距离平方
  register_->optimizer.max_iterations = max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  // Publish the downsampled source at the optimizer's latest candidate pose.
  // Publishing failed candidates too makes initialization errors visible in RViz.
  pcl::PointCloud<pcl::PointXYZ> source_xyz;
  source_xyz.reserve(source_->size());
  for (const auto & point : source_->points) {
    source_xyz.emplace_back(point.x, point.y, point.z);
  }
  pcl::PointCloud<pcl::PointXYZ> aligned_scan;
  pcl::transformPointCloud(source_xyz, aligned_scan, result.T_target_source.matrix());
  sensor_msgs::msg::PointCloud2 aligned_scan_msg;
  pcl::toROSMsg(aligned_scan, aligned_scan_msg);
  aligned_scan_msg.header.frame_id = map_frame_;
  aligned_scan_msg.header.stamp = last_scan_time_;
  aligned_scan_pub_->publish(aligned_scan_msg);

  if (result.converged) {
    const Eigen::Isometry3d raw =
      constrain_planar_ ? constrainPlanar(result.T_target_source) : result.T_target_source;
    // 低通平滑: 点云密度波动会让单次 GICP 解在厘米级跳动, 与上次结果插值过渡抑制抖动
    Eigen::Isometry3d smoothed = Eigen::Isometry3d::Identity();
    smoothed.translation() = correction_alpha_ * raw.translation() +
                             (1.0 - correction_alpha_) * previous_result_t_.translation();
    smoothed.linear() =
      Eigen::Quaterniond(previous_result_t_.rotation())
        .slerp(correction_alpha_, Eigen::Quaterniond(raw.rotation()))
        .toRotationMatrix();
    result_t_ = previous_result_t_ = smoothed;
    publishTrackingStatus(true, result.iterations, result.error);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "GICP tracking OK (iterations=%zu, error=%.3f, source_points=%zu)",
      result.iterations, result.error, source_->size());
  } else {
    // 关键：收敛失败时不更新位姿，但 publishTransform() 仍会按上一次的位姿发
    // map->odom —— 也就是"位姿已经不可信、TF 却还在更新"。这里把判定发出去，
    // 让安全层能在位姿失效时掐掉自动驾驶（看门狗靠 TF 新鲜度发现不了这种失效）。
    publishTrackingStatus(false, result.iterations, result.error);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "GICP did not converge (iterations=%zu, error=%.3f, source_points=%zu). "
      "Set an approximate pose with RViz 2D Pose Estimate, or use global KISS relocalization.",
      result.iterations, result.error, source_->size());
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTrackingStatus(
  bool converged, std::size_t iterations, double error)
{
  if (!tracking_ok_pub_) {
    return;
  }

  // 只看 converged 是保守的默认；tracking_error_warn 是给"收敛了但误差偏大"留的
  // 可选加严开关，需要先看一段实际 error 分布再定值。
  const bool confident =
    converged && (tracking_error_warn_ <= 0.0 || error <= tracking_error_warn_);

  std_msgs::msg::Bool ok_msg;
  ok_msg.data = confident;
  tracking_ok_pub_->publish(ok_msg);

  std_msgs::msg::Float32 error_msg;
  error_msg.data = static_cast<float>(error);
  tracking_error_pub_->publish(error_msg);

  if (converged && !confident) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "GICP converged (iterations=%zu) but error=%.3f exceeds tracking_error_warn=%.3f; "
      "reporting map->odom as untrusted.", iterations, error, tracking_error_warn_);
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // This must use the source scan timestamp so map -> odom is valid at the
  // same instant as odom -> base_footprint and the incoming LaserScan.
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = constrainPlanar(map_to_odom);
    accumulated_cloud_->clear();
    RCLCPP_INFO(
      this->get_logger(), "Initial pose accepted; GICP will refine map -> odom from this guess.");
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

Eigen::Isometry3d SmallGicpRelocalizationNode::constrainPlanar(const Eigen::Isometry3d & pose) const
{
  const Eigen::Vector3d yaw_direction = pose.linear() * Eigen::Vector3d::UnitX();
  const double yaw = std::atan2(yaw_direction.y(), yaw_direction.x());
  Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
  planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // z 平移必须保留：先验地图原点与当前 odom 原点的 z 差是真实存在的
  planar.translation() << pose.translation().x(), pose.translation().y(), pose.translation().z();
  return planar;
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
