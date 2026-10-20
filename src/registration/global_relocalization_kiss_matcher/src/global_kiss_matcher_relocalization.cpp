/**
 * @brief KISSMatcher 与 small_gicp 融合的全局激光重定位节点
 *
 * @details
 * 核心思想:
 * 启动时使用 KISSMatcher 做无初值全局粗配准，获得初始 map -> odom。
 * 初始化成功后，使用 small_gicp 以上一帧位姿为初值持续跟踪 map -> odom。
 * 当 GICP 连续失败时，切回 KISSMatcher 做全局恢复，成功后继续 GICP 跟踪。
 *
 * 输入:
 * 订阅当前累计点云: /registered_scan
 * 订阅人工初始位姿: /initialpose
 *
 * 输出:
 * 发布全局定位 TF: map -> odom
 */

#include "global_kiss_matcher_relocalization/global_kiss_matcher_relocalization.hpp"

namespace global_kiss_matcher_relocalization
{

GlobalKissMatcherRelocalizationNode::GlobalKissMatcherRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("kiss_matcher_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  reloc_state_(RelocState::KISS_GLOBAL_INIT)
{
  this->declareParameters();

  const auto lc_config = this->createLoopClosureConfig();
  reg_module_ = std::make_shared<kiss_matcher::LoopClosure>(lc_config, this->get_logger());

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }

  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&GlobalKissMatcherRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&GlobalKissMatcherRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // 跟踪健康度发布（与 small_gicp_relocalization 用同一话题，安全层的看门狗
  // 默认盯 /localization/tracking_ok，两个重定位节点都发它，切换方案时不用改配置）
  publish_tracking_status_ = this->declare_parameter<bool>("publish_tracking_status", true);
  tracking_ok_topic_ = this->declare_parameter<std::string>(
    "tracking_ok_topic", "/localization/tracking_ok");
  tracking_status_topic_ = this->declare_parameter<std::string>(
    "tracking_status_topic", "/localization/tracking_status");
  if (publish_tracking_status_) {
    auto status_qos = rclcpp::QoS(1).transient_local();
    tracking_ok_pub_ = this->create_publisher<std_msgs::msg::Bool>(tracking_ok_topic_, status_qos);
    tracking_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      tracking_status_topic_, status_qos);
    RCLCPP_INFO(
      this->get_logger(), "Tracking status published on %s / %s",
      tracking_ok_topic_.c_str(), tracking_status_topic_.c_str());
  }

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GlobalKissMatcherRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&GlobalKissMatcherRelocalizationNode::publishTransform, this));
}

void GlobalKissMatcherRelocalizationNode::declareParameters()
{
  global_leaf_size_ = this->declare_parameter<double>("global_leaf_size", 0.25);
  registered_leaf_size_ = this->declare_parameter<double>("registered_leaf_size", 0.25);
  coarse_leaf_size_ = this->declare_parameter<double>("coarse_leaf_size", 0.8);
  medium_leaf_size_ = this->declare_parameter<double>("medium_leaf_size", 0.5);
  fine_leaf_size_ = this->declare_parameter<double>("fine_leaf_size", 0.25);

  num_threads_ = this->declare_parameter<int>("num_threads", 4);
  num_neighbors_ = this->declare_parameter<int>("num_neighbors", 20);
  max_dist_sq_ = this->declare_parameter<double>("max_dist_sq", 1.0);
  voxel_resolution_ = this->declare_parameter<double>("voxel_resolution", 0.25);

  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "");
  robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "");
  lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "");
  prior_pcd_file_ = this->declare_parameter<std::string>("prior_pcd_file", "");
  input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "registered_scan");

  init_pose_ = this->declare_parameter<std::vector<double>>(
    "init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  // 是否使用 KISS 全局初始化
  use_global_initialization_ =
    this->declare_parameter<bool>("use_global_initialization", true);
  use_kiss_recovery_ = 
    this->declare_parameter<bool>("use_kiss_recovery", true);
  verify_kiss_with_gicp_ = 
    this->declare_parameter<bool>("verify_kiss_with_gicp", true);
  gicp_max_consecutive_failures_ =
    this->declare_parameter<int>("gicp_max_consecutive_failures", 3);
  // 0 = 只看 converged；>0 时把“收敛但 error 超过阈值”也当作失败，
  // 连续失败后触发 KISS 全局恢复。建议先观察实际 error 分布再设置。
  gicp_error_max_ =
    this->declare_parameter<double>("gicp_error_max", 0.0);
  recovery_min_points_ = 
    this->declare_parameter<int>("recovery_min_points", 1000);
  recovery_cooldown_sec_ = 
    this->declare_parameter<double>("recovery_cooldown_sec", 2.0);
}

kiss_matcher::LoopClosureConfig GlobalKissMatcherRelocalizationNode::createLoopClosureConfig()
{
  kiss_matcher::LoopClosureConfig lc_config;
  auto & gc = lc_config.gicp_config_;

  lc_config.voxel_res_ = voxel_resolution_;
  lc_config.verbose_ = this->declare_parameter<bool>("loop.verbose", true);
  lc_config.enable_global_registration_ =
    this->declare_parameter<bool>("enable_global_registration", true);
  lc_config.num_inliers_threshold_ =
    this->declare_parameter<int>("loop.num_inliers_threshold", 3);  // 测试中

  gc.num_threads_ = num_threads_;
  gc.correspondence_randomness_ =
    this->declare_parameter<int>("loop.correspondence_randomness", 20);
  gc.max_num_iter_ = 
    this->declare_parameter<int>("loop.max_num_iter", 32);
  gc.scale_factor_for_corr_dist_ =
    this->declare_parameter<double>("loop.scale_factor_for_corr_dist", 5.0);
  gc.overlap_threshold_ = 
    this->declare_parameter<double>("loop.overlap_threshold", 80.0);

  return lc_config;
}

void GlobalKissMatcherRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

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

void GlobalKissMatcherRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  *accumulated_cloud_ += *scan;
}

void GlobalKissMatcherRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  switch (reloc_state_) {
    case RelocState::KISS_GLOBAL_INIT: {
      if (!use_global_initialization_) {
        initial_aligned_ = true;
        reloc_state_ = RelocState::GICP_TRACKING;
        RCLCPP_INFO(
          this->get_logger(), "KISS global initialization disabled. Switching to GICP tracking.");
        return;
      }

      Eigen::Isometry3d kiss_pose = Eigen::Isometry3d::Identity();
      size_t inliers = 0;
      if (tryKissAlignment(accumulated_cloud_, kiss_pose, inliers)) {
        result_t_ = previous_result_t_ = kiss_pose;
        initial_aligned_ = true;
        reloc_state_ = RelocState::GICP_TRACKING;
        gicp_fail_count_ = 0;
        accumulated_cloud_->clear();
        RCLCPP_INFO(
          this->get_logger(), "KISSMatcher initialization succeeded. Inliers: %zu", inliers);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "KISSMatcher initialization failed. Inliers: %zu. Retrying next cycle...", inliers);
      }
      return;
    }

    case RelocState::GICP_TRACKING: {
      Eigen::Isometry3d gicp_pose = Eigen::Isometry3d::Identity();
      if (tryGicpAlignment(accumulated_cloud_, previous_result_t_, gicp_pose)) {
        result_t_ = previous_result_t_ = gicp_pose;
        initial_aligned_ = true;
        reloc_state_ = RelocState::GICP_TRACKING;
        gicp_fail_count_ = 0;
        accumulated_cloud_->clear();
        return;
      }

      ++gicp_fail_count_;
      RCLCPP_WARN(
        this->get_logger(), "GICP did not converge. fail_count=%d/%d",
        gicp_fail_count_, gicp_max_consecutive_failures_);

      if (use_kiss_recovery_ && gicp_fail_count_ >= gicp_max_consecutive_failures_) {
        reloc_state_ = RelocState::GLOBAL_RECOVERY;
        last_recovery_attempt_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        RCLCPP_WARN(this->get_logger(), "GICP lost. Switching to KISSMatcher recovery.");
      } else if (!use_kiss_recovery_) {
        accumulated_cloud_->clear();
      }
      return;
    }

    case RelocState::GLOBAL_RECOVERY: {
      if (static_cast<int>(accumulated_cloud_->size()) < recovery_min_points_) {
        RCLCPP_WARN(
          this->get_logger(), "KISSMatcher recovery waiting for points: %zu/%d",
          accumulated_cloud_->size(), recovery_min_points_);
        return;
      }

      const auto now = this->now();
      const bool cooldown_elapsed =
        last_recovery_attempt_time_.nanoseconds() == 0 ||
        (now - last_recovery_attempt_time_).seconds() >= recovery_cooldown_sec_;
      if (!cooldown_elapsed) {
        return;
      }
      last_recovery_attempt_time_ = now;

      Eigen::Isometry3d kiss_pose = Eigen::Isometry3d::Identity();
      size_t inliers = 0;
      if (!tryKissAlignment(accumulated_cloud_, kiss_pose, inliers)) {
        RCLCPP_WARN(
          this->get_logger(),
          "KISSMatcher recovery failed. Inliers: %zu. Continue accumulating...", inliers);
        return;
      }

      Eigen::Isometry3d recovered_pose = kiss_pose;
      if (verify_kiss_with_gicp_) {
        if (!tryGicpAlignment(accumulated_cloud_, kiss_pose, recovered_pose)) {
          RCLCPP_WARN(
            this->get_logger(),
            "KISSMatcher recovery produced a pose, but GICP verification failed. Continue recovery.");
          return;
        }
      }

      result_t_ = previous_result_t_ = recovered_pose;
      initial_aligned_ = true;
      reloc_state_ = RelocState::GICP_TRACKING;
      gicp_fail_count_ = 0;
      accumulated_cloud_->clear();
      RCLCPP_INFO(
        this->get_logger(), "KISSMatcher recovery succeeded. Inliers: %zu", inliers);
      return;
    }
  }
}

bool GlobalKissMatcherRelocalizationNode::tryKissAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud,
  Eigen::Isometry3d & pose_out,
  size_t & inliers_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_down(new pcl::PointCloud<pcl::PointXYZ>());
  vg.setInputCloud(source_cloud);
  vg.filter(*src_down);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_down(new pcl::PointCloud<pcl::PointXYZ>());
  vg.setInputCloud(global_map_);
  vg.filter(*tgt_down);

  RCLCPP_INFO(
    this->get_logger(), "KISSMatcher alignment: src=%zu, tgt=%zu (downsampled from %zu, %zu)",
    src_down->size(), tgt_down->size(), source_cloud->size(), global_map_->size());

  pcl::PointCloud<PointType>::Ptr src_i(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr tgt_i(new pcl::PointCloud<PointType>());
  src_i->reserve(src_down->size());
  tgt_i->reserve(tgt_down->size());

  for (const auto & p : *src_down) {
    src_i->push_back({p.x, p.y, p.z, 0.f});
  }
  for (const auto & p : *tgt_down) {
    tgt_i->push_back({p.x, p.y, p.z, 0.f});
  }

  const auto & reg_output = reg_module_->coarseToFineAlignment(*src_i, *tgt_i);
  inliers_out = reg_output.num_final_inliers_;
  if (!reg_output.is_valid_) {
    return false;
  }

  pose_out = Eigen::Isometry3d::Identity();
  pose_out.linear() = reg_output.pose_.block<3, 3>(0, 0);
  pose_out.translation() = reg_output.pose_.block<3, 1>(0, 3);
  return true;
}

bool GlobalKissMatcherRelocalizationNode::tryGicpAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud,
  const Eigen::Isometry3d & initial_guess,
  Eigen::Isometry3d & pose_out)
{
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *source_cloud, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return false;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, initial_guess);
  if (!result.converged) {
    return false;
  }
  if (gicp_error_max_ > 0.0 && result.error > gicp_error_max_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "GICP converged but error=%.4f exceeds gicp_error_max=%.4f; treating as lost.",
      result.error, gicp_error_max_);
    return false;
  }

  pose_out = result.T_target_source;
  return true;
}

bool GlobalKissMatcherRelocalizationNode::isTrackingHealthy() const
{
  // 只有"已完成初始化 + 处于 GICP 连续跟踪 + 没有累积失败"才算位姿可信。
  // KISS_GLOBAL_INIT 阶段 map->odom 还没有；GLOBAL_RECOVERY 阶段位姿是旧的、
  // 正在被推翻重算 —— 这两种情况都不能让自动驾驶继续跑。
  return initial_aligned_ &&
         reloc_state_ == RelocState::GICP_TRACKING &&
         gicp_fail_count_ == 0;
}

const char * GlobalKissMatcherRelocalizationNode::relocStateName() const
{
  switch (reloc_state_) {
    case RelocState::KISS_GLOBAL_INIT:
      return "KISS_GLOBAL_INIT";
    case RelocState::GICP_TRACKING:
      return "GICP_TRACKING";
    case RelocState::GLOBAL_RECOVERY:
      return "GLOBAL_RECOVERY";
  }
  return "UNKNOWN";
}

void GlobalKissMatcherRelocalizationNode::publishTrackingStatus()
{
  if (!tracking_ok_pub_) {
    return;
  }

  const bool healthy = isTrackingHealthy();

  std_msgs::msg::Bool ok_msg;
  ok_msg.data = healthy;
  tracking_ok_pub_->publish(ok_msg);

  std_msgs::msg::String status;
  status.data = std::string("reloc_state=") + relocStateName() +
                " gicp_fail_count=" + std::to_string(gicp_fail_count_) +
                " tracking_ok=" + (healthy ? "true" : "false");
  tracking_status_pub_->publish(status);
}

void GlobalKissMatcherRelocalizationNode::publishTransform()
{
  // 先发健康度：即使位姿还没算出来（result_t_ 为零）也要发 false。
  // 否则看门狗只能靠"TF 缺失"判断，而真正危险的是"位姿陈旧、TF 却还在按老值发"。
  publishTrackingStatus();

  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
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

void GlobalKissMatcherRelocalizationNode::initialPoseCallback(
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

    previous_result_t_ = result_t_ = map_to_odom;
    initial_aligned_ = true;
    reloc_state_ = RelocState::GICP_TRACKING;
    gicp_fail_count_ = 0;
    accumulated_cloud_->clear();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace global_kiss_matcher_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(global_kiss_matcher_relocalization::GlobalKissMatcherRelocalizationNode)
