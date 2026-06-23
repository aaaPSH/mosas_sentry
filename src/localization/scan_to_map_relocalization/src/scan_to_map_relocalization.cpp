#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using KdTreeT = pcl::KdTreeFLANN<PointT>;

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-6) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = q.toRotationMatrix();
  tf.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return tf;
}

geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d & tf)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond q(tf.linear());
  pose.position.x = tf.translation().x();
  pose.position.y = tf.translation().y();
  pose.position.z = tf.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::TransformStamped isoToTransformMsg(
  const Eigen::Isometry3d & tf, const builtin_interfaces::msg::Time & stamp,
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = tf.translation().x();
  msg.transform.translation.y = tf.translation().y();
  msg.transform.translation.z = tf.translation().z();
  const Eigen::Quaterniond q(tf.linear());
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

Eigen::Isometry3d vectorToPose(const std::vector<double> & xyz_rpy)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  if (xyz_rpy.size() != 6) {
    return tf;
  }

  const Eigen::AngleAxisd roll(xyz_rpy[3], Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(xyz_rpy[4], Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(xyz_rpy[5], Eigen::Vector3d::UnitZ());
  tf.linear() = (yaw * pitch * roll).toRotationMatrix();
  tf.translation() = Eigen::Vector3d(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2]);
  return tf;
}

CloudT::Ptr voxelDownsample(const CloudT::ConstPtr & cloud, double leaf_size)
{
  CloudT::Ptr filtered(new CloudT);
  if (leaf_size <= 0.0) {
    *filtered = *cloud;
    return filtered;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(cloud);
  voxel.filter(*filtered);
  return filtered;
}
}  // namespace

class ScanToMapRelocalizationNode : public rclcpp::Node
{
public:
  ScanToMapRelocalizationNode() : Node("scan_to_map_relocalization")
  {
    declareAndReadParameters();
    loadMapPyramid();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50,
      std::bind(&ScanToMapRelocalizationNode::odomCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanToMapRelocalizationNode::scanCallback, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&ScanToMapRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    corrected_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    aligned_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/relocalization/aligned_scan", 5);
    local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/relocalization/local_map", 5);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/relocalization/map", rclcpp::QoS(1).transient_local().reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    publishMap();

    map_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ScanToMapRelocalizationNode::publishMap, this));

    RCLCPP_INFO(
      get_logger(),
      "local GICP relocalization ready: levels=%zu, local_radius=%.2f, map=%s",
      voxel_leaf_sizes_.size(), local_map_radius_, map_path_.c_str());
    if (require_initial_pose_) {
      RCLCPP_INFO(
        get_logger(), "waiting for RViz /initialpose. Set RViz Fixed Frame to %s.",
        map_frame_.c_str());
    }
  }

private:
  struct RegistrationResult
  {
    bool converged = false;
    double score = std::numeric_limits<double>::infinity();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    CloudT::Ptr aligned{new CloudT};
    CloudT::Ptr local_map{new CloudT};
  };

  void declareAndReadParameters()
  {
    map_path_ = declare_parameter<std::string>("map_path", "maps/localization_map.pcd");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered_body");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/relocalization/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "body");

    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_aligned_scan_ = declare_parameter<bool>("publish_aligned_scan", true);
    publish_local_map_ = declare_parameter<bool>("publish_local_map", true);
    require_initial_pose_ = declare_parameter<bool>("require_initial_pose", true);
    use_odom_prediction_ = declare_parameter<bool>("use_odom_prediction", true);
    relocalize_every_n_scans_ =
      std::max(1, static_cast<int>(declare_parameter<int>("relocalize_every_n_scans", 1)));
    min_scan_points_ = static_cast<int>(declare_parameter<int>("min_scan_points", 80));

    voxel_leaf_sizes_ =
      declare_parameter<std::vector<double>>("voxel_leaf_sizes", std::vector<double>{1.0, 0.5, 0.25});
    if (voxel_leaf_sizes_.empty()) {
      voxel_leaf_sizes_.push_back(0.25);
    }

    local_map_radius_ = declare_parameter<double>("local_map_radius", 8.0);
    min_local_map_points_ =
      std::max(10, static_cast<int>(declare_parameter<int>("min_local_map_points", 300)));

    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.5);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 0.01);
    euclidean_fitness_epsilon_ = declare_parameter<double>("euclidean_fitness_epsilon", 0.01);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 1.0);
    max_iterations_ = std::max(1, static_cast<int>(declare_parameter<int>("max_iterations", 30)));
    gicp_correspondence_randomness_ =
      std::max(1, static_cast<int>(declare_parameter<int>("gicp_correspondence_randomness", 20)));
    gicp_max_optimizer_iterations_ =
      std::max(1, static_cast<int>(declare_parameter<int>("gicp_max_optimizer_iterations", 20)));
    ransac_iterations_ = std::max(0, static_cast<int>(declare_parameter<int>("ransac_iterations", 0)));
    ransac_outlier_rejection_threshold_ =
      declare_parameter<double>("ransac_outlier_rejection_threshold", 0.05);

    enable_sor_ = declare_parameter<bool>("enable_sor", true);
    sor_mean_k_ = std::max(1, static_cast<int>(declare_parameter<int>("sor_mean_k", 50)));
    sor_stddev_mul_thresh_ = declare_parameter<double>("sor_stddev_mul_thresh", 1.0);

    // --- Adaptive relocalization parameters ---
    consistency_translation_thresh_ =
      declare_parameter<double>("consistency_translation_thresh", 0.5);
    consistency_yaw_thresh_ = declare_parameter<double>("consistency_yaw_thresh", 0.1);
    max_correction_translation_ =
      declare_parameter<double>("max_correction_translation", 2.0);
    max_correction_yaw_ = declare_parameter<double>("max_correction_yaw", 0.5);
    drift_score_threshold_ =
      declare_parameter<double>("drift_score_threshold", 0.3);
    drift_consistency_frames_ =
      std::max(2, static_cast<int>(declare_parameter<int>("drift_consistency_frames", 3)));
    drift_consistency_translation_ =
      declare_parameter<double>("drift_consistency_translation", 0.3);
    drift_consistency_yaw_ =
      declare_parameter<double>("drift_consistency_yaw", 0.1);
    min_recovery_map_points_ =
      std::max(50, static_cast<int>(declare_parameter<int>("min_recovery_map_points", 200)));
    lost_relocalize_every_n_ =
      std::max(1, static_cast<int>(declare_parameter<int>("lost_relocalize_every_n", 1)));
    drift_relocalize_every_n_ =
      std::max(1, static_cast<int>(declare_parameter<int>("drift_relocalize_every_n", 3)));

    const std::vector<double> initial_pose_vec{
      declare_parameter<double>("initial_x", 0.0), declare_parameter<double>("initial_y", 0.0),
      declare_parameter<double>("initial_z", 0.0), declare_parameter<double>("initial_roll", 0.0),
      declare_parameter<double>("initial_pitch", 0.0), declare_parameter<double>("initial_yaw", 0.0)};
    initial_pose_ = vectorToPose(initial_pose_vec);
    have_initial_pose_ = !require_initial_pose_;
  }

  void loadMapPyramid()
  {
    CloudT::Ptr raw_map(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(map_path_, *raw_map) != 0 || raw_map->empty()) {
      throw std::runtime_error("failed to load non-empty PCD map: " + map_path_);
    }

    map_pyramid_.clear();
    map_kdtrees_.clear();
    for (const double leaf_size : voxel_leaf_sizes_) {
      CloudT::Ptr level_map = voxelDownsample(raw_map, leaf_size);
      KdTreeT::Ptr tree(new KdTreeT);
      tree->setInputCloud(level_map);
      map_pyramid_.push_back(level_map);
      map_kdtrees_.push_back(tree);
      RCLCPP_INFO(
        get_logger(), "map level leaf=%.3f points=%zu", leaf_size, level_map->size());
    }

    map_for_publish_ = map_pyramid_.back();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    latest_odom_tf_ = poseMsgToIso(msg->pose.pose);
    have_odom_ = true;

    if (!initialized_ && have_initial_pose_) {
      initializeLocked();
    }
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "/initialpose frame is %s, but this node treats it as %s",
        msg->header.frame_id.c_str(), map_frame_.c_str());
    }

    initial_pose_ = poseMsgToIso(msg->pose.pose);
    have_initial_pose_ = true;
    if (have_odom_) {
      initializeLocked();
    } else {
      RCLCPP_INFO(get_logger(), "received /initialpose, waiting for odometry");
    }
  }

  void initializeLocked()
  {
    map_to_odom_ = initial_pose_ * latest_odom_tf_.inverse();
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "initialized relocalization prior from /initialpose and odometry");
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom;
    Eigen::Isometry3d odom_tf;
    Eigen::Isometry3d map_to_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_odom_ || !initialized_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000, "waiting for odometry and RViz /initialpose");
        return;
      }
      odom = latest_odom_;
      odom_tf = latest_odom_tf_;
      map_to_odom = map_to_odom_;
    }

    // --- Adaptive frequency: choose skip interval based on current mode ---
    const int effective_interval =
      (relocalization_mode_ == MODE_LOST) ? lost_relocalize_every_n_ :
      (relocalization_mode_ == MODE_DRIFT) ? drift_relocalize_every_n_ :
      relocalize_every_n_scans_;

    if (++scan_count_ % effective_interval != 0) {
      publishCorrectedOdom(odom, map_to_odom, map_to_odom * odom_tf);
      return;
    }

    CloudT::Ptr scan(new CloudT);
    pcl::fromROSMsg(*msg, *scan);
    CloudT::Ptr scan_filtered = preprocessScan(scan);
    if (static_cast<int>(scan_filtered->size()) < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "scan rejected: only %zu points after filtering",
        scan_filtered->size());
      return;
    }

    const Eigen::Isometry3d predicted_map_base =
      use_odom_prediction_ ? map_to_odom * odom_tf : map_to_odom;
    const RegistrationResult result = runLocalMapPyramidGicp(scan_filtered, predicted_map_base);

    if (!result.converged || result.score > fitness_score_threshold_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "GICP rejected: converged=%d score=%.3f",
        result.converged, result.score);
      pending_corrections_.clear();
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      return;
    }

    // --- Evaluate relocalization result ---
    const Eigen::Isometry3d relocalization_pose = result.pose;
    const Eigen::Isometry3d lio_pose = map_to_odom * odom_tf;
    const Eigen::Isometry3d delta = lio_pose.inverse() * relocalization_pose;

    const double delta_translation = delta.translation().norm();
    const Eigen::AngleAxisd delta_aa(delta.linear());
    const double delta_yaw = std::abs(delta_aa.angle());

    // --- Classify into three modes based on delta ---
    if (delta_translation > max_correction_translation_ ||
        delta_yaw > max_correction_yaw_) {
      // Case 3: Lost localization — large discrepancy
      if (relocalization_mode_ != MODE_LOST) {
        RCLCPP_WARN(
          get_logger(),
          "LOST LOCALIZATION: delta=%.2fm/%.2frad, need %d consistent GICP frames to recover",
          delta_translation, delta_yaw, drift_consistency_frames_);
        pending_corrections_.clear();
      }
      relocalization_mode_ = MODE_LOST;
    } else if (delta_translation > consistency_translation_thresh_ ||
               delta_yaw > consistency_yaw_thresh_) {
      // Case 2: LIO has drifted — moderate discrepancy
      if (relocalization_mode_ != MODE_DRIFT) {
        RCLCPP_WARN(
          get_logger(),
          "LIO DRIFT suspected: delta=%.2fm/%.2frad, collecting GICP evidence",
          delta_translation, delta_yaw);
        pending_corrections_.clear();
      }
      relocalization_mode_ = MODE_DRIFT;
    } else {
      // Case 1: Normal — small discrepancy, LIO is fine
      if (relocalization_mode_ != MODE_NORMAL) {
        RCLCPP_INFO(
          get_logger(), "relocalization back to normal: delta=%.3fm/%.3frad",
          delta_translation, delta_yaw);
        pending_corrections_.clear();
      }
      relocalization_mode_ = MODE_NORMAL;
    }

    // --- GICP validation: score check ---
    const bool strict_score = (relocalization_mode_ != MODE_NORMAL);
    const double score_limit = strict_score ? drift_score_threshold_ : fitness_score_threshold_;
    if (result.score > score_limit) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GICP score too high for mode %d: %.3f > %.3f",
        static_cast<int>(relocalization_mode_), result.score, score_limit);
      pending_corrections_.clear();
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      return;
    }

    // --- GICP validation: local map quality ---
    if (strict_score &&
        static_cast<int>(result.local_map->size()) < min_recovery_map_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "local map too sparse for recovery: %zu points (need %d)",
        result.local_map->size(), min_recovery_map_points_);
      pending_corrections_.clear();
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      return;
    }

    // --- NORMAL mode: LIO is fine, just confirm, don't update ---
    if (relocalization_mode_ == MODE_NORMAL) {
      pending_corrections_.clear();
      // No need to update map→odom — LIO hasn't drifted, delta is small
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "NORMAL: GICP confirms LIO (delta=%.3fm), no correction needed",
        delta_translation);
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      publishAlignedScan(*result.aligned, msg->header.stamp);
      publishLocalMap(*result.local_map, msg->header.stamp);
      return;
    }

    // --- DRIFT / LOST mode: collect GICP candidates, require consistency ---
    const Eigen::Isometry3d candidate_m2o = relocalization_pose * odom_tf.inverse();
    pending_corrections_.push_back(candidate_m2o);

    // Keep only the last N candidates
    while (static_cast<int>(pending_corrections_.size()) > drift_consistency_frames_) {
      pending_corrections_.erase(pending_corrections_.begin());
    }

    // Check if we have enough candidates
    if (static_cast<int>(pending_corrections_.size()) < drift_consistency_frames_) {
      RCLCPP_INFO(
        get_logger(),
        "collecting GICP evidence (%d/%d): score=%.3f delta=%.3fm",
        static_cast<int>(pending_corrections_.size()), drift_consistency_frames_,
        result.score, delta_translation);
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      publishAlignedScan(*result.aligned, msg->header.stamp);
      publishLocalMap(*result.local_map, msg->header.stamp);
      return;
    }

    // Check consistency: all recent candidates must agree with each other
    bool consistent = true;
    for (int i = 1; i < drift_consistency_frames_; ++i) {
      const Eigen::Isometry3d diff =
        pending_corrections_[i - 1].inverse() * pending_corrections_[i];
      const double t_diff = diff.translation().norm();
      const double y_diff = std::abs(Eigen::AngleAxisd(diff.linear()).angle());
      if (t_diff > drift_consistency_translation_ || y_diff > drift_consistency_yaw_) {
        consistent = false;
        RCLCPP_INFO(
          get_logger(),
          "GICP candidates inconsistent: frame %d vs %d: t=%.3fm y=%.3frad",
          i - 1, i, t_diff, y_diff);
        break;
      }
    }

    if (!consistent) {
      // GICP results are jumping around — not trustworthy
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GICP results inconsistent across %d frames, rejecting",
        drift_consistency_frames_);
      pending_corrections_.clear();
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      publishAlignedScan(*result.aligned, msg->header.stamp);
      publishLocalMap(*result.local_map, msg->header.stamp);
      return;
    }

    // All checks passed — accept the correction (use the latest candidate)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_to_odom_ = candidate_m2o;
      map_to_odom = map_to_odom_;
    }
    pending_corrections_.clear();

    // After correction, re-evaluate: if delta is now small, return to normal
    const Eigen::Isometry3d new_delta =
      (map_to_odom * odom_tf).inverse() * relocalization_pose;
    if (new_delta.translation().norm() < consistency_translation_thresh_ &&
        std::abs(Eigen::AngleAxisd(new_delta.linear()).angle()) < consistency_yaw_thresh_) {
      relocalization_mode_ = MODE_NORMAL;
      RCLCPP_INFO(get_logger(), "correction applied, returning to NORMAL mode");
    }

    RCLCPP_INFO(
      get_logger(),
      "correction accepted: mode=%d score=%.3f delta=%.3fm local_map=%zu points",
      static_cast<int>(relocalization_mode_), result.score,
      delta_translation, result.local_map->size());

    publishCorrectedOdom(odom, map_to_odom, relocalization_pose);
    publishAlignedScan(*result.aligned, msg->header.stamp);
    publishLocalMap(*result.local_map, msg->header.stamp);
  }

  CloudT::Ptr preprocessScan(const CloudT::ConstPtr & scan)
  {
    CloudT::Ptr filtered(new CloudT);
    *filtered = *scan;

    if (enable_sor_ && static_cast<int>(filtered->size()) > sor_mean_k_) {
      CloudT::Ptr sor_filtered(new CloudT);
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(filtered);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_mul_thresh_);
      sor.filter(*sor_filtered);
      filtered = sor_filtered;
    }

    return filtered;
  }

  RegistrationResult runLocalMapPyramidGicp(
    const CloudT::ConstPtr & source, const Eigen::Isometry3d & initial_guess)
  {
    RegistrationResult result;
    Eigen::Matrix4f current_guess = initial_guess.matrix().cast<float>();

    for (std::size_t i = 0; i < voxel_leaf_sizes_.size(); ++i) {
      CloudT::Ptr source_down = voxelDownsample(source, voxel_leaf_sizes_[i]);
      if (static_cast<int>(source_down->size()) < min_scan_points_) {
        return result;
      }

      const Eigen::Vector3d center(
        current_guess(0, 3), current_guess(1, 3), current_guess(2, 3));
      CloudT::Ptr local_map = extractLocalMap(i, center);
      if (local_map->empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "local map empty at level %zu", i);
        return result;
      }
      // Only enforce min_local_map_points at the finest level (last iteration).
      // Coarse levels may have fewer points due to heavy downsampling; they are
      // only used to refine the initial guess for the next finer level.
      if (i == voxel_leaf_sizes_.size() - 1 &&
          static_cast<int>(local_map->size()) < min_local_map_points_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "local map too small at finest level %zu: %zu points (need %d)",
          i, local_map->size(), min_local_map_points_);
        return result;
      }

      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setMaximumIterations(max_iterations_);
      gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
      gicp.setTransformationEpsilon(transformation_epsilon_);
      gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
      gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);
      gicp.setMaximumOptimizerIterations(gicp_max_optimizer_iterations_);
      gicp.setRANSACIterations(ransac_iterations_);
      gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);
      gicp.setInputSource(source_down);
      gicp.setInputTarget(local_map);

      CloudT::Ptr aligned(new CloudT);
      gicp.align(*aligned, current_guess);
      if (!gicp.hasConverged()) {
        return result;
      }

      current_guess = gicp.getFinalTransformation();
      result.converged = true;
      result.score = gicp.getFitnessScore();
      result.aligned = aligned;
      result.local_map = local_map;
    }

    result.pose = Eigen::Isometry3d::Identity();
    result.pose.matrix() = current_guess.cast<double>();
    return result;
  }

  CloudT::Ptr extractLocalMap(std::size_t level, const Eigen::Vector3d & center)
  {
    CloudT::Ptr local_map(new CloudT);
    if (level >= map_pyramid_.size()) {
      return local_map;
    }

    PointT search_point;
    search_point.x = static_cast<float>(center.x());
    search_point.y = static_cast<float>(center.y());
    search_point.z = static_cast<float>(center.z());

    std::vector<int> indices;
    std::vector<float> sq_distances;
    map_kdtrees_[level]->radiusSearch(search_point, local_map_radius_, indices, sq_distances);
    local_map->reserve(indices.size());
    for (const int index : indices) {
      local_map->push_back((*map_pyramid_[level])[index]);
    }
    local_map->width = static_cast<uint32_t>(local_map->size());
    local_map->height = 1;
    local_map->is_dense = map_pyramid_[level]->is_dense;
    return local_map;
  }

  void publishCorrectedOdom(
    const nav_msgs::msg::Odometry & odom, const Eigen::Isometry3d & map_to_odom,
    const Eigen::Isometry3d & corrected_map_base)
  {
    nav_msgs::msg::Odometry corrected = odom;
    corrected.header.frame_id = map_frame_;
    corrected.child_frame_id = base_frame_;
    corrected.pose.pose = isoToPoseMsg(corrected_map_base);
    corrected_odom_pub_->publish(corrected);

    if (publish_tf_) {
      tf_broadcaster_->sendTransform(
        isoToTransformMsg(map_to_odom, odom.header.stamp, map_frame_, odom_frame_));
    }
  }

  void publishAlignedScan(const CloudT & cloud, const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_aligned_scan_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    aligned_scan_pub_->publish(msg);
  }

  void publishLocalMap(const CloudT & cloud, const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_local_map_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    local_map_pub_->publish(msg);
  }

  void publishMap()
  {
    if (map_for_publish_->empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map is empty, nothing to publish");
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_for_publish_, msg);
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    map_pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "published map on %s: %zu points",
      map_pub_->get_topic_name(), map_for_publish_->size());
  }

  std::string map_path_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string output_odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_tf_ = true;
  bool publish_aligned_scan_ = true;
  bool publish_local_map_ = true;
  bool require_initial_pose_ = true;
  bool use_odom_prediction_ = true;
  bool enable_sor_ = true;

  double local_map_radius_ = 8.0;
  double max_correspondence_distance_ = 1.5;
  double transformation_epsilon_ = 0.01;
  double euclidean_fitness_epsilon_ = 0.01;
  double fitness_score_threshold_ = 1.0;
  double ransac_outlier_rejection_threshold_ = 0.05;
  double sor_stddev_mul_thresh_ = 1.0;

  int min_scan_points_ = 80;
  int min_local_map_points_ = 300;
  int max_iterations_ = 30;
  int gicp_correspondence_randomness_ = 20;
  int gicp_max_optimizer_iterations_ = 20;
  int ransac_iterations_ = 0;
  int relocalize_every_n_scans_ = 1;
  int scan_count_ = 0;
  int sor_mean_k_ = 50;

  // --- Adaptive relocalization parameters ---
  int lost_relocalize_every_n_ = 1;
  int drift_relocalize_every_n_ = 3;
  int drift_consistency_frames_ = 3;
  int min_recovery_map_points_ = 200;
  double consistency_translation_thresh_ = 0.5;
  double consistency_yaw_thresh_ = 0.1;
  double max_correction_translation_ = 2.0;
  double max_correction_yaw_ = 0.5;
  double drift_score_threshold_ = 0.3;
  double drift_consistency_translation_ = 0.3;
  double drift_consistency_yaw_ = 0.1;

  // --- Adaptive relocalization state ---
  enum RelocalizationMode { MODE_NORMAL, MODE_DRIFT, MODE_LOST };
  RelocalizationMode relocalization_mode_ = MODE_NORMAL;
  std::vector<Eigen::Isometry3d> pending_corrections_;

  std::vector<double> voxel_leaf_sizes_;
  std::vector<CloudT::Ptr> map_pyramid_;
  std::vector<KdTreeT::Ptr> map_kdtrees_;
  CloudT::Ptr map_for_publish_{new CloudT};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex mutex_;
  bool have_odom_ = false;
  bool have_initial_pose_ = false;
  bool initialized_ = false;
  nav_msgs::msg::Odometry latest_odom_;
  Eigen::Isometry3d latest_odom_tf_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_to_odom_ = Eigen::Isometry3d::Identity();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToMapRelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
