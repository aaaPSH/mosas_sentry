#include <algorithm>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

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
}  // namespace

class ScanToMapRelocalizationNode : public rclcpp::Node
{
public:
  ScanToMapRelocalizationNode() : Node("scan_to_map_relocalization")
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
    require_initial_pose_ = declare_parameter<bool>("require_initial_pose", true);
    use_odom_prediction_ = declare_parameter<bool>("use_odom_prediction", true);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.25);
    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.5);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 0.01);
    euclidean_fitness_epsilon_ = declare_parameter<double>("euclidean_fitness_epsilon", 0.01);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 1.0);
    min_scan_points_ = static_cast<int>(declare_parameter<int>("min_scan_points", 80));
    max_iterations_ = static_cast<int>(declare_parameter<int>("max_iterations", 30));
    relocalize_every_n_scans_ =
      std::max(1, static_cast<int>(declare_parameter<int>("relocalize_every_n_scans", 1)));

    const std::vector<double> initial_pose_vec{
      declare_parameter<double>("initial_x", 0.0), declare_parameter<double>("initial_y", 0.0),
      declare_parameter<double>("initial_z", 0.0), declare_parameter<double>("initial_roll", 0.0),
      declare_parameter<double>("initial_pitch", 0.0), declare_parameter<double>("initial_yaw", 0.0)};
    initial_pose_ = vectorToPose(initial_pose_vec);
    have_initial_pose_ = !require_initial_pose_;

    loadMap();

    icp_.setInputTarget(map_);
    icp_.setMaximumIterations(max_iterations_);
    icp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp_.setTransformationEpsilon(transformation_epsilon_);
    icp_.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

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
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/relocalization/map", rclcpp::QoS(1).transient_local().reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    publishMap();

    if (require_initial_pose_) {
      RCLCPP_INFO(
        get_logger(), "loaded map and waiting for RViz /initialpose. Set RViz Fixed Frame to %s.",
        map_frame_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "loaded map and will initialize from initial_* parameters.");
    }
  }

private:
  void loadMap()
  {
    CloudT raw_map;
    if (pcl::io::loadPCDFile<PointT>(map_path_, raw_map) != 0 || raw_map.empty()) {
      throw std::runtime_error("failed to load non-empty PCD map: " + map_path_);
    }

    map_.reset(new CloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.setInputCloud(raw_map.makeShared());
    voxel.filter(*map_);

    RCLCPP_INFO(
      get_logger(), "loaded localization map: raw=%zu filtered=%zu", raw_map.size(), map_->size());
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

    if (++scan_count_ % relocalize_every_n_scans_ != 0) {
      publishCorrectedOdom(odom, map_to_odom, map_to_odom * odom_tf);
      return;
    }

    CloudT::Ptr scan(new CloudT);
    pcl::fromROSMsg(*msg, *scan);
    if (static_cast<int>(scan->size()) < min_scan_points_) {
      return;
    }

    CloudT::Ptr scan_filtered(new CloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.setInputCloud(scan);
    voxel.filter(*scan_filtered);
    if (static_cast<int>(scan_filtered->size()) < min_scan_points_) {
      return;
    }

    const Eigen::Isometry3d predicted_map_base = use_odom_prediction_ ? map_to_odom * odom_tf : map_to_odom;
    CloudT::Ptr aligned(new CloudT);
    icp_.setInputSource(scan_filtered);
    icp_.align(*aligned, predicted_map_base.matrix().cast<float>());

    if (!icp_.hasConverged() || icp_.getFitnessScore() > fitness_score_threshold_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "ICP rejected: converged=%d score=%.3f",
        icp_.hasConverged(), icp_.getFitnessScore());
      publishCorrectedOdom(odom, map_to_odom, predicted_map_base);
      return;
    }

    const Eigen::Matrix4d corrected_mat = icp_.getFinalTransformation().cast<double>();
    Eigen::Isometry3d corrected_map_base = Eigen::Isometry3d::Identity();
    corrected_map_base.matrix() = corrected_mat;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_to_odom_ = corrected_map_base * odom_tf.inverse();
      map_to_odom = map_to_odom_;
    }

    publishCorrectedOdom(odom, map_to_odom, corrected_map_base);
    publishAlignedScan(*aligned, msg->header.stamp);
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

  void publishMap()
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg);
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    map_pub_->publish(msg);
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
  bool require_initial_pose_ = true;
  bool use_odom_prediction_ = true;
  double voxel_leaf_size_ = 0.25;
  double max_correspondence_distance_ = 1.5;
  double transformation_epsilon_ = 0.01;
  double euclidean_fitness_epsilon_ = 0.01;
  double fitness_score_threshold_ = 1.0;
  int min_scan_points_ = 80;
  int max_iterations_ = 30;
  int relocalize_every_n_scans_ = 1;
  int scan_count_ = 0;

  CloudT::Ptr map_{new CloudT};
  pcl::IterativeClosestPoint<PointT, PointT> icp_;
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToMapRelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
