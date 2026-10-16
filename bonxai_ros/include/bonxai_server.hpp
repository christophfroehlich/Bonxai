#ifndef BONXAI_SERVER__BONXAI_SERVER_HPP_
#define BONXAI_SERVER__BONXAI_SERVER_HPP_

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "bonxai/bonxai.hpp"
#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/message_filter.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace bonxai_server {

using sensor_msgs::msg::PointCloud2;

class BonxaiServer : public rclcpp_lifecycle::LifecycleNode {
 public:
  using PCLPoint = pcl::PointXYZ;
  using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using BonxaiT = Bonxai::ProbabilisticMap;
  using ResetSrv = std_srvs::srv::Empty;

  explicit BonxaiServer(const rclcpp::NodeOptions& node_options);

  bool resetSrv(
      const std::shared_ptr<ResetSrv::Request> req, const std::shared_ptr<ResetSrv::Response> resp);

  virtual void insertCloudCallback(const PointCloud2::ConstSharedPtr cloud);

  virtual void insertCloudsCallback(
      const PointCloud2::ConstSharedPtr hit_cloud, const PointCloud2::ConstSharedPtr free_cloud);

 protected:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  virtual void publishAll(const rclcpp::Time& rostime);

    bool isActive() const;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult onParameter(
      const std::vector<rclcpp::Parameter>& parameters);

    rclcpp_lifecycle::LifecyclePublisher<PointCloud2>::SharedPtr point_cloud_pub_;
  message_filters::Subscriber<PointCloud2, rclcpp_lifecycle::LifecycleNode> point_cloud_sub_;
  message_filters::Subscriber<PointCloud2, rclcpp_lifecycle::LifecycleNode> free_point_cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> tf_point_cloud_sub_;
  using CloudSyncPolicy = message_filters::sync_policies::ExactTime<PointCloud2, PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<CloudSyncPolicy>> cloud_sync_;
  // rclcpp::Service<BBoxSrv>::SharedPtr clear_bbox_srv_;
  rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::unique_ptr<BonxaiT> bonxai_;
  std::vector<Bonxai::CoordT> key_ray_;

  double max_range_;
  bool use_separate_free_cloud_;
    int64_t cloud_sync_queue_size_;
  std::string world_frame_id_;  // the map frame
  std::string base_frame_id_;   // base of the robot for ground plane filtering

  bool latched_topics_;

  double res_;

  double occupancy_min_z_;
  double occupancy_max_z_;

  bool publish_2d_map_;
  bool map_origin_changed;
  // octomap::OcTreeKey padded_min_key_;
  unsigned multires_2d_scale_;
  bool project_complete_map_;
};
}  // namespace bonxai_server

#endif  // BONXAI_SERVER__BONXAI_SERVER_HPP_
