#ifndef FAR_PLANNER_ROS_H
#define FAR_PLANNER_ROS_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "far_planner/graph_planner.h"
#include "far_planner/dynamic_graph.h"
#include "far_planner/point_struct.h"

namespace far_planner {

class FarPlannerROS : public nav_core::BaseGlobalPlanner {
public:
  FarPlannerROS();
  FarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  bool initialized_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  // FAR internal objects
  GraphPlanner graph_planner_;

  // loaded graph nodes
  NodePtrStack loaded_graph_;

  // planning frame
  std::string planner_frame_;
  std::string graph_file_path_;

  // publishers
  ros::Publisher plan_pub_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // helpers
  bool loadGraphFromFile(const std::string& file_path, NodePtrStack& out_graph);
  void connectGraphIndices(NodePtrStack& graph, const std::unordered_map<std::size_t, std::size_t>& id2idx,
                           const std::vector<std::vector<std::size_t>>& connect_idx,
                           const std::vector<std::vector<std::size_t>>& poly_idx,
                           const std::vector<std::vector<std::size_t>>& contour_idx,
                           const std::vector<std::vector<std::size_t>>& traj_idx);
  bool transformToPlannerFrame(const geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out);
  void convertNavPathToPlan(const NodePtrStack& node_path, const std::string& frame,
                            std::vector<geometry_msgs::PoseStamped>& plan_out);
};

} // namespace far_planner

#endif // FAR_PLANNER_ROS_H
