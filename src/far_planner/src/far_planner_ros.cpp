#include "far_planner/far_planner_ros.h"

#include <fstream>
#include <sstream>
#include <tf/tf.h>

namespace far_planner {

FarPlannerROS::FarPlannerROS()
  : initialized_(false), costmap_ros_(nullptr), tf_listener_(tf_buffer_) {}

FarPlannerROS::FarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false), costmap_ros_(nullptr), tf_listener_(tf_buffer_) {
  initialize(name, costmap_ros);
}

void FarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("FarPlannerROS: already initialized");
    return;
  }

  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~" + name);
  costmap_ros_ = costmap_ros;

  pnh_.param<std::string>("planner_frame", planner_frame_, std::string("map"));
  pnh_.param<std::string>("graph_file", graph_file_path_, std::string(""));

  plan_pub_ = nh_.advertise<nav_msgs::Path>("plan", 1, true);

  // initialize graph planner (use default params)
  GraphPlannerParams gp_params;
  graph_planner_.Init(nh_, gp_params);

  if (!graph_file_path_.empty()) {
    if (!loadGraphFromFile(graph_file_path_, loaded_graph_)) {
      ROS_WARN("FarPlannerROS: failed to load graph from %s", graph_file_path_.c_str());
    } else {
      // set graph into planner
      graph_planner_.UpdaetVGraph(loaded_graph_);
      ROS_INFO("FarPlannerROS: loaded graph with %zu nodes", loaded_graph_.size());
    }
  }

  initialized_ = true;
  ROS_INFO("FarPlannerROS initialized (planner_frame=%s)", planner_frame_.c_str());
}

bool FarPlannerROS::transformToPlannerFrame(const geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out) {
  if (in.header.frame_id == planner_frame_) {
    out = in;
    return true;
  }
  try {
    tf_buffer_.transform(in, out, planner_frame_, ros::Duration(0.5));
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "FarPlannerROS: transform failed: %s", ex.what());
    return false;
  }
}

void FarPlannerROS::convertNavPathToPlan(const NodePtrStack& node_path, const std::string& frame,
                                         std::vector<geometry_msgs::PoseStamped>& plan_out) {
  plan_out.clear();
  ros::Time now = ros::Time::now();
  for (const auto& n : node_path) {
    if (!n) continue;
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = now;
    ps.header.frame_id = frame;
    ps.pose.position.x = n->position.x;
    ps.pose.position.y = n->position.y;
    ps.pose.position.z = n->position.z;
    // orientation: try to infer from next node if available
    ps.pose.orientation.w = 1.0;
    plan_out.push_back(ps);
  }
}

// parse file format used by graph_decoder and data/*.vgh
bool FarPlannerROS::loadGraphFromFile(const std::string& file_path, NodePtrStack& out_graph) {
  out_graph.clear();
  std::ifstream graph_file(file_path);
  if (!graph_file.is_open()) {
    ROS_ERROR("FarPlannerROS: cannot open graph file %s", file_path.c_str());
    return false;
  }

  std::string line;
  std::vector<std::vector<std::size_t>> connect_idx, poly_idx, contour_idx, traj_idx;
  std::unordered_map<std::size_t, std::size_t> id2idx;

  std::size_t idx = 0;
  while (std::getline(graph_file, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::vector<std::string> components;
    std::string token;
    while (ss >> token) components.push_back(token);

    NavNodePtr node = std::make_shared<NavNode>();
    std::vector<std::size_t> cidx, pidx, ctidx, tidx;

    for (std::size_t i=0; i<components.size(); ++i) {
      if (i == 0) node->id = static_cast<std::size_t>(std::stoul(components[i]));
      else if (i == 1) node->free_direct = static_cast<NodeFreeDirect>(std::stoi(components[i]));
      else if (i == 2) node->position.x = std::stof(components[i]);
      else if (i == 3) node->position.y = std::stof(components[i]);
      else if (i == 4) node->position.z = std::stof(components[i]);
      else if (i == 5) node->surf_dirs.first.x = std::stof(components[i]);
      else if (i == 6) node->surf_dirs.first.y = std::stof(components[i]);
      else if (i == 7) node->surf_dirs.first.z = std::stof(components[i]);
      else if (i == 8) node->surf_dirs.second.x = std::stof(components[i]);
      else if (i == 9) node->surf_dirs.second.y = std::stof(components[i]);
      else if (i == 10) node->surf_dirs.second.z = std::stof(components[i]);
      else if (i == 11) node->is_covered  = std::stoi(components[i]) != 0;
      else if (i == 12) node->is_frontier = std::stoi(components[i]) != 0;
      else if (i == 13) node->is_navpoint = std::stoi(components[i]) != 0;
      else if (i == 14) node->is_boundary = std::stoi(components[i]) != 0;
      else {
        // after index 14, the lists are separated by '|' tokens
        // We'll collect all remaining tokens, then split by '|'
      }
    }

    // collect lists by re-parsing tail
    // find the substring starting after the 14th token
    std::size_t pos = 0; int cnt = 0; std::size_t startpos = 0;
    while (cnt < 14 && pos != std::string::npos) {
      pos = line.find(' ', pos);
      if (pos == std::string::npos) break;
      pos++;
      cnt++;
      startpos = pos;
    }
    if (startpos < line.size()) {
      std::string tail = line.substr(startpos);
      // split by spaces, but handle '|' separators
      std::istringstream tss(tail);
      std::string tk;
      int phase = 0; // 0: connect,1:poly,2:contour,3:traj
      while (tss >> tk) {
        if (tk == "|") { phase++; continue; }
        std::size_t val = static_cast<std::size_t>(std::stoul(tk));
        if (phase == 0) cidx.push_back(val);
        else if (phase == 1) pidx.push_back(val);
        else if (phase == 2) ctidx.push_back(val);
        else if (phase == 3) tidx.push_back(val);
      }
    }

    // store temporary idx lists
    connect_idx.push_back(cidx);
    poly_idx.push_back(pidx);
    contour_idx.push_back(ctidx);
    traj_idx.push_back(tidx);

    // push node
    out_graph.push_back(node);
    id2idx.insert({node->id, idx});
    idx++;
  }

  // connect nodes using indices
  connectGraphIndices(out_graph, id2idx, connect_idx, poly_idx, contour_idx, traj_idx);

  graph_file.close();
  return true;
}

void FarPlannerROS::connectGraphIndices(NodePtrStack& graph, const std::unordered_map<std::size_t, std::size_t>& id2idx,
                                       const std::vector<std::vector<std::size_t>>& connect_idx,
                                       const std::vector<std::vector<std::size_t>>& poly_idx,
                                       const std::vector<std::vector<std::size_t>>& contour_idx,
                                       const std::vector<std::vector<std::size_t>>& traj_idx) {
  const std::size_t N = graph.size();
  for (std::size_t i=0; i<N; ++i) {
    const auto& cvec = connect_idx[i];
    for (const auto& id : cvec) {
      auto it = id2idx.find(id);
      if (it != id2idx.end()) {
        graph[i]->connect_nodes.push_back(graph[it->second]);
      }
    }
    const auto& pvec = poly_idx[i];
    for (const auto& id : pvec) {
        auto it = id2idx.find(id);
        if (it != id2idx.end()) graph[i]->poly_connects.push_back(graph[it->second]);
    }
    const auto& ctvec = contour_idx[i];
    for (const auto& id : ctvec) {
        auto it = id2idx.find(id);
        if (it != id2idx.end()) graph[i]->contour_connects.push_back(graph[it->second]);
    }
    const auto& tvec = traj_idx[i];
    for (const auto& id : tvec) {
        auto it = id2idx.find(id);
        if (it != id2idx.end()) graph[i]->trajectory_connects.push_back(graph[it->second]);
    }
  }
}

bool FarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!initialized_) {
    ROS_ERROR("FarPlannerROS: not initialized");
    return false;
  }

  geometry_msgs::PoseStamped start_pf, goal_pf;
  if (!transformToPlannerFrame(start, start_pf) || !transformToPlannerFrame(goal, goal_pf)) {
    ROS_WARN("FarPlannerROS: transform to planner frame failed");
    return false;
  }

  // create odom and goal nodes
  NavNodePtr odom_node = NULL, goal_node = NULL;
  Point3D spt(start_pf.pose.position.x, start_pf.pose.position.y, start_pf.pose.position.z);
  Point3D gpt(goal_pf.pose.position.x, goal_pf.pose.position.y, goal_pf.pose.position.z);
  DynamicGraph::CreateNavNodeFromPoint(spt, odom_node, true, false, false);
  DynamicGraph::CreateNavNodeFromPoint(gpt, goal_node, false, false, true);

  // update graph traversability and goal connectivity
  graph_planner_.UpdateGraphTraverability(odom_node, goal_node);
  graph_planner_.UpdateGoalNavNodeConnects(goal_node);

  NodePtrStack global_path;
  NavNodePtr nav_ptr;
  Point3D adjusted_goal;
  bool is_fail = false, is_succeed = false, is_free_nav = false;

  const bool ok = graph_planner_.PathToGoal(goal_node, global_path, nav_ptr, adjusted_goal, is_fail, is_succeed, is_free_nav);
  if (!ok || global_path.empty()) {
    ROS_WARN("FarPlannerROS: PathToGoal failed or returned empty path");
    return false;
  }

  convertNavPathToPlan(global_path, planner_frame_, plan);

  // publish nav_msgs::Path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = planner_frame_;
  path_msg.poses = plan;
  plan_pub_.publish(path_msg);

  return !plan.empty();
}

// Export plugin
PLUGINLIB_EXPORT_CLASS(far_planner::FarPlannerROS, nav_core::BaseGlobalPlanner)

} // namespace far_planner
