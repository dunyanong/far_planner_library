// far_planner_util.cpp
#include "far_planner/far_planner.h"
#include <limits>

// Define all static member variables of FARUtil
PointCloudPtr  FARUtil::surround_obs_cloud_;
PointCloudPtr  FARUtil::surround_free_cloud_;
PointCloudPtr  FARUtil::stack_new_cloud_;
PointCloudPtr  FARUtil::cur_new_cloud_;
PointCloudPtr  FARUtil::cur_dyobs_cloud_;
PointCloudPtr  FARUtil::stack_dyobs_cloud_;
PointCloudPtr  FARUtil::cur_scan_cloud_;
PointCloudPtr  FARUtil::local_terrain_obs_;
PointCloudPtr  FARUtil::local_terrain_free_;
PointKdTreePtr FARUtil::kdtree_new_cloud_;
PointKdTreePtr FARUtil::kdtree_filter_cloud_;

// Define static const members
const float FARUtil::kEpsilon = 1e-7f;
const float FARUtil::kINF = std::numeric_limits<float>::max();

// Define static primitive type members
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise = 0.0f;
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim = 0.0f;
float   FARUtil::vehicle_height = 0.0f;
float   FARUtil::kLeafSize = 0.0f;
float   FARUtil::kHeightVoxel = 0.0f;
float   FARUtil::kNavClearDist = 0.0f;
float   FARUtil::kCellLength = 0.0f;
float   FARUtil::kCellHeight = 0.0f;
float   FARUtil::kNewPIThred = 0.0f;
float   FARUtil::kSensorRange = 0.0f;
float   FARUtil::kMarginDist = 0.0f;
float   FARUtil::kMarginHeight = 0.0f;
float   FARUtil::kTerrainRange = 0.0f;
float   FARUtil::kLocalPlanRange = 0.0f;
float   FARUtil::kFreeZ = 0.0f;
float   FARUtil::kVizRatio = 0.0f;
double  FARUtil::systemStartTime = 0.0;
float   FARUtil::kObsDecayTime = 0.0f;
float   FARUtil::kNewDecayTime = 0.0f;
float   FARUtil::kNearDist = 0.0f;
float   FARUtil::kMatchDist = 0.0f;
float   FARUtil::kProjectDist = 0.0f;  // This is the missing symbol!
int     FARUtil::kDyObsThred = 0;
int     FARUtil::KNewPointC = 0;
int     FARUtil::kObsInflate = 0;
float   FARUtil::kTolerZ = 0.0f;
float   FARUtil::kAcceptAlign = 0.0f;
bool    FARUtil::IsStaticEnv = false;
bool    FARUtil::IsDebug = false;
bool    FARUtil::IsMultiLayer = false;
TimeMeasure FARUtil::Timer;

// Define static variables for other classes
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_ = 0;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;

CTNodeStack ContourGraph::polys_ctnodes_;
CTNodeStack ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;