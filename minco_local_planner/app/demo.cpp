/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 09:52:24
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-30 22:20:19
 */
#include "demo.h"

#include "basis/laser_scan.h"
#include "basis/time.h"
#include "module_manager/module_manager.h"

using namespace minco_local_planner::module_manager;
using namespace minco_local_planner::basis;
using namespace minco_local_planner::utils;
Demo::Demo(const std::string config_file_path)
    : pnh_("~"), config_file_path_(config_file_path) {}

bool Demo::Init() {
  if (!InitPlanner()) {
    return false;
  }
  InitRos();
  return true;
}
void Demo::Run() {
  Singleton<TimerManager>()->Schedule(100, std::bind(&Demo::VisTimer, this));
  ModuleManager::GetInstance()->Run();
  ros::spin();
}

void Demo::InitRos() {
  odom_sub_ = nh_.subscribe("/odom", 1, &Demo::OdomCallback, this);
  laser_scan_sub_ = nh_.subscribe("/scan", 1, &Demo::LaserCallback, this);
  goals_pose_sub_ = nh_.subscribe("goals", 50, &Demo::GoalsCallback, this);
  goal_pose_sub_ =
      nh_.subscribe("/move_base_simple/goal", 50, &Demo::GoalCallback, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  start_navi_sub_ =
      nh_.subscribe("start_nav", 50, &Demo::StartNaviCallback, this);
  load_map_sub_ =
      nh_.subscribe("load_roadmap", 50, &Demo::LoadRoadmapCallback, this);

  cancel_sub_ =
      nh_.subscribe("cancel_navi", 50, &Demo::CancelNaviCallback, this);
}

bool Demo::InitPlanner() {
  bool flag = ModuleManager::GetInstance()->Init(config_file_path_);
  return flag;
}

void Demo::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  const auto x = msg->pose.pose.position.x;
  const auto y = msg->pose.pose.position.y;
  const auto v_x = msg->twist.twist.linear.x;
  const auto v_y = msg->twist.twist.linear.y;
  const auto angular_vel = msg->twist.twist.angular.z;
  const auto yaw = NormalizeAngleRad(
      2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  VehiclePose cur_pose;
  cur_pose.SetX(x);
  cur_pose.SetY(y);
  cur_pose.SetYaw(yaw);
  cur_pose.SetVelX(v_x);
  cur_pose.SetVelY(v_y);
  cur_pose.SetAngularVel(angular_vel);
  ModuleManager::GetInstance()->GetRuntimeManager()->UpdateVehiclePose(
      cur_pose);
}
void Demo::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  LaserScan laser_scan;
  laser_scan.time_stemp = GetTimeNowDouble();
  laser_scan.time_increment = msg->time_increment;
  laser_scan.angle_increment = msg->angle_increment;
  laser_scan.angle_max = msg->angle_max;
  laser_scan.angle_min = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    laser_scan.ranges.emplace_back(msg->ranges[i]);
    laser_scan.intensities.emplace_back(msg->intensities[i]);
  }
  ModuleManager::GetInstance()->GetRuntimeManager()->UpdateScan(laser_scan);
}

void Demo::GoalsCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {}
void Demo::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  std::cout << "in GoalCallback " << std::endl;
  const auto x = msg->pose.position.x;
  const auto y = msg->pose.position.y;
  auto global_map =
      ModuleManager::GetInstance()->GetMapManager()->GetGlobalMap();
  Vec2d pt(x, y);
  std::cout << "pt : " << pt << "is " << global_map->IsOccupied(pt)
            << std::endl;
}

void Demo::LoadRoadmapCallback(const std_msgs::Empty::ConstPtr &msg) {
  std::string roadmap_path;

  pnh_.param<std::string>("roadmap_path", roadmap_path, "");
  std::ifstream ifs(roadmap_path.data());  // open file example.json
  Json::Value json_roadmap;
  Json::Reader reader;
  std::cout << "roadmap_path is " << roadmap_path << std::endl;
  if (!reader.parse(ifs, json_roadmap)) {
    std::cout << "failed to load config file " << std::endl;
    return;
  }

  BezierSegments bezier_segs;
  for (auto &json_lane : json_roadmap) {
    Vec2d p0, p1, p2, p3;
    p0.x() = json_lane["start"]["x"].asDouble();
    p0.y() = json_lane["start"]["y"].asDouble();
    p1.x() = json_lane["p1"]["x"].asDouble();
    p1.y() = json_lane["p1"]["y"].asDouble();
    p2.x() = json_lane["p2"]["x"].asDouble();
    p2.y() = json_lane["p2"]["y"].asDouble();
    p3.x() = json_lane["end"]["x"].asDouble();
    p3.y() = json_lane["end"]["y"].asDouble();
    BezierSegment bezier_seg(p0, p1, p2, p3);
    bezier_seg.SetId(json_lane["id"].asInt());
    bezier_seg.SetSpeed(json_lane["speed"].asDouble());
    bezier_seg.SetWidth(json_lane["width"].asDouble());
    bezier_seg.SetStartId(json_lane["start"]["id"].asInt());
    bezier_seg.SetEndId(json_lane["end"]["id"].asInt());
    bezier_seg.SetLocalPlanFlag(json_lane["local_plan"].asBool());
    bezier_segs.emplace_back(bezier_seg);
  }
  bezier_segments_ = bezier_segs;
  b_get_bezier_segments_ = true;
}

void Demo::CancelNaviCallback(const std_msgs::Empty::ConstPtr &msg) {
  std::cout << "cancel navigation " << std::endl;
}
void Demo::StartNaviCallback(const std_msgs::Empty::ConstPtr &msg) {
  std::cout << "start navigation " << std::endl;
}

void Demo::VisTimer() {
  //  显示局部栅格地图
  if (ModuleManager::GetInstance()->GetMapManager()->HaveLocalMap()) {
    const auto local_gird_map =
        ModuleManager::GetInstance()->GetMapManager()->GetLocalMap();
    visualizer_.LocalGridMapVis(
        local_gird_map->GetOrigin(), local_gird_map->GetDim(),
        local_gird_map->GetData(), local_gird_map->GetRes());
  }
  // 显示全局栅格地图
  if (ModuleManager::GetInstance()->GetMapManager()->HaveGlobalMap()) {
    const auto global_gird_map =
        ModuleManager::GetInstance()->GetMapManager()->GetGlobalMap();

    visualizer_.GlobalGridMapVis(
        global_gird_map->GetOrigin(), global_gird_map->GetDim(),
        global_gird_map->GetData(), global_gird_map->GetRes());
  }

  const PointCloud3d transformed_pcd =
      ModuleManager::GetInstance()->GetMapManager()->GetTransformedPointcloud();
  visualizer_.TransformedPcdVis(transformed_pcd);

  visualizer_.SafetyBoundingBoxesVis(
      ModuleManager::GetInstance()->GetSafetyManager()->GetBoundingBoxs());
  if (b_get_bezier_segments_) {
    Path2d bezier_path;
    Points2d control_points;
    bezier_segments_.GetPath(bezier_path);
    bezier_segments_.GetAllControlPoints(control_points);
    visualizer_.BezierSegmentsVis(bezier_path, control_points);
  }
}
