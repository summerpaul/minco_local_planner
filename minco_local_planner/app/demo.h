/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 09:52:15
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-30 11:05:20
 */
#include <stdint.h>

#ifndef __DEMO_H__
#define __DEMO_H__

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>

#include "segments/bezier_segment.h"
#include "visualizer/visualizer.h"

using namespace minco_local_planner::segments;
class Demo {
 public:
  explicit Demo(const std::string config_file_path);

 public:
  bool Init();
  void Run();

 private:
  bool InitPlanner();
  void InitRos();
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  void GoalsCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void LoadRoadmapCallback(const std_msgs::Empty::ConstPtr &msg);
  void StartNaviCallback(const std_msgs::Empty::ConstPtr &msg);
  void CancelNaviCallback(const std_msgs::Empty::ConstPtr &msg);


  void VisTimer();

 private:
  ros::NodeHandle nh_, pnh_;

  std::string config_file_path_;

  ros::Subscriber odom_sub_;        // 定位车辆的实时位置
  ros::Subscriber laser_scan_sub_;  // 定位激光雷达
  ros::Subscriber goals_pose_sub_;  // 订阅多目标点，生成Bezier曲线段
  ros::Subscriber goal_pose_sub_;   // 目标点订阅
  ros::Subscriber cancel_sub_;
  ros::Publisher cmd_pub_;
  ros::Subscriber load_map_sub_;
  ros::Subscriber start_navi_sub_;
  ros::Timer vis_timer_;

  visualizer::Visualizer visualizer_;

  BezierSegments bezier_segments_;
  bool b_get_bezier_segments_ = false;
};

#endif /* __DEMO_H__ */
