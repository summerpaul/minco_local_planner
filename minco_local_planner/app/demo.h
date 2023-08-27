/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-25 09:52:15
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-25 10:10:17
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

 private:
  ros::NodeHandle nh_, pnh_;

  std::string config_file_path_;
};

#endif /* __DEMO_H__ */
