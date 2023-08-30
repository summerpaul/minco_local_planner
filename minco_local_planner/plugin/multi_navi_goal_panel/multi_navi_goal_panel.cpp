/**
 * @Author: Yunkai Xia
 * @Date:   2023-01-06 13:35:19
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-05-29 17:27:04
 */
#include "multi_navi_goal_panel.h"

#include <QtWidgets/qheaderview.h>
#include <ros/console.h>

#include <QDebug>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <QtWidgets/QTableWidget>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace navi_multi_goals_pub_rviz_plugin {

MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
    : rviz::Panel(parent), nh_(), maxNumGoal_(1) {
  initRos();

  QVBoxLayout *root_layout = new QVBoxLayout;
  // create a panel about "maxNumGoal"
  // 设定最大目标点数量
  QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
  maxNumGoal_layout->addWidget(new QLabel("目标最大数量"));
  output_maxNumGoal_editor_ = new QLineEdit;
  maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
  output_maxNumGoal_button_ = new QPushButton("确定");
  maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
  root_layout->addLayout(maxNumGoal_layout);

  // 设置路网名称
  QHBoxLayout *roadmapNameSet_layout = new QHBoxLayout;
  roadmapNameSet_layout->addWidget(new QLabel("路网名"));
  output_roadmap_name_editor_ = new QLineEdit;
  roadmapNameSet_layout->addWidget(output_roadmap_name_editor_);
  output_roadmapName_button_ = new QPushButton("设置路网");
  roadmapNameSet_layout->addWidget(output_roadmapName_button_);

  load_roadmapName_button_ = new QPushButton("加载路网");
  roadmapNameSet_layout->addWidget(load_roadmapName_button_);
  root_layout->addLayout(roadmapNameSet_layout);

  //
  QHBoxLayout *cycleTimesSet_layout = new QHBoxLayout;
  output_cycle_times_editor_ = new QLineEdit;
  cycleTimesSet_layout->addWidget(new QLabel("循环次数"));
  cycleTimesSet_layout->addWidget(output_cycle_times_editor_);
  out_set_cycle_times_button_ = new QPushButton("确定");
  root_layout->addLayout(cycleTimesSet_layout);

  cycle_checkbox_ = new QCheckBox("循环");
  root_layout->addWidget(cycle_checkbox_);
  // creat a QTable to contain the poseArray
  // 显示关键点的table
  poseArray_table_ = new QTableWidget;
  initPoseTable();
  root_layout->addWidget(poseArray_table_);
  laneArray_table_ = new QTableWidget;

  // creat a manipulate layout
  QHBoxLayout *manipulate_layout = new QHBoxLayout;
  output_reset_button_ = new QPushButton("重置");
  manipulate_layout->addWidget(output_reset_button_);
  output_cancel_button_ = new QPushButton("取消");
  manipulate_layout->addWidget(output_cancel_button_);

  out_draw_roadmap_button_ = new QPushButton("绘制路网");
  manipulate_layout->addWidget(out_draw_roadmap_button_);
  output_startNavi_button_ = new QPushButton("开始导航!");
  manipulate_layout->addWidget(output_startNavi_button_);
  root_layout->addLayout(manipulate_layout);

  setLayout(root_layout);
  // set a Qtimer to start a spin for subscriptions
  QTimer *output_timer = new QTimer(this);
  output_timer->start(200);

  // 设置信号与槽的连接
  connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
          SLOT(updateMaxNumGoal()));
  connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
          SLOT(updatePoseTable()));
  connect(output_roadmapName_button_, SIGNAL(clicked()), this,
          SLOT(updateRoadMapName()));
  connect(out_set_cycle_times_button_, SIGNAL(clicked()), this,
          SLOT(setCycleTimes()));
  connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
  connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
  connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
  connect(out_draw_roadmap_button_, SIGNAL(clicked()), this,
          SLOT(drawRoadMap()));
  connect(load_roadmapName_button_, SIGNAL(clicked()), this,
          SLOT(loadRoadMap()));
  connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));
}

void MultiNaviGoalsPanel::initRos() {
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 1, &MultiNaviGoalsPanel::goalCntCB, this);

  goals_pub_ = nh_.advertise<geometry_msgs::PoseArray>("goals", 1);

  cancel_pub_ = nh_.advertise<std_msgs::Empty>("cancel_navi", 1);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("way_points", 1);

  vehicle_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      "odom", 1, &MultiNaviGoalsPanel::vehiclePoseCB, this);

  roadmap_name_pub_ = nh_.advertise<std_msgs::String>("roadmap_name", 1);

  start_nav_pub_ = nh_.advertise<std_msgs::Empty>("start_nav", 1);
  load_roadmap_pub_ = nh_.advertise<std_msgs::Empty>("load_roadmap", 1);

  cycle_times_pub_ = nh_.advertise<std_msgs::Int16>("cycle_times", 1);
}

// 更新maxNumGoal命名
void MultiNaviGoalsPanel::updateMaxNumGoal() {
  setMaxNumGoal(output_maxNumGoal_editor_->text());
}

// set up the maximum number of goals
void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal) {
  // 检查maxNumGoal是否发生改变.
  if (new_maxNumGoal != output_maxNumGoal_) {
    output_maxNumGoal_ = new_maxNumGoal;

    // 如果命名为空，不发布任何信息
    if (output_maxNumGoal_ == "") {
      nh_.setParam("maxNumGoal_", 1);
      maxNumGoal_ = 1;
    } else {
      nh_.setParam("maxNumGoal_", output_maxNumGoal_.toInt());
      maxNumGoal_ = output_maxNumGoal_.toInt();
    }
    ROS_INFO_STREAM("maxNumGoal_ is " << maxNumGoal_);
    Q_EMIT configChanged();
  }
}

void MultiNaviGoalsPanel::updateRoadMapName() {
  setRoadMapName(output_roadmap_name_editor_->text());
}

void MultiNaviGoalsPanel::setRoadMapName(const QString &road_map_name) {
  std_msgs::String road_map_name_msg;
  road_map_name_msg.data = road_map_name.toStdString();
  roadmap_name_pub_.publish(road_map_name_msg);
  ROS_INFO_STREAM("set road_map_name " << road_map_name.toStdString());
}

// initialize the table of pose
// 初始化表头
void MultiNaviGoalsPanel::initPoseTable() {
  ROS_INFO("Initialize");
  curGoalIdx_ = 0, cycleCnt_ = 0;
  permit_ = false, cycle_ = false;
  // 清空表头
  poseArray_table_->clear();
  pose_array_.poses.clear();
  deleteMark();
  poseArray_table_->setRowCount(maxNumGoal_);
  poseArray_table_->setColumnCount(3);
  poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  poseArray_table_->horizontalHeader()->setSectionResizeMode(
      QHeaderView::Stretch);
  QStringList pose_header;
  pose_header << "x"
              << "y"
              << "yaw";
  poseArray_table_->setHorizontalHeaderLabels(pose_header);
  cycle_checkbox_->setCheckState(Qt::Unchecked);
}

// delete marks in the map
void MultiNaviGoalsPanel::deleteMark() {
  visualization_msgs::Marker marker_delete;
  marker_delete.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(marker_delete);
}

// update the table of pose
void MultiNaviGoalsPanel::updatePoseTable() {
  poseArray_table_->setRowCount(maxNumGoal_);
  //        pose_array_.poses.resize(maxNumGoal_);
  QStringList pose_header;
  pose_header << "x"
              << "y"
              << "yaw";
  poseArray_table_->setHorizontalHeaderLabels(pose_header);
  poseArray_table_->show();
}

// call back function for counting goals
// 接受目标点的回调函数
void MultiNaviGoalsPanel::goalCntCB(
    const geometry_msgs::PoseStamped::ConstPtr &pose) {
  if (static_cast<int>(pose_array_.poses.size()) < maxNumGoal_) {
    pose_array_.poses.emplace_back(pose->pose);
    pose_array_.header.frame_id = pose->header.frame_id;
    writePose(pose->pose);
    markPose(pose->pose, pose->header.frame_id);
  } else {
    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
  }
}

// write the poses into the table
void MultiNaviGoalsPanel::writePose(geometry_msgs::Pose pose) {
  poseArray_table_->setItem(
      pose_array_.poses.size() - 1, 0,
      new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
  poseArray_table_->setItem(
      pose_array_.poses.size() - 1, 1,
      new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
  poseArray_table_->setItem(
      pose_array_.poses.size() - 1, 2,
      new QTableWidgetItem(QString::number(
          tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
}

// when setting a Navi Goal, it will set a mark on the map
void MultiNaviGoalsPanel::markPose(const geometry_msgs::Pose &pose,
                                   const std::string &frame_id) {
  if (ros::ok()) {
    visualization_msgs::Marker arrow;
    visualization_msgs::Marker number;
    arrow.header.frame_id = number.header.frame_id = frame_id;
    arrow.ns = "navi_point_arrow";
    number.ns = "navi_point_number";
    arrow.action = number.action = visualization_msgs::Marker::ADD;
    arrow.type = visualization_msgs::Marker::ARROW;
    number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    arrow.pose = number.pose = pose;
    number.pose.position.z += 0.5;
    arrow.scale.x = 0.5;
    arrow.scale.y = 0.1;
    number.scale.z = 0.5;
    arrow.color.r = number.color.r = 1.0f;
    arrow.color.g = number.color.g = 0.98f;
    arrow.color.b = number.color.b = 1.0f;
    arrow.color.a = number.color.a = 1.0;
    arrow.id = number.id = pose_array_.poses.size();
    number.text = std::to_string(pose_array_.poses.size());
    marker_pub_.publish(arrow);
    marker_pub_.publish(number);
  }
}

// check whether it is in the cycling situation
void MultiNaviGoalsPanel::checkCycle() {
  cycle_ = cycle_checkbox_->isChecked();
  ROS_INFO_STREAM("set circle " << cycle_);
}

// start to navigate, and only command the first goal
void MultiNaviGoalsPanel::startNavi() {
  std_msgs::Empty msg;
  start_nav_pub_.publish(msg);
}

void MultiNaviGoalsPanel::loadRoadMap() {
  std_msgs::Empty msg;
  load_roadmap_pub_.publish(msg);
}

void MultiNaviGoalsPanel::drawRoadMap() { goals_pub_.publish(pose_array_); }

// complete the remaining goals
void MultiNaviGoalsPanel::completeNavi() {
  if (curGoalIdx_ < static_cast<int>(pose_array_.poses.size())) {
    geometry_msgs::PoseStamped goal;
    goal.header = pose_array_.header;
    goal.pose = pose_array_.poses.at(curGoalIdx_);
    goals_pub_.publish(goal);
    ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
    poseArray_table_->item(curGoalIdx_, 0)
        ->setBackgroundColor(QColor(255, 69, 0));
    poseArray_table_->item(curGoalIdx_, 1)
        ->setBackgroundColor(QColor(255, 69, 0));
    poseArray_table_->item(curGoalIdx_, 2)
        ->setBackgroundColor(QColor(255, 69, 0));
    curGoalIdx_ += 1;
    permit_ = true;
  } else {
    ROS_ERROR("All goals are completed");
    permit_ = false;
  }
}

// command the goals cyclically
void MultiNaviGoalsPanel::cycleNavi() {
  if (permit_) {
    geometry_msgs::PoseStamped goal;
    goal.header = pose_array_.header;
    goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
    goals_pub_.publish(goal);
    ROS_INFO("Navi to the Goal%lu, in the %dth cycle",
             curGoalIdx_ % pose_array_.poses.size() + 1, cycleCnt_ + 1);
    bool even = ((cycleCnt_ + 1) % 2 != 0);
    QColor color_table;
    if (even)
      color_table = QColor(255, 69, 0);
    else
      color_table = QColor(100, 149, 237);
    poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 0)
        ->setBackgroundColor(color_table);
    poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 1)
        ->setBackgroundColor(color_table);
    poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 2)
        ->setBackgroundColor(color_table);
    curGoalIdx_ += 1;
    cycleCnt_ = curGoalIdx_ / pose_array_.poses.size();
  }
}

void MultiNaviGoalsPanel::setCycleTimes() {
  std_msgs::Int16 msg;
  int cycle_times = output_cycle_times_editor_->text().toInt();
  msg.data = cycle_times;
  cycle_times_pub_.publish(msg);
}

// cancel the current command
void MultiNaviGoalsPanel::cancelNavi() {
  std_msgs::Empty msg;

  cancel_pub_.publish(msg);
  ROS_ERROR("Navigation have been canceled");
}

void MultiNaviGoalsPanel::addVehiclePose() {
  if (!is_add_vehicle_pose_) {
    is_add_vehicle_pose_ = true;
    ROS_INFO("add vehicle pose");
  }
}

void MultiNaviGoalsPanel::vehiclePoseCB(
    const nav_msgs::Odometry::ConstPtr &pose) {
  if (!is_add_vehicle_pose_) {
    return;
  }
  is_add_vehicle_pose_ = false;
  if (static_cast<int>(pose_array_.poses.size()) < maxNumGoal_) {
    pose_array_.poses.emplace_back(pose->pose.pose);
    pose_array_.header.frame_id = pose->header.frame_id;
    writePose(pose->pose.pose);
    markPose(pose->pose.pose, pose->header.frame_id);
  } else {
    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
  }
}


// spin for subscribing
void MultiNaviGoalsPanel::startSpin() {
  if (ros::ok()) {
    ros::spinOnce();
  }
}

}  // namespace navi_multi_goals_pub_rviz_plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel,
                       rviz::Panel)
