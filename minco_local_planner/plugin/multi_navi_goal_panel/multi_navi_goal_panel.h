
/**
 * @Author: Yunkai Xia
 * @Date:   2023-01-06 13:35:19
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 09:22:39
 */
#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <string>

#include <ros/console.h>
#include <ros/ros.h>

#include <rviz/panel.h>

#include <QCheckBox>
#include <QPushButton>
#include <QTableWidget>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>
namespace navi_multi_goals_pub_rviz_plugin
{

  class MultiNaviGoalsPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    explicit MultiNaviGoalsPanel(QWidget *parent = 0);

  public Q_SLOTS:

    void setMaxNumGoal(const QString &maxNumGoal);

    void writePose(geometry_msgs::Pose pose);
    void markPose(const geometry_msgs::Pose &pose,
                  const std::string &frame_id = "odom");
    void deleteMark();

    void setRoadMapName(const QString &road_map_name);

  protected Q_SLOTS:

    void updateMaxNumGoal(); // update max number of goal
    void initPoseTable();    // initialize the pose table

    void initLaneTable(); //  初始化段的表头

    void updatePoseTable(); // update the pose table
    void startNavi();       // start navigate for the first pose
    void loadRoadMap();
    void cancelNavi();
    void addVehiclePose();

    void setCycleTimes();

    void drawRoadMap();

    void updateRoadMapName();

    void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr
                       &pose); // goal count sub callback function
                       
    void vehiclePoseCB(const nav_msgs::Odometry::ConstPtr &pose);

    void checkCycle();

    void completeNavi(); // after the first pose, continue to navigate the rest of
                         // poses
    void cycleNavi();


    static void startSpin(); // spin for sub

    void initRos(); // 初始化ros相关
  protected:
    QLineEdit *output_maxNumGoal_editor_;
    QLineEdit *output_roadmap_name_editor_;
    QLineEdit *output_cycle_times_editor_;

    // 面板上的按钮
    QPushButton *output_maxNumGoal_button_; // 确定目标点的数量按钮
    QPushButton *output_roadmapName_button_;
    QPushButton *load_roadmapName_button_;
    QPushButton *output_reset_button_;       // 重置的按钮
    QPushButton *output_startNavi_button_;   // 启动导航的按钮
    QPushButton *output_cancel_button_;      // 取消的按钮
    QPushButton *out_draw_roadmap_button_; 
    QPushButton *out_set_cycle_times_button_;
    

    QTableWidget *poseArray_table_; // 显示关键点的table
    QTableWidget *laneArray_table_; // 显示路段的table
    QCheckBox *cycle_checkbox_;

    QString output_maxNumGoal_;
    QString output_roadMapName_;

    // The ROS node handle.
    ros::NodeHandle nh_;
    ros::Publisher goals_pub_, cancel_pub_, marker_pub_;
    ros::Publisher roadmap_name_pub_;
    ros::Publisher start_nav_pub_;
    ros::Publisher load_roadmap_pub_;
    ros::Publisher cycle_times_pub_;
    ros::Subscriber goal_sub_, status_sub_, vehicle_pose_sub_;

    int maxNumGoal_;
    int curGoalIdx_ = 0, cycleCnt_ = 0;
    bool permit_ = false, cycle_ = false, arrived_ = false;
    geometry_msgs::PoseArray pose_array_;

    bool is_add_vehicle_pose_{false};
  };

} // namespace navi_multi_goals_pub_rviz_plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
