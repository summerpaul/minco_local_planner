/**
 * @Author: Yunkai Xia
 * @Date:   2023-03-08 13:28:17
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-02 09:25:25
 */
#include <stdint.h>

#ifndef __MODE_CHANGE_PANEL_H__
#define __MODE_CHANGE_PANEL_H__
#include <QCheckBox>
#include <QPushButton>
#include <QTableWidget>
#include <ros/ros.h>
#include <rviz/panel.h>
namespace plugin {
class ModeChangePanel : public rviz::Panel {
  Q_OBJECT
public:
  explicit ModeChangePanel(QWidget *parent = 0);

protected Q_SLOTS:
  void manualModePub();
  void debugModePub();
  void autoModePub();
  void reloadConfigPub();

private:
  // 配置ros发布相关
  void initRosConnection();

protected:
  QPushButton *manual_mode_btn_; //手动模式按钮
  QPushButton *debug_mode_btn_;  //调试模式按钮
  QPushButton *auto_mode_btn_;
  QPushButton *reload_config_btn_;

  // ros相关
private:
  ros::NodeHandle nh_;            // ros句柄
  ros::Publisher mode_change_pub_; //发布车辆前进的任务，float类型
  ros::Publisher reload_config_pub_;
};
} // namespace plugin

#endif /* __MODE_CHANGE_PANEL_H__ */
