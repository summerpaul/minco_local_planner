/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-27 22:12:28
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-27 22:24:59
 */
#include <stdint.h>

#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__
#include <ros/ros.h>

#include "vis_tools.h"
namespace visualizer {
class Visualizer {
 public:
 private:
  ros::NodeHandle nh_;
};
}  // namespace visualizer

#endif /* __VISUALIZER_H__ */
