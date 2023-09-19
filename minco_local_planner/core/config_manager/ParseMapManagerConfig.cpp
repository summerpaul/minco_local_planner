/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-19 11:09:33
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-19 11:16:29
 */
#include <iostream>

using namespace std;

#include "config_manager.h"
namespace minco_local_planner::config_manager {

bool ConfigManager::ParseMapManagerConfig(const Json& map_manager_cfg_json) {
  if (!GetField(map_manager_cfg_json, "base_to_laser_x",
                map_manager_cfg_->base_to_laser_x)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "base_to_laser_y",
                map_manager_cfg_->base_to_laser_y)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "base_to_laser_yaw",
                map_manager_cfg_->base_to_laser_yaw)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "down_sampling_res",
                map_manager_cfg_->down_sampling_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "local_map_height",
                map_manager_cfg_->local_map_height)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "grid_map_inf_size",
                map_manager_cfg_->grid_map_inf_size)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "grid_map_res",
                map_manager_cfg_->grid_map_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "local_map_width",
                map_manager_cfg_->local_map_width)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "laser_max_range",
                map_manager_cfg_->laser_max_range)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "laser_min_range",
                map_manager_cfg_->laser_min_range)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "map_generate_time",
                map_manager_cfg_->map_generate_time)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "pcd_map_path",
                map_manager_cfg_->pcd_map_path)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "raycast_dis",
                map_manager_cfg_->raycast_dis)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "raycast_res",
                map_manager_cfg_->raycast_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "use_global_map",
                map_manager_cfg_->use_global_map)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "esdf_map_res",
                map_manager_cfg_->esdf_map_res)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "use_esdf_map",
                map_manager_cfg_->use_esdf_map)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "width_offset_scale",
                map_manager_cfg_->width_offset_scale)) {
    return false;
  }

  if (!GetField(map_manager_cfg_json, "height_offset_scale",
                map_manager_cfg_->height_offset_scale)) {
    return false;
  }

  return true;
}
}  // namespace minco_local_planner::config_manager