/**
 * @Author: Xia Yunkai
 * @Date:   2023-08-24 20:05:56
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-08-24 21:10:26
 */
#include <stdint.h>

#ifndef __SAFETY_MANAGER_H__
#define __SAFETY_MANAGER_H__

#include "basis/base_module.h"
#include "bounding_box.h"
#include "basis/trajectory.h"
namespace minco_local_planner::runtime_manager {

class SafetyManager : public BaseModule {};
}  // namespace minco_local_planner::runtime_manager

#endif /* __SAFETY_MANAGER_H__ */
