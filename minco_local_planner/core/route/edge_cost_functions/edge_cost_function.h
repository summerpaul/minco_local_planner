/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 13:12:41
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 17:15:34
 */

#ifndef __EDGE_COST_FUNCTION_H__
#define __EDGE_COST_FUNCTION_H__
#include <memory>
#include <string>

#include "route/types.h"
namespace minco_local_planner::route {

class EdgeCostFunction {
 public:
  using Ptr = std::shared_ptr<EdgeCostFunction>;

  EdgeCostFunction() = default;

  virtual ~EdgeCostFunction() = default;

  virtual bool Init() = 0;

  virtual bool Score(const EdgePtr edge, float& cost) = 0;

  virtual std::string GetName() = 0;

  virtual void Prepare() {}
};
}  // namespace minco_local_planner::route

#endif /* __EDGE_COST_FUNCTION_H__ */
