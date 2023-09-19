/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 21:44:23
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 13:17:44
 */
#include <stdint.h>

#ifndef __GRAPH_FILE_LOADER_H__
#define __GRAPH_FILE_LOADER_H__

#include <memory>
#include <string>

#include "route/types.h"
namespace minco_local_planner::route {

class GraphFileLoader {
 public:
  using Ptr = std::shared_ptr<GraphFileLoader>;
  GraphFileLoader() = default;
  virtual ~GraphFileLoader() = default;
  virtual bool LoadGraphFromFile(Graph& graph, GraphToIDMap& graph_to_id_map,
                                 const std::string& filepath) = 0;
};
}  // namespace minco_local_planner::route
#endif /* __GRAPH_FILE_LOADER_H__ */
