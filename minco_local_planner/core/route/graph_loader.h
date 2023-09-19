/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 16:45:38
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 16:49:06
 */
#include <stdint.h>

#ifndef __GRAPH_LOADER_H__
#define __GRAPH_LOADER_H__
#include "graph_file_loaders/graph_file_loader.h"
namespace minco_local_planner::route {

class GraphLoader {
 public:
  GraphLoader() = default;
  ~GraphLoader() = default;
  // 用于初始化参数
  bool Init();

  bool LoadGraphFromFile(Graph& graph, GraphToIDMap& idx_map,
                         std::string filepath = "");

 private:
  GraphFileLoader::Ptr graph_file_loader_;
};
}  // namespace minco_local_planner::route

#endif /* __GRAPH_LOADER_H__ */
