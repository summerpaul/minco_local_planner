/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 16:45:44
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 16:52:03
 */
#include "graph_loader.h"

#include <iostream>
using namespace std;

namespace minco_local_planner::route {

bool GraphLoader::Init() { return true; }

bool GraphLoader::LoadGraphFromFile(Graph& graph, GraphToIDMap& graph_to_id_map,
                                    std::string filepath) {
  if (!graph_file_loader_->LoadGraphFromFile(graph, graph_to_id_map,
                                             filepath)) {
    return false;
  }
  return true;
}
}  // namespace minco_local_planner::route