/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 21:49:48
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-04 22:18:38
 */
#include "geojson_graph_file_loader.h"

#include <iostream>

namespace minco_local_planner::route {

bool GeoJsonGraphFileLoader::LoadGraphFromFile(Graph& graph,
                                               GraphToIDMap& graph_to_id_map,
                                               const std::string& filepath) {
  if (!fs::IsFileExist(filepath)) {
    return false;
  }
  Json json_graph;

  try {
    json_graph = Load(filepath);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}
}  // namespace minco_local_planner::route