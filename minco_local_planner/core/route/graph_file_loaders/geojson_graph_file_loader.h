/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 21:48:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-04 22:15:42
 */
#include <stdint.h>

#ifndef __GEOJSON_GRAPH_FILE_LOADER_H__
#define __GEOJSON_GRAPH_FILE_LOADER_H__
#include "graph_file_loader.h"
#include "utils/fs.hpp"
#include "utils/json.hpp"
namespace minco_local_planner::route {
using namespace utils;
class GeoJsonGraphFileLoader : public GraphFileLoader {
 public:
  GeoJsonGraphFileLoader() = default;
  ~GeoJsonGraphFileLoader() = default;
  bool LoadGraphFromFile(Graph& graph, GraphToIDMap& graph_to_id_map,
                         const std::string& filepath) override;

 protected:
};
}  // namespace minco_local_planner::route

#endif /* __GEOJSON_GRAPH_FILE_LOADER_H__ */
