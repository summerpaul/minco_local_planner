/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 21:48:49
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 17:13:55
 */
#include <stdint.h>

#ifndef __GEOJSON_GRAPH_FILE_LOADER_H__
#define __GEOJSON_GRAPH_FILE_LOADER_H__
#include "basis/logger.h"
#include "exceptions/route_exceptions.h"
#include "graph_file_loader.h"
#include "utils/fs.hpp"
#include "utils/json.hpp"
namespace minco_local_planner::route {
using namespace utils;
using namespace exceptions;
class GeoJsonGraphFileLoader : public GraphFileLoader {
 public:
  GeoJsonGraphFileLoader() = default;
  virtual ~GeoJsonGraphFileLoader();
  bool LoadGraphFromFile(Graph& graph, GraphToIDMap& graph_to_id_map,
                         const std::string& filepath) override;

 protected:
  //  获取路网中的点与段
  void GetGraphElements(const Json& features, std::vector<Json>& nodes,
                        std::vector<Json>& edges);

  void AddNodesToGraph(Graph& graph, GraphToIDMap& graph_to_id_map,
                       std::vector<Json>& nodes);
  void AddEdgesToGraph(Graph& graph, GraphToIDMap& graph_to_id_map,
                       std::vector<Json>& edges);

  Coordinates ConvertCoordinatesFromJson(const Json& node);
  Metadata ConvertMetaDataFromJson(const Json& properties,
                                   const std::string& key = "metadata");
  Operation ConvertOperationFromJson(const Json& json_operation);
  Operations ConvertOperationsFromJson(const Json& properties);
  EdgeCost ConvertEdgeCostFromJson(const Json& properties);
};
}  // namespace minco_local_planner::route

#endif /* __GEOJSON_GRAPH_FILE_LOADER_H__ */
