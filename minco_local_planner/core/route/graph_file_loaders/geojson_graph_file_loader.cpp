/**
 * @Author: Xia Yunkai
 * @Date:   2023-09-04 21:49:48
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 17:14:11
 */
#include "geojson_graph_file_loader.h"

#include <iostream>

namespace minco_local_planner::route {

GeoJsonGraphFileLoader::~GeoJsonGraphFileLoader() {}
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
  // json文件中的点与路段
  auto features = json_graph["features"];
  std::vector<Json> nodes, edges;

  GetGraphElements(features, nodes, edges);

  if (nodes.empty() || edges.empty()) {
    LOG_ERROR(
        "The graph is malformed. Is does not contain nodes or edges. Please "
        "check ");
    return false;
  }
  graph.resize(nodes.size());
  AddNodesToGraph(graph, graph_to_id_map, nodes);
  AddEdgesToGraph(graph, graph_to_id_map, edges);
  return true;
}

void GeoJsonGraphFileLoader::GetGraphElements(const Json& features,
                                              std::vector<Json>& nodes,
                                              std::vector<Json>& edges) {
  for (const auto& feature : features) {
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    } else if (feature["geometry"]["type"] == "MultiLineString") {
      edges.emplace_back(feature);
    }
  }
}

void GeoJsonGraphFileLoader::AddNodesToGraph(Graph& graph,
                                             GraphToIDMap& graph_to_id_map,
                                             std::vector<Json>& nodes) {
  int idx = 0;
  for (const auto& node : nodes) {
    const auto properties = node["properties"];
    graph[idx].nodeid = properties["id"];
    graph_to_id_map[graph[idx].nodeid] = idx;
    graph[idx].coords = ConvertCoordinatesFromJson(node);
    graph[idx].operations = ConvertOperationsFromJson(properties);
    graph[idx].metadata = ConvertMetaDataFromJson(properties);
    idx++;
  }
}
void GeoJsonGraphFileLoader::AddEdgesToGraph(Graph& graph,
                                             GraphToIDMap& graph_to_id_map,
                                             std::vector<Json>& edges) {
  for (const auto& edge : edges) {
    // Required data
    const auto properties = edge["properties"];
    unsigned int id = properties["id"];
    unsigned int start_id = properties["startid"];
    unsigned int end_id = properties["endid"];

    if (graph_to_id_map.find(start_id) == graph_to_id_map.end()) {
      LOG_ERROR("Start id {} does not exist for edge id {}", start_id, id);
      throw NoValidGraph("Start id does not exist");
    }

    if (graph_to_id_map.find(end_id) == graph_to_id_map.end()) {
      LOG_ERROR("End id {} does not exist for edge id {}", end_id, id);
      throw NoValidGraph("End id does not exist");
    }

    EdgeCost edge_cost = ConvertEdgeCostFromJson(properties);
    Operations operations = ConvertOperationsFromJson(properties);
    Metadata metadata = ConvertMetaDataFromJson(properties);

    graph[graph_to_id_map[start_id]].AddEdge(
        edge_cost, &graph[graph_to_id_map[end_id]], id, metadata, operations);
  }
}

Coordinates GeoJsonGraphFileLoader::ConvertCoordinatesFromJson(
    const Json& node) {
  Coordinates coords;
  const auto& properties = node["properties"];
  if (properties.contains("frame")) {
    coords.frame_id = properties["frame"];
  }

  const auto& coordinates = node["geometry"]["coordinates"];
  coords.x = coordinates[0];
  coords.y = coordinates[1];

  return coords;
}
Metadata GeoJsonGraphFileLoader::ConvertMetaDataFromJson(
    const Json& properties, const std::string& key) {
  Metadata metadata;
  if (!properties.contains(key)) {
    return metadata;
  }

  for (const auto& data : properties[key].items()) {
    if (data.value().is_object()) {
      Metadata new_metadata =
          ConvertMetaDataFromJson(properties[key], data.key());
      metadata.SetValue(data.key(), new_metadata);
      continue;
    }

    const auto setPrimitiveType = [&](const auto& value) -> std::any {
      if (value.is_number()) {
        if (value.is_number_unsigned()) {
          return static_cast<unsigned int>(value);
        } else if (value.is_number_integer()) {
          return static_cast<int>(value);
        } else {
          return static_cast<float>(value);
        }
      }

      if (value.is_boolean()) {
        return static_cast<bool>(value);
      }

      if (value.is_string()) {
        return static_cast<std::string>(value);
      }

      LOG_ERROR("Failed to convert the key: {} to a value", data.key());
      throw std::runtime_error("Failed to convert");
    };

    if (data.value().is_array()) {
      std::vector<std::any> array;
      for (const auto& el : data.value()) {
        auto value = setPrimitiveType(el);
        array.push_back(value);
      }
      metadata.SetValue(data.key(), array);
      continue;
    }

    auto value = setPrimitiveType(data.value());
    metadata.SetValue(data.key(), value);
  }

  return metadata;
}

Operation GeoJsonGraphFileLoader::ConvertOperationFromJson(
    const Json& json_operation) {
  Operation operation;
  json_operation.at("type").get_to(operation.type);
  Json trigger = json_operation.at("trigger");
  operation.trigger = trigger.get<OperationTrigger>();
  Metadata metadata = ConvertMetaDataFromJson(json_operation);
  operation.metadata = metadata;

  return operation;
}

Operations GeoJsonGraphFileLoader::ConvertOperationsFromJson(
    const Json& properties) {
  Operations operations;
  if (properties.contains("operations")) {
    for (const auto& json_operation : properties["operations"]) {
      operations.push_back(ConvertOperationFromJson(json_operation));
    }
  }
  return operations;
}

EdgeCost GeoJsonGraphFileLoader::ConvertEdgeCostFromJson(
    const Json& properties) {
  EdgeCost edge_cost;
  if (properties.contains("cost")) {
    edge_cost.cost = properties["cost"];
  }

  if (properties.contains("overridable")) {
    edge_cost.overridable = properties["overridable"];
  }
  return edge_cost;
}
}  // namespace minco_local_planner::route