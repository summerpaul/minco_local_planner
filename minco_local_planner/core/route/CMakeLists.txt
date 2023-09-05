

set(route_src 
   graph_file_loaders/geojson_graph_file_loader.cpp
   edge_cost_functions/distance_scorer.cpp
   edge_scorer.cpp
   route_planner.cpp)


add_library(route  ${route_src})