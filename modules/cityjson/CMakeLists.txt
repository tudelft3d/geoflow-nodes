
add_library(cityjson_nodes STATIC
  cityjson_nodes.cpp
)
target_link_libraries( cityjson_nodes
  geoflow-core
)
set_target_properties(
  cityjson_nodes
  PROPERTIES CXX_STANDARD 17
)