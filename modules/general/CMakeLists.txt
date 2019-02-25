
add_library(general_nodes STATIC
  general_nodes.cpp
)
target_link_libraries( general_nodes
  geoflow-core
)
set_target_properties(
  general_nodes
  PROPERTIES CXX_STANDARD 17
)