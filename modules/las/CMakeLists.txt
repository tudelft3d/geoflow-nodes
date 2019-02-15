
add_library(las_nodes STATIC
  las_nodes.cpp
)
target_link_libraries( las_nodes
  geoflow-core
  LASlib
)
set_target_properties(
  las_nodes
  PROPERTIES CXX_STANDARD 17
)