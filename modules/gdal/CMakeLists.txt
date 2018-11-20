
add_library(gdal_nodes STATIC
  gdal_nodes.cpp
)
target_link_libraries( gdal_nodes
  flowchart-core
  ${GDAL_LIBRARY}
)
set_target_properties(
  gdal_nodes
  PROPERTIES CXX_STANDARD 17
)