
add_library(gdal_nodes STATIC
  gdal_nodes.cpp
)
target_link_libraries( gdal_nodes
  geoflow-core
  ${GDAL_LIBRARY}
  ${GEOS_LIBRARY}
)
set_target_properties(
  gdal_nodes
  PROPERTIES CXX_STANDARD 17
)