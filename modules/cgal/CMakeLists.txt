cmake_minimum_required(VERSION 3.14)
project (processing-cgal VERSION 0.0.1)

add_definitions(-DGF_PLUGIN_NAME=\"${CMAKE_PROJECT_NAME}\")

find_package(geoflow REQUIRED)

# CGAL
find_package( CGAL QUIET COMPONENTS )
include_directories(${CGAL_INCLUDE_DIR})

find_package(LASlib)

add_library(cgal_nodes MODULE
  cgal_nodes.cpp
  isolines.cpp
  linesimp.cpp
  tinsimp.cpp
  plugin.cpp
)
target_link_libraries( cgal_nodes
  geoflow-core
  ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} 
  LASlib
)
set_target_properties(
  cgal_nodes
  PROPERTIES CXX_STANDARD 17
)