cmake_minimum_required(VERSION 3.14)
project (misc VERSION 0.0.1)

add_definitions(-DGF_PLUGIN_NAME=\"${CMAKE_PROJECT_NAME}\")

find_package(geoflow REQUIRED)

add_library(gfp_misc MODULE
  general_nodes.cpp
  plugin.cpp
)
target_link_libraries( gfp_misc
  geoflow-core
)
set_target_properties(
  gfp_misc
  PROPERTIES CXX_STANDARD 17
)

install(TARGETS gfp_misc DESTINATION ${geoflow_PLUGIN_FOLDER})