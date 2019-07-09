cmake_minimum_required(VERSION 3.14)
project (io-gdal VERSION 0.0.1)

add_definitions(-DGF_PLUGIN_NAME=\"${CMAKE_PROJECT_NAME}\")

find_package(geoflow REQUIRED)

# GDAL
find_package( GDAL )
include_directories(${GDAL_INCLUDE_DIR})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
# GEOS
find_package( GEOS )
if (GEOS_FOUND)
  include_directories(${GEOS_INCLUDE_DIR})
endif()

add_library(gfp_gdal MODULE
  gdal_nodes.cpp
  plugin.cpp
)
target_link_libraries( gfp_gdal
  geoflow-core
  ${GDAL_LIBRARY}
  ${GEOS_LIBRARY}
)
set_target_properties(
  gfp_gdal
  PROPERTIES CXX_STANDARD 17
)

install(TARGETS gfp_gdal DESTINATION ${geoflow_PLUGIN_FOLDER})