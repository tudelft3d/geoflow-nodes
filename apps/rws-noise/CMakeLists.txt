if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise /Zi /DEBUG")
endif()

set (geoflowlib geoflow-core)
if(${GF_BUILD_GUI})
  set (geoflowlib geoflow-gui)
endif()

add_executable(hoogtelijnen hoogtelijnen.cpp)
target_link_libraries( hoogtelijnen
  ${geoflowlib}
  Boost::program_options
  general_nodes
  gdal_nodes
  cgal_nodes
  las_nodes
)
set_target_properties(
  hoogtelijnen
  PROPERTIES CXX_STANDARD 17
)

add_executable(lod13 lod13.cpp)
target_link_libraries( lod13 
  ${geoflowlib}
  stepedge_nodes
  cgal_nodes
  gdal_nodes
  Boost::program_options
  LASlib
)
add_executable(lod10 lod10.cpp)
target_link_libraries( lod10 
  ${geoflowlib}
  stepedge_nodes
  gdal_nodes
  Boost::program_options
  LASlib
)
set_target_properties(
  lod13 lod10
  PROPERTIES CXX_STANDARD 17
)