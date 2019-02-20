if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
endif()

add_executable(hoogtelijnen hoogtelijnen.cpp)
target_link_libraries( hoogtelijnen
  geoflow-gui
  Boost::program_options
  gdal_nodes
  cgal_nodes
)
set_target_properties(
  hoogtelijnen
  PROPERTIES CXX_STANDARD 17
)

add_executable(footprint-simp footprint-simp.cpp)
target_link_libraries( footprint-simp 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
  stepedge_nodes
  gdal_nodes
  las_nodes
  cgal_nodes
)
set_target_properties(
  footprint-simp
  PROPERTIES CXX_STANDARD 17
)

add_executable(footprint-simp-cli footprint-simp-cli.cpp)
target_link_libraries( footprint-simp-cli 
  Boost::program_options
  ${GDAL_LIBRARY}
  gdal_nodes
  cgal_nodes
)
set_target_properties(
  footprint-simp-cli
  PROPERTIES CXX_STANDARD 17
)

add_executable(lod13 lod13.cpp)
target_link_libraries( lod13 
  geoflow-gui
  stepedge_nodes
  cgal_nodes
  gdal_nodes
  Boost::program_options
  LASlib
)
add_executable(lod10 lod10.cpp)
target_link_libraries( lod10 
  # geoflow-gui
  stepedge_nodes
  gdal_nodes
  Boost::program_options
  LASlib
)
set_target_properties(
  lod13 lod10
  PROPERTIES CXX_STANDARD 17
)