if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
endif()

add_executable(hoogtelijnen hoogtelijnen.cpp)
target_link_libraries( hoogtelijnen 
  flowchart-gui
  gdal_nodes
  las_nodes
  cgal_nodes
)
set_target_properties(
  hoogtelijnen
  PROPERTIES CXX_STANDARD 17
)
add_executable(hoogtelijnen-cli hoogtelijnen-cli.cpp)
target_link_libraries( hoogtelijnen-cli
  flowchart-core
  Boost::program_options
  gdal_nodes
  cgal_nodes
)
set_target_properties(
  hoogtelijnen-cli
  PROPERTIES CXX_STANDARD 17
)

# add_executable(footprint-simp footprint-simp.cpp ../src/tinsimp.cpp)
# target_link_libraries( footprint-simp 
#   point_edge_lib 
#   flowchart-gui
#   Boost::program_options
#   ${GDAL_LIBRARY}
#   ptinpoly_lib
# )
# set_target_properties(
#   footprint-simp
#   PROPERTIES CXX_STANDARD 17
# )

add_executable(lod13 lod13.cpp)
target_link_libraries( lod13 
  flowchart-gui
  stepedge_nodes
  gdal_nodes
  las_nodes
)
set_target_properties(
  lod13
  PROPERTIES CXX_STANDARD 17
)

add_executable(lod13-cli lod13-cli.cpp)
target_link_libraries( lod13-cli 
  flowchart-core
  Boost::program_options
  stepedge_nodes
  gdal_nodes
  las_nodes
)
set_target_properties(
  lod13-cli
  PROPERTIES CXX_STANDARD 17
)