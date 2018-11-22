if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
endif()

add_library(stepedge_nodes STATIC
  stepedge_nodes.cpp
  point_edge.cpp
  region_growing_plane.cpp
  region_growing.cpp
)
target_link_libraries( stepedge_nodes
  flowchart-core
  ptinpoly
)
set_target_properties(
  stepedge_nodes
  PROPERTIES CXX_STANDARD 17
)