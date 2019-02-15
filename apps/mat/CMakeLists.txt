add_executable(mat main.cpp)
target_link_libraries( mat  
  geoflow-gui
  masb_nodes
  las_nodes
  cgal_nodes
)

set_target_properties(
  mat
  PROPERTIES CXX_STANDARD 17
)