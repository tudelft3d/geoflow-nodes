add_executable(gf_viewer main.cpp)
target_link_libraries( gf_viewer 
  flowchart-gui
  cgal_nodes
  gdal_nodes
  las_nodes
  stepedge_nodes
  masb_nodes
)
set_target_properties(
  gf_viewer
  PROPERTIES CXX_STANDARD 17
)