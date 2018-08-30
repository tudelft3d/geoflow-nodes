include_directories(
  ../thirdparty/geoflow/src/flowchart
  ../thirdparty/geoflow/src/viewer
  ../thirdparty/geoflow/thirdparty/glad/include 
  ../thirdparty/geoflow/thirdparty/imgui
  ../thirdparty/geoflow/thirdparty/imgui/examples)

add_executable(gf_viewer main.cpp)
target_link_libraries( gf_viewer 
  point_edge_lib 
  flowchart 
  Boost::program_options
)