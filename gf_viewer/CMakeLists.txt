include_directories(
  ../thirdparty/geoflow/src
  ../thirdparty/geoflow/thirdparty/povi/src 
  ../thirdparty/geoflow/thirdparty/povi/glad/include 
  ../thirdparty/geoflow/thirdparty/povi/imgui)

add_executable(gf_viewer main.cpp 
  ../thirdparty/geoflow/src/nodes.cpp 
  ../thirdparty/geoflow/src/geoflow.cpp)
target_link_libraries( gf_viewer 
  point_edge_lib 
  povi_lib 
  Boost::program_options
)