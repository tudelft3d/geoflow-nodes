include_directories(
  ../thirdparty/json/include
  ../thirdparty/geoflow/src/flowchart
  ../thirdparty/geoflow/src/viewer
  ../thirdparty/geoflow/thirdparty/glad/include 
  ../thirdparty/geoflow/thirdparty/imgui
  ../thirdparty/geoflow/thirdparty/imgui/examples)

add_executable(gf_viewer main.cpp)
target_link_libraries( gf_viewer 
  point_edge_lib 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
)
if(NOT MSVC)
  target_link_libraries(gf_viewer c++fs)
endif()

set_target_properties(
  gf_viewer
  PROPERTIES CXX_STANDARD 17
)