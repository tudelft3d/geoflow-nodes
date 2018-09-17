include_directories(
  ../thirdparty/geoflow/src/flowchart
  ../thirdparty/geoflow/thirdparty/imgui)

add_executable(point_distance main.cpp)
target_link_libraries( point_distance 
  point_edge_lib
  flowchart-core
  Boost::program_options
  ${GDAL_LIBRARY}
)