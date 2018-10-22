include_directories(
  ../thirdparty/json/include
  ../thirdparty/geoflow/src/flowchart
  ../thirdparty/geoflow/src/viewer
  ../thirdparty/geoflow/thirdparty/glad/include 
  ../thirdparty/geoflow/thirdparty/imgui
  ../thirdparty/geoflow/thirdparty/imgui/examples)

add_executable(hoogtelijnen hoogtelijnen.cpp ../src/tinsimp.cpp)
target_link_libraries( hoogtelijnen 
  point_edge_lib 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
)
if(NOT MSVC)
  target_link_libraries(hoogtelijnen c++fs)
endif()

set_target_properties(
  hoogtelijnen
  PROPERTIES CXX_STANDARD 17
)

add_executable(footprint-simp footprint-simp.cpp ../src/tinsimp.cpp)
target_link_libraries( footprint-simp 
  point_edge_lib 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
)
if(NOT MSVC)
  target_link_libraries(footprint-simp c++fs)
endif()

set_target_properties(
  footprint-simp
  PROPERTIES CXX_STANDARD 17
)