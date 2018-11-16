include_directories(
  ../thirdparty/json/include
  ../thirdparty/geoflow/src/flowchart
  ../thirdparty/geoflow/src/viewer
  ../thirdparty/geoflow/thirdparty/glad/include 
  ../thirdparty/geoflow/thirdparty/imgui
  ../thirdparty/geoflow/thirdparty/imgui/examples)

add_executable(hoogtelijnen hoogtelijnen.cpp ../src/tinsimp.cpp ../src/linesimp.cpp)
target_link_libraries( hoogtelijnen 
  point_edge_lib 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
  ptinpoly_lib
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
  ptinpoly_lib
)
if(NOT MSVC)
  target_link_libraries(footprint-simp c++fs)
endif()

set_target_properties(
  footprint-simp
  PROPERTIES CXX_STANDARD 17
)

add_executable(lod13 lod13.cpp ../src/tinsimp.cpp)
target_link_libraries( lod13 
  point_edge_lib 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
  ptinpoly_lib
)
if(NOT MSVC)
  target_link_libraries(lod13 c++fs)
endif()

set_target_properties(
  lod13
  PROPERTIES CXX_STANDARD 17
)

add_executable(lod13-cli lod13-cli.cpp ../src/tinsimp.cpp)
target_link_libraries( lod13-cli 
  point_edge_lib 
  flowchart-gui
  Boost::program_options
  ${GDAL_LIBRARY}
  ptinpoly_lib
)
if(NOT MSVC)
  target_link_libraries(lod13-cli c++fs)
endif()

set_target_properties(
  lod13-cli
  PROPERTIES CXX_STANDARD 17
)