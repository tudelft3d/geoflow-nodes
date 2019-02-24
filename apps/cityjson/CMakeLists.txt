include_directories(../../thirdparty/geoflow/thirdparty/json/include)
add_executable(cjv main.cpp)
target_link_libraries( cjv  
  geoflow-gui
)

set_target_properties(
  cjv
  PROPERTIES CXX_STANDARD 17
)