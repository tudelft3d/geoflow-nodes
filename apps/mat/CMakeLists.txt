include_directories(
  ../thirdparty/geoflow/src/flowchart
  ../thirdparty/geoflow/src/viewer
  # ../thirdparty/geoflow/thirdparty/glad/include 
  ../thirdparty/geoflow/thirdparty/imgui
  # ../thirdparty/geoflow/thirdparty/imgui/examples
  )

add_executable(mat main.cpp)
target_link_libraries( mat  
  flowchart-gui
  masb_nodes
  las_nodes
)

set_target_properties(
  mat
  PROPERTIES CXX_STANDARD 17
)