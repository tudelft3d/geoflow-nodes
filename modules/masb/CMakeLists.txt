add_library(masb_nodes STATIC
  masb_nodes.cpp
)
target_link_libraries( masb_nodes
  geoflow-core
  masbcpp
)
set_target_properties(
  masb_nodes
  PROPERTIES CXX_STANDARD 17
)