
add_library(cgal_nodes STATIC
  cgal_nodes.cpp
  isolines.cpp
  linesimp.cpp
  tinsimp.cpp
)
target_link_libraries( cgal_nodes
  geoflow-core
  ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} 
  ${LASlib}
)
set_target_properties(
  cgal_nodes
  PROPERTIES CXX_STANDARD 17
)
