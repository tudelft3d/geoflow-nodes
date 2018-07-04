add_executable(point_edge cli.cpp)
target_link_libraries( point_edge point_edge_lib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GDAL_LIBRARY} /usr/local/lib/liblas.dylib )