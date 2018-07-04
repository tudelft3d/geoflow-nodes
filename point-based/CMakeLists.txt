add_executable(point_edge cli.cpp point_edge.cpp region_growing.cpp)
target_link_libraries( point_edge ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GDAL_LIBRARY} /usr/local/lib/liblas.dylib )

add_executable(rg_test region_growing.cpp test_regiongrower.cpp)
target_link_libraries( rg_test ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})