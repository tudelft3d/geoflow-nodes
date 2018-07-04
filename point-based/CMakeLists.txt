add_executable(point_edge cli.cpp)
target_link_libraries( point_edge point_edge_lib /usr/local/lib/liblas.dylib )