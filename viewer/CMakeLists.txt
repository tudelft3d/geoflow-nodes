include_directories(../thirdparty/geoflow/thirdparty/povi/src ../thirdparty/geoflow/thirdparty/povi/glad/include ../thirdparty/geoflow/thirdparty/povi/imgui)

add_executable(viewer main.cpp)
target_link_libraries( viewer point_edge_lib povi_lib Boost::program_options)