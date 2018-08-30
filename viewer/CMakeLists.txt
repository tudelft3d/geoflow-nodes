include_directories(../thirdparty/geoflow/src/viewer ../thirdparty/geoflow/thirdparty/glad/include ../thirdparty/geoflow/thirdparty/imgui ../thirdparty/geoflow/thirdparty/imgui/examples)

add_executable(basic-viewer main.cpp)
target_link_libraries( basic-viewer point_edge_lib viewer Boost::program_options)