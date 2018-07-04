include_directories(../thirdparty/povi/src ../thirdparty/povi/glad/include ../thirdparty/povi/imgui)

add_executable(viewer main.cpp)
target_link_libraries( viewer point_edge_lib povi_lib )