add_executable(rg_test region_growing.cpp test_regiongrower.cpp)
target_link_libraries( rg_test ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})