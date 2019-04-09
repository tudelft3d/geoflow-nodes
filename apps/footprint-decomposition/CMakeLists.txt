if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise /Zi /DEBUG")
endif()

set (geoflowlib geoflow-core)
if(${GF_BUILD_GUI})
  set (geoflowlib geoflow-gui)
endif()

add_executable(footprint-decomposition footprint-decomposition.cpp)
target_link_libraries( footprint-decomposition
  ${geoflowlib}
  Boost::program_options
  general_nodes
  gdal_nodes
  cgal_nodes
)
set_target_properties(
  footprint-decomposition
  PROPERTIES CXX_STANDARD 17
)