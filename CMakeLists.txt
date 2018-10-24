cmake_minimum_required (VERSION 3.10)

if(APPLE)
  set(CMAKE_C_COMPILER "/usr/local/opt/llvm/bin/clang")
  set(CMAKE_CXX_COMPILER "/usr/local/opt/llvm/bin/clang++")
  set(CMAKE_EXE_LINKER_FLAGS -Wl)
  set(CMAKE_MACOSX_RPATH 1)
  set(CMAKE_INSTALL_RPATH "/usr/local/opt/llvm/lib")
  link_directories("/usr/local/opt/llvm/lib")
endif()

project( edge_detect )


# Boost
find_package( Boost REQUIRED COMPONENTS program_options)
if ( NOT Boost_FOUND )
  message(SEND_ERROR "edge_detect requires the Boost library")
  return()  
endif()
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# GDAL
find_package( GDAL )
if ( NOT GDAL_FOUND )
  message(SEND_ERROR "edge_detect requires the GDAL library")
endif()

# CGAL
find_package( CGAL QUIET COMPONENTS  )
if ( NOT CGAL_FOUND )
  message(SEND_ERROR "edge_detect requires the CGAL library")
  return()  
endif()

# LASlib
add_subdirectory(thirdparty/LAStools/LASlib/src)

# include helper file
include( ${CGAL_USE_FILE} )

if (MSVC)
  add_definitions(-DNOMINMAX)
endif()

add_definitions(
  -DGLM_FORCE_CTOR_INIT # FIXME: get rid of this (by doing proper initialisation of glm objects, especially the quaternions)
)
include_directories( src thirdparty ${CGAL_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${GDAL_INCLUDE_DIR} thirdparty/LAStools/LASlib/inc thirdparty/LAStools/LASzip/inc thirdparty/LAStools/LASzip/src thirdparty/geoflow/thirdparty/glm thirdparty/ptinpoly)

add_library(point_edge_lib STATIC src/point_edge.cpp src/region_growing.cpp)
target_link_libraries(point_edge_lib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} LASlib)
set_target_properties(
  point_edge_lib
  PROPERTIES CXX_STANDARD 17
)

add_library(ptinpoly_lib STATIC thirdparty/ptinpoly/ptinpoly.c)
set_target_properties(
  ptinpoly_lib
  PROPERTIES C_STANDARD 11
)
# Creating entries for target

add_subdirectory(thirdparty/geoflow)

add_subdirectory(point-based)
add_subdirectory(raster-based)


add_subdirectory(viewer)
add_subdirectory(gf_viewer)
add_subdirectory(point_distance)
add_subdirectory(rws-noise)
