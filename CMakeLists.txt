
project( edge_detect )

add_definitions(-std=c++17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

set(CMAKE_CXX_FLAGS "-O2")
 
if ( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )  
endif()

# Boost
find_package( Boost )
if ( NOT Boost_FOUND )
  message(SEND_ERROR "3dfier requires the Boost library")
  return()  
endif()
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# GDAL
find_package( GDAL )
if ( NOT GDAL_FOUND )
  message(SEND_ERROR "3dfier requires the GDAL library")
endif()

# CGAL
find_package( CGAL QUIET COMPONENTS  )
if ( NOT CGAL_FOUND )
  message(SEND_ERROR "3dfier requires the CGAL library")
  return()  
endif()
set(CMAKE_BUILD_TYPE "RELEASE")

#find_package(PROJ)

# include helper file
include( ${CGAL_USE_FILE} )

include_directories( src thirdparty ${CGAL_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${GDAL_INCLUDE_DIR} /usr/local/include/LASlib /usr/local/include/LASzip)

# Creating entries for target
# FILE(GLOB SRC_FILES src/*.cpp)
add_executable(raster_edge src/raster_edge.cpp src/Raster.cpp thirdparty/lsd/lsd.c)
target_link_libraries( raster_edge ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GDAL_LIBRARY} /usr/local/lib/liblas.dylib )

add_executable(point_edge src/point_edge.cpp src/region_growing.cpp)
target_link_libraries( point_edge ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GDAL_LIBRARY} /usr/local/lib/liblas.dylib )

add_executable(rg_test src/region_growing.cpp src/test_regiongrower.cpp)
target_link_libraries( rg_test ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
