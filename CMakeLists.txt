set(CMAKE_C_COMPILER "/usr/local/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/usr/local/opt/llvm/bin/clang++")

project( edge_detect )

# add_definitions(-std=c++17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++17> $<$<COMPILE_LANGUAGE:C>:-std=c99>)

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

#set(CMAKE_CXX_FLAGS "-O2")
 
if ( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )  
endif()

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

#find_package(PROJ)

# include helper file
include( ${CGAL_USE_FILE} )

include_directories( src thirdparty ${CGAL_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${GDAL_INCLUDE_DIR} /usr/local/include/LASlib /usr/local/include/LASzip)

add_library(point_edge_lib STATIC src/point_edge.cpp src/region_growing.cpp)
target_link_libraries(point_edge_lib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} /usr/local/lib/LASlib/liblas.a)
# Creating entries for target

add_subdirectory(thirdparty/geoflow)

add_subdirectory(point-based)
add_subdirectory(raster-based)


add_subdirectory(viewer)
add_subdirectory(gf_viewer)
add_subdirectory(point_distance)
# add_subdirectory(test)
