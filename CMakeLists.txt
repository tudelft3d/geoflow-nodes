cmake_minimum_required (VERSION 3.10)

# if(APPLE)
#   set(CMAKE_C_COMPILER "/usr/local/opt/llvm/bin/clang")
#   set(CMAKE_CXX_COMPILER "/usr/local/opt/llvm/bin/clang++")
#   set(CMAKE_EXE_LINKER_FLAGS -Wl)
#   set(CMAKE_MACOSX_RPATH 1)
#   set(CMAKE_INSTALL_RPATH "/usr/local/opt/llvm/lib")
#   link_directories("/usr/local/opt/llvm/lib")
# endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

project( geoflow_nodes )

if (MSVC)
  add_definitions(-DNOMINMAX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise /Zi /DEBUG")
endif()

# Boost
find_package( Boost REQUIRED COMPONENTS program_options)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

# GDAL
find_package( GDAL )
include_directories(${GDAL_INCLUDE_DIR})

# GEOS
find_package( GEOS )
if (GEOS_FOUND)
  include_directories(${GEOS_INCLUDE_DIR})
endif()

# CGAL
find_package( CGAL QUIET COMPONENTS )
include_directories(${CGAL_INCLUDE_DIR})

add_definitions(
  -DGLM_FORCE_CTOR_INIT # FIXME: get rid of this (by doing proper initialisation of glm objects, especially the quaternions)
)

# LASlib
add_subdirectory(thirdparty/LAStools/LASlib/src)
include_directories(
  thirdparty/LAStools/LASlib/inc
  thirdparty/LAStools/LASzip/inc
  thirdparty/LAStools/LASzip/src
)

# masbcpp
add_subdirectory(thirdparty/masbcpp)
include_directories(
  thirdparty/masbcpp/src
  thirdparty/masbcpp/thirdparty
  thirdparty/masbcpp/thirdparty/vrui
)

# ptinpoly
add_library(ptinpoly STATIC thirdparty/ptinpoly/ptinpoly.c)
set_target_properties(
  ptinpoly
  PROPERTIES C_STANDARD 11
)
include_directories(thirdparty/ptinpoly)

# geoflow
add_subdirectory(thirdparty/geoflow)
include_directories(
  thirdparty/geoflow/src/
  thirdparty/geoflow/thirdparty/glad/include 
  thirdparty/geoflow/thirdparty/imgui
  thirdparty/geoflow/thirdparty/imgui/examples
  thirdparty/geoflow/thirdparty/glm
  thirdparty/geoflow/thirdparty/osdialog
)
if(${GF_BUILD_GUI})
  add_definitions(-DGF_BUILD_GUI)
endif()
#earcut
include_directories(
  thirdparty/earcut
)

# MODULES

if (CGAL_FOUND)
  # include helper file
  include( ${CGAL_USE_FILE} )
  add_subdirectory(modules/cgal)
  include_directories(modules/cgal)
endif()

if (GDAL_FOUND AND Boost_FOUND AND GEOS_FOUND)
  # include helper file
  add_subdirectory(modules/gdal)
  include_directories(modules/gdal)
endif()

add_subdirectory(modules/las)
include_directories(modules/las)

add_subdirectory(modules/masb)
include_directories(modules/masb)

# add_subdirectory(modules/cityjson)
include_directories(modules/cityjson)

add_subdirectory(modules/general)
include_directories(modules/general)

if (CGAL_FOUND AND Boost_FOUND)
  add_subdirectory(modules/stepedge)
  include_directories(modules/stepedge)
endif()

# APPS

if (CGAL_FOUND)
  add_subdirectory(apps/gf_viewer)
endif()

if (CGAL_FOUND AND Boost_FOUND AND GDAL_FOUND AND GEOS_FOUND)
  add_subdirectory(apps/rws-noise)
endif()

add_subdirectory(apps/mat)

add_subdirectory(apps/cityjson)

# add_subdirectory(point-based)
# add_subdirectory(raster-based)
# add_subdirectory(viewer)
# add_subdirectory(point_distance)