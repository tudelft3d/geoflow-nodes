## Building
Requires compiler with c++17 support (see https://en.cppreference.com/w/cpp/compiler_support).

External dependencies:
1. CGAL [tested with version 4.12/4.13]
1. GDAL [requires version 2.3+]
1. Boost [tested with version 1.68]


To build checkout the submodules and run cmake/make:
```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```
 
