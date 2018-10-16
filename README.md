## Building
Requires compiler with c++17 support (see https://en.cppreference.com/w/cpp/compiler_support). Can be installed on macOS with:
```
brew install llvm
```

External dependencies:
1. CGAL
1. GDAL
1. Boost 


To build checkout the submodules and run cmake/make:
```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```
 
