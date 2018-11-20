## Building
Requires compiler with c++17 support (see https://en.cppreference.com/w/cpp/compiler_support).

External dependencies:
1. CGAL [tested with version 4.12/4.13]
1. GDAL [requires version 2.3+]
1. Boost [tested with version 1.68]

Windows 10 instructions on how to install these dependencies as well as cmake (the build system) can be found on (this wiki)[https://github.com/tudelft3d/3dfier/wiki/Building-on-Windows-10], see sections 1 and 2.

The build system will try to automatically detect these dependencies and build only the geoflow nodes whose dependencies have been found.

To build checkout the submodules and run cmake/make:
```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -G <generator>
```

The `-G <generator>` flag can be ommitted on linux and macOS systems. For windows I recommend Microsoft Visual Studio 2017 and then you should use `-G "Visual Studio 15 2017 Win64"`.
