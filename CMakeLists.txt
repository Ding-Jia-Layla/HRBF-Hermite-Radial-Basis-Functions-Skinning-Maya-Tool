cmake_minimum_required(VERSION 3.8)
project(add-buildHRBF-finalAdjust)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Set up include and lib/external paths
set(EXTERNAL "external")
include_directories("${EXTERNAL}/include")

add_subdirectory(src)
# mkdir build
# cmake -S. -Bbuild -G "Visual Studio 17 2022" 
# cmake --build build 