cmake_minimum_required(VERSION 3.13)

project(RayScan-JPS
	VERSION 0.0.1
	LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include(cmake/submodule.cmake)
warthog_submodule(warthog-core)
include(cmake/submodule.cmake)
warthog_submodule(warthog-jps)

add_library(mylib src/scanner.cpp)
target_include_directories(mylib PUBLIC include)
include(cmake/headers.cmake)

add_executable(RayScan-jps main.cpp)	

add_subdirectory(src)

target_link_libraries(RayScan-jps PUBLIC mylib)
target_link_libraries(mylib PUBLIC warthog::libjps)
# add_subdirectory(include)