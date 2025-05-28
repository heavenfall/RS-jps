cmake_minimum_required(VERSION 3.13)

project(Owen-JPS
	VERSION 0.0.1
	LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include(cmake/submodule.cmake)
warthog_submodule(warthog-core) # repeat for each [module]
include(cmake/submodule.cmake)
warthog_submodule(warthog-jps)
