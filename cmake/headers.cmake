cmake_minimum_required(VERSION 3.13)

# find include/jps -type f -name '*.h' | sort
target_sources(mylib PUBLIC
include/scanner.h
include/solver.h
include/Log.h
include/Ray.h
include/rjps.h
)
