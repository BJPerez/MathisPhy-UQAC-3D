cmake_minimum_required(VERSION 3.10)

file(GLOB PHYSICSLIB_SOURCES
       src/*.cpp
	   src/collisions/*.cpp
       src/forceGenerator/*.cpp
	   src/math/*.cpp)

file(GLOB PHYSICSLIB_HEADERS
       include/*.hpp
	   include/collisions/*.hpp
       include/forceGenerator/*.hpp
	   include/math/*.hpp)

add_library(physicslib ${PHYSICSLIB_SOURCES} ${PHYSICSLIB_HEADERS})
target_include_directories(physicslib SYSTEM INTERFACE include)
target_include_directories(physicslib PRIVATE include)