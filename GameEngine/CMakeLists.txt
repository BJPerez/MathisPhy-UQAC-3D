cmake_minimum_required(VERSION 3.10)

project(GameEngine VERSION 0.0.0
				DESCRIPTION "A little physical game engine made for a class."
				LANGUAGES CXX C)
				
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BASIC_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/output)
set(ARCHIVE_OUTPUT_DIRECTORY ${BASIC_OUTPUT_DIR}/lib)
set(RUNTIME_OUTPUT_DIRECTORY ${BASIC_OUTPUT_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${BASIC_OUTPUT_DIR}/lib)

add_subdirectory(external)
add_subdirectory(main)