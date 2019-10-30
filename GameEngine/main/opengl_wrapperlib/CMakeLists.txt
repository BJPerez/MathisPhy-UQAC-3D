cmake_minimum_required(VERSION 3.10)

set(OPENGL_WRAPPER_SOURCES src/OpenGlWrapper.cpp src/shader.cpp)
set(OPENGL_WRAPPER_HEADERS include/OpenGlWrapper.hpp include/shader.hpp)

add_library(opengl_wrapperlib ${OPENGL_WRAPPER_SOURCES} ${OPENGL_WRAPPER_HEADERS})
target_link_libraries(opengl_wrapperlib PUBLIC glfw glad)
target_include_directories(opengl_wrapperlib SYSTEM INTERFACE include)
target_include_directories(opengl_wrapperlib PRIVATE include)