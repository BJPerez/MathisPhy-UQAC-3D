cmake_minimum_required(VERSION 3.10)

add_subdirectory(glad)
add_subdirectory(glfw)
add_subdirectory(glm)
add_library(glm::glm ALIAS glm)