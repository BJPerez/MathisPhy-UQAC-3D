cmake_minimum_required(VERSION 3.10)

file(GLOB GAME_ENGINE_SOURCES
       src/*.cpp)

file(GLOB GAME_ENGINE_HEADERS
       include/*.hpp)

add_executable(GameEngine ${GAME_ENGINE_SOURCES} ${GAME_ENGINE_HEADERS})

target_link_libraries(GameEngine PUBLIC physicslib opengl_wrapperlib glm)

file(COPY ${PROJECT_SOURCE_DIR}/resources DESTINATION .)