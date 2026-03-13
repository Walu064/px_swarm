cmake_minimum_required(VERSION 3.16)
project(px_swarm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Helpful defaults
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Warnings (optional)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(MAVSDK REQUIRED)

add_library(px_swarm_core
  src/app/cli.cpp
  src/app/mission.cpp
  src/control/px4_interface.cpp
  src/control/leader_controller.cpp
  src/control/follower_controller.cpp
  src/swarm/swarm_bus.cpp
  src/swarm/swarm_state.cpp
  src/swarm/pso.cpp
  src/utils/keyboard.cpp
  src/utils/time.cpp
)

target_include_directories(px_swarm_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# NOTE: Some MAVSDK packages export only MAVSDK::mavsdk (no per-plugin targets).
target_link_libraries(px_swarm_core PUBLIC
  MAVSDK::mavsdk
)

add_executable(px_swarm_node
  src/app/px_swarm_node_main.cpp
)

target_link_libraries(px_swarm_node PRIVATE px_swarm_core)

install(TARGETS px_swarm_node RUNTIME DESTINATION bin)
