cmake_minimum_required(VERSION 3.0.0)
project(traj VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_library(get_path  lib_route.cpp DWA.cpp Astar.cpp)

# add_executable(traj route.cpp)
# target_link_libraries(traj get_path)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
