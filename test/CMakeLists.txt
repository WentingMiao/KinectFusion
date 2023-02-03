cmake_minimum_required(VERSION 3.14)
project(unit_test)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include_directories(${CMAKE_SOURCE_DIR}/..)

enable_testing()
add_executable(
  unit_test
  Voxel_tests.cpp
  ${CMAKE_SOURCE_DIR}/../Voxels.cpp
)
target_link_libraries(
  unit_test
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_test)
