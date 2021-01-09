cmake_minimum_required(VERSION 3.13)

project(nanoplan)

option(BUILD_DEMO "Build the planning demo program." ON)
option(BUILD_TESTS "Build unit tests." ON)

set(CMAKE_CXX_STANDARD 11)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if( ${BUILD_DEMO} )
  find_package(fmt REQUIRED)
  add_executable(demo_${PROJECT_NAME} demo/demo.cpp)
  target_link_libraries(demo_${PROJECT_NAME} nanoplan fmt::fmt)
endif()

if( ${BUILD_TESTS} )
  find_package(Catch2 REQUIRED)
  find_package(fmt REQUIRED)
  add_executable(test_${PROJECT_NAME}
    test/test_main.cpp
    test/test_queues.cpp
    test/test_planners.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} nanoplan Catch2 fmt::fmt)
endif()
