cmake_minimum_required(VERSION 3.14)

project(nanoplan)

option(BUILD_DEMO "Build the planning demo program." ON)
option(BUILD_TESTS "Build unit tests." ON)

set(CMAKE_CXX_STANDARD 11)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if( ${BUILD_DEMO} )
  include(FetchContent)
  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 7.1.3
  )
  FetchContent_MakeAvailable(fmt)

  add_executable(demo_${PROJECT_NAME} demo/demo.cpp)
  target_link_libraries(demo_${PROJECT_NAME} nanoplan fmt::fmt)
endif()

if( ${BUILD_TESTS} )
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v2.13.4
  )
  FetchContent_MakeAvailable(Catch2)

  find_package(fmt REQUIRED)
  add_executable(test_${PROJECT_NAME}
    test/test_main.cpp
    test/test_queues.cpp
    test/test_planners.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} nanoplan Catch2 fmt::fmt)
endif()
