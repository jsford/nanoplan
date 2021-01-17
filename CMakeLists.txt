cmake_minimum_required(VERSION 3.14)

project(nanoplan
        VERSION 0.0.0
        DESCRIPTION "A C++11 header-only library for search-based robot planning."
        HOMEPAGE_URL "https://github.com/jsford/nanoplan")

option(BUILD_DEMO "Build the planning demo program." ON)
option(BUILD_TESTS "Build unit tests." ON)

set(CMAKE_CXX_STANDARD 11)

add_library(${PROJECT_NAME} INTERFACE)
#target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---------------------------------------------------------------
# Build the planning demo program.
# ---------------------------------------------------------------

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

# ---------------------------------------------------------------
# Build the unit tests.
# ---------------------------------------------------------------

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

# ---------------------------------------------------------------
# Install
# ---------------------------------------------------------------

include(GNUInstallDirs)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_NAME}ConfigVersion.cmake"
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY SameMajorVersion)

install(EXPORT ${PROJECT_NAME}_Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/nanoplan DESTINATION include)




