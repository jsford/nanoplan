if( ${NANOPLAN_BUILD_EXAMPLES} )
  include(FetchContent)
  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 7.1.3
  )
  FetchContent_MakeAvailable(fmt)

  add_executable(example_grid2d grid2d.cpp)
  target_link_libraries(example_grid2d nanoplan fmt::fmt)

  add_executable(example_grid3d grid3d.cpp)
  target_link_libraries(example_grid3d nanoplan fmt::fmt)
endif()
