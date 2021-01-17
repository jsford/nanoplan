if( ${BUILD_DEMO} )
  include(FetchContent)
  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 7.1.3
  )
  FetchContent_MakeAvailable(fmt)

  add_executable(demo_${PROJECT_NAME} demo.cpp)
  target_link_libraries(demo_${PROJECT_NAME} nanoplan fmt::fmt)
endif()
