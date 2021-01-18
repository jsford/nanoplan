if( ${NANOPLAN_BUILD_TESTS} )
  enable_testing()

  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v2.13.4
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(test_${PROJECT_NAME}
    test_main.cpp
    test_queues.cpp
    test_planners.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} nanoplan Catch2)

  include(CTest)
  include(ParseAndAddCatchTests)
  ParseAndAddCatchTests(test_${PROJECT_NAME})
endif()
