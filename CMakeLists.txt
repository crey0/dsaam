CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(dsaam)

add_definitions(-std=c++11)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")


SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")


MESSAGE(STATUS "Build type:" ${CMAKE_BUILD_TYPE})

INCLUDE_DIRECTORIES(cxx/include)


install(DIRECTORY cxx/ DESTINATION ${CMAKE_INSTALL_PREFIX}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

IF(BUILD_TEST)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)

  add_executable(nbody_threads EXCLUDE_FROM_ALL test/cxx/nbody_threads.cpp)
  target_link_libraries(nbody_threads Threads::Threads)
  set(TEST_DEPENDS_ALL_FLAG "")
  IF(BUILD_TEST_ALWAYS)
    set(TEST_DEPENDS_ALL_FLAG "ALL")
  ENDIF()
  add_custom_target(build_tests ${TEST_DEPENDS_ALL_FLAG} DEPENDS nbody_threads)
ENDIF()

IF(BUILD_TEST AND BUILD_ROS)
  add_custom_target(nbody_ros
    COMMAND catkin_make -C ${dsaam_SOURCE_DIR}/test/ros --build ${CMAKE_BINARY_DIR}/ros/build --cmake-args -DCATKIN_DEVEL_PREFIX=${CMAKE_BINARY_DIR}/ros/devel -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/ros/install
    WORKING_DIRECTORY ${dsaam_SOURCE_DIR}/test/ros
    )

  add_dependencies(build_tests nbody_ros)
  set_directory_properties(PROPERTIES  ADDITIONAL_MAKE_CLEAN_FILES ros)
ENDIF()


