CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(dsaam)

add_definitions(-std=c++11)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")


SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -Werror")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror")


MESSAGE(STATUS "Build type:" ${CMAKE_BUILD_TYPE})

INCLUDE_DIRECTORIES(cxx/include)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(nbody_threads test/cxx/nbody_threads.cpp)
target_link_libraries(nbody_threads Threads::Threads)

install(DIRECTORY cxx/ DESTINATION ${CMAKE_INSTALL_PREFIX}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

#add_subdirectory(test/ros/src)
