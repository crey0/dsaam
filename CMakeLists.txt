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


MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

INCLUDE_DIRECTORIES(cxx/include)

set(DSAAM_SRCS
  cxx/src/dsaam/node.cpp
  )

add_library(dsaam ${DSAAM_SRCS})
target_link_libraries(dsaam)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(nbody_threads test/cxx/nbody_threads.cpp)
target_link_libraries(nbody_threads dsaam Threads::Threads)

add_subdirectory(test/ros/src)
