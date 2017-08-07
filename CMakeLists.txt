CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(dsaam)

add_definitions(-std=c++11)

find_package(Boost REQUIRED)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

INCLUDE_DIRECTORIES(cxx/include ${Boost_INCLUDE_DIRS})

set(DSAAM_SRCS
  cxx/src/dsaam/node.cpp
  )
  
add_library(dsaam ${DSAAM_SRCS})
target_link_libraries(dsaam)


