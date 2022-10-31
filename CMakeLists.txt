cmake_minimum_required(VERSION 2.8.3)
project(lanelet2_parser)

# Enable support for C++14
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "{CMAKE_CXX_FLAGS} -std=c++14")
else ()
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lanelet2_parser
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set (LL2_LIBS
  lib/lanelet2_parser.cpp
)

add_library(lanelet2_parser ${LL2_LIBS})
target_link_libraries(lanelet2_parser ${catkin_LIBRARIES})
add_dependencies(lanelet2_parser ${catkin_EXPORTED_TARGETS})
