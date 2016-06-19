cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PCL_TEST)

find_package(PCL 1.7 REQUIRED COMPONENTS commo io feature visualization)

include_directories(${PCL_INCLUDE_DIRS} include )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(LFSH_test src/LFSH_test.cpp )
target_link_libraries(LFSH_test ${PCL_LIBRARIES} )
