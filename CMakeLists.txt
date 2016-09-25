cmake_minimum_required (VERSION 3.0.2)
project (fdas3)

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
endif()


add_subdirectory(utils)
add_subdirectory(devices)
