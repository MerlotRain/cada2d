cmake_minimum_required(VERSION 3.29)

project(cada2d LANGUAGES CXX C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(BUILD_EXAMPLES "Build examples" OFF)

add_subdirectory(thirdparty)
find_package(Eigen3 REQUIRED)

add_library(cada2d SHARED)
add_subdirectory(include)
add_subdirectory(src) 

if(WIN32)
	target_compile_definitions(cada2d PRIVATE CADA_EXPORTS)
endif()

target_link_libraries(cada2d Eigen3::Eigen apollonius OpenNURBS)

# add_subdirectory(capi)
# if(BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()
# add_subdirectory(tools)