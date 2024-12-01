cmake_minimum_required(VERSION 3.29)

project(cada2d LANGUAGES CXX C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(thirdparty/CavalierContours)
find_package(Eigen3 REQUIRED)

set(cada_srcs
    shape/arc.cc
    shape/bbox.cc
    shape/spline.cc
    shape/circle.cc
    shape/ellipse.cc
    shape/line.cc
    shape/math.cc
    shape/point.cc
    shape/polyline.cc
    shape/ray.cc
    shape/shape.cc
    shape/shapefactory.cc
    shape/vec2d.cc
    shape/xline.cc

    algorithm/algorithm.cc
    algorithm/angle.cc
    algorithm/boundingbox.cc
    algorithm/direction.cc
    algorithm/distance.cc
    algorithm/equal.cc
    algorithm/flip.cc
    algorithm/intersection.cc
    algorithm/length.cc
    algorithm/mirror.cc
    algorithm/move.cc
    algorithm/offset.cc
    algorithm/pointonshape.cc
    algorithm/pointstobspline.cc
    algorithm/polylinecontains.cc
    algorithm/reverse.cc
    algorithm/rotate.cc
    algorithm/roundallcorners.cc
    algorithm/scale.cc
    algorithm/self-intersection.cc
    algorithm/side.cc
    algorithm/split.cc
    algorithm/stretch.cc
    algorithm/trim.cc
    algorithm/vectorto.cc
)

add_library(cada2d STATIC ${cada_srcs}) 

target_include_directories(cada2d PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/apollonius
)

target_include_directories(cada2d PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/algorithm
)

target_link_libraries(cada2d PRIVATE Eigen3::Eigen CavalierContours)

add_subdirectory(capi)
add_subdirectory(examples)
# add_subdirectory(tools)