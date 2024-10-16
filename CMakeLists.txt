cmake_minimum_required(VERSION 3.10)
project(cadsa3d LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 11)

add_library(cada2d STATIC
    arc.cc
    bbox.cc
    bspline.cc
    circle.cc
    ellipse.cc
    intcurvecurve.cc
    line.cc
    math.cc
    point.cc
    pointstobspline.cc
    polyline.cc
    ray.cc
    vec3d.cc
    xline.cc
)