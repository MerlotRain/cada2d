polyline_files = files(
    'polyline.c',
)

polyline_include = include_directories('.')

libpolyline = static_library(
    'polyline',
    polyline_files,
    include_directories: polyline_include,
    install: true,
)

polyline_dep = declare_dependency(link_with: libpolyline, include_directories: polyline_include)