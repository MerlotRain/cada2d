project(
    'cada2d',
    'cpp',
    version: '0.1',
    default_options: ['warning_level=3', 'cpp_std=c++14'],
    license: 'MIT',
)

inc_dirs = include_directories('include')

subdir('algorithm')
subdir('shape')

cada2d_lib = static_library(
    'cada2d',
    shape_src + algorithm_src,
    include_directories: inc_dirs,
    cpp_args: ['-D_USE_MATH_DEFINES'],
)

subdir('test')