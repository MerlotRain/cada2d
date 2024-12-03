project(
    'cada2d',
    'cpp',
    'c',
    version: '0.1',
    default_options: ['warning_level=3', 'cpp_std=c++17'],
    license: 'MIT',
)

build_test = get_option('build_test')

# apo library

inc_dirs = include_directories('include')

subdir('algorithm')
subdir('shape')

cada2d_lib = static_library(
    'cada2d',
    shape_src + algorithm_src,
    include_directories: inc_dirs,
    cpp_args: ['-D_USE_MATH_DEFINES'],
)

if build_test
    gtest_dep = dependency('gtest', main: true, required: false)
    subdir('tests')
endif