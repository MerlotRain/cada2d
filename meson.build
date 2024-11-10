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
subdir('apollonius') 

eigen_dep = dependency('eigen3', required: true)
# apo_dep = dependency('apollonius', required: true) 
inc_dirs = include_directories('include')

subdir('algorithm')
subdir('shape')
subdir('index')

cada2d_lib = static_library(
    'cada2d',
    shape_src + algorithm_src + index_src,
    include_directories: inc_dirs,
    dependencies: [eigen_dep, apo_dep],
    cpp_args: ['-D_USE_MATH_DEFINES'],
)

if build_test
    gtest_dep = dependency('gtest', main: true, required: false)
    subdir('tests')
endif