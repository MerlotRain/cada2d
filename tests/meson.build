test_executable = executable(
    'cada2d_test',
    'shape/arcTest.cpp',
    include_directories: inc_dirs,
    link_with: [cada2d_lib],
    dependencies: [gtest_dep],
)

test(
    'test',
    test_executable,
    args: ['--log-file', 'test.log', '--log-level', 'debug'],
)