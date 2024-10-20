test_executable = executable(
    'test_exec',
    'shape/arcTest.cpp',
    include_directories: inc_dirs,
    link_with: [cada2d_lib],
)

test(
    'test',
    test_executable,
    args: ['--log-file', 'test.log', '--log-level', 'debug'],
)