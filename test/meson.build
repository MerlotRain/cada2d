test_executable = executable('test', 'shape/arcTest.cpp', link_with: [cada2d_lib])

test(
    'test',
    test_executable,
    args: ['--log-file', 'test.log', '--log-level', 'debug'],
)