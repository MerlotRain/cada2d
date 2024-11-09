fs = import('fs')

test_sources = files('shape/arcTest.cpp', 'shape/circleTest.cpp')

test_exe = executable(
    'cada2d_test',
    test_sources,
    dependencies: [gtest_dep],
    link_with: [cada2d_lib],
    include_directories: [inc_dirs],
)

foreach test_file : test_sources
    suite_name = fs.stem(test_file.full_path())
    message('Adding test suite: ' + suite_name)
    test(suite_name, test_exe, args: ['--gtest_filter=' + suite_name + '.*'])
endforeach