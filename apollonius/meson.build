apo_files = files(
    'apollonius.c',
    'circle.c',
    'line.c',
    'point.c',
)

apo_include = include_directories('.')

libapo = static_library('apollonius', apo_files, include_directories: apo_include, install: true)

apo_dep = declare_dependency(link_with: libapo, include_directories: apo_include)