# Don't generate debug symbols (remove -g) and minimize binary size with -Os.
# Debug symbol are useless for CI build (we don't retrieve any binaries) and can stop the build
# by taking all the disk space.
set(CMAKE_CXX_FLAGS_DEBUG
    "-Os"
    CACHE STRING "")
