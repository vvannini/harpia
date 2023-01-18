

## Top level `CMakeLists.txt`

Maybe there should be a `CMakeLists.txt` in the root of the project. This could
be moved from `src/rosplan/CMakeLists.txt` since there is no real use for it
there (as far as I know).

## Rosplan as submodule

If we can modularize away the custom dispatch interface currently inside
`src/rosplan/rosplan_planning_system/` we could use rosplan only as a submodule.
This would separate nicely what was actually custom implemented for the project
and what is external using git submodules.

## No `src/` directory

It seems to be a common practice in the ROS community to not have a top level
`src/` directory, instead the packages are placed directly in the root of the
project and each one would have it's own `src/` or `scripts/`. Then there is an
additional metapackage with the same name as the project which is responsible for
depending on all packages.

## Possible data racing in `RPHarpiaExecutor.cpp`

There is a possible data race in `src/rosplan_interface_harpia/src/RPHarpiaExecutor.cpp`
on the global variables `mission` and `uav`. They are updated asynchronously by
different subscriber and read in `concreteCallback()` which means it is possible
for an update to race with the callback. Possible fixes would include using some
lock like a mutex. It may cause a deadlock if used incorrectly, but should be
faster than an atomic solution.

## Python utils library

Some functions such as `wait_for()` and `get_harpia_root_dir()` are used in
several different python scripts. Ideally there would be some shared library
with all of this common functionality.
