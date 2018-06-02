# libslam

Heavily modified ORB-SLAM2 to be packaged as a reusable library (Linux only)

**PRs are not welcome!** Contribute back to the main ORB-SLAM2 repository instead.

## build

This repository is Linux-only (specifically tested on Ubuntu 14, 16, 17, and 18).

```bash
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make -j4
...
[100%] Linking CXX shared library libslam.so
[100%] Built target slam
```

This should leave you with `libslam.so` with all its dependencies linked into it statically.

## changelog

- removed RGBD and Sterero SLAM codes
- checked in all dependencies under `thridparty/`
- compiled everything into a single shared library `libslam.so`
- replaced DBoW with DBoW3 for lightning fast vocabulary loading times
- minor code optimizations and changes here and there to implrove performance
- removed code for GUI and debug logging and basically anything that does not belong in a library

## license

This repository is dual licensed under MIT and [ORB-SLAM2's original license](https://github.com/raulmur/ORB_SLAM2/blob/master/LICENSE.txt).
