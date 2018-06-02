#! /usr/bin/env bash

set -e
set -x

rm -rf build
mkdir build
pushd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
popd
