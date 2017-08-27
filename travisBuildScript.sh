#!/bin/sh
set -e

mkdir $BUILD_DIR
cd $BUILD_DIR

CMAKE_FLAGS="-DCMAKE_CXX_COMPILER=$COMPILER"

if [ -n "$COMPILER_FLAGS" ]; then
    CMAKE_FLAGS="$CMAKE_FLAGS -DCMAKE_CXX_FLAGS=$COMPILER_FLAGS"
fi

cmake $CMAKE_FLAGS ..

make -j${nproc}
ctest -VV
