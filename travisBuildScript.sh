#!/bin/sh
set -e

mkdir $BUILD_DIR
cd $BUILD_DIR

CMAKE_FLAGS="-DCMAKE_CXX_COMPILER=$COMPILER"

cmake $CMAKE_FLAGS ..

make -j${nproc} $COMPILER_FLAGS
ctest -VV
