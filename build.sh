#!/bin/bash

# exit on error and print each command
set -ex

# cmake
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DXYBER_X1_INFER_BUILD_TESTS=OFF \
    -DXYBER_X1_INFER_SIMULATION=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    $@

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

cmake --build build --config Release --target install --parallel $(nproc)
