#!/bin/sh

# arm64-make.sh - Making kernel for ARM64
# * Default build configuration
# ./arm64-make.sh defconfig
# * Build the kernel, modules, and Device Tree blobs
# ./arm64-make.sh all

# To speed up compilation on multiprocessor systems, and get some
# improvement on single processor ones, use -j n, where n is the number of
# processors * 1.5. Alternatively, feel free to experiment and see what works!

CWD="$(cd "$(dirname "$0")"; pwd)"
#echo $CWD

make -C $CWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- O=$CWD/.obj64 $@
