#!/bin/sh

# arm-make.sh - Making kernel for ARM
# * Default build configuration
# ./arm32-make.sh defconfig
# * Build the kernel, modules, and Device Tree blobs
# ./arm32-make.sh all

# To speed up compilation on multiprocessor systems, and get some
# improvement on single processor ones, use -j n, where n is the number of
# processors * 1.5. Alternatively, feel free to experiment and see what works!

CWD="$(cd "$(dirname "$0")"; pwd)"
#echo $CWD

make -C $CWD ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- O=$CWD/.obj $@
