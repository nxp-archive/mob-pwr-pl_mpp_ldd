#!/bin/sh

# rpi-make.sh - Making kernel for RPI3
# * Default build configuration
# ./rpi-make.sh bcm2709_defconfig
# * Build the kernel, modules, and Device Tree blobs
# ./rpi-make.sh zImage modules dtbs

# To speed up compilation on multiprocessor systems, and get some
# improvement on single processor ones, use -j n, where n is the number of
# processors * 1.5. Alternatively, feel free to experiment and see what works!

CWD="$(cd "$(dirname "$0")"; pwd)"
#echo $CWD

HOST_MACH=`uname -m`
if [ "$HOST_MACH" = "x86_64" ]
then
	HOST_MACH="-x64"
else
	HOST_MACH=""
fi
#echo $HOST_MACH

PATH=$PATH:$CWD/../rpi-tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian$HOST_MACH/bin/ \
KERNEL=kernel7 \
make -C $CWD ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- O=$CWD/.obj $@
