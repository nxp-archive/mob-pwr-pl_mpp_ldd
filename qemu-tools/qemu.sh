#!/bin/bash

out_dir=$PWD/.out
build_out_dir=$out_dir/obj

if [ ! -f "$build_out_dir/arch/arm/boot/zImage" ]; then
	echo "zImage not found"
	exit 1
fi

if [ ! -f "$build_out_dir/arch/arm/boot/dts/versatile-pb.dtb" ]; then
	echo "DTB not found"
	exit 1
fi

mkdir -p .out/qemu

# Kernel Command-line
cmdline=""
cmdline="$cmdline root=/dev/ram"
cmdline="$cmdline rdinit=/sbin/init"
cmdline="$cmdline ignore_loglevel"
cmdline="$cmdline console=ttyAMA0"
cmdline="$cmdline ip=dhcp"
#cmdline="$cmdline $*"

# User Args
user_args=""
user_args="$user_args $*"

# OPTION "-net"
opt_net_nic="-net nic,vlan=0"
opt_net_smb="-net user,smb=$out_dir/qemu"

# OPTION "-kernel"
cat "$build_out_dir/arch/arm/boot/zImage" "$build_out_dir/arch/arm/boot/dts/versatile-pb.dtb" > "$build_out_dir/arch/arm/boot/zImage_w_dtb"
chmod a+r "$build_out_dir/arch/arm/boot/zImage_w_dtb"
opt_kernel="-kernel $build_out_dir/arch/arm/boot/zImage_w_dtb"

# OPTION "-initrd"
opt_initrd="-initrd rootfs.img.gz"

sudo qemu-system-arm \
-m 256M \
-M versatilepb \
$opt_kernel \
$opt_initrd \
$opt_net_nic \
$opt_net_smb \
-usb -usbdevice host:0b6a:434d \
-usb -usbdevice host:0403:6001 \
-nographic \
-no-reboot \
-append "$cmdline" \
$user_args

#reset
