# Branches
 - `nxp-lcdseg.rpi.linux-4.9.41` - NXP LCD segment driver. PCA8553, PCA8561, ...
 - `pca9468.arm.linux-4.4.1` - MP12 E2E demo supports. PCA9468, NX30P6093 and PCA9491.
 - `pca9956b.rpi.linux-4.9.41` - PCA9956B LED driver.

&nbsp;
# Dev System for RPI-Linux

## Files
 - `env` - Contains environment variables required by following scripts. (*env.example* could be an example.)
 - `build.sh` - Script file building your developing modules.
 - `release.cfg` - Configurations for release process.
 - `release.sh` - Script file releasing your output, requires *release.cfg*.
 - `kernel/rpi-make.sh` - Script file making the kernel for RPI3.
 - `rpi-tools/` - git submodule, tracking `https://github.com/raspberrypi/tools`.

## Cross toolchain for RPI
Update and initialize (if required) the registered submodule `rpi-tools` through following command when using for the first time after checkout.
```sh
$ git submodule update --init
```

## New branch for your development
Generally, base branch for yours would be `base.rpi.linux-4.9.41`, tracking `rpi.linux-4.9.41` &#x2190; `linux-4.9.41` &#x2190; `linux-4.x` &#x2190; `master`.
```sh
$ git checkout -b <your_branch_name> base.rpi.linux-4.9.41
```

&nbsp;

And then, create the `env` file for your development environment.
```sh
$ cp env.example env
$ <edit the file 'env'>
```

## Generating `.config`, a kernel compile configuration
By following command, kernel will be configured to make a target for RPI3.
```sh
$ kernel/rpi-make.sh bcm2709_defconfig
```

## Build your developing drivers as built-in (32-bit)
Just run the generic kernel build command, for built-in files which are inside the kernel tree.
```sh
$ kernel/rpi-make.sh all
```

## Build your developing drivers as modules (32-bit)
Assume that the files your are developing are in a separate directory. Go to the directory where your files are located and run `build.sh` of the top-level directory.
```sh
$ cd <directory_where_your_files_are>
$ ../build.sh
```

&nbsp;

&nbsp;
# Dev System for QEMU/ARM-Linux 

## Files
 - `build32.sh` - Script file building your developing modules for 32-bit kernel.
 - `build64.sh` - Script file building your developing modules for 64-bit kernel.
 - `kernel/arch/arm/configs/defconfig` - 32-bit kernel compile configuration for your development 
 - `kernel/arch/arm64/configs/defconfig` - 64-bit kernel compile configuration for your development
 - `kernel/arm32-make.sh` - Script file making the kernel for ARM.
 - `kernel/arm64-make.sh` - Script file making the kernel for ARM64.
 - `qemu-tools/` - QEMU/ARM supports
 
## New branch for your development
Generally, base branch for yours would be `base.arm.linux-x.y.z`, tracking `linux-x.y.z` &#x2190; `linux-x.y` &#x2190; `master`.
```sh
$ git checkout -b <your_branch_name> base.arm.linux-x.y.z
```

## Generating `.config`, a kernel compile configuration
By following command, kernel will be configured to make a target for ARM or ARM64.
```sh
$ kernel/arm32-make.sh defconfig
```
```sh
$ kernel/arm64-make.sh defconfig
```

## Build your developing drivers as built-in
Just run the generic kernel build command, for built-in files which are inside the kernel tree.
```sh
$ kernel/arm32-make.sh all
```
```sh
$ kernel/arm64-make.sh all
```

## Build your developing drivers as modules
Assume that the files your are developing are in a separate directory. Go to the directory where your files are located and run `build32.sh` or `build64.sh` of the top-level directory.
```sh
$ cd <directory_where_your_files_are>
$ ../build32.sh
```
```sh
$ cd <directory_where_your_files_are>
$ ../build64.sh
```

