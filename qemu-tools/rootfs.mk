# QEMU ARM INITRD Makefile
# date: JUL 08 2015

# 2015-03-23
BUSYBOX_NAME=busybox-1.23.2
BUSYBOX_TAR=$(BUSYBOX_NAME).tar.bz2
BUSYBOX_URL_BASE=http://busybox.net/downloads

# 2015-02-06
GLIBC_NAME=glibc-2.21
GLIBC_TAR=$(GLIBC_NAME).tar.gz
GLIBC_URL_BASE=http://ftp.gnu.org/gnu/libc


# 2011-12-04
I2CTOOLS_NAME=i2c-tools-3-1-0
I2CTOOLS_TAR=V3-1-0.tar.gz
I2CTOOLS_URL_BASE=https://github.com/groeck/i2c-tools/archive

JOBS=8
CWD=$(shell pwd)

OUT_DIR=$(CWD)/.out
INSTALL_ROOT=$(OUT_DIR)/rootfs

Q=@
E=/bin/echo -e

TARGET=arm-linux-gnueabi
HOST=i686-pc-linux-gnu

help:
	$(Q) $(E) "QEMU ARM INITRD Makefile"
	$(Q) $(E) "------------------------"
	$(Q) $(E) ""
	$(Q) $(E) "busybox_config  Menuconfig $(BUSYBOX_NAME)"
	$(Q) $(E) "busybox         Install $(BUSYBOX_NAME)"
	$(Q) $(E) "glibc           Install $(GLIBC_NAME)"
	$(Q) $(E) "i2ctools        Install $(I2CTOOLS_NAME)"
	$(Q) $(E) ""
	$(Q) $(E) "install_rootfs  Install all for rootfs.img"
	$(Q) $(E) "release_rootfs  Release rootfs.img.gz"
	$(Q) $(E) "make_rootfs     Make rootfs.img"
	$(Q) $(E) "extract_rootfs  Extract rootfs.img.gz"
	$(Q) $(E) ""
	$(Q) $(E) "TARGETS FROM SCRATCH:"
	$(Q) $(E) "  busybox_config -> install_rootfs -> release_rootfs"
	$(Q) $(E) ""
#	$(Q) $(E) "OUT_DIR         $(OUT_DIR)"
#	$(Q) $(E) "INSTALL_ROOT    $(INSTALL_ROOT)"

define cmd_msg
	$(E) $1"  "$(subst $(CWD)/,,$2)
endef

$(INSTALL_ROOT):
	$(Q) mkdir -p $@

PREREQUISITES=$(INSTALL_ROOT)

.FORCE:

$(INSTALL_ROOT)/etc/inittab:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) echo "::sysinit:/etc/init.d/rcS" > $@
	$(Q) echo "ttyAMA0::respawn:-/bin/sh" >> $@
	$(Q) echo "::restart:/sbin/init" >> $@
	$(Q) echo "::ctrlaltdel:/sbin/reboot" >> $@
	$(Q) echo "::shutdown:/bin/umount -a -r" >> $@
	$(Q) echo "::shutdown:/sbin/swapoff -a" >> $@
$(INSTALL_ROOT)/etc/group:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) echo "root:x:0:" > $@
$(INSTALL_ROOT)/etc/passwd:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) echo "root:x:0:0:root:/root:/bin/sh" > $@
$(INSTALL_ROOT)/etc/init.d/rcS:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) echo "#!/bin/sh" > $@
	$(Q) echo "mount -t proc none /proc" >> $@
	$(Q) echo "mount -t sysfs none /sys" >> $@
	$(Q) echo "echo /sbin/mdev > /proc/sys/kernel/hotplug" >> $@
	$(Q) echo "/sbin/mdev -s" >> $@
	$(Q) echo "mount -t cifs //10.0.2.4/qemu /mnt/qemu -o username=guest,rw" >> $@
	$(Q) echo "ln -sf /mnt/qemu/lib/modules /lib/modules" >> $@
$(INSTALL_ROOT)/etc/network/interfaces:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) $(E) "auto lo" > $@
	$(Q) $(E) "iface lo inet loopback" >> $@
	$(Q) $(E) "auto eth0" >> $@
	$(Q) $(E) "iface eth0 inet dhcp" >> $@
$(INSTALL_ROOT)/etc/network/if-pre-up.d:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) $(E) "" > $@
$(INSTALL_ROOT)/etc/network/if-up.d:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) $(E) "" > $@
$(INSTALL_ROOT)/etc/network/if-down.d:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) $(E) "" > $@
$(INSTALL_ROOT)/etc/network/if-post-down.d:
	$(Q) $(call cmd_msg,"GEN",$@)
	$(Q) $(E) "" > $@

ETC_FILES=\
	$(INSTALL_ROOT)/etc/inittab \
	$(INSTALL_ROOT)/etc/group \
	$(INSTALL_ROOT)/etc/passwd \
	$(INSTALL_ROOT)/etc/init.d/rcS \
	$(INSTALL_ROOT)/etc/network/interfaces \
	$(INSTALL_ROOT)/etc/network/if-pre-up.d \
	$(INSTALL_ROOT)/etc/network/if-up.d \
	$(INSTALL_ROOT)/etc/network/if-down.d \
	$(INSTALL_ROOT)/etc/network/if-post-down.d

install_rootfs: $(PREREQUISITES) glibc busybox i2ctools

release_rootfs: $(PREREQUISITES) rootfs.img.gz
rootfs.img.gz: $(OUT_DIR)/rootfs.img
	$(Q) $(call cmd_msg,"ZIP","$@ <- $<")
	$(Q) gzip --verbose --stdout $< > $@
$(OUT_DIR)/rootfs.img: prepare_rootfs ldconfig $(ETC_FILES)
	$(Q) chmod a+x $(INSTALL_ROOT)/etc/init.d/rcS
	$(Q) $(call cmd_msg,"GEN","$@ <- $(INSTALL_ROOT)")
	$(Q) cd $(INSTALL_ROOT) ; find . | cpio --create --format=newc --owner=root:root > $@
prepare_rootfs:
	$(Q) mkdir -v -p $(INSTALL_ROOT)/proc
	$(Q) mkdir -v -p $(INSTALL_ROOT)/sys
	$(Q) mkdir -v -p $(INSTALL_ROOT)/dev
	$(Q) mkdir -v -p $(INSTALL_ROOT)/etc/init.d
	$(Q) mkdir -v -p $(INSTALL_ROOT)/etc/network
	$(Q) mkdir -v -p $(INSTALL_ROOT)/usr/lib
	$(Q) mkdir -v -p $(INSTALL_ROOT)/var/log
	$(Q) mkdir -v -p $(INSTALL_ROOT)/var/run
	$(Q) mkdir -v -p $(INSTALL_ROOT)/mnt/qemu
ldconfig:
	$(Q) echo /usr/lib > $(INSTALL_ROOT)/etc/ld.so.conf

make_rootfs: $(PREREQUISITES) $(OUT_DIR)/rootfs.img

extract_rootfs: $(PREREQUISITES)
	$(Q) $(call cmd_msg,"RM","$(INSTALL_ROOT)")
	$(Q) rm -rf $(INSTALL_ROOT)
	$(Q) mkdir -p $(INSTALL_ROOT)
	$(Q) $(call cmd_msg,"UNZIP","rootfs.img.gz -> $(INSTALL_ROOT)")
	$(Q) cd $(INSTALL_ROOT) ; gzip --decompress --stdout $(CWD)/rootfs.img.gz | cpio --extract --make-directories

define cmd_download
$$(OUT_DIR)/$2:
	$$(Q) wget $1/$2 -O $$@
endef

define cmd_unpack
$$(OUT_DIR)/$2: $$(OUT_DIR)/$1
	$$(Q) if [ ! -d $$@ ]; then $$(E) "UNTAR $$<"; tar -C $$(OUT_DIR) -xf $$<; fi
endef

busybox_config: $(PREREQUISITES) $(OUT_DIR)/$(BUSYBOX_NAME)/.config
	$(Q) make -C $(OUT_DIR)/$(BUSYBOX_NAME) ARCH=arm CROSS_COMPILE=$(TARGET) menuconfig
busybox: $(PREREQUISITES) $(OUT_DIR)/$(BUSYBOX_NAME)/.config
	$(Q) make -C $(OUT_DIR)/$(BUSYBOX_NAME) -j$(JOBS) ARCH=arm CROSS_COMPILE=$(TARGET)- CONFIG_PREFIX=$(INSTALL_ROOT) all install
$(OUT_DIR)/$(BUSYBOX_NAME)/.config: $(OUT_DIR)/$(BUSYBOX_NAME)
	$(Q) make -C $(OUT_DIR)/$(BUSYBOX_NAME) ARCH=arm CROSS_COMPILE=$(TARGET) defconfig
$(eval $(call cmd_download,$(BUSYBOX_URL_BASE),$(BUSYBOX_TAR)))
$(eval $(call cmd_unpack,$(BUSYBOX_TAR),$(BUSYBOX_NAME)))

glibc: $(PREREQUISITES) $(OUT_DIR)/glibc-build/Makefile
	$(Q) make -C $(OUT_DIR)/glibc-build -j1 install_root=$(INSTALL_ROOT) all install
	$(Q) cp -v -f $(shell $(TARGET)-gcc -print-file-name=libgcc_s.so.1) $(INSTALL_ROOT)/lib
$(OUT_DIR)/glibc-build/Makefile: $(OUT_DIR)/$(GLIBC_NAME)
	$(Q) mkdir -p $(OUT_DIR)/glibc-build
	$(Q) cd $(OUT_DIR)/glibc-build ; $(OUT_DIR)/$(GLIBC_NAME)/configure $(TARGET) --target=$(TARGET) --build=$(HOST) --prefix= --enable-add-ons
$(eval $(call cmd_download,$(GLIBC_URL_BASE),$(GLIBC_TAR)))
$(eval $(call cmd_unpack,$(GLIBC_TAR),$(GLIBC_NAME)))

I2CTOOLS_TARGETS=tools/i2cdetect tools/i2cdump tools/i2cset tools/i2cget
I2CTOOLS_INSTALL_FLAGS=--target-directory=$(INSTALL_ROOT)/usr/bin
I2CTOOLS_INSTALL_FLAGS+=--strip-program=$(TARGET)-strip --strip
#I2CTOOLS_INSTALL_FLAGS+=--verbose
i2ctools: $(PREREQUISITES) $(OUT_DIR)/$(I2CTOOLS_NAME)/Makefile
	$(Q) make -C $(OUT_DIR)/$(I2CTOOLS_NAME) -j$(JOBS) CC=$(TARGET)-gcc all
	$(Q)	for T in $(I2CTOOLS_TARGETS);\
			do \
				$(call cmd_msg,"INSTALL","$$T");\
				install $(I2CTOOLS_INSTALL_FLAGS) $(OUT_DIR)/$(I2CTOOLS_NAME)/$$T;\
			done
	$(Q) install $(I2CTOOLS_INSTALL_FLAGS) $(addprefix $(OUT_DIR)/$(I2CTOOLS_NAME)/,$(I2CTOOLS_TARGETS))
$(OUT_DIR)/$(I2CTOOLS_NAME)/Makefile: $(OUT_DIR)/$(I2CTOOLS_NAME)
$(eval $(call cmd_download,$(I2CTOOLS_URL_BASE),$(I2CTOOLS_TAR)))
$(eval $(call cmd_unpack,$(I2CTOOLS_TAR),$(I2CTOOLS_NAME)))