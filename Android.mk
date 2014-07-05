#Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_USE_ST_ERICSSON_KERNEL),true)

# Give other modules a nice, symbolic name to use as a dependent
# Yes, there are modules that cannot build unless the kernel has
# been built. Typical (only?) example: loadable kernel modules.
.PHONY: build-kernel clean-kernel

PRIVATE_KERNEL_ARGS := -C kernel ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) LOCALVERSION=$(LOCALVERSION)

PRIVATE_OUT := $(abspath $(PRODUCT_OUT)/root)

PRODUCT_JB_UPG := GT-I9070 GT-I8160 GT-I8530

ifeq ($(CONNECTIVITY_ENABLE_FEATURE_STE_WLAN),true)
export FEATURE_STE_WLAN=y
# For compat-wireless gits to compile with kernel
export STERICSSON_WLAN_BUILT_IN=y
endif
ifeq ($(CONNECTIVITY_ENABLE_FEATURE_STE_BT),true)
export FEATURE_STE_BT=y
endif

# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
PRIVATE_KERNEL_ARGS += O=$(KERNEL_OUTPUT)
endif
else
KERNEL_OUTPUT := $(call my-dir)
endif

# STE Connectivity configuration
PRIVATE_KERNEL_ARGS += \
	CONNECTIVITY_ENABLE_FEATURE_STE_CONNECTIVITY=$(CONNECTIVITY_ENABLE_FEATURE_STE_CONNECTIVITY)
build-kernel: $(PRODUCT_OUT)/zImage

# Include kernel in the Android build system
include $(CLEAR_VARS)

KERNEL_LIBPATH := $(KERNEL_OUTPUT)/arch/arm/boot
LOCAL_PATH := $(KERNEL_LIBPATH)
LOCAL_SRC_FILES := zImage
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)

include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
KERNEL_LIBPATH := $(KERNEL_OUTPUT)
LOCAL_PATH := $(KERNEL_LIBPATH)
LOCAL_SRC_FILES := vmlinux
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)
$(KERNEL_LIBPATH)/$(LOCAL_SRC_FILES): build-kernel
include $(BUILD_PREBUILT)
# Configures, builds and installs the kernel. KERNEL_DEFCONFIG usually
# comes from the BoardConfig.mk file, but can be overridden on the
# command line or by an environment variable.
# If KERNEL_DEFCONFIG is set to 'local', configuration is skipped.
# This is useful if you want to play with your own, custom configuration.

#iifeq ($(ONE_SHOT_MAKEFILE),)
#$(KERNEL_OUTPUT)/arch/arm/boot/uImage: $(UBOOT_OUTPUT)/tools/mkimage FORCE
#else
$(KERNEL_OUTPUT)/arch/arm/boot/zImage: FORCE
#endif

# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
	@mkdir -p $(KERNEL_OUTPUT)
endif
endif

ifeq ($(KERNEL_DEFCONFIG),local)
	@echo Skipping kernel configuration, KERNEL_DEFCONFIG set to local
else
	$(MAKE) $(PRIVATE_KERNEL_ARGS) $(KERNEL_DEFCONFIG)
endif

ifeq ($(SEC_PRODUCT_SHIP),true) 
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--enable CONFIG_SAMSUNG_PRODUCT_SHIP
endif

ifeq ($(SEC_DEBUG_LEVEL),high)
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--set-val CONFIG_SEC_DEBUG_LEVEL 2
else
ifeq ($(SEC_DEBUG_LEVEL),low)
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--set-val CONFIG_SEC_DEBUG_LEVEL 0
endif
endif
# Disable SHRM Signature verification feature from here
ifeq ($(SHRM_ENABLE_FEATURE_SIGNATURE_VERIFICATION),false)
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_U8500_SHRM_ENABLE_FEATURE_SIGNATURE_VERIFICATION
endif

ifneq ($(CONNECTIVITY_ENABLE_FEATURE_STE_WLAN),true)
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_COMPAT_WIRELESS
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_COMPAT_WIRELESS_MODULES
#	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_CFG80211
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_MAC80211
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_LIB80211
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--disable CONFIG_COMPAT_MAC80211_RC_DEFAULT
endif
ifneq ($(findstring $(TARGET_PRODUCT), $(PRODUCT_JB_UPG)),)
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--set-str CONFIG_INITRAMFS_SOURCE "$(abspath $(TOP)/$(TARGET_ROOT_OUT))" \
		--set-val CONFIG_INITRAMFS_ROOT_UID 0 \
		--set-val CONFIG_INITRAMFS_ROOT_GID 0 \
		--enable CONFIG_INITRAMFS_COMPRESSION_NONE \
		--disable CONFIG_INITRAMFS_COMPRESSION_GZIP
endif

# Enable openMAC from here, since the defconfig is now set for UMAC
ifeq ($(WLAN_ENABLE_OPEN_MAC_SOLUTION),true)
ifneq ($(shell [ -f kernel/net/compat-wireless-openmac/Makefile ] && echo "OK"), OK)
	$(shell ln -s ../../vendor/st-ericsson/variant/connectivity/wlan/compat-wireless-openmac/ kernel/net/)
endif
endif
	$(MAKE) $(PRIVATE_KERNEL_ARGS) zImage
ifeq ($(KERNEL_NO_MODULES),)
	$(MAKE) $(PRIVATE_KERNEL_ARGS) modules
	$(MAKE) $(PRIVATE_KERNEL_ARGS) INSTALL_MOD_PATH:=$(PRIVATE_OUT) modules_install
else
	@echo Skipping building of kernel modules, KERNEL_NO_MODULES set
endif
	cp -u $(KERNEL_OUTPUT)/vmlinux $(PRODUCT_OUT)

ifneq ($(findstring $(TARGET_PRODUCT), $(PRODUCT_JB_UPG)),)
	cd $(KERNEL_OUTPUT)/arch/arm/boot; \
	cp -f zImage kernel.bin; \
	cp -f zImage $(abspath $(PRODUCT_OUT)); \
	cp -f zImage $(abspath $(PRODUCT_OUT))/kernel.bin; \
	cp -f zImage $(abspath $(PRODUCT_OUT))/kernel2.bin; \
	tar cvf $(abspath $(PRODUCT_OUT))/kernel.tar kernel.bin
endif

build-kernel2: build-kernel init-symlinks
	$(MAKE) bootimage
	$(MAKE) recoveryimage

build-kernel3: ramdisk init-symlinks recoveryimage build-kernel
	@echo "== build-kernel3 =="

# Configures and runs menuconfig on the kernel based on
# KERNEL_DEFCONFIG given on commandline or in BoardConfig.mk.
# The build after running menuconfig must be run with
# KERNEL_DEFCONFIG=local to not override the configuration modification done.

menuconfig-kernel:
# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
	@mkdir -p $(KERNEL_OUTPUT)
endif
endif

	$(MAKE) $(PRIVATE_KERNEL_ARGS) $(KERNEL_DEFCONFIG)
	$(MAKE) $(PRIVATE_KERNEL_ARGS) menuconfig

clean clobber : clean-kernel

clean-kernel:
	$(MAKE) $(PRIVATE_KERNEL_ARGS) clean
endif
