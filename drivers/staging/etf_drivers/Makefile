
TARGET :=  $(shell basename `echo $(ANDROID_PRODUCT_OUT)`)
KERNELDIR := $(ANDROID_BUILD_TOP)/kernel
KERNEL_OUTPUT := $(ANDROID_BUILD_TOP)/out/target/product/$(TARGET)/obj/kernel

ARCH := arm
ARM_ARCH_VERSION := armv7-a
CROSS_COMPILE := $(ARM_EABI_TOOLCHAIN)/arm-eabi-

ETF_FLAGS := CROSS_COMPILE=$(CROSS_COMPILE)
ETF_FLAGS += ARCH=$(ARCH)
ETF_FLAGS += ARM_ARCH_VERSION=$(ARM_ARCH_VERSION)
ETF_FLAGS += O=$(KERNEL_OUTPUT)
ETF_FLAGS += M=$(shell pwd)
ETF_FLAGS += -C $(KERNELDIR)

# for SDIO interrupts, remove macro GPIO_BASED_IRQ

# using GPIO based interrupts
EXTRA_CFLAGS += -DMOP_WORKAROUND -DPIGGYBACK -DCW1260_DETECTION_LOGIC -DGPIO_BASED_IRQ

etf_driver-objs := ./sh_ta.o ./sdio_wrapper.o ./sh_adap.o ./sbus.o ./fwio.o ./hwio.o
obj-m += etf_driver.o

all:
	make $(ETF_FLAGS) modules

clean:
	make $(ETF_FLAGS) clean
