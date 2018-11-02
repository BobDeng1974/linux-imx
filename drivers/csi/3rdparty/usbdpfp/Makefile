DRIVER_VERSION	 := 2.0.0.7
CFLAGS_usbdpfp.o := -DDRIVER_VERSION=\"v$(DRIVER_VERSION)\"
mod_usbdpfp-objs := usbdpfp.o
obj-$(CONFIG_DPFINGERPRINT) += mod_usbdpfp.o
