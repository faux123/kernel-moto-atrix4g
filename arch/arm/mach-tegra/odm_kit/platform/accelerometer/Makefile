ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

obj-$(CONFIG_TEGRA_ODM_CONCORDE)	+= nvodm_accelerometer_adi340.o
obj-$(CONFIG_TEGRA_ODM_WHISTLER)	+= nvodm_accelerometer_adi340.o
obj-$(CONFIG_TEGRA_ODM_HARMONY)		+= nvodm_priv_accelerometer.o
obj-$(CONFIG_TEGRA_ODM_HARMONY)		+= nvodm_accelerometer_bma150.o
obj-$(CONFIG_TEGRA_ODM_HARMONY)		+= nvodm_accelerometer_kxtf9.o
obj-$(CONFIG_TEGRA_ODM_VENTANA)		+= nvodm_priv_accelerometer.o
obj-$(CONFIG_TEGRA_ODM_VENTANA)		+= nvodm_accelerometer_bma150.o
obj-$(CONFIG_TEGRA_ODM_VENTANA)		+= nvodm_accelerometer_kxtf9.o
obj-$(CONFIG_TEGRA_ODM_OLYMPUS) += nvodm_accelerometer_adi340.o
obj-$(CONFIG_TEGRA_ODM_ETNA) += nvodm_accelerometer_adi340.o

obj-$(CONFIG_TEGRA_ODM_DAYTONA) += nvodm_accelerometer_adi340.o
obj-$(CONFIG_TEGRA_ODM_SUNFIRE) += nvodm_accelerometer_adi340.o
