#
# Makefile for regulator drivers.
#


obj-$(CONFIG_REGULATOR) += core.o
obj-$(CONFIG_REGULATOR_FIXED_VOLTAGE) += fixed.o
obj-$(CONFIG_REGULATOR_VIRTUAL_CONSUMER) += virtual.o
obj-$(CONFIG_REGULATOR_USERSPACE_CONSUMER) += userspace-consumer.o

obj-$(CONFIG_REGULATOR_BQ24022) += bq24022.o
obj-$(CONFIG_REGULATOR_LP3971) += lp3971.o
obj-$(CONFIG_REGULATOR_MAX1586) += max1586.o
obj-$(CONFIG_REGULATOR_TEGRA) += tegra-regulator.o
obj-$(CONFIG_REGULATOR_TWL4030) += twl4030-regulator.o
obj-$(CONFIG_REGULATOR_WM831X) += wm831x-dcdc.o
obj-$(CONFIG_REGULATOR_WM831X) += wm831x-isink.o
obj-$(CONFIG_REGULATOR_WM831X) += wm831x-ldo.o
obj-$(CONFIG_REGULATOR_WM8350) += wm8350-regulator.o
obj-$(CONFIG_REGULATOR_WM8400) += wm8400-regulator.o
obj-$(CONFIG_REGULATOR_DA903X)	+= da903x.o
obj-$(CONFIG_REGULATOR_PCF50633) += pcf50633-regulator.o
obj-$(CONFIG_REGULATOR_PCAP) += pcap-regulator.o
obj-$(CONFIG_REGULATOR_MC13783) += mc13783.o
obj-$(CONFIG_REGULATOR_AB3100) += ab3100.o
obj-$(CONFIG_REGULATOR_CPCAP) += cpcap-regulator.o

obj-$(CONFIG_REGULATOR_TPS65023) += tps65023-regulator.o
obj-$(CONFIG_REGULATOR_TPS6507X) += tps6507x-regulator.o

ccflags-$(CONFIG_REGULATOR_DEBUG) += -DDEBUG
