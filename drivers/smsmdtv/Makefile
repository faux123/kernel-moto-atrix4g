# For build-in
# with the flag on is build for P0 mortable sms1186 chip, otherwise for P1 sms2186 chip
#EXTRA_CFLAGS += -DSMS1186_MORTABLE_BOARD
SMSOBJ := smschar.o sms-cards.o smsendian.o smsspilog.o smsspicommon.o smsspiphy_ap20.o

EXTRA_CFLAGS += -DSMS_HOSTLIB_SUBSYS
EXTRA_CFLAGS += -DSMS_SPI_NVIDIA_AP20_DRV
SMSOBJ += smscoreapi.o

smsmdtv-objs := $(SMSOBJ)

obj-y := smsmdtv.o

clean:
	rm -rf *.o .*.cmd modules.order *.mod.c



# For module build
# with the flag on is build for P0 mortable sms1186 chip, otherwise for P1 sms2186 chip
#EXTRA_CFLAGS += -DSMS1186_MORTABLE_BOARD

#SMSOBJ := smschar.o sms-cards.o smsendian.o smsspilog.o smsspicommon.o smsspiphy_ap20.o

#EXTRA_CFLAGS += -DSMS_HOSTLIB_SUBSYS
#EXTRA_CFLAGS += -DSMS_SPI_NVIDIA_AP20_DRV
#SMSOBJ += smscoreapi.o

#smsmdtv-objs := $(SMSOBJ)

#obj-m := smsmdtv.o

#clean:
#	rm -rf *.o .*.cmd modules.order *.ko *.mod.c

