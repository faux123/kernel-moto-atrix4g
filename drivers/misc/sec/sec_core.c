/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/sec_linux.h>
//#include <mach/iomap.h>

#include "sec_core.h"
#include "sec_ks_external.h"

#define __file__	"sec_drv"

#define NV_MAX_FUSE_LEN		256

#define NV_DEBUG			    0
#undef  SYSFS_FUSE_VOLTAGE
#define SYSFS_RAM_INFO
#define NV_DDK_FUSE_API_AVAIL   1

#include "nvcommon.h"
#include "../nvrm/core/common/nvrm_chipid.h"
#include "../nvrm/core/common/nvrm_structure.h"
#include "../nvrm/core/ap20/ap20rm_misc_private.h"
#include "nvrm_init.h"
#ifdef NV_DDK_FUSE_API_AVAIL
#include "nvddk_fuse.h"
#endif

static int SecGetMIO(void *data, unsigned int parameter);
static int SecProvisionModelID(unsigned int model_id);
static int SecGPIO(unsigned int parameter);
static int SecBSDis(void);
static int SecBlowSBK(void *data);
static int SecBlowProduction(void);
static int SecFuseInit(void);
static int SecVfuseOff(void);
static int SecVfuseOn(void);
static int SecRaiseVfuse(void);
static int SecLowerVfuse(void);
static int SecVfuseRegulatorVoltage(void);

static void SecGetProcUID(void *data);
static void SecGetModelId(void *data);

static struct regulator *sec_efuse_regulator;

static uint32_t sec_debug=NV_DEBUG;

#define SEC_DBG(args...)	{sec_printk(sec_debug, args);}
#define SEC_INFO(args...)	SEC_DBG(args)
#define SEC_ERR(args...)	{sec_printk(0xff,args);}
#define SEC_MIN(a,b)		((a)>(b)?(a):(b))

#define SEC_FUSE_PROG_VOLTAGE	3300000	// uV

static void sec_printk (int dbg, char *fmt, ...)
{
	if ( dbg )
	{
		static va_list args;
		va_start(args, fmt);
		vprintk(fmt, args);
		va_end(args);
	}
}

#ifdef SYSFS_FUSE_VOLTAGE
#include <linux/sysfs.h>
#include <linux/kobject.h>

static int SecVfuseRegulatorEnabled(void)
{
	int mode=0;
	if (sec_efuse_regulator) {
		mode = regulator_is_enabled (sec_efuse_regulator);
	}
	return mode;
}

static struct kobject *sec_nvfuse_kobj;

static ssize_t sysfsfuse_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return ((ssize_t)sprintf (buf, "%s:%duV\n", 
			 SecVfuseRegulatorEnabled() ? "on" : "off", 
			 SecVfuseRegulatorVoltage() 
			));
}

static ssize_t sysfsfuse_store(struct kobject *kobj,
    struct kobj_attribute *attr, char *buf, size_t count)
{
	int state = -1;
	if (!strncmp (buf, "on", 2)) {
		state = 1;
	} else if (!strncmp (buf, "off", 3)) {
		state = 0;
	}
	if (state != -1) {
		if (! state && 		// turning regulator off kills USB, so we opt out here
			SecVfuseRegulatorEnabled ()) {
			//SecLowerVfuse();
			state = state;
		}
		if (state && 
			SecVfuseRegulatorEnabled () == 0) {	// turn on
			SecRaiseVfuse();
		}
	}
    return count;
}

static struct kobj_attribute sec_nvfuse_FuseVoltage_attribute =
    __ATTR(FuseVoltage, 0600, sysfsfuse_show, sysfsfuse_store);    
#endif

#ifdef SYSFS_RAM_INFO
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/apanic.h>
#include "nvrm_hardware_access.h"
#include "nvrm_module.h"
#include <ap20/aremc.h>

#define SEC_UNKNOWN         "unknown"
#define SEC_RAM_MAN_ELPIDA  "Elpida-"
#define SEC_RAM_MAN_HYNIX   "Hynix-"
#define SEC_RAM_MAN_MICRON  "Micron"
#define SEC_RAM_ELPIDA_40NM "40nm"
#define SEC_RAM_ELPIDA_50NM "50nm"
#define SEC_RAM_HYNIX_44NM  "44nm"
#define SEC_RAM_HYNIX_54NM  "54nm"
#define SEC_RAM_TYPE_DDR1   "DDR1"
#define SEC_RAM_TYPE_DDR2   "DDR2"
#define SEC_RAM_TYPE_LPDDR2 "LPDDR2"
#define SEC_RAM_IO_W16      "x16"
#define SEC_RAM_IO_W32      "x32"
#define SEC_RAM_S4          "S4"
#define SEC_RAM_S2          "S2"
#define SEC_RAM_N           "N"
#define SEC_RAM_128MB       0x8000000
#define SEC_RAM_512B_BLK    0x40000     // # of 512b blocks in 128MB chunk

static struct kobject *sec_ram_kobj;
static const char *sec_ram_type         = SEC_UNKNOWN;
static const char *sec_ram_manufacturer = SEC_UNKNOWN;
static const char *sec_ram_io           = SEC_UNKNOWN;
static const char *sec_ram_stype        = SEC_UNKNOWN;
static const char *sec_ram_lithography  = NULL;
static u32 sec_ram_size   =  0;
static u32 sec_ram_serial =  0;
static int sec_ram_manid  = -1;

#define SEC_NV_REGR(reg)         NV_REGR(pDev, NvRmPrivModuleID_ExternalMemoryController, 0, reg)
#define SEC_NV_REGW(reg,val)     NV_REGW(pDev, NvRmPrivModuleID_ExternalMemoryController, 0, reg, val)

static int ram_info_apanic_annotate(void);

static void smart_hw_write(NvRmDeviceHandle pDev, u32 value)
{
    int i = 0;
    u32 status;
    do {
        ++i;
        SEC_NV_REGR (EMC_MRR_0); // dummy read
        status = SEC_NV_REGR (EMC_EMC_STATUS_0) & EMC_EMC_STATUS_0_MRR_DIVLD_FIELD;
    } while (status);
    SEC_DBG ("%s: MRR write status counter %d\n",__file__, i);
    SEC_NV_REGW (EMC_MRR_0, value);
}

static u32 smart_hw_read(NvRmDeviceHandle pDev)
{
    int i = 0;
    u32 status;
    do {
        ++i;
        status = (SEC_NV_REGR (EMC_EMC_STATUS_0) >> EMC_EMC_STATUS_0_MRR_DIVLD_SHIFT) & 0x1;
    } while (status != 1);
    SEC_DBG ("%s: MRR read status counter %d\n",__file__, i);
    return SEC_NV_REGR (EMC_MRR_0);
}

static int get_ram_info(void)
{
	NvRmDeviceHandle pDev=NULL;
	if (NvRmOpen (&pDev, 0) != NvError_Success) {
		SEC_ERR ("%s: NvRmOpen() error\n",__file__);
		return -1;
	} else {
        u32  reg, width_1, width_2, density_1, density_2, stype;
        // read DRAM config register
        reg = SEC_NV_REGR (EMC_FBIO_CFG5_0);
        SEC_DBG ("%s: FBIO_CFG5 %x\n",__file__, reg);

        switch (reg&0x3) {
         case 1 : sec_ram_type = SEC_RAM_TYPE_DDR1; break;
         case 2 : sec_ram_type = SEC_RAM_TYPE_LPDDR2; break;
         case 3 : sec_ram_type = SEC_RAM_TYPE_DDR2; break;
         default: sec_ram_type = SEC_UNKNOWN;
        }

        // read Manufacturer ID
        smart_hw_write (pDev, (1<<30)|(5<<16));
        reg = smart_hw_read (pDev);
        SEC_DBG ("%s: MR5 read %x\n",__file__, reg);

        sec_ram_manid = (reg & 0xFF);

        switch (sec_ram_manid) {
         case 3  : sec_ram_manufacturer = SEC_RAM_MAN_ELPIDA;
                   // read lithography for Elpida
                   smart_hw_write (pDev, (1<<30)|(6<<16));
                   reg = smart_hw_read (pDev);
                   SEC_DBG ("%s: MR6 read %x\n",__file__, reg);
                   if ((reg & 0xFF))    // 1 for 40nm
                            sec_ram_lithography = SEC_RAM_ELPIDA_40NM;
                   else     sec_ram_lithography = SEC_RAM_ELPIDA_50NM;
                        break;
         case 6   : sec_ram_manufacturer = SEC_RAM_MAN_HYNIX; break;
         case 255 : sec_ram_manufacturer = SEC_RAM_MAN_MICRON; break;
         default: sec_ram_manufacturer = SEC_UNKNOWN;
        }

        // read I/O width and density
        smart_hw_write (pDev, (1<<30)|(8<<16));
        reg = smart_hw_read (pDev);
        SEC_DBG ("%s: MR8-1 read %x\n",__file__, reg);

        stype = (reg&0x3);
        width_1 = ((reg&(0x3<<6)) >> 6);  // x32 or x16

        // set S type
        switch (stype) {
         case 0 : sec_ram_stype = SEC_RAM_S4; break;
         case 1 : sec_ram_stype = SEC_RAM_S2; break;
         case 2 : sec_ram_stype = SEC_RAM_N; break;
         default: sec_ram_stype = SEC_UNKNOWN;
        }

        // set I/O width
        switch (width_1) {
         case 0 : sec_ram_io = SEC_RAM_IO_W32; break;
         case 1 : sec_ram_io = SEC_RAM_IO_W16; break;
         default: sec_ram_io = SEC_UNKNOWN;
        }

        // count size again
        sec_ram_size = 0;

        density_1 = ((reg&(0xF<<2)) >> 2);// 64Mb to 32Gb
        switch (density_1) {
         case 4 : sec_ram_size += 1; break; // 128MB chunks
         case 5 : sec_ram_size += 2; break;
         case 6 : sec_ram_size += 4; break;
         case 7 : sec_ram_size += 8; break;
        }

        // read I/O width and density
        smart_hw_write (pDev, (2<<30)|(8<<16));
        reg = smart_hw_read (pDev);
        SEC_DBG ("%s: MR8-2 read %x\n",__file__, reg);

        width_2 = ((reg&(0x3<<6)) >> 6);  // x32 or x16
        density_2 = ((reg&(0xF<<2)) >> 2);// 64Mb to 32Gb

        if (!((width_1 - width_2)|(density_1 - density_2))) {
            switch (density_2) {
             case 4 : sec_ram_size += !width_1 ? 1 : 3; break;
             case 5 : sec_ram_size += !width_1 ? 2 : 6; break;
             case 6 : sec_ram_size += !width_1 ? 4 : 12; break;
             case 7 : sec_ram_size += !width_1 ? 8 : 24; break;
            }
        } else
            SEC_ERR ("Ambiguous RAM configuration!!!");

        if (sec_ram_manid == 6)
            sec_ram_lithography = (sec_ram_size >= 8) ? SEC_RAM_HYNIX_54NM : SEC_RAM_HYNIX_44NM;

        // set RAM size
        SEC_DBG ("%s: RAM chunks %d, size 0x%x\n",__file__, sec_ram_size, sec_ram_size*SEC_RAM_128MB);
	}
	NvRmClose (pDev);
	return 0;
}

static ssize_t sysfsram_size_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return ((ssize_t)sprintf (buf, "%u\n", sec_ram_size*SEC_RAM_512B_BLK));
}

static ssize_t sysfsram_type_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return ((ssize_t)sprintf (buf, "%s\n", sec_ram_type));
}

static ssize_t sysfsram_serial_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return ((ssize_t)sprintf (buf, "0x%x\n", sec_ram_serial));
}

static ssize_t sysfsram_info_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return ((ssize_t)sprintf (buf, "%s%s:%s:%s:%s:%u%s\n",
            sec_ram_manufacturer,
            sec_ram_lithography ? sec_ram_lithography : "",
            sec_ram_type,
            sec_ram_io,
            sec_ram_stype,
            sec_ram_size >= 8 ? (sec_ram_size >> 3) : (sec_ram_size << 7),
            sec_ram_size >= 8 ? "GB" : "MB"));
}

static const struct kobj_attribute sec_ram_info_attribute =
    __ATTR(info, 0444, sysfsram_info_show, NULL);
static const struct kobj_attribute sec_ram_type_attribute =
    __ATTR(type, 0444, sysfsram_type_show, NULL);
static const struct kobj_attribute sec_ram_size_attribute =
    __ATTR(size, 0444, sysfsram_size_show, NULL);
static const struct kobj_attribute sec_ram_serial_attribute =
    __ATTR(serial, 0444, sysfsram_serial_show, NULL);

static int ram_info_apanic_annotate(void)
{
#ifdef CONFIG_APANIC_MMC
	char buf[80] = "RAM: ";

	sysfsram_info_show(NULL, NULL, buf + 5);
	return apanic_annotate(buf);
#else
	return 0;
#endif
}
#endif

#ifdef NV_DDK_FUSE_API_AVAIL
typedef enum
{
    TegraFuseSizeInBytes_DeviceKey = 4,
    TegraFuseSizeInBytes_JtagDisable = 1, // 1 bit
    TegraFuseSizeInBytes_KeyProgrammed = 1,
    TegraFuseSizeInBytes_OdmProduction = 1, // 1 bit
    TegraFuseSizeInBytes_SecBootDeviceConfig = 2, // 14 bits
    TegraFuseSizeInBytes_SecBootDeviceSelect = 1, // 3 bits
    TegraFuseSizeInBytes_SecureBootKey = 16,
    TegraFuseSizeInBytes_Sku = 4,
    TegraFuseSizeInBytes_SpareBits = 4,
    TegraFuseSizeInBytes_SwReserved = 1, // 4 bit
    TegraFuseSizeInBytes_SkipDevSelStraps = 1,  // 1 bit
    TegraFuseSizeInBytes_SecBootDeviceSelectRaw = 4,
    TegraFuseSizeInBytes_ReservedOdm = 32
} TegraFuseSizeInBytes;

static int GetFuseDataSize (NvDdkFuseDataType type, unsigned int* pSize)
{
    int ret_code = 0;
    switch (type)
    {
     case NvDdkFuseDataType_DeviceKey:
        *pSize = TegraFuseSizeInBytes_DeviceKey;
            break;
     case NvDdkFuseDataType_JtagDisable:
        *pSize = TegraFuseSizeInBytes_JtagDisable;
            break;
     case NvDdkFuseDataType_KeyProgrammed:
        *pSize = TegraFuseSizeInBytes_KeyProgrammed;
            break;
     case NvDdkFuseDataType_OdmProduction:
        *pSize = TegraFuseSizeInBytes_OdmProduction;
            break;
     case NvDdkFuseDataType_SecBootDeviceConfig:
        *pSize = TegraFuseSizeInBytes_SecBootDeviceConfig;
            break;
     case NvDdkFuseDataType_SecBootDeviceSelect:
        *pSize = TegraFuseSizeInBytes_SecBootDeviceSelect;
            break;
     case NvDdkFuseDataType_SecureBootKey:
        *pSize = TegraFuseSizeInBytes_SecureBootKey;
            break;
     case NvDdkFuseDataType_Sku:
        *pSize = TegraFuseSizeInBytes_Sku;
            break;
     case NvDdkFuseDataType_SpareBits:
        *pSize = TegraFuseSizeInBytes_SpareBits;
            break;
     case NvDdkFuseDataType_SwReserved:
        *pSize = TegraFuseSizeInBytes_SwReserved;
            break;
     case NvDdkFuseDataType_SkipDevSelStraps:
        *pSize = TegraFuseSizeInBytes_SkipDevSelStraps;
            break;
     case NvDdkFuseDataType_SecBootDeviceSelectRaw:
        *pSize = TegraFuseSizeInBytes_SecBootDeviceSelectRaw;
            break;
     case NvDdkFuseDataType_ReservedOdm:
        *pSize = TegraFuseSizeInBytes_ReservedOdm;
            break;
    default : ret_code = -1;
        SEC_ERR("Invalid Fuse type (%d) ... Find out reason\n", type);
    }
    return ret_code;
}

static NvError ReadByteFuseData (NvDdkFuseDataType type, unsigned char *Fuse)
{
    unsigned int size = 0;
    GetFuseDataSize (type, &size);
    NvDdkFuseSense ();
    NvDdkFuseClear ();
    return NvDdkFuseGet (type, Fuse, &size);
}
#endif

/* Structure that declares the usual file */
/* access functions */
const struct file_operations sec_fops = {
	.read = sec_read,
	.open = sec_open,
	.release = sec_release,
	.ioctl = sec_ioctl
	//.compat_ioctl = sec_ioctl
};

/* Mapping of the module init and exit functions */
module_init(sec_init);
module_exit(sec_exit);

static struct platform_driver sec_driver = {
	.probe = sec_probe,
	.driver = {
		   .name = "sec",
		   },
};

static struct miscdevice sec_dev = {
	MISC_DYNAMIC_MINOR,
	"sec",
	&sec_fops
};

/* These may be read from the command line, if they exist */
static char	*sec_serial_no;
static char	*sec_model_id;
static char	*sec_product_id;

SEC_DATA_T	*sec_data=NULL;

/******************************************************************************/
/*   KERNEL DRIVER APIs, ONLY IOCTL RESPONDS BACK TO USER SPACE REQUESTS      */
/******************************************************************************/
static int __init sec_serial_no_setup(char *options)
{
    sec_serial_no = options;
    pr_info("%s: sec_serial_no: %s\n", __func__, sec_serial_no);
    return 0;
}
__setup("androidboot.serialno=", sec_serial_no_setup);

int sec_init(void)
{
	return platform_driver_register(&sec_driver);
}

static int __devinit sec_probe(struct platform_device *pdev)
{
	struct sec_platform_data *sec_plat =
        	(struct sec_platform_data*)pdev->dev.platform_data;
	int result;
	NvU64	uId=0ULL;
	NvRmDeviceHandle pDev=NULL;

	result = misc_register(&sec_dev);
	if (result) {
		SEC_ERR ("%s: cannot obtain major number \n",__file__);
		return result;
	}

	sec_data = (SEC_DATA_T *)kzalloc (sizeof(SEC_DATA_T), GFP_KERNEL);
	if (! sec_data) {
		SEC_ERR ("%s: unable to allocate sec_data\n",__file__);
		return -ENOMEM;
	}

	if (NvRmOpen (&pDev, 0) != NvError_Success) {
		SEC_ERR ("%s: NvRmOpen() error\n",__file__);
	} else {
		sec_data->chipId = pDev->ChipId.Id;
		sec_data->chipSKU = pDev->ChipId.SKU; 
		SEC_DBG ("%s: chipId: %x, chipSku: %x\n",__file__, sec_data->chipId, sec_data->chipSKU);
	}
	if (NvRmPrivAp20ChipUniqueId (pDev, (void *)&uId) != NvError_Success) {
		SEC_ERR ("%s: NvRmPrivAp20ChipUniqueId() error\n",__file__);
	} else {
		NvU32	ub, lb;
		char buf[64];

		lb = uId & 0xFFFFFFFF; 
		ub = uId >> 32;
		memset (buf, 0, sizeof(buf));
		NvOsSnprintf(buf, sizeof(buf), "%08X%08X", ub, lb);
		memcpy (sec_data->proc_uid, buf, SEC_PROC_UID_SIZE);
		SEC_DBG ("%s: procUId: %s\n",__file__, buf);
	}
	NvRmClose (pDev);

#ifdef NV_DDK_FUSE_API_AVAIL
	if (1)
	{
        unsigned int size;
        unsigned char FuseBuff[32];

        NvDdkFuseSense ();
        NvDdkFuseClear ();
        
        size = TegraFuseSizeInBytes_OdmProduction;
        if (NvDdkFuseGet (NvDdkFuseDataType_OdmProduction, FuseBuff, &size) == NvSuccess) {      
	        sec_data->is_production = FuseBuff[0];
        }
        
        size = TegraFuseSizeInBytes_KeyProgrammed;
        if (NvDdkFuseGet (NvDdkFuseDataType_KeyProgrammed, FuseBuff, &size) == NvSuccess) {      
	        sec_data->is_secure = FuseBuff[0];
        }
#endif
    	sec_data->is_in_factory  = sec_plat->fl_factory ? 1 : 0;
	    SEC_DBG ("%s: prod: %x, secure: %x, in_factory: %x\n",__file__, 
    		sec_data->is_production, sec_data->is_secure, sec_data->is_in_factory);

    	if (sec_serial_no)
    		memcpy (sec_data->serial_no, sec_serial_no,
    			SEC_MIN(strlen (sec_serial_no), SIZE_OF_SER_NO));

    	if (sec_model_id)
    		memcpy (sec_data->model_id, sec_model_id,
    			SEC_MIN(strlen (sec_model_id), SIZE_OF_MODEL_ID));

    	if (sec_product_id)
    		memcpy (sec_data->prod_id, sec_product_id,
    			SEC_MIN(strlen (sec_product_id), SIZE_OF_PROD_ID));

    	// since having model id provisioned means indicates that phone is no more in initial factory state
    	//  let's fake it by initializing it to something when is_in_factory == 0
    	if (! sec_data->is_in_factory)
    		*((int *)&sec_data->model_id[0]) = 666;
#ifdef NV_DDK_FUSE_API_AVAIL
	}
#else
	sec_data->is_production  = sec_plat->fl_production ? 1 : 0;
	sec_data->is_secure      = sec_plat->fl_keys ? 1 : 0;
#endif

#ifdef SYSFS_FUSE_VOLTAGE
    sec_nvfuse_kobj = kobject_create_and_add ("fuse_pwr", firmware_kobj);
    sysfs_create_file (sec_nvfuse_kobj, &sec_nvfuse_FuseVoltage_attribute.attr);
#endif

#ifdef SYSFS_RAM_INFO
    get_ram_info ();
    sec_ram_kobj = kobject_create_and_add ("ram", NULL);
    sysfs_create_file (sec_ram_kobj, &sec_ram_info_attribute.attr);
    sysfs_create_file (sec_ram_kobj, &sec_ram_size_attribute.attr);
    sysfs_create_file (sec_ram_kobj, &sec_ram_type_attribute.attr);
    sysfs_create_file (sec_ram_kobj, &sec_ram_serial_attribute.attr);
    ram_info_apanic_annotate();
#endif
	// init regulator
	result = SecFuseInit();
	if (result != 0) {
		SEC_ERR ("%s: registration regulator framework failed:%d\n",
			     __file__, result);
	}
	return 0;
}

void sec_exit(void)
{
	/* Freeing the major number */
	misc_deregister(&sec_dev);
#ifdef SYSFS_RAM_INFO
    sysfs_remove_file (sec_ram_kobj, &sec_ram_info_attribute.attr);
    sysfs_remove_file (sec_ram_kobj, &sec_ram_size_attribute.attr);
    sysfs_remove_file (sec_ram_kobj, &sec_ram_type_attribute.attr);
    sysfs_remove_file (sec_ram_kobj, &sec_ram_serial_attribute.attr);
    kobject_del (sec_ram_kobj);
#endif
#ifdef SYSFS_FUSE_VOLTAGE
	sysfs_remove_file (sec_nvfuse_kobj, &sec_nvfuse_FuseVoltage_attribute.attr);
    kobject_del (sec_nvfuse_kobj);
#endif
}

int sec_open(struct inode *inode, struct file *filp)
{
	/* Not supported, return Success */
	return 0;
}

int sec_release(struct inode *inode, struct file *filp)
{
	/* Not supported, return Success */
	return 0;
}

ssize_t sec_read(struct file *filp, char *buf,
		 size_t count, loff_t *f_pos)
{
	/* Not supported, return Success */
	return 0;
}

ssize_t sec_write(struct file *filp, char *buf,
		  size_t count, loff_t *f_pos)
{
	/* Not supported, return Success */
	return 0;
}

int sec_ioctl(struct inode *inode, struct file *file,
	      unsigned int ioctl_num, unsigned long ioctl_param)
{
	unsigned long count = 0xDEADBEEF;
	void *sec_buffer = NULL;
	unsigned int parameter=0;
	int buffer_len, ret_val = 99;

	switch (ioctl_num) {
	case SEC_IOCTL_SBK:
		SEC_DBG ("%s: SBK ioctl\n",__file__);
		sec_buffer = kmalloc(SIZE_OF_SBK, GFP_KERNEL);
		if (sec_buffer != NULL) {
			count = 
			    copy_from_user ((void *)sec_buffer, 
					(void __user *) ioctl_param,
					SIZE_OF_SBK);
		    SEC_DBG ("%s: copy_from_user %d\n",__file__, count);
			if (count != 0) {
				count = 0; // no need to complain about a fault
				ret_val = EINVAL;
				break;
			}
		    SEC_DBG ("%s: calling SecBlowSBK\n",__file__);
			ret_val = SecBlowSBK(sec_buffer);
			kfree(sec_buffer);
		}

		break;

	case SEC_IOCTL_PRODUCTION:
		ret_val = SecBlowProduction();
        count = 0;
		break;

	case SEC_IOCTL_MODEL:
		sec_buffer = kmalloc(SIZE_OF_MODEL_ID, GFP_KERNEL);
		if (sec_buffer != NULL) {
			SecGetModelId(sec_buffer);
			count =
			    copy_to_user((void __user *) ioctl_param,
					 (const void *) sec_buffer,
					 SIZE_OF_MODEL_ID);
			kfree(sec_buffer);
			ret_val = 0;
		}

		break;

	case SEC_IOCTL_MODELID_PROV:
		ret_val = SecProvisionModelID(ioctl_param);

		break;

	case SEC_IOCTL_MIO:
		sec_buffer = kmalloc(SIZE_OF_MIO, GFP_KERNEL);
		if (sec_buffer != NULL) {
			count = 
			    copy_from_user ((void *)&parameter, 
					(void __user *) ioctl_param,
					sizeof(parameter));
			if (count != 0) {
				count = 0; // no need to complain about a fault
				ret_val = EINVAL;
				break;
			}
			buffer_len = SecGetMIO(sec_buffer, parameter);
			if (buffer_len > 0) {
    			count =
			        copy_to_user((void __user *) ioctl_param,
					    (const void *) sec_buffer,
					    buffer_len);
			    ret_val = 0;
			} else {
			    ret_val = -ENOSYS;
			}
			kfree(sec_buffer);
		}
		break;

	case SEC_IOCTL_PROC_ID:
		sec_buffer = kmalloc(SIZE_OF_PROC_UID, GFP_KERNEL);
		if (sec_buffer != NULL) {
			SecGetProcUID(sec_buffer);
			count =
			    copy_to_user((void __user *) ioctl_param,
					 (const void *) sec_buffer,
					 SIZE_OF_PROC_UID);
			kfree(sec_buffer);
			ret_val = 0;
		}
		break;

	case SEC_IOCTL_SERNO_ID:
		sec_buffer = kmalloc(SIZE_OF_SER_NO, GFP_KERNEL);
		if (sec_buffer != NULL) {
			SecGetSerialID(sec_buffer);
			count =
			    copy_to_user((void __user *) ioctl_param,
					 (const void *) sec_buffer,
					 SIZE_OF_SER_NO);
			kfree(sec_buffer);
			ret_val = 0;
		}
		break;

	case SEC_IOCTL_EFUSE_RAISE:
		count = SecRaiseVfuse();
		ret_val = 0;
		break;

	case SEC_IOCTL_EFUSE_LOWER:
		count = SecLowerVfuse();
		ret_val = 0;
		break;

	case SEC_IOCTL_BS_DIS:
		ret_val = SecBSDis();
		break;

	default:
		SEC_ERR ("%s: ioctl called with bad cmd: %d\n",__file__, ioctl_num);
		break;
	}

	if (count != 0) {
		SEC_ERR ("%s: ioctl operation: %d failed, \
			copy_to_user returned: 0x%lX\n",__file__, ioctl_num, count);
	}

	return ret_val;
}

static SEC_MODE_T SecICType(void)
{
	SEC_MODE_T mode=SEC_ENGINEERING;
#ifdef NV_DDK_FUSE_API_AVAIL
	if (SecIsUnitProduction ())
#else
	if (sec_data->is_production)
#endif
		mode = SEC_PRODUCTION;
	return mode;
}

static void SecGetModelId(void *data)
{
	if (sec_data->is_in_factory) {
		*((int *)data) = 0;
	} else {
		memset (data, 0, SIZE_OF_MODEL_ID);
		memcpy (data, sec_data->model_id, SIZE_OF_MODEL_ID);
	}
	return;
}

static void SecGetProcUID(void *data)
{
	memset (data, 0, SIZE_OF_PROC_UID);
	memcpy (data, sec_data->proc_uid, SIZE_OF_PROC_UID);
	return;
}

static void SecGetSerialID(void *data)
{
	memset (data, 0, SIZE_OF_SER_NO);
	memcpy (data, sec_data->serial_no, SIZE_OF_SER_NO);
	return;
}

static int SecGetMIO(void *data, unsigned int parameter)
{
	int buffer_len=0;
	
	switch (SEC_MASKED_OP(parameter)) {
	 case SEC_OP_FUSE :
	 
	    switch (SEC_MASKED_FUSE(parameter)) {
         case SEC_NV_FUSE_IC_TYPE : // check ODM production
		    *((unsigned int *)data) = (unsigned int)(SecICType() - SEC_MODE_BASE);
		    //SEC_DBG ("%s: IC type 0x%08x\n",__file__, *((unsigned int *)data));
		    buffer_len = sizeof (unsigned int);
			    break;
	     case SEC_NV_FUSE_KEY_PROGRAMMED :  // check secure vs. unsecure
	        *((unsigned int *)data) = (unsigned int)SecIsUnitSecure();
		    buffer_len = sizeof (unsigned int);
	            break;
	     case SEC_NV_FUSE_ODM_PRODUCTION_MODE :  // check ODM production
	        *((unsigned int *)data) = (unsigned int)SecIsUnitProduction();
		    buffer_len = sizeof (unsigned int);
	            break;
	     default:
	            break;
	    }
	        break;
	        
	 case SEC_OP_FLAGS : // flags
	    SecUnitGetFlags ((unsigned long *)data);
		buffer_len = sizeof (unsigned int);
	        break;
	        
	 case SEC_OP_GPIO_READ : // GPIO operations
	 case SEC_OP_GPIO_WRITE : // GPIO operations
	    *((unsigned int *)data) = (unsigned int)SecGPIO (parameter);
		buffer_len = sizeof (unsigned int);
	        break;
	        
	 default:
		SEC_ERR ("%s: unknown operation 0x%x\n",__file__, SEC_MASKED_OP(parameter));
	 
	}
	return buffer_len;
}

static int SecRaiseVfuse(void)
{
	int ret_value = 99;
	if (sec_efuse_regulator) {
		ret_value = SecVfuseOn();
		if (ret_value != 0)
			SEC_ERR ("%s: efuse voltage raising failed : %d\n",
				__file__, ret_value);
	}
	return 0;
}

static int SecLowerVfuse(void)
{
	int ret_value = 99;
	if (sec_efuse_regulator) {
		ret_value = SecVfuseOff();
		if (ret_value != 0)
			SEC_ERR ("%s: efuse voltage lowering failed : %d\n",
				__file__, ret_value);
	}
	return 0;
}

static int SecProvisionModelID(unsigned int model_id)
{
	return -ENOSYS;
}

#define GPIO_BANK(x)    ((x) >> 5)
#define GPIO_PORT(x)    (((x) >> 3) & 0x3)
#define GPIO_BIT(x)     ((x) & 0x7)
//#define GPIO_REG(x)     ((IO_TO_VIRT(TEGRA_GPIO_BASE) + GPIO_BANK(x)*0x80) +  GPIO_PORT(x)*4)
#define GPIO_REG(x)     ((IO_TO_VIRT(0x6000D000) + GPIO_BANK(x)*0x80) +  GPIO_PORT(x)*4)

#define GPIO_CNF(x)     (GPIO_REG(x) + 0x00)
#define GPIO_OE(x)      (GPIO_REG(x) + 0x10)
#define GPIO_OUT(x)     (GPIO_REG(x) + 0X20)
#define GPIO_IN(x)      (GPIO_REG(x) + 0x30)
#define GPIO_INT_STA(x) (GPIO_REG(x) + 0x40)
#define GPIO_INT_ENB(x) (GPIO_REG(x) + 0x50)
#define GPIO_INT_LVL(x) (GPIO_REG(x) + 0x60)
#define GPIO_MSK_OUT(x) (GPIO_REG(x) + 0X820)

static void tegra_gpio_mask_write(u32 reg, int pin, int value)
{
	u32 val;
	val = 0x100 << pin;
	if (value)
		val |= 1 << pin;
    SEC_DBG ("WRITE: reg=%x, value=%x\n", reg, val);
	__raw_writel(val, reg);
}

static u32 tegra_gpio_mask_read(u32 reg, int pin)
{
    u32 val = __raw_readl(reg);
    SEC_DBG ("READ: reg=%x, value=%x\n", reg, val);
	return ((val >> pin) & 0x1);
}

static u8 sec_gpio_requested[ARCH_NR_GPIOS]={0};

static int SecGPIO(unsigned int parameter)
{
	int ret_val = 99;
	u32 val;
	int gpio_id, gpio, offset, bank, slot;
	int state, port, pin;
	
	state = SEC_MASKED_GPIO_STATE(parameter);
	port = SEC_MASKED_GPIO_PORT(parameter);
	pin = SEC_MASKED_GPIO_PIN(parameter);
	SEC_DBG ("state=%x, tegra-gpio: %c%d\n", state, port, pin);
	
	if (port < 0x41 || port > 0x5C || pin < 0 || pin > 7)
	    return ret_val;
	
	for (bank=0; bank < 7; bank++)
	{
	    offset = bank*4 + 'A';
	    slot = port - offset;
	    if (slot < 4)
	        break;
	}
	SEC_DBG ("bank=%x, port=%x, pin=%x\n", bank, slot, pin);
	
	gpio = ((bank << 5) | ((slot & 0x3) << 3));
	val = tegra_gpio_mask_read(GPIO_CNF(gpio), pin);  
	if (val)   // check if pin is GPIO
	{
 	    SEC_DBG ("gpio/pin: %x[%x] is GPIO\n", gpio, pin);
        val = tegra_gpio_mask_read(GPIO_OE(gpio), pin);
        if (val)   // GPIO is configured as output
        {
 	        SEC_DBG ("gpio/pin: %x[%x] configured as OUT\n", gpio, pin);
    	    if (SEC_MASKED_OP(parameter) == SEC_OP_GPIO_WRITE)
    	    {
    	        tegra_gpio_mask_write(GPIO_MSK_OUT(gpio), pin, state);
	            ret_val = 0;
    	    }
	        if (SEC_MASKED_OP(parameter) == SEC_OP_GPIO_READ)
	        {
	            ret_val = tegra_gpio_mask_read(GPIO_OUT(gpio), pin);
	        }
    	} else    // GPIO configured as input
    	{
 	        SEC_DBG ("gpio/pin: %x[%x] configured as IN\n", gpio, pin);
    	    val = tegra_gpio_mask_read(GPIO_INT_ENB(gpio), pin);
    	    if (1) // (val) GPIO_INT_ENB does not seem to be working ... talking to NV
    	    {
    	        u32 lvl = __raw_readl(GPIO_INT_LVL(gpio));
    	        int edge = 0, delta = 0;
    	        const char *activation, *triggering;
    	        edge = lvl & (0x100 << pin);     // 1-edge, 0-level
    	        delta = lvl & (0x10000 << pin);    // 1-any
    	        activation = edge ? "edge" : "level";
    	        if (edge) {
    	            if (delta)
    	                    triggering = "both";
    	            else    triggering = (lvl & (0x1 << pin)) ? "raising" : "falling";
    	        } else    
    	            triggering = (lvl & (0x1 << pin)) ? "high" : "low";
    	        SEC_DBG ("gpio/pin: %x[%x] configured as %s-%s IRQ\n", gpio, pin,
    	                activation, triggering);
    	    }
    	    if (SEC_MASKED_OP(parameter) == SEC_OP_GPIO_READ)
    	    {
    	        ret_val = tegra_gpio_mask_read(GPIO_IN(gpio), pin);
    	    }
        }
	} else 
	{
	    SEC_DBG ("%s: gpio/pin: %x[%x] is SFIO\n", __file__, gpio, pin);
	    if (SEC_MASKED_OP(parameter) == SEC_OP_GPIO_READ)
	    {
	        SEC_DBG ("%s: re-configuring gpio/pin: %x[%x] for reading\n", __file__, gpio, pin);
	        gpio_id = gpio | (pin & 0x7);
	        // sec driver must request GPIO
            if (gpio_is_valid (gpio_id) && ! sec_gpio_requested[gpio_id])
                if(! gpio_request (gpio_id, "sec_gpio_in"))
                {
                    sec_gpio_requested[gpio_id] = 1;
	                SEC_DBG ("%s: gpio/pin: %x[%x] GPIO-%d has been requested\n", __file__, gpio, pin, gpio_id);
                }

            // make sure GPIO has been requested by sec driver
            if (sec_gpio_requested[gpio_id])
            {
                gpio_direction_input (gpio_id);
                ret_val = gpio_get_value (gpio_id);
	            SEC_DBG ("%s: gpio/pin: %x[%x] GPIO-%d configured as input, value is %d\n", __file__, gpio, pin, gpio_id, ret_val);
            }            
	    } 
	    else if (SEC_MASKED_OP(parameter) == SEC_OP_GPIO_WRITE)
	    {
	        SEC_DBG ("%s: re-configuring gpio/pin: %x[%x] for writing\n", __file__, gpio, pin);
	        gpio_id = gpio | (pin & 0x7);
	        // sec driver must request GPIO
            if (gpio_is_valid (gpio_id) && ! sec_gpio_requested[gpio_id])
                if (! gpio_request (gpio_id, "sec_gpio_out"))
                {
                    sec_gpio_requested[gpio_id] = 1;
	                SEC_DBG ("%s: gpio/pin: %x[%x] GPIO-%d has been requested\n", __file__, gpio, pin, gpio_id);
                }

            // make sure GPIO has been requested by sec driver
            if (sec_gpio_requested[gpio_id])
            {
                gpio_direction_output (gpio_id, 0);
                gpio_set_value (gpio_id, state);
                ret_val = 0;
	            SEC_DBG ("%s: gpio/pin: %x[%x] GPIO-%d configured as output, value set to %d\n", __file__, gpio, pin, gpio_id, state);
            }
	    }
	}
	return ret_val;
}

static int SecBlowSBK(void *data)
{
	int ret_value = 99;
	if (sec_efuse_regulator) {
	    int voltage = SecVfuseRegulatorVoltage();
		if (voltage != SEC_FUSE_PROG_VOLTAGE) {
			SEC_ERR ("%s: incorrect fuse voltage: %d\n",
				__file__, ret_value);
		} else 
		{
#ifdef NV_DDK_FUSE_API_AVAIL
		    unsigned char Fuse = 1;
		    unsigned int size = TegraFuseSizeInBytes_OdmProduction;
		    NvError Err;
		    
		    Err = ReadByteFuseData (NvDdkFuseDataType_OdmProduction, &Fuse);
		    if (Err == NvSuccess && Fuse)
		    {
		        SEC_ERR ("%s: Production already!\n", __file__);
		        return ret_value;
		    }
		    size = TegraFuseSizeInBytes_SecureBootKey;
		    Err = ReadByteFuseData (NvDdkFuseDataType_KeyProgrammed, &Fuse);
		    if (Err == NvSuccess && Fuse)
		    {
		        SEC_ERR ("%s: SBK has been programmed already!\n", __file__);
		        return ret_value;
		    }
		    // program SBK fuses here
            NvDdkFuseClear ();
            Err = NvDdkFuseSet (NvDdkFuseDataType_SecureBootKey, data, &size);
            if (Err == NvSuccess)
            {
                NvDdkFuseProgram ();
                NvDdkFuseSense ();
                NvDdkFuseVerify ();
    		    ret_value = 0;
            } else
                SEC_ERR ("%s: SBK programming failed with error 0x%x\n", __file__, Err);
#else
		    SEC_ERR ("%s: SBK programming API unavailable!\n", __file__);
#endif
   		}		    
	}
	return ret_value;
}

static NvError SecProgramSingleFuse(NvDdkFuseDataType Type, unsigned char *Fuse, unsigned int *Size)
{
    NvError Err;
    NvDdkFuseClear ();
    Err = NvDdkFuseSet (Type, Fuse, Size);
    if (Err == NvSuccess)
    {
        NvDdkFuseProgram ();
        NvDdkFuseSense ();
        Err = NvDdkFuseVerify ();
    } else
        SEC_ERR ("%s: fuse 0x%x programming failed with error 0x%x\n", __file__, Type, Err);
    return Err;
}

static int SecBlowProduction(void)
{
	int ret_value = 99;
	if (sec_efuse_regulator) {
	    int voltage = SecVfuseRegulatorVoltage();
		if (voltage != SEC_FUSE_PROG_VOLTAGE) {
			SEC_ERR ("%s: incorrect fuse voltage: %d\n",
				__file__, ret_value);
		} else 
		{
#ifdef NV_DDK_FUSE_API_AVAIL
		    unsigned char bootFuse[2], strapFuse;
		    unsigned int  size;
		    NvError Err;
		    
		    ReadByteFuseData (NvDdkFuseDataType_SecBootDeviceConfig, bootFuse);
		    ReadByteFuseData (NvDdkFuseDataType_SkipDevSelStraps, &strapFuse);
	        SEC_DBG ("%s: current SecBootDeviceConfig value [%x,%x], SkipDevSelStraps [%x]\n", __file__,
	                                bootFuse[0], bootFuse[1], strapFuse);
	        if ((bootFuse[0] | bootFuse[1] | strapFuse) == 0)   // fail if current values are not 0
	        {
		        bootFuse[0] = 0x1D; // program eMMC Boot mode, low voltage, 8-bit fuses here
		        size = TegraFuseSizeInBytes_SecBootDeviceConfig;
		        Err = SecProgramSingleFuse (NvDdkFuseDataType_SecBootDeviceConfig, bootFuse, &size);
		        // If programming of SecBootDeviceConfig failed, then to avoid bricking
		        //   a phone just refrain from blowing SkipDevSelStraps
                if (Err == NvSuccess)
                {
		            strapFuse = 0x01;   // use fuses to select boot device
		            size = TegraFuseSizeInBytes_SkipDevSelStraps;
                    Err = SecProgramSingleFuse (NvDdkFuseDataType_SkipDevSelStraps, &strapFuse, &size);
                }
                if (Err == NvSuccess)
                {
                    ret_value = 0;
                } else
                {
                    SEC_ERR ("%s: production fuses programming failed\n", __file__);
                }
            } else
            {
		        SEC_ERR ("%s: production fuses programming failed due to wrong initial fuse state\n", __file__);
            }
#else
		    SEC_ERR ("%s: production fuses programming API unavailable!\n", __file__);
#endif
   		}		    
	}
	return ret_value;
}

static int SecBSDis(void)
{
	return -ENOSYS;
}

static int SecFuseInit(void)
{
	int ret_value = 99;
	struct regulator *reg;
	reg = regulator_get(NULL, "vwlan2");
	if (reg) {
		sec_efuse_regulator = reg;
		ret_value = 0;  // indicate a success
		SEC_DBG ("successfully init-ed vwlan2\n");
	}
	return ret_value;
}

static int SecVfuseRegulatorVoltage(void)
{
	int voltage=-1;
	if (sec_efuse_regulator) {
	    if (regulator_is_enabled (sec_efuse_regulator)) {
		    voltage = regulator_get_voltage (sec_efuse_regulator);
		    SEC_DBG ("vwlan2 voltage is %d\n", voltage);
		} else
		    SEC_ERR ("%s: regulator vwlan2 is disabled\n", __file__);
	}
	return voltage;
}

static int SecVfuseOn(void)
{
	int ret_value = 99;
	if (sec_efuse_regulator) {
		SEC_DBG ("setting vwlan2 voltage...\n");
		regulator_set_voltage (sec_efuse_regulator, SEC_FUSE_PROG_VOLTAGE, 
													SEC_FUSE_PROG_VOLTAGE);
		msleep (100);
		SEC_DBG ("verifying vwlan2 voltage...\n");
		if (regulator_get_voltage (sec_efuse_regulator) != 
			SEC_FUSE_PROG_VOLTAGE)	// make sure requred voltage is setup
        {
       		SEC_ERR ("%s: incorrect vwlan2 voltage...\n", __file__);
			return ret_value;
	    }
		SEC_DBG ("correct voltage - enabling vwlan2...\n");
		// do not enable regulator until NV fixes ODM kit regulator API
		//ret_value = regulator_enable (sec_efuse_regulator);
	}
	return ret_value;
}

static int SecVfuseOff(void)
{
	int ret_value = 99;
	if (sec_efuse_regulator && 
		regulator_is_enabled (sec_efuse_regulator)) {
		// do not disable regulator until NV fixes ODM kit regulator API
		//ret_value = regulator_disable (sec_efuse_regulator);
		SEC_DBG ("disabling vwlan2...\n");
	}
	return ret_value;
}

/******************************************************************************/
/*Functions exported to Kernel for other kernel services                      */
/******************************************************************************/
SEC_STAT_T SecProcessorUID(unsigned char *buffer, int length)
{
	if (length < SIZE_OF_PROC_UID)
		return SEC_FAIL;
	memset (buffer, 0, length);
	memcpy (buffer, sec_data->proc_uid, SIZE_OF_PROC_UID);
	return SEC_SUCCESS;
}

EXPORT_SYMBOL(SecProcessorUID);

SEC_STAT_T SecSerialID(unsigned char *buffer, int length)
{
	if (length < SIZE_OF_SER_NO)
		return SEC_FAIL;
	memset (buffer, 0, length);
	memcpy (buffer, sec_data->serial_no, SIZE_OF_SER_NO);
	return SEC_SUCCESS;
}

EXPORT_SYMBOL(SecSerialID);

SEC_STAT_T SecModelID(unsigned char *buffer, int length)
{
	if (length < SIZE_OF_MODEL_ID)
		return SEC_FAIL;
	memset (buffer, 0, length);
	memcpy (buffer, sec_data->model_id, SIZE_OF_MODEL_ID);
	return SEC_SUCCESS;

}

EXPORT_SYMBOL(SecModelID);

SEC_STAT_T SecProductID(unsigned char *buffer, int length)
{
	if (length < SIZE_OF_PROD_ID)
		return SEC_FAIL;
	memset (buffer, 0, length);
	memcpy (buffer, sec_data->prod_id, SIZE_OF_PROD_ID);
	return SEC_SUCCESS;

}

EXPORT_SYMBOL(SecProductID);


SEC_MODE_T SecProcessorType(void)
{
    return SecICType();
}

EXPORT_SYMBOL(SecProcessorType);

int SecIsUnitSecure (void)
{
#ifdef NV_DDK_FUSE_API_AVAIL
    int ret_val = 0;
    unsigned char Fuse;
    if (ReadByteFuseData (NvDdkFuseDataType_KeyProgrammed, &Fuse) == NvSuccess) {
        ret_val = (int)Fuse;
    }
    return ret_val;
#else
    return sec_data->is_secure;
#endif
}

EXPORT_SYMBOL(SecIsUnitSecure);

int SecIsUnitProduction (void)
{
#ifdef NV_DDK_FUSE_API_AVAIL
    int ret_val = 0;
    unsigned char Fuse;
    if (ReadByteFuseData (NvDdkFuseDataType_OdmProduction, &Fuse) == NvSuccess) {
        ret_val = (int)Fuse;
    }
    return ret_val;
#else
    return sec_data->is_production;
#endif
}

EXPORT_SYMBOL(SecIsUnitProduction);

int SecUnitGetFlags (unsigned long *flags)
{
    int retval=-1;
    if (flags) {
	    *flags = 0UL;
#ifdef NV_DDK_FUSE_API_AVAIL
	    if (SecIsUnitProduction ())
#else
	    if (sec_data->is_production)
#endif
	    		*flags |= SEC_FLAG_PRODUCTION;
	    else	*flags |= SEC_FLAG_ENGINEERING;
#ifdef NV_DDK_FUSE_API_AVAIL
	    if (SecIsUnitSecure ())
#else
	    if (sec_data->is_secure)
#endif
		    *flags |= SEC_FLAG_SECURE;
		if (sec_data->is_in_factory)
		    *flags |= SEC_FLAG_INFACTORY;
	    retval = 0;
    }
    return retval;	
}

EXPORT_SYMBOL(SecUnitGetFlags);

/******************************************************************************/
/*Kernel Module License Information                                           */
/******************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MOTOROLA");
