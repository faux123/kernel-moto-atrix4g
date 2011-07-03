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

/* Necessary includes for device drivers */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <linux/uaccess.h>	/* copy_from/to_user */
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/io.h>

#define SIZE_OF_MIO		    4
#define SIZE_OF_MODEL_ID	4
#define SIZE_OF_PROD_ID		16
#define SIZE_OF_SER_NO		10
#define SIZE_OF_PROC_UID	16
#define SIZE_OF_SBK     	16

#define LARGER_OF_TWO (n, m) (n > m ? n : m)
#define LARGER_OF_THREE (x, y, z) \
	(x > (LARGER_OF_TWO(y, z)) ? x : LARGER_OF_TWO(y, z))

#define API_HAL_KM_SOFTWAREREVISION_READ          33

#define FLAG_IRQFIQ_MASK            0x3
#define FLAG_START_HAL_CRITICAL     0x4
#define FLAG_IRQ_ENABLE             0x2
#define FLAG_FIQ_ENABLE             0x1

#define SMICODEPUB_IRQ_END   0xFE
#define SMICODEPUB_FIQ_END   0xFD
#define SMICODEPUB_RPC_END   0xFC

#define REGISTER_ADDRESS_DIE_ID 	0x4830A218
#define REGISTER_ADDRESS_MSV		0x480023B4

#define SEC_IOCTL_MODEL			_IOWR(0x99, 100, int*)
#define SEC_IOCTL_MIO			_IOWR(0x99, 101, int*)
#define SEC_IOCTL_PROC_ID		_IOWR(0x99, 102, int*)
#define SEC_IOCTL_EFUSE_RAISE	_IOWR(0x99, 103, int*)
#define SEC_IOCTL_EFUSE_LOWER	_IOWR(0x99, 104, int*)
#define SEC_IOCTL_MODELID_PROV	_IOWR(0x99, 105, int)
#define SEC_IOCTL_BS_DIS   		_IOWR(0x99, 106, int)
#define SEC_IOCTL_SERNO_ID  	_IOWR(0x99, 107, int*)
#define SEC_IOCTL_SBK         	_IOWR(0x99, 108, int*)
#define SEC_IOCTL_PRODUCTION	_IOWR(0x99, 109, int)

#define API_HAL_NB_MAX_SVC         39
#define API_HAL_MOT_EFUSE            (API_HAL_NB_MAX_SVC + 10)
#define API_HAL_MOT_EFUSE_READ       (API_HAL_NB_MAX_SVC + 15)

/*===========================*
   struct SEC_PA_PARAMS
*============================*/
typedef struct {
	unsigned int component;
	unsigned int efuse_value;
	unsigned int bch_value;
} SEC_PA_PARAMS;

/*============================
   enum SEC_SV_COMPONENT_T
*============================*/
typedef enum {
	/*Starting with random non zero value for component type */
	SEC_AP_PA_PPA = 0x00000065,
	SEC_BP_PPA,
	SEC_BP_PA,
	SEC_ML_PBRDL,
	SEC_MBM,
	SEC_RRDL_BRDL,
	SEC_BPL,
	SEC_AP_OS,
	SEC_BP_OS,
	SEC_BS_DIS,
	SEC_ENG,
	SEC_PROD,
	SEC_CUST_CODE,
	SEC_PKC,
	SEC_MODEL_ID,
	SEC_MAX
} SEC_SV_COMPONENT_T;

/*==================================================
   SEC_FUSE NV - must be the same as in SecPublic.h
====================================================*/
typedef enum
{
  SEC_NV_FUSE_START = 0,
  SEC_NV_FUSE_DK,
  SEC_NV_FUSE_JTAG_DIS,
  SEC_NV_FUSE_KEY_PROGRAMMED,
  SEC_NV_FUSE_ODM_PRODUCTION_MODE,
  SEC_NV_FUSE_BOOT_DEV_CFG,
  SEC_NV_FUSE_BOOT_DEV_SEL,
  SEC_NV_FUSE_SBK,
  SEC_NV_FUSE_SKU,
  SEC_NV_FUSE_SPARE_BITS,
  SEC_NV_FUSE_SW_RSRVD,
  SEC_NV_FUSE_SKIP_DS_STRAP,
  SEC_NV_FUSE_SECBOOT_DEVSEL_RAW,
  SEC_NV_FUSE_RSVD_ODM,
  SEC_NV_FUSE_END,
  
  SEC_NV_FUSE_IC_TYPE = 0x0F,
  SEC_NV_FUSE_MODEL_ID,
  SEC_NV_FUSE_SIG_ALLOWED

} SEC_NV_FUSES_T;

/*==================================================
   SEC_OP - operations supported by SEC_IOCTL_MIO
====================================================*/
typedef enum
{
  SEC_OP_FUSE        = 0,  // get fuse info; operation data corresponds to SEC_NV_FUSE enum above
  SEC_OP_GPIO_READ   = 1,   // read GPIO; operation data
  SEC_OP_GPIO_WRITE  = 2,  // write GPIO; operation data
  SEC_OP_FLAGS       = 3,  // get flags; operation data
} SEC_OPS_T;

/**** Data format passed to SecGetMIO **************
 * FUSE id corresponds to nvddk_fuse.h
 * GPIO port  [bits 9-14]    - 'A' to 'Z' or 0x5B=AA and 0x5C=BB
 * GPIO pin   [bits 4-7]     - 0-7
 * GPIO state [bit  8]       - 0/1
 * GPIO conf  [bits 15-23]   - TBD
 ***************************************************/
#define SEC_MASKED_FUSE(a)           (a & 0x0000000F)
#define SEC_MASKED_GPIO_PORT(a)     ((a & 0x0000FF00)>>8)
#define SEC_MASKED_GPIO_CFG(a)      ((a & 0x00FF0000)>>16)
#define SEC_MASKED_GPIO_PIN(a)      ((a & 0x00000070)>>4)
#define SEC_MASKED_GPIO_STATE(a)    ((a & 0x00000080)>>7)
#define SEC_MASKED_OP(a)            ((a & 0x0F000000)>>24)

/*===============================================
    struct SEC_DATA_T
================================================*/
typedef struct {
	unsigned char	model_id[SIZE_OF_MODEL_ID];	/* unit's model id */
	unsigned char	prod_id[SIZE_OF_PROD_ID];	/* unit's product id */
	unsigned char	proc_uid[SIZE_OF_PROC_UID];	/* proc's unique id */
	unsigned char	serial_no[SIZE_OF_SER_NO];	/* unit's serial number */
	unsigned char	is_in_factory,	/* in factory flag */
			        is_production,	/* production */
			        is_secure;	/* encryption keys entered */
	unsigned short	chipId,
			        chipSKU;
} SEC_DATA_T;

struct fti_container {
	unsigned char	pre_padding[21],
			        track_id[10],
			        post_padding[97];
};

/* Declaration of sec.c functions */
int sec_open(struct inode *inode, struct file *filp);
int sec_release(struct inode *inode, struct file *filp);
ssize_t sec_read(struct file *filp, char *buf, size_t count,
		loff_t *f_pos);
ssize_t sec_write(struct file *filp, char *buf, size_t count,
		loff_t *f_pos);
int sec_ioctl(struct inode *inode, struct file *file,
	      unsigned int ioctl_num, unsigned long ioctl_param);

extern u32 pub2sec_bridge_entry(u32 appl_id, u32 proc_ID, u32 flag,
				char *ptr);
extern u32 rpc_handler(u32 p1, u32 p2, u32 p3, u32 p4);
extern u32 v7_flush_kern_cache_all(void);

void sec_exit(void);

int sec_init(void);

static int __devinit sec_probe(struct platform_device *pdev);

static void SecGetModelId(void *);

static int SecGetMIO(void *, unsigned int);

static void SecGetProcUID(void *);

static void SecGetSerialID(void *);
