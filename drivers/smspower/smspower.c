#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <mach/pinmux.h>

#define LP3907_I2C_SLAVE_ADDR   (0x61)
#define MTV_VREG_EN	145	// TEGRA_GPIO_PS1

/* e13386, add suspend counter reader */
#define SMS_POWER_GET_SUSPEND_COUNTER     _IOR('K', 9, int)
int sms_suspend_count;

// switch between P0 borad(sms1186) and P1/2 and later HW(sms2186)
#ifdef  SMS1186_MORTABLE_BOARD
	#define MTV_PWDN	204	// TEGRA_GPIO_PZ4
	#define MTV_RST_N	94	// TEGRA_GPIO_PL6
#else
	#define MTV_PWDN	94	// TEGRA_GPIO_PL6
#endif

extern unsigned int u_msgres_count;
extern int sms2186_irq_number;
extern int sms2186_int_counter;
int sms2186_irq_enabled = 0;

static struct i2c_adapter* smsmdtv_adap;

typedef enum {
	/* Reg Name		Reg Addr */
	LP3907_ICRA_REG		= 0x02,  // READ ONLY
	LP3907_SCR1_REG         = 0x07,
	LP3907_BKLDOEN_REG      = 0x10,
	LP3907_BKLDOSR_REG      = 0x11,	 // READ ONLY
	LP3907_VCCR_REG         = 0x20,
	LP3907_B1TV1_REG        = 0x23,
	LP3907_B1TV2_REG        = 0x24,
	LP3907_B1RC_REG         = 0x25,
	LP3907_B2TV1_REG        = 0x29,
	LP3907_B2TV2_REG        = 0x2A,	
	LP3907_B2RC_REG         = 0x2B,
	LP3907_BFCR_REG         = 0x38,
	LP3907_LDO1VCR_REG      = 0x39,
	LP3907_LDO2VCR_REG      = 0x3A,
	LP3907_REG_MAX          = LP3907_LDO2VCR_REG,
	LP3907_INVALID_REG      = LP3907_REG_MAX + 1,
} LP3907_REG;

typedef struct {
	LP3907_REG reg;
	u8	   data;
} LP3907_CFG;

#ifdef SMS1186_MORTABLE_BOARD	// For Arowana_TD_2md_mortable_PCB with SMS1186
static LP3907_CFG lp3907_powerup_tbl[] = {
	{LP3907_SCR1_REG,    0x28,},
	{LP3907_BKLDOEN_REG, 0x71,},	/* Enable BUCK1 & LDO1 & LDO2 */
	{LP3907_VCCR_REG,    0x00,},
	{LP3907_B1TV1_REG,   0x09,},	/* BUCK1 - 1.2V */
	{LP3907_B1RC_REG,    0x28,},
	{LP3907_LDO1VCR_REG, 0x02,},	/* LDO1  - 1.2V */
	{LP3907_LDO2VCR_REG, 0x08,},	/* LDO2  - 1.8V */
	/* must end invalid register */
	{LP3907_INVALID_REG, 0x00,},
};

static LP3907_CFG lp3907_powerdown_tbl[] = {
	{LP3907_SCR1_REG,    0x28,},
	{LP3907_BKLDOEN_REG, 0x20,},	/* Disable all output */
	{LP3907_VCCR_REG,    0x00,},
	{LP3907_B1TV1_REG,   0x09,},	/* BUCK1 - 1.2V */
	{LP3907_B1RC_REG,    0x28,},
	{LP3907_LDO1VCR_REG, 0x02,},	/* LDO1  - 1.2V */
	{LP3907_LDO2VCR_REG, 0x08,},	/* LDO2  - 1.8V */
	/* must end invalid register */
	{LP3907_INVALID_REG, 0x00,},
};
#else	// For P1/2 HW with SMS2186
static LP3907_CFG lp3907_powerup_tbl[] = {
	{LP3907_SCR1_REG,    0x28,},
	{LP3907_BKLDOEN_REG, 0x61,},	/* Enable BUCK1 & LDO2 */
	{LP3907_VCCR_REG,    0x00,},
	{LP3907_B1TV1_REG,   0x09,},	/* BUCK1 - 1.2V */
	{LP3907_B1RC_REG,    0x28,},
	{LP3907_LDO2VCR_REG, 0x08,},	/* LDO2  - 1.8V */
	/* must end invalid register */
	{LP3907_INVALID_REG, 0x00,},
};

static LP3907_CFG lp3907_powerdown_tbl[] = {
	{LP3907_SCR1_REG,    0x28,},
	{LP3907_BKLDOEN_REG, 0x20,},	/* Disable all output */
	{LP3907_VCCR_REG,    0x00,},
	{LP3907_B1TV1_REG,   0x09,},	/* BUCK1 - 1.2V */
	{LP3907_B1RC_REG,    0x28,},
	{LP3907_LDO2VCR_REG, 0x08,},	/* LDO2  - 1.8V */
	/* must end invalid register */
	{LP3907_INVALID_REG, 0x00,},
};
#endif


#ifdef CONFIG_PROC_FS

static long is_sms_power_on = 0;

static int lp3907_i2c_write(u8 reg_addr, u8 data)
{
	u8 value[2];
	struct i2c_msg msg;

	value[0] = reg_addr;
	value[1] = data;

	msg.addr = LP3907_I2C_SLAVE_ADDR;
	msg.flags = 0;		
	msg.buf = value;
	msg.len = sizeof(value);

	return i2c_transfer(smsmdtv_adap, &msg, 1);
}

int smsmdtv_power_up(void)
{
	int i = 0;
	int ret = -1;

	smsmdtv_adap = i2c_get_adapter(3);
	if(smsmdtv_adap == 0)
	{
		printk("Cann't get I2C asapter\n");
		return ret;
	}
	
	//=================add for config SPI_CS pin using pinmux function===================
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LM0, TEGRA_PUPD_PULL_DOWN);

#ifdef  SMS1186_MORTABLE_BOARD
	gpio_set_value(MTV_RST_N, 0);
	printk("MTV_RST_N state: %d\n", gpio_get_value(MTV_RST_N));
#endif
	gpio_set_value(MTV_PWDN, 0);
	printk("MTV_PWDN state: %d\n", gpio_get_value(MTV_PWDN));
	msleep(10);

	printk("Write LP3907 reg to enable & set SW1=1.2V, LDO2=1.8V.\n");
	for(i = 0; lp3907_powerup_tbl[i].reg != LP3907_INVALID_REG; i++)
	{
		printk("Write data 0x%2x to LP3907 reg 0x%2x\n", lp3907_powerup_tbl[i].data, lp3907_powerup_tbl[i].reg);
		ret = lp3907_i2c_write(lp3907_powerup_tbl[i].reg, lp3907_powerup_tbl[i].data);
	}

	gpio_set_value(MTV_VREG_EN, 1);
	printk("MTV_VREG_EN enable state: %d\n", gpio_get_value(MTV_VREG_EN));
	printk("Delay 10ms\n");
	msleep(10);	// for sms2186 T1 >= 200usec, for sms1186 T1 >= 10usec

	gpio_set_value(MTV_PWDN, 1);
	printk("MTV_PWDN enable state: %d\n", gpio_get_value(MTV_PWDN));

#ifdef  SMS1186_MORTABLE_BOARD
	printk("Delay 25ms\n");
	msleep(25);	// for sms1186 T2 at least 20msec
#else
	printk("Delay 110ms\n");
	msleep(110);	// for sms2186 T2 at least 100msec
#endif

#ifdef  SMS1186_MORTABLE_BOARD
	gpio_set_value(MTV_RST_N, 1);
	printk("MTV_RST_N enable state: %d\n", gpio_get_value(MTV_RST_N));
	printk("Delay 30msec\n");
	msleep(30);	// for sms1186 T3 at least 25msec
#endif

	if(sms2186_irq_enabled == 0)
	{
		enable_irq(sms2186_irq_number);
		sms2186_irq_enabled = 1;
		printk("sms2186 interupt enabled\n");
	}
	sms2186_int_counter = 0;
	u_msgres_count = 0;

	return ret;
}

int smsmdtv_power_down(void)
{
	int i, ret = -1;
	if(sms2186_irq_enabled == 1)
	{
		disable_irq(sms2186_irq_number);
		sms2186_irq_enabled = 0;
		printk("sms2186 interrupt disabled\n");
	}

#ifdef  SMS1186_MORTABLE_BOARD
	gpio_set_value(MTV_RST_N, 0);
	printk("MTV_RST_N enable state: %d\n", gpio_get_value(MTV_RST_N));
#endif
	gpio_set_value(MTV_PWDN, 0);
	printk("MTV_PWDN enable state: %d\n", gpio_get_value(MTV_PWDN));
	msleep(10);
	printk("Delayed 10msec\n");
/*
// ==================================deleted when stable==================================
// ---------------------------------------------------------------------------------------
#ifndef SMS1186_MORTABLE_BOARD	// test for 2186 only, need to change when P2 HW available.
	// Disable LDO2 1.8V output first
	lp3907_i2c_write(LP3907_BKLDOEN_REG, 0x21);
	printk("Disable LDO2 1.8V output\n");
	msleep(10);
	printk("Delayed 10msec\n");

	// Disable SW1 1.2V output following VDD-IO(1.8V)
	lp3907_i2c_write(LP3907_BKLDOEN_REG, 0x20);
	printk("Disable SW1 1.2V output\n");
	msleep(10);
	printk("Delayed 10msec\n");
#endif
//-----------------------------------------------------------------------------------------
*/
	
	// if no current leak when disable LP3907 by MTV_VREG_EN, the following can be deleted.
	printk("Write LP3907 registers to disable output...\n");
	for(i = 0; lp3907_powerdown_tbl[i].reg != LP3907_INVALID_REG; i++)
	{
		ret = lp3907_i2c_write(lp3907_powerdown_tbl[i].reg, lp3907_powerdown_tbl[i].data);
		printk("Write data 0x%2x to LP3907 reg 0x%2x\n", lp3907_powerdown_tbl[i].data, lp3907_powerdown_tbl[i].reg);
	}	
	msleep(30);

	gpio_set_value(MTV_VREG_EN, 0);
	printk("MTV_VREG_EN enable state:%d\n", gpio_get_value(MTV_VREG_EN));

	//=================add for config SPI_CS pin using pinmux function===================
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LM0, TEGRA_PUPD_NORMAL);

	
	return ret;

}

static int sms_power_sel(int on)
{
	printk("SMSMDTV Power ON/OFF(1/0) select: %s\n", on ? "Power ON CMMB Chip" : "Turn OFF CMMB Chip");

	if (on) {
		smsmdtv_power_up();
		sms_suspend_count = 0;
	} else {
		smsmdtv_power_down();
	}

	return 0;

}

static int sms_power_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "sms_power_proc_show: %ld\n", is_sms_power_on);

	printk("CMMB_POWER_EN = %x\n", gpio_get_value(MTV_VREG_EN));
	printk("CMMB_MTV_PWDN = %x\n", gpio_get_value(MTV_PWDN));
#ifdef  SMS1186_MORTABLE_BOARD
	printk("CMMB_MTV_RST_N = %x\n", gpio_get_value(MTV_RST_N));
#endif
	return 0 ;
}

static int sms_power_proc_open(struct inode *node, struct file *flip)
{
	return single_open(flip, sms_power_proc_show, NULL);
}

static int sms_power_proc_write(struct file* file, const char __user* buffer, size_t count, loff_t* data) 
{
	char* pVal = NULL;
	int ret = -ENOMEM;

	pVal = (char*)kmalloc(sizeof(char)* count+1, GFP_KERNEL);
	if(!pVal) 
		return -ENOMEM;

	memset(pVal, 0, sizeof(char)*count+1);

	if(copy_from_user(pVal, buffer, count))
		ret = -EFAULT ;
	else
	{
		if(strncmp("1", pVal, 1) == 0)
			is_sms_power_on = 1;
		else 
			is_sms_power_on = 0;
           
		sms_power_sel(is_sms_power_on);
		ret = count;
	}
	kfree(pVal);
	return ret;
}

static int sms_prower_proc_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	void __user *up = (void __user *)arg;
	if (!up)
		return -EINVAL;
	if (SMS_POWER_GET_SUSPEND_COUNTER == cmd) {
		if (put_user(sms_suspend_count, (int *)up))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static struct file_operations sms_switch_fileops = {
	.owner  = THIS_MODULE,
	.open   = sms_power_proc_open,
	.write  = sms_power_proc_write,
	.read   = seq_read,
	.llseek	= seq_lseek,
	.release= single_release,
	.ioctl = sms_prower_proc_ioctl,
};

static int sms_power_proc_init(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("smspower", S_IRWXUGO, NULL);
	if (!entry)
		return -ENOMEM;

	//entry->owner = THIS_MODULE;
	entry->proc_fops = &sms_switch_fileops;

	return 0 ;
}

static void sms_power_proc_exit(void)
{
	remove_proc_entry("smspower", NULL);
}
#else
static int sms_power_proc_init(void) {}
static int sms_power_proc_exit(void) {}
#endif

static int __init sms_power_init(void)
{
	int err = 0 ; 

	//=================add for config SPI_CS pin using pinmux function===================
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LM0, TEGRA_PUPD_PULL_DOWN);

	// Init the CMMB PWR_EN GPIO pin used for LP3907
	if(gpio_request(MTV_VREG_EN, NULL) < 0)	// MTV_VREG_EN
		printk("!!!unable to get TEGRA_GPIO_PS1 to init MTV_VREG_EN state\n");

	printk("Disable LP3907 output voltage for SMSx186 when board power up\n");
	gpio_direction_output(MTV_VREG_EN, 0);	// Disable LP3907 when init

#ifdef SMS1186_MORTABLE_BOARD
	if(gpio_request(MTV_RST_N, NULL) < 0)	// MTV_RST_N
		printk("!!!unable to get MTV_RST_N to init\n");
	printk("set MTV_RST_N to LOW when init\n");
	gpio_direction_output(MTV_RST_N, 0);
#endif

	if(gpio_request(MTV_PWDN, NULL) < 0)	// MTV_PWRDN
		printk("!!!unable to get MTV_PWDN to init MTV_PWRDN state\n");
	printk("set MTV_PWDN to LOW when init\n");
	gpio_direction_output(MTV_PWDN, 0);
	msleep(20);

	//=================add for config SPI_CS pin using pinmux function===================
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LM0, TEGRA_PUPD_NORMAL);

	err = sms_power_proc_init();
	printk("@@ == create smspower proc == @@\n");
	if(err) {
		printk("create smspower proc failed, err = %d\n",err);
		return err;
	}
	return err ;
}

static void __exit sms_power_exit(void)
{
	sms_power_proc_exit();

#ifdef  SMS1186_MORTABLE_BOARD
	gpio_free(MTV_RST_N);
#endif
	gpio_free(MTV_PWDN);
	gpio_free(MTV_VREG_EN);
}

module_init(sms_power_init);
module_exit(sms_power_exit);

MODULE_LICENSE("GPL");

