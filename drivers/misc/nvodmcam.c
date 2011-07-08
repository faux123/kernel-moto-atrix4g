/*
 * Copyright (C) 2007-2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <../../arch/arm/mach-tegra/clock.h> /* for get_tegra_clock_by_name */

#include <mach/nvodmcam.h>
#include <linux/nvodmcam.h>

#define OFF 0
#define ON  1

#define MAX_CLIENTS 3

#define DEBUG_RW_INTF 0

enum {
	CAMERAS_OFF,
	CAMERA_ON,
};

struct clientinfo {
	int open;
	int pwr_requestor;  /* can be only 1 */
};

static int state = CAMERAS_OFF;
static struct regulator *reg_vcam = NULL;
static struct regulator *reg_vcsi = NULL;
static struct nvodmcam_platform_data p;
static struct clientinfo clients[MAX_CLIENTS];
static struct mutex lock;

#define CHKRET(x) {\
	int __ret = (x);\
	if (__ret) {\
		printk(KERN_ERR "nvodmcam: failed line %d\n", __LINE__);\
		return __ret;\
	}\
}

static int gpio_set(struct nvodmcam_gpio_cfg *gpio, int on)
{
	if (gpio->num >= 0) {
		gpio_set_value(gpio->num,
			on ? gpio->state : !gpio->state);
	}
	return 0;
}

static int mclk(bool enable)
{
	int ret;
	struct clk *mclk;

	mclk = get_tegra_clock_by_name("vimclk");
	if (mclk == NULL) {
		printk("nvodmcam: unable to find mclk by name\n");
		return -EFAULT;
	}

	if (enable) {
		printk("nvodm: mclk enable\n");
		ret = clk_set_rate(mclk, 24000000);
		if (ret != 0) {
			printk(KERN_ERR "nvodmcam: clk_set_rate failed\n");
			return ret;
		}

		ret = clk_enable(mclk);
		if (ret != 0) {
			printk(KERN_ERR" nvodmcam: clk_enable failed\n");
			return ret;
		}
	} else {
		printk("nvodm: mclk disable\n");
		clk_disable(mclk);
	}

	return 0;
}

/*
	IKSTABLEFOUR-8256 Functions camera_on() and camera_off() need to conditionally control the vcsi
	regulator and, thusly, argument ignore_vcsi is added. The decision to ignore control (or not)
	is made by the caller. During kernel initialization, the ignore flag is set to ensure the state
	of vcsi is cleanly inherited from the bootloader.
*/
static int camera_on(int num, bool ignore_vcsi)
{
	if (state != CAMERAS_OFF) {
		printk(KERN_ERR "nvodmcam: camera_on: BAD STATE\n");
		return -EFAULT;
	}

	if (num >= p.num_cameras) {
		printk(KERN_ERR "nvodmcam: invalid camera number\n");
		return -EFAULT;
	}

	/* regulator requests must be balanced */
	if (!ignore_vcsi) {
		CHKRET( regulator_enable(reg_vcsi) );
	}

	CHKRET( regulator_enable(reg_vcam) );

	/* set CAM_PD on ON state */
	CHKRET( gpio_set( &p.camera[num].cam_pd, ON ) );
	mdelay(5);

	/* toggle CAM_RS to ON state */
	CHKRET( gpio_set( &p.camera[num].cam_rs, ON ) );
	mdelay(5);
	CHKRET( gpio_set( &p.camera[num].cam_rs, OFF ) );
	mdelay(50);  // FIXME - check this
	CHKRET( gpio_set( &p.camera[num].cam_rs, ON ) );

	/* set FLASH_RS to ON state */
	CHKRET( gpio_set( &p.camera[num].flash_rs, ON ) );

	state = CAMERA_ON;

	return 0;
}

static int cameras_off(bool ignore_vcsi)
{
	int i;

	for (i=0; i<p.num_cameras; i++) {
		CHKRET( gpio_set( &p.camera[i].flash_rs, OFF ) );
		CHKRET( gpio_set( &p.camera[i].cam_rs, OFF ) );
		CHKRET( gpio_set( &p.camera[i].cam_pd, OFF ) );
	}

	if (state != CAMERAS_OFF) {
		/* regulator requests must be balanced */
		CHKRET( regulator_disable(reg_vcam) );
		if (!ignore_vcsi) {
			CHKRET( regulator_disable(reg_vcsi) );
		}
	}

	state = CAMERAS_OFF;

	return 0;
}

static int cameras_init(void)
{
	int i;

	CHKRET(cameras_off(1));

	for (i=0; i<p.num_cameras; i++) {
		switch (p.camera[i].sensor) {
		case OV5650:
			// FIXME: to do
			break;
		case AP8140:
			mclk(1);
			mdelay(2); // min 2400 cycles
			camera_on(i, 1);
			mdelay(4); // min 5000 cycles
			cameras_off(1);
			mdelay(2); // min 2400 cycles
			mclk(0);
			break;
		case OV7692:
		case OV7739:
		case SOC380:
			// do nothing for these
			break;
		default:
			printk(KERN_ERR "unsupported sensor type\n");
			return -ENODEV;
		}
	}

	return 0;
}

#if (DEBUG_RW_INTF)
static ssize_t nvodmcam_read(struct file *file, char *buf,
		size_t count, loff_t *ppos)
{
	static int eof = 0;
	unsigned char retbuf[50];
	int cnt;
	int ret = -EFAULT;

	mutex_lock(&lock);

	if (eof) {
		eof = 0;
		return 0;
	} else {
		eof = 1;
		cnt = sprintf(retbuf, "%d\n", p.num_cameras);

		if (copy_to_user(buf, retbuf, cnt)) {
			printk(KERN_ERR "nvodmcam: failed to copy_to_user\n");
		} else {
			ret = cnt;
		}
	}

	mutex_unlock(&lock);

	return cnt;
}

static ssize_t nvodmcam_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
#define CMDBUFLEN 100
	unsigned char cmdbuf[CMDBUFLEN+1];
	int lcount = (count < CMDBUFLEN) ? count : CMDBUFLEN;
	int ret = -EFAULT;

	mutex_lock(&lock);

	if (lcount <= 0) {
		goto done;
	}
	if (copy_from_user(cmdbuf, buf, lcount)) {
		printk(KERN_ERR "nvodmcam: failed to copy_from_user\n");
		goto done;
	}
	cmdbuf[lcount] = 0;
	if (cmdbuf[lcount-1] == '\n') {
		cmdbuf[lcount-1] = 0;
		lcount--;
	}

	printk("nvodmcam: cmd = '%s'\n", cmdbuf);

	if (!strcmp(cmdbuf,"on1")) { ret = camera_on(0, 0); }
	else
	if (!strcmp(cmdbuf,"on2")) { ret = camera_on(1, 0); }
	else
	if (!strcmp(cmdbuf,"off")) { ret = cameras_off(0); }
	else
	if (!strcmp(cmdbuf,"mclkon")){ ret = mclk(1); }
	else
	if (!strcmp(cmdbuf,"mclkoff")){ ret = mclk(0); }
	else {
		printk(KERN_ERR "nvodmcam: unknown command: %s\n", cmdbuf);
		goto done;
	}

	if (ret) {
		printk(KERN_ERR "nvodmcam: camera on failed: %d\n", ret);
	} else {
		ret = count;
	}

done:
	mutex_unlock(&lock);

	return ret;
}
#endif
static int nvodmcam_open(struct inode *inode, struct file *file)
{
	int i;
	int ret = 0;

	mutex_lock(&lock);

	//printk("nvodmcam_open\n");

	for (i=0; i<MAX_CLIENTS; i++) {
		if (!clients[i].open) {
			clients[i].open = 1;
			break;
		}
	}
	if (i >= MAX_CLIENTS) {
		printk("nvodmcam: device busy\n");
		ret = -EBUSY;
	} else {
		file->private_data = &clients[i];
	}

	mutex_unlock(&lock);

	return ret;
}

static int nvodmcam_release(struct inode *inode, struct file *file)
{
	struct clientinfo *cinfo = (struct clientinfo *)file->private_data;

	mutex_lock(&lock);

	//printk("nvodmcam_release\n");

	cinfo->open = 0;
	if (cinfo->pwr_requestor) {
		printk("nvodmcam: cleaning up after bad client\n");
		cameras_off(0);
		cinfo->pwr_requestor = 0;
	}
	file->private_data = 0;

	mutex_unlock(&lock);

	return 0;
}

static long nvodmcam_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int *param = (int *)arg;
	int ret = -EINVAL;
	struct clientinfo *cinfo = (struct clientinfo *)file->private_data;

	mutex_lock(&lock);

	//printk("nvodmcam_ioctl cmd=%u arg=%lu\n", cmd, arg);

	switch(cmd) {
	case NVODMCAM_IOCTL_GET_NUM_CAM:
		printk("nvodmcam: num cameras %u\n", p.num_cameras);
		ret = put_user(p.num_cameras, param);
		break;

	case NVODMCAM_IOCTL_CAMERA_OFF:
		printk("nvodmcam: camera off\n");
		ret = cameras_off(0);
		cinfo->pwr_requestor = 0;
		break;

	case NVODMCAM_IOCTL_CAMERA_ON:
		printk("nvodmcam: camera on %lu\n", arg);
		ret = camera_on(arg, 0);
		if (ret == 0) {
			cinfo->pwr_requestor = 1;
		}
		break;

	default:
		printk(KERN_ERR "nvodmcam: invalid cmd: %d\n", cmd);
		break;
	}

	if (ret)
		printk(KERN_ERR "nvodcam: cmd failed: %d\n", ret);

	mutex_unlock(&lock);

	return ret;
}

const static struct file_operations nvodmcam_fops = {
	.owner = THIS_MODULE,
#if (DEBUG_RW_INTF)
	.read  = nvodmcam_read,
	.write = nvodmcam_write,
#endif
	.open  = nvodmcam_open,
	.release = nvodmcam_release,
	.unlocked_ioctl = nvodmcam_ioctl,
};

static struct miscdevice nvodmcam_mdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NVODMCAM_DRIVER_NAME,
	.fops = &nvodmcam_fops,
};

static int gpio_get(int gpionum)
{
	int ret;
	if (gpionum >= 0) {
		ret = gpio_request(gpionum, "nvodmcam");
		if (ret) {
			printk(KERN_ERR "nvodmcam: failed gpio request %d\n",
					gpionum);
			return ret;
		}
		ret = gpio_direction_output(gpionum, 0);
		if (ret) {
			printk(KERN_ERR "nvodmcam: failed gpio direction %d\n",
					gpionum);
			return ret;
		}
	}
	return 0;
}

static int __init nvodmcam_probe(struct platform_device *pdev)
{
	int ret;
	int i;

	printk("nvodmcam_probe\n");

	mutex_init(&lock);

	memcpy(&p, pdev->dev.platform_data, sizeof(p));

	for(i=0; i<p.num_cameras; i++) {
		pr_info("nvodmcam: %d: pd={%d,%d} rs={%d,%d} fl={%d,%d}",
			i,
			p.camera[i].cam_pd.num,   p.camera[i].cam_pd.state,
			p.camera[i].cam_rs.num,   p.camera[i].cam_rs.state,
			p.camera[i].flash_rs.num, p.camera[i].flash_rs.state);

		if (gpio_get(p.camera[i].cam_pd.num))
			return -EFAULT;
		if (gpio_get(p.camera[i].cam_rs.num))
			return -EFAULT;
		if (gpio_get(p.camera[i].flash_rs.num))
			return -EFAULT;
	}

	reg_vcam = regulator_get(&pdev->dev, "vcam");
	if (reg_vcam == NULL) {
		printk("nvodmcam: failed to get vcam\n");
		return -EFAULT;
	}
	reg_vcsi = regulator_get(&pdev->dev, "vcsi");
	if (reg_vcsi == NULL) {
		printk("nvodmcam: failed to get vcsi\n");
		return -EFAULT;
	}

	ret = cameras_init();
	if (ret != 0)
		return ret;

	ret = misc_register(&nvodmcam_mdev);
	if (ret != 0)
		printk(KERN_ERR "nvodmcam register failed\n");

	return ret;
}

static struct platform_driver nvodmcam_pdrv = {
	.driver = { .name = NVODMCAM_DRIVER_NAME, },
	.probe = nvodmcam_probe,
};

static int __init nvodmcam_init(void)
{
	int ret;
	printk("nvodmcam_init\n");

	memset(&p, 0, sizeof(p));
	memset(&clients, 0, sizeof(clients));

	ret = platform_driver_register(&nvodmcam_pdrv);
	if (ret) {
		printk(KERN_ERR "nvodmcam: failed to register pdrv %d\n", ret);
		return ret;
	}

	return 0;
}

static void __exit nvodmcam_exit(void)
{
	printk("nvodmcam_exit\n");
	misc_deregister(&nvodmcam_mdev);
	platform_driver_unregister(&nvodmcam_pdrv);

	// FIXME: release gpios?

	if (reg_vcam)
		regulator_put(reg_vcam);
	if (reg_vcsi)
		regulator_put(reg_vcsi);
}

module_init(nvodmcam_init);
module_exit(nvodmcam_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("Misc driver for nVidia ODM Camera");
MODULE_LICENSE("GPL");

