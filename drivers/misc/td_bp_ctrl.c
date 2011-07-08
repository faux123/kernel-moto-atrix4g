/*
     Copyright (C) 2010 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/

/* BP control driver for TD modem. Its main features are:
 * provide interfaces to user space to reset BP, power down BP and power up BP
 * notify user space when a BP reset /power down event happens.
 * interfaces to drivers for registration of uninterruptable waite queues
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/td_bp_ctrl.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/cpcap-accy.h>
/* Fix me: we should not operate Vbus pin in this driver
 * It is the USB IPC driver's job. Just enable it before
 * interfaces to IPC drivers are implemented.
 */
#define TD_BP_USB_VBUS 1

#define FORCE_PANIC_ON_BP_RESET

/*
 * event mechanism
 */

//event type
typedef enum BP_EVENT_TYPE {
	BP_RESET = 0,
	BP_POWER_LOSS,
	AP_TRIGGED
} E_BP_EVENT_TYPE;

struct td_bp_event{
	struct timeval time_stamp; //for do_gettimeofday()...
	E_BP_EVENT_TYPE type;		//event type...
	const char * ap_mod_name;  //the name of modules in AP changing BP power status...
	const char * ap_mod_msg;	//the msg string from trigger module
	struct list_head	list;

};
/* Internal data structures
 * Fix me: put these into device struct?
 */
static 	struct mot_td_bpp_pdata * bpp_pdata;
static	struct device * td_bp_dev;	//stand device of td bp ctrl driver, for access kobj and uevent...
static LIST_HEAD(event_list);
static LIST_HEAD(ext_intf_list);
static spinlock_t event_lock = SPIN_LOCK_UNLOCKED;
static struct wake_lock lp0wdi_wakelock;

#ifdef FORCE_PANIC_ON_BP_RESET
static struct timer_list panic_timer;
static void bp_reset_or_off(unsigned long data);
#define FORCE_PANIC_TIMEOUT (15*HZ)
#endif

static void td_bp_wdi_uevent(struct work_struct *work)
{
	kobject_uevent(&td_bp_dev->kobj,KOBJ_CHANGE);
	printk(KERN_WARNING "%s: uevent sent in work\n", __func__);
}

DECLARE_WORK(td_uevnt_work,td_bp_wdi_uevent);

/*
 * td_bp_intf_mutex: mutex lock for interfaces to user
 * e.g. sysfs calls, misc driver calls, proc interfaces...
 */
static DECLARE_MUTEX(td_bp_intf_mutex);

#define RUN_EACH_EXT_INTERFACE(__func)	{\
	struct td_bp_ext_interface * intf=NULL;	\
		list_for_each_entry(intf,&ext_intf_list,list){	\
			if(intf->__func) intf->__func();	\
		}	\
	}

static void td_bp_new_event(const char * mod_name,
				const char * msg,
				E_BP_EVENT_TYPE type){

	unsigned long flag;
	struct td_bp_event * evt=NULL;
	if(!mod_name)
		panic("%s: Receive null mod name!!!\n", __func__);


	if(in_interrupt())
		evt=kmalloc(sizeof(struct td_bp_event),GFP_ATOMIC);
	else
		evt=kmalloc(sizeof(struct td_bp_event),GFP_KERNEL);
	if(!evt)
		panic("%s: Fail to alloc new bp event!!!\n", __func__);

	evt->ap_mod_name=mod_name;
	evt->ap_mod_msg=msg;
	evt->type=type;

	/*
	 * Fix me: check if sleep-able in do_gettimeofday()
	 */
	do_gettimeofday(&evt->time_stamp);

/*
 * enqueue the new event, consider interrupt context
 */
	spin_lock_irqsave(&event_lock,flag);
	list_add_tail(&evt->list,&event_list);
	spin_unlock_irqrestore(&event_lock,flag);

}

#ifdef FORCE_PANIC_ON_BP_RESET
static void bp_reset_or_off(unsigned long data)
{
	printk(KERN_WARNING "\n%s: Can not reboot phone in %d seconds!!!\n",
			__func__, FORCE_PANIC_TIMEOUT / HZ);
	BUG();
}
#endif

static int __td_bp_get_wdi(void){
	return gpio_get_value(bpp_pdata->gpio_bp_wdi);
}

/*
 * Set/Get BP status functions. 
 * basic tool functions without any sync protection
 */

#define BP_ON_KEY_OFF_DURATION_MS 1100

static void __td_bp_shutdown(void){
	/*
	 * do nothing if BP has been off
	 */
	enum cpcap_accy cur_accy = CPCAP_ACCY_NONE;
	if(!__td_bp_get_wdi())
		return;
#if TD_BP_USB_VBUS

#define BP_VBUS_PD5 29 /*TEGRA_GPIO_PD5   */

  /* disconnect TD USB IPC before shutdown BP */
  gpio_request(BP_VBUS_PD5, "TD USB IPC VBUS");
  gpio_direction_output(BP_VBUS_PD5, 0);
#endif

  /*
   * Call each external interface's prior shutdown functions.
   * Mutex of external interface's must be acquired first
   */
  RUN_EACH_EXT_INTERFACE(prior_shutdown);
  printk(KERN_WARNING "%s: disable wdi irq when shutdown\n",
		  __func__);
  disable_irq(gpio_to_irq(bpp_pdata->gpio_bp_wdi));
  gpio_set_value(bpp_pdata->gpio_bp_on_key,1);
  printk(KERN_WARNING "%s: pull-up bp on key\n", __func__);
  if(!in_interrupt() && ! in_atomic()){
   msleep(BP_ON_KEY_OFF_DURATION_MS);
  }
  else{
    panic("%s: Fix me: support shutdown BP in interrupt context\n", __func__);

  }
  gpio_set_value(bpp_pdata->gpio_bp_on_key,0);
  printk(KERN_WARNING "%s: pull-down bp on key\n", __func__);
  /* Fix me:add wait BB power off safely and force shut down
   * There has no force shutdown pin on Arowana's TD PMU...
   * What we can do is ignoring this, since we can not nothing
   * with a failure of BP powering off
   * */
#if 1
  /*
   * Add workaround on Arowana, BP can not be shut down if factory cable
   * is attached in factory, the workaround is bypassing wdi check in
   * this case.
   */
  while((cur_accy = cpcap_accy_get_crrent_accry()) != CPCAP_ACCY_FACTORY
		  && __td_bp_get_wdi()) {
	  printk(KERN_WARNING "%s: Waiting BP off, current accy = %d.\n", __func__,
			  cur_accy);
	  msleep(200);
  }
  /* After shut down.
   * Call each external interface's post shutdown function.
   * Mutex of external interface's must be acquired first
   */
  RUN_EACH_EXT_INTERFACE(post_shutdown);

  printk(KERN_WARNING "%s: BP has been shutdown.\n", __func__);
#endif
}

/* Fix me: need confirm the final timing of ON duration
   Now give it the longest setting.
 */
#define BP_ON_KEY_ON_DURATION_MS 2048
static void __td_bp_power_up(void){
	/*
	 * do nothing if BP has been on
	 */
	if(__td_bp_get_wdi())
		return;

	  /*
	   * Call each external interface's prior power-on functions.
	   * Mutex of external interface's must be acquired first
	   */
	RUN_EACH_EXT_INTERFACE(prior_on);

#if TD_BP_USB_VBUS
  /* enable TD USB IPC VBUS pin */
  gpio_request(BP_VBUS_PD5, "TD USB IPC VBUS");
  gpio_direction_output(BP_VBUS_PD5, 1);
#endif
  printk(KERN_WARNING "%s: enable wdi irq before power up BP\n", __func__);
  enable_irq(gpio_to_irq(bpp_pdata->gpio_bp_wdi));
  gpio_set_value(bpp_pdata->gpio_bp_on_key,1);
  if(!in_interrupt() && ! in_atomic()){
	  msleep(BP_ON_KEY_ON_DURATION_MS);
  }
  else{
    panic("%s: Fix me: support shutdown BP in interrupt context\n", __func__);

  }
  gpio_set_value(bpp_pdata->gpio_bp_on_key,0);

#if 1
  while(!__td_bp_get_wdi())
	  msleep(200);
  /*
   * Call each external interface's post power-on functions.
   * Mutex of external interface's must be acquired first
   */
  RUN_EACH_EXT_INTERFACE(post_on);

  printk(KERN_WARNING "%s: BP has been powered up.\n", __func__);
#endif
}


#define TD_BP_UNSAFE_RESET 1
static void __td_bp_reset(void){

#if TD_BP_UNSAFE_RESET
	/*
	 * do nothing if BP has been off
	 */
	if(!__td_bp_get_wdi())
		return;
	gpio_set_value(bpp_pdata->gpio_bp_reset,0);

	if(!in_interrupt() && ! in_atomic()){
		msleep(10);
	}
	else{
		mdelay(10);
	}

	gpio_set_value(bpp_pdata->gpio_bp_reset,1);

#else
  __td_bp_shutdown();

  __td_bp_power_up();
#endif

#if 1
  while(!__td_bp_get_wdi())
	  msleep(50);

  printk(KERN_WARNING "%s: BP has been reset.\n", __func__);
#endif
}
void tp_bp_enter_tat_mode(void){
	printk(KERN_WARNING "%s: disable wdi irq when enter tat\n",
			__func__);
	disable_irq(gpio_to_irq(bpp_pdata->gpio_bp_wdi));
	RUN_EACH_EXT_INTERFACE(prior_tat_mode);
	printk(KERN_WARNING "%s,Bring BP to TAT mode\n", __func__);
	__td_bp_reset();
}
/*
 * interface to kernel space
 */

//reset BP interface when it has a failure, to ap modules
void tp_bp_failure_reset(const char * mod_name, const char * msg){

	/*
	 * call td_bp_new_event() to add new event.
	 * flag = AP_TRIGGED
	 */
	td_bp_new_event(mod_name,msg,AP_TRIGGED);
	/* Fix me: need protection of sync in process context at least*/
	__td_bp_reset();
}
EXPORT_SYMBOL(tp_bp_failure_reset);

/*
 * irq and notification implementations
 */
irqreturn_t bp_wdi_irq_handler(int irq, void *data){
	/*
	 * add event
	 */

	/*
	 * start sending signals in work queue & wake up processes...
	 */
	disable_irq_nosync(gpio_to_irq(bpp_pdata->gpio_bp_wdi));
	printk(KERN_WARNING "%s,receive BP_WDI interrupt,start uevent work & disable our wdi irq %d\n",
			__func__, gpio_to_irq(bpp_pdata->gpio_bp_wdi));
	wake_lock(&lp0wdi_wakelock);
	RUN_EACH_EXT_INTERFACE(on_wdi_interrupt);
	schedule_work(&td_uevnt_work);
	td_bp_new_event("bpctrl","BP (wdi) falling edge detected",BP_RESET);
#ifdef FORCE_PANIC_ON_BP_RESET
/*
 * fire timer to force a panic
 * when we found BP can not be off nicely
 * after its reset...
 */
	mod_timer(&panic_timer, jiffies+FORCE_PANIC_TIMEOUT);
#endif
	return IRQ_HANDLED;
}

/*
 * external interfaces
 */
int td_bp_register_ext_interface(struct td_bp_ext_interface * intf){

	if(!intf)
		return -EINVAL;
	if(!intf->name)
		return -EINVAL;

	down(&td_bp_intf_mutex);
	list_add_tail(&intf->list,&ext_intf_list);
	up(&td_bp_intf_mutex);

	return 0;
}

int td_bp_unregister_ext_interface_by_name(const char * name){
	struct td_bp_ext_interface * intf=NULL;
	if(!name)
		return -EINVAL;

	down(&td_bp_intf_mutex);
	list_for_each_entry(intf,&ext_intf_list,list){
		if(!strcmp(name,intf->name)){
			list_del(&intf->list);
			up(&td_bp_intf_mutex);
			return 0;
		}

	}
	up(&td_bp_intf_mutex);
	return -EINVAL;
}
/*
 * Sysfs interfaces
 */
#ifdef CONFIG_SYSFS
static ssize_t show_ctrl(struct device *dev,struct device_attribute *attr, char *buf){
	return snprintf(buf, PAGE_SIZE, "Accept values:\non\noff\nreset\ntat\ncheck wdi for state\n");
}

static ssize_t store_ctrl(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count){
	const char * comm = current->comm;
	down(&td_bp_intf_mutex);
//ensure BP is off before power it on
	if(!strncmp("on",buf,strlen("on"))){
		/*
		 * Fix me: if BP happens a reset when getting wdi???
		 */
		printk(KERN_WARNING "%s: power on BP\n", __func__);
	td_bp_new_event(comm,"pow on BP:sysfs",AP_TRIGGED);
		__td_bp_power_up();

	}else if (!strncmp("off",buf,strlen("off"))){
		//ensure BP is on before power it off
		printk(KERN_WARNING "%s: power off BP\n", __func__);
	td_bp_new_event(comm,"pow off BP:sysfs",AP_TRIGGED);
		__td_bp_shutdown();

	}else if (!strncmp("reset",buf,strlen("reset"))){
		//ensure BP is on before reset it
		printk(KERN_WARNING "%s: reset BP\n", __func__);
	td_bp_new_event(comm,"reset BP:sysfs",AP_TRIGGED);
		__td_bp_reset();
	}else if (!strncmp("tat",buf,strlen("tat"))){
		//ensure BP is on before reset it
		printk(KERN_WARNING "%s: BP enters TAT mode\n", __func__);
		td_bp_new_event(comm,"TAT mode:sysfs",AP_TRIGGED);
		tp_bp_enter_tat_mode();
	}

	up(&td_bp_intf_mutex);
	return count;

}

static ssize_t show_wdi(struct device *dev,struct device_attribute *attr, char *buf){
	return snprintf(buf, PAGE_SIZE, "%d\n",__td_bp_get_wdi()?1:0);
}

static ssize_t show_events(struct device *dev,struct device_attribute *attr, char *buf){
	struct td_bp_event * evt=NULL;
	size_t count=0;  /*counter for copied string's length, excluding trailing null */
	char * c=buf;  /*to reduce loop times on string operations...*/

	count+=snprintf(c,PAGE_SIZE-count,"%s\n", \
			"#Module Name |  Message | TYPE | Time Stamp(sec since 1970) | Time Stamp(usec from the last sec)");
	c=buf+count;

	count+=snprintf(c,PAGE_SIZE-count,"%s\n\n", \
			"#TYPE:0 BP WDI Falling edge, 2, Triggered by AP");
	c=buf+count;

	list_for_each_entry(evt,&event_list,list){

		count+=snprintf(c,PAGE_SIZE-count,"\"%s\" | ",evt->ap_mod_name);
		c=buf+count;

		count+=snprintf(c,PAGE_SIZE-count,"\"%s\" | ",evt->ap_mod_msg);
		c=buf+count;

		count+=snprintf(c,PAGE_SIZE-count,"%d | ",evt->type);
		c=buf+count;

		count+=snprintf(c,PAGE_SIZE-count,"%ld | ",evt->time_stamp.tv_sec);
		c=buf+count;

		count+=snprintf(c,PAGE_SIZE-count,"%ld\n",evt->time_stamp.tv_usec);
		c=buf+count;

		if(count==PAGE_SIZE-1)
			break;
	}
	return count+1;
}

static struct device_attribute td_bp_dev_attrs[]={
	{
	 .attr = {"ctrl",THIS_MODULE,(S_IRUSR|S_IWUSR)},
	 .show=show_ctrl,
	 .store=store_ctrl,
	},
	{
	 .attr = {"wdi", THIS_MODULE,(S_IRUSR | S_IRGRP | S_IROTH)},
	 .show = show_wdi,
	 .store = NULL,
	},
	{
	 .attr = {"events",THIS_MODULE, (S_IRUSR | S_IRGRP | S_IROTH)},
	 .show = show_events,
	 .store = NULL,
	},
};

static int td_bp_sysfs_init(struct device * dev){
	int i=0;
	for (i=0;i<ARRAY_SIZE(td_bp_dev_attrs);i++){
		if(device_create_file(dev,&td_bp_dev_attrs[i]))
			goto err;
	}
	printk(KERN_WARNING "%s: create %d attributes\n", __func__, i);
	return 0;
err:
	printk(KERN_WARNING "%s: error at init No. %d attr\n", __func__, i+1);
	return -1;

}

static int td_bp_sysfs_exit(struct device * dev){
	int i=0;
	for (i=0;i<ARRAY_SIZE(td_bp_dev_attrs);i++)
		device_remove_file(dev,&td_bp_dev_attrs[i]);

	printk(KERN_WARNING "%s: remove %d attributes\n", __func__, i);
	return 0;
}

#endif //config_sysfs

/*
 * misc device interface... temporary  interface
 */
static int td_bp_ioctl(struct inode *inode,struct file *file, unsigned int cmd,unsigned long arg)
{
	/*
	 * fix me: add ioctl parameters for msg to app.
	 */

	const char * comm = current->comm;
	const char * msg = "Triggered by application";
	printk(KERN_WARNING "%s,cmd=0x%x", __func__, cmd);
	down(&td_bp_intf_mutex);
	switch(cmd)
	{

	case TDBP_IOCTL_BP_RESET:
		__td_bp_reset();
		break;
	case TDBP_IOCTL_BP_FAIL_RESET:
		tp_bp_failure_reset(comm,msg);
		break;
	default:
		up(&td_bp_intf_mutex);
		return -EINVAL;
	}
	up(&td_bp_intf_mutex);
 return 0;
}

static const struct file_operations td_bp_fops={
	.owner=THIS_MODULE,
//	.open=td_bp_open,
	.ioctl=td_bp_ioctl,
};

static struct miscdevice td_bp_misc_device={
   .minor= MISC_DYNAMIC_MINOR,
   .name="td_bp",
   .fops=&td_bp_fops,
};


/*
 * Platform driver section
 */
static int __devinit td_bpp_probe(struct platform_device *pdev){
//	int irq=0;
	int value;
	int ret=0;
	int errno=0;
#if 0
	struct resource * irq_src;
#endif
	bpp_pdata = pdev->dev.platform_data;
	td_bp_dev = &pdev->dev;
/*
 * request GPIO for output from pdata
 */
	printk(KERN_WARNING "%s: enter !!!\n", __func__);
	ret=gpio_request(bpp_pdata->gpio_bp_on_key, "TD BP ON_KEY");
	if(ret)
		goto err0;
	value = gpio_get_value(bpp_pdata->gpio_bp_on_key);
	gpio_direction_output(bpp_pdata->gpio_bp_on_key, value);

	ret=gpio_request(bpp_pdata->gpio_bp_wdi,"td_bp_wdi");
	if(ret)
		goto err1;
	gpio_direction_input(bpp_pdata->gpio_bp_wdi);

	ret=gpio_request(bpp_pdata->gpio_bp_reset, "TD BP reset");
	if(ret)
		goto err2;
	value = gpio_get_value(bpp_pdata->gpio_bp_reset);
	gpio_direction_output(bpp_pdata->gpio_bp_reset, value);

	ret=gpio_request(bpp_pdata->gpio_bp_flash_en, "TD BP flash enable");
	if(ret)
		goto err3;
	value = gpio_get_value(bpp_pdata->gpio_bp_flash_en);
	gpio_direction_output(bpp_pdata->gpio_bp_flash_en, value);

#if 1
	value = gpio_get_value(bpp_pdata->gpio_bp_wdi);
	printk(KERN_WARNING "%s: bp_wdi has been at %d !!!\n", __func__, value);
#endif

/*
 * initialize/alloc event queue
 */

/*
 * initialize wake up queue
 */
#ifdef FORCE_PANIC_ON_BP_RESET
/*
 * initialize timer to force a panic
 * when we found BP can not be off nicely
 * after its reset...
 */
	init_timer(&panic_timer);
	panic_timer.function = bp_reset_or_off;
	panic_timer.data = 0;

#endif
/*
 * get irq resource and request irq
 */

#if 0
	irq_src = platform_get_resource_byname(pdev,IORESOURCE_IRQ,"wdi_irq");

	gpio_request(irq_to_gpio(irq_src->start),irq_src->name);

	gpio_direction_input(irq_to_gpio(irq_src->start));

	request_irq(irq_src->start,
			    bp_wdi_irq_handler,
			    irq_src->flags,
			    irq_src->name,
			    NULL);

#else
	ret=request_irq(gpio_to_irq(bpp_pdata->gpio_bp_wdi),
			    bp_wdi_irq_handler,
			    IORESOURCE_IRQ_LOWEDGE,
			    "td_bp_wdi_irq",
			    NULL);
	if(ret)
		goto err4;

#endif

/*
 * register misc device
 */
	 ret=misc_register(&td_bp_misc_device);
	 if(ret)
		 goto err5;

/*
 * register proc/seq interface
 */
/*
 * register sysfs interface
 */
#ifdef CONFIG_SYSFS
	 if(td_bp_sysfs_init(&pdev->dev))
		 goto err6;
#endif

	printk(KERN_WARNING "%s: exit successfully \n", __func__);
	return 0;
err6:
	errno++;
	misc_deregister(&td_bp_misc_device);
err5:
	errno++;
	free_irq(gpio_to_irq(bpp_pdata->gpio_bp_wdi),NULL);
err4:
	errno++;
	gpio_free(bpp_pdata->gpio_bp_flash_en);
err3:
	errno++;
	gpio_free(bpp_pdata->gpio_bp_reset);
err2:
	errno++;
	gpio_free(bpp_pdata->gpio_bp_wdi);
err1:
	errno++;
	gpio_free(bpp_pdata->gpio_bp_on_key);
err0:
	printk(KERN_WARNING "%s: exit at label err%d \n", __func__, errno);
	return -EBUSY;

}

static void __devexit td_bpp_plat_shutdown(struct platform_device *pdev)
{
	/*
	 * Fix me: an unfinished BP operation may block whole system shutdown
	 * process, but there is no way to cut down BP power by force...
	 */
	down(&td_bp_intf_mutex);
	__td_bp_shutdown();
	up(&td_bp_intf_mutex);
	printk(KERN_WARNING "%s: exit \n", __func__);
}

static int __devexit td_bpp_plat_remove(struct platform_device *pdev){
/*
 * deregister misc driver
 */
  misc_deregister(&td_bp_misc_device);

/*
 * free irq
 */
  free_irq(bpp_pdata->gpio_bp_wdi,NULL);

/*
 * unregister proc/seq interface
 */

/*
 * deregister sysfs interface
 */
#ifdef CONFIG_SYSFS
  td_bp_sysfs_exit(&pdev->dev);
#endif
/*
 * free GPIO for output from pdata
 */
  gpio_free(bpp_pdata->gpio_bp_on_key);
  gpio_free(bpp_pdata->gpio_bp_wdi);
  gpio_free(bpp_pdata->gpio_bp_reset);
  gpio_free(bpp_pdata->gpio_bp_flash_en);
/*
* free event queue
*/

/*
 * free wake up queue
 */


	printk(KERN_WARNING "%s: exit \n", __func__);
  return 0;
}

static struct platform_driver td_bpp_plat_drv = {
	.probe = td_bpp_probe,
	.remove = __devexit_p(td_bpp_plat_remove),
	.shutdown = __devexit_p(td_bpp_plat_shutdown),
	.driver = {
		   .name = "td_bp_ctrl",
		   .owner = THIS_MODULE,
		   },
};


/*
 * Module init/exit functions
 */
static int __init bpp_mod_init(void){
	int ret;
	ret=platform_driver_register(&td_bpp_plat_drv);

	printk(KERN_WARNING "%s: ret = %d\n", __func__, ret);
//  return platform_driver_register(&td_bpp_plat_drv);
	wake_lock_init(&lp0wdi_wakelock, WAKE_LOCK_SUSPEND,
			"wdilp0");
	return ret;
}



static void __exit bpp_mod_exit(void){
	wake_lock_destroy(&lp0wdi_wakelock);
	platform_driver_unregister(&td_bpp_plat_drv);
	printk(KERN_WARNING "%s: exit \n", __func__);
	return;
}

MODULE_LICENSE ("GPL");
module_init (bpp_mod_init);
module_exit (bpp_mod_exit);

