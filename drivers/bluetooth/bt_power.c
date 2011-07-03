#include <asm/delay.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/synaptics_i2c_rmi.h>
#include <../arch/arm/mach-tegra/gpio-names.h>

//////////////////////////////////////////////////////////////////////
static struct proc_dir_entry *proc_entry;
static int power_is_on;

//////////////////////////////////////////////////////////////////////

// #define TEGRA_GPIO_PU0		160
// #define TEGRA_GPIO_PU1		161
// #define TEGRA_GPIO_PU2		162
// #define TEGRA_GPIO_PU3		163
// #define TEGRA_GPIO_PU4		164
// #define TEGRA_GPIO_PU5		165
// #define TEGRA_GPIO_PU6		166
// #define TEGRA_GPIO_PU7		167

// bluetooth GPIOs
#define BT_POWER TEGRA_GPIO_PU0

/*Note: TODO. Need to wrap this change (instead of product specific #ifdef) into
        product specific structure and then pass it to driver.
*/
#ifdef CONFIG_TEGRA_ODM_DAYTONA
#define BT_RESET TEGRA_GPIO_PE5
#else
#define BT_RESET TEGRA_GPIO_PU4
#endif

#define EXT_WAKE TEGRA_GPIO_PU1
#define HOST_WAKE TEGRA_GPIO_PU6

// WLAN GPIOs
#define WL_POWER TEGRA_GPIO_PU3
#define WL_RESET TEGRA_GPIO_PU2
#define WL_HOST_WAKE TEGRA_GPIO_PU5

void bluetooth_gpio_init(void)
{
  gpio_request(BT_POWER, "Bluetooth Power");
  gpio_request(BT_RESET, "Bluetooth Reset");
  //  gpio_request(EXT_WAKE, "Bluetooth EXT_WAKE");
  //  gpio_request(HOST_WAKE,"Bluetooth HOST_WAKE");

  if(gpio_direction_output(BT_POWER, 0) < 0)
    { printk(KERN_ERR "BLUETOOTH: failed to set gpio output direction for BT_POWER\n"); }

  if(gpio_direction_output(BT_RESET, 0) < 0)
    { printk(KERN_ERR "BLUETOOTH: failed to set gpio output direction for BT_RESET\n"); }

  //  if(gpio_direction_output(EXT_WAKE, 0) < 0)
  //    { printk(KERN_ERR "BLUETOOTH: failed to set gpio output direction for EXT_WAKE\n"); }

  //  if(gpio_direction_input(HOST_WAKE) < 0)
  //    { printk(KERN_ERR "BLUETOOTH: failed to set gpio input direction for HOST_WAKE\n"); }
}

// state: 0 = low/off, 1 = high/on
void set_power(int state)   {gpio_set_value(BT_POWER, state); mdelay(100);}
void set_reset(int state)   {gpio_set_value(BT_RESET, state); mdelay(100);}
void set_bt_wake(int state) {gpio_set_value(EXT_WAKE, state); mdelay(100);}

int get_power(void)         {return(gpio_get_value(BT_POWER));}
int get_reset(void)         {return(gpio_get_value(BT_RESET));}
int get_bt_wake(void)       {return(gpio_get_value(EXT_WAKE));}
int get_host_wake(void)     {return(gpio_get_value(HOST_WAKE));}

int get_WL_power(void)      {return(gpio_get_value(WL_POWER));}
int get_WL_reset(void)      {return(gpio_get_value(WL_RESET));}
int get_WL_host_wake(void)  {return(gpio_get_value(WL_HOST_WAKE));}

//////////////////////////////////////////////////////////////////////
static int bt_power_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  char *str;
  int r;
  int len;
  
  r = get_power();
  
  if(r)
    {str = "1 (Bluetooth power is ON)\n";}
  else
    {str = "0 (Bluetooth power is OFF)\n";}
  
  len = strlen(str);
  if (size < len) {return -EINVAL;}
  if (offset != 0){return 0;}
  
  strcpy(buffer, str);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////
void xstrcat(char *d, char *s)
{
  while(*d){d++;}
  while(*s){*d++ = *s++;}
  *d=0;
}

//////////////////////////////////////////////////////////////////////
static int bt_stat_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  static unsigned int op;
  unsigned int cp;
  char str[64*13];
  char line[64];
  int len=0;

  str[0]=0;
  cp=0;

  cp |= gpio_get_value(BT_POWER); cp <<= 1;
  cp |= gpio_get_value(BT_RESET); cp <<= 1;

  cp |= gpio_get_value(EXT_WAKE); cp <<= 1;
  cp |= gpio_get_value(HOST_WAKE); cp <<= 1;
  
  cp |= gpio_get_value(WL_POWER); cp <<= 1;
  cp |= gpio_get_value(WL_RESET); cp <<= 1;

  cp |= gpio_get_value(WL_HOST_WAKE); 
  
  if(cp != op)
    {
      op = cp;

      sprintf(line,"%d",   gpio_get_value(BT_POWER)); strcat(str,line);
      sprintf(line,"%d ",  gpio_get_value(BT_RESET)); strcat(str,line);

      sprintf(line,"%d",   gpio_get_value(EXT_WAKE)); strcat(str,line);
      sprintf(line,"%d ",  gpio_get_value(HOST_WAKE)); strcat(str,line);

      sprintf(line,"%d",   gpio_get_value(WL_POWER)); strcat(str,line);
      sprintf(line,"%d ",  gpio_get_value(WL_RESET)); strcat(str,line);

      sprintf(line,"%d\n", gpio_get_value(WL_HOST_WAKE)); strcat(str,line);

      len = strlen(str);
      if (size < len) {return -EINVAL;}
    }

  if (offset != 0){return 0;}
  
  strcpy(buffer, str);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////
static int bt_status_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  char str[64*13];
  char line[64];
  int len;

  str[0]=0;

  sprintf(line,"%d = BT_RESET\n",         get_reset());        strcat(str,line);
  sprintf(line,"%d = BT_POWER\n\n",       get_power());        strcat(str,line);

  sprintf(line,"%d = BT_EXTWAKE\n",       get_bt_wake());      strcat(str,line);
  sprintf(line,"%d = BT_HOST_WAKE\n\n",   get_host_wake());    strcat(str,line);

  sprintf(line,"%d = WLAN_RESET\n",       get_WL_reset());     strcat(str,line);
  sprintf(line,"%d = WLAN_POWER\n\n",     get_WL_power());     strcat(str,line);

  sprintf(line,"%d = WLAN_HOST_WAKE\n\n", get_WL_host_wake()); strcat(str,line);

  len = strlen(str);
  if (size < len) {return -EINVAL;}
  if (offset != 0){return 0;}
  
  strcpy(buffer, str);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////
static char proc_buf[1];
// write '1' to turn on power, '0' to turn it off
// write 'w' to set BT WAKE to 0,  'W' to set BT WAKE to 1
// write 'a' to trigger A2DP mode transition to send broadcom VSC for BT prioritization [disabled, built-in to FroYo]
int bt_power_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
  //  extern int A2DP_prioritization_trigger;
  
  if(copy_from_user(proc_buf, buffer, 1)) {return(-EFAULT);}
  
  //  if('a' == *proc_buf)
  //    {
  //      printk(KERN_INFO "BLUETOOTH: A2DP VSC for BT prioritization triggered (was %d)\n",
  //	     A2DP_prioritization_trigger);
  //      A2DP_prioritization_trigger = 2;
  //      return(0);
  //    }
  
  if('w' == *proc_buf)
    {
      set_bt_wake(0);
      printk(KERN_INFO "BLUETOOTH: BT_WAKE: low\n");
      return(0);
    }
  
  if('W' == *proc_buf)
    {
      set_bt_wake(1);
      printk(KERN_INFO "BLUETOOTH: BT_WAKE: high\n");
      return(0);
    }
  
  if('1' == *proc_buf)
    {
      if(!power_is_on)
	{
	  set_power(1);  // turn power on to BT module
	  set_reset(1);  // take BT out of reset
	  
	  power_is_on = 1;
	  printk(KERN_INFO "BLUETOOTH: bt_power: ON\n");
	}
    }
  else
    {
      if(power_is_on)
	{
	  set_reset(0);  // put BT into reset
	  set_power(0);  // turn off internal BT VREG
	  
	  power_is_on = 0;
	  printk(KERN_INFO "BLUETOOTH: bt_power: OFF\n");
	}
    }

  return(0);
}

//////////////////////////////////////////////////////////////////////
static int __init bt_power_init(void)
{
  bluetooth_gpio_init();

  //  if (create_proc_read_entry("bt_power", 0, NULL, bt_power_read_proc,NULL) == 0) 
  proc_entry = create_proc_entry("bt_power", 0666, NULL);
  if(!proc_entry)
    {
      printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_power\" file failed\n");
      return(-ENOMEM);
    }

  proc_entry->read_proc  = bt_power_read_proc;
  proc_entry->write_proc = bt_power_write_proc;

  printk(KERN_INFO "BLUETOOTH: /proc/bt_power created\n");

  {
    proc_entry = create_proc_entry("bt_stat", 0666, NULL);
    if(!proc_entry)
      {
	printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_stat\" file failed\n");
	return(-ENOMEM);
      }
    
    proc_entry->read_proc  = bt_stat_read_proc;
    proc_entry->write_proc = 0;
    
    printk(KERN_INFO "BLUETOOTH: /proc/bt_stat created\n");
  }

  {
    proc_entry = create_proc_entry("bt_status", 0666, NULL);
    if(!proc_entry)
      {
	printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_status\" file failed\n");
	return(-ENOMEM);
      }
    
    proc_entry->read_proc  = bt_status_read_proc;
    proc_entry->write_proc = 0;
    
    printk(KERN_INFO "BLUETOOTH: /proc/bt_status created\n");
  }

  { // turn off power to BT and hold it in reset
    set_reset(0);  // put BT into reset
    set_power(0);  // turn off internal BT VREG
    printk(KERN_INFO "BLUETOOTH: bt_power: Bluetooth power is off, and BT module is in reset.\n");
    
    power_is_on = 0;
  }
  
  return 0;
}

//////////////////////////////////////////////////////////////////////
module_init(bt_power_init);

static void __exit bt_power_exit(void)
{
  remove_proc_entry("bt_power", NULL);
  remove_proc_entry("bt_stat", NULL);
  remove_proc_entry("bt_status", NULL);
}

//////////////////////////////////////////////////////////////////////
module_exit(bt_power_exit);

//////////////////////////////////////////////////////////////////////
MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Zafiris <John.Zafiris@motorola.com>");
MODULE_DESCRIPTION("\"bt_power\" Bluetooth power control for the Broadcom BCM4329");
MODULE_VERSION("1.1");

//////////////////////////////////////////////////////////////////////
