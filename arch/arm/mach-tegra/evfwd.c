#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/input.h>
#include <linux/platform_device.h>

MODULE_LICENSE("GPL");

#define DEVICE_NAME "evfwd"

/*
 * The x/y limits are taken from the Synaptics TouchPad interfacing Guide,
 * section 2.3.2, which says that they should be valid regardless of the
 * actual size of the sensor.
 */
#define XMIN_NOMINAL 1472
#define XMAX_NOMINAL 5472
#define YMIN_NOMINAL 1408
#define YMAX_NOMINAL 4448

struct input_dev *evfwd_input_dev;        /* Representation of an input device */
static struct platform_device *evfwd_dev; /* Device structure */

/* Sysfs method to input simulated
   events to the event forwarder */
static ssize_t
write_evfwd(struct device *dev,
          struct device_attribute *attr,
          const char *buffer, size_t count)
{
    struct input_event inev;

    if ( (count % sizeof(inev)) == 0)
    {
      int i;
      for (i = 0; i < count; i += sizeof(inev) )
      {
        memcpy( (char*)&inev, buffer+i, sizeof(inev));
        //printk("Event received (%d,%d,%d)\n", inev.type,inev.code, inev.value);
        switch (inev.type)
        {
          case EV_ABS:
              input_event(evfwd_input_dev, EV_ABS, inev.code, inev.value);
              break;
          case EV_REL:
              input_event(evfwd_input_dev, EV_REL, inev.code, inev.value);
              break;
          case EV_KEY:
              if (inev.code != KEY_UNKNOWN) {
                input_event(evfwd_input_dev, EV_KEY, inev.code, inev.value);
              }
              break;
          case EV_SYN:
              input_event(evfwd_input_dev, EV_SYN, SYN_REPORT, 0);
              break;
          default:
              printk("Bad event type (%d)\n", inev.type);
        }
      }
    }
    else
    {
        printk("Bad event block size (%d)\n", count);
    }

    return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(coordinates, 0644, NULL, write_evfwd);

/* Attribute Descriptor */
static struct attribute *evfwd_attrs[] = {
    &dev_attr_coordinates.attr,
    NULL
};

/* Attribute group */
static struct attribute_group evfwd_attr_group = {
    .attrs = evfwd_attrs,
};

/* Driver Initialization */
int __init
evfwd_init(void)
{
    int i;

    /* Register a platform device */
    evfwd_dev = platform_device_register_simple(DEVICE_NAME, -1, NULL, 0);
    if (IS_ERR(evfwd_dev)){
        printk ("evfwd_init: error\n");
        return PTR_ERR(evfwd_dev);
    }

    /* Create a sysfs node to read simulated coordinates */
    sysfs_create_group(&evfwd_dev->dev.kobj, &evfwd_attr_group);

    /* Allocate an input device data structure */
    evfwd_input_dev = input_allocate_device();
    if (!evfwd_input_dev) {
        printk("Bad input_allocate_device()\n"); return -ENOMEM;
    }

    evfwd_input_dev->name = DEVICE_NAME;

    // Announce that the virtual mouse will generate relative coordinates
    set_bit(EV_REL,     evfwd_input_dev->evbit);
    set_bit(REL_X,      evfwd_input_dev->relbit);
    set_bit(REL_Y,      evfwd_input_dev->relbit);
    set_bit(REL_WHEEL,  evfwd_input_dev->relbit);
    set_bit(REL_HWHEEL, evfwd_input_dev->relbit);

    // Announce the device will generate key and scan codes
    set_bit(EV_KEY, evfwd_input_dev->evbit);
    // set_bit(EV_MSC, evfwd_input_dev->evbit);

    // Set key autorepeat on
    set_bit(EV_REP, evfwd_input_dev->evbit);

    // set key codes for mouse buttons
    set_bit(BTN_0, evfwd_input_dev->keybit);
    set_bit(BTN_TOUCH, evfwd_input_dev->keybit);
    set_bit(BTN_LEFT, evfwd_input_dev->keybit);
    set_bit(BTN_RIGHT, evfwd_input_dev->keybit);
    set_bit(BTN_MIDDLE, evfwd_input_dev->keybit);

    // set key codes for keyboard (all key codes 1..255)
    for (i = KEY_ESC; i < BTN_MISC; i++) {
      set_bit(i, evfwd_input_dev->keybit);
    }

    // this driver doesn't handle reasigning keys so it doesn't send
    // KEY_UNKNOWN code
    clear_bit(KEY_UNKNOWN, evfwd_input_dev->keybit);

    /* Register with the input subsystem */
    input_register_device(evfwd_input_dev);

    printk("'%s' Initialized.\n", DEVICE_NAME);
    return 0;
}

/* Driver Exit */
void
evfwd_cleanup(void)
{
    /* Unregister from the input subsystem */
    input_unregister_device(evfwd_input_dev);

    /* Cleanup sysfs node */
    sysfs_remove_group(&evfwd_dev->dev.kobj, &evfwd_attr_group);

    /* Unregister driver */
    platform_device_unregister(evfwd_dev);

    printk("'%s' unloaded.\n", DEVICE_NAME);

    return;
}

module_init(evfwd_init);
module_exit(evfwd_cleanup);
