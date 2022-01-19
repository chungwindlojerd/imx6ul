#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_access.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


#define KEYINPUT_CNT        1
#define KEYINPUT_NAME       "keyinput"
#define KEY0VALUE           0x01
#define INVAKEY             0xFF
#define KEY_NUM             1


struct irq_keydesc
{
    int gpio;
    int irqnum;
    unsigned char value;
    char name[20];
    irqreturn_t (*handler)(int, void *);
};

struct keyinput_dev = 
{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    struct timer_list timer;
    struct irq_keydesc irqkeydesc[KEY_NUM];
    unsigned char cur_keynum;
    struct input_dev *inputdev;
};


struct keyinput_dev keyinputdev;

static irqreturn_t key0_handler(int irqnum, void *dev_id)
{
    struct keyinput_dev *dev = (struct keyinput_dev *)dev_id;
    dev->cur_keynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10));
    return IRQ_RETVAL(IRQ_HANDLED);
}



