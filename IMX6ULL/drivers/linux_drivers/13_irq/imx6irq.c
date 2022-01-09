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
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define IMX6UIRQ_CNT        1
#define IMX6UIRQ_NAME       "imx6uirq"
#define KEY0VALUE           0x01
#define INVAKEY             0xFF
#define KEY_NUM             1

struct irq_keydesc 
{
    int gpio;                               /** gpio value */
    int irqnum;                             /** 中断号 */
    unsigned char value;                    /** 按键对应键值 */
    char name[10];                          
    irqreturn_t (*handler)(int, void *);    /** 中断服务函数 */    
};

struct imx6uirq_dev
{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    int major;
    int minor;
    struct device_node *nd;
    atomic_t key_value;
    atomic_t release_key;
    struct timer_list timer;
    struct irq_keydesc irqkeydesc[KEY_NUM];
    unsigned char cur_keynum;
};

struct imx6uirq_dev imx6uirq;

static irqreturn_t key0_handler(int irqnum, void *dev_id)
{
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)dev_id;
    dev->cur_keynum = 0;
    dev->timer.data = (volatile long *)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10));
    return IRQ_RETVAL(IRQ_HANDLED);
}

void timer_function(unsigned long arg)
{
    unsigned char value;
    unsigned char num;
    struct irq_keydesc *keydesc;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)arg;

    num = imx6uirq_dev->cur_keynum;
    keydesc = &dev->irqkeydesc[num];
    value = gpio_get_value(keydesc->gpio);
    if (value == 0) atomic_set(&dev->keyvalue, keydesc->value);
    else
    {
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->release_key, 1);
    }
}

static int keyio_init(void)
{
    unsigned char i = 0;
    int ret = 0;
    
    imx6uirq.nd = of_find_node_by_path("/key");
    if (imx6uirq.nd == NULL) 
    {
        printk("can't find key node.\r\n");
        return -EINVAL;
    }

    /* get gpio num */
    for (i = 0; i < KEY_NUM; i++)
    {
        imx6uirq.irqkeydesc[i].gpio = of_get_named_gpio(imx6uirq.nd, 
                                                        "key-gpio", i);
        if (imx6uirq.irqkeydesc[i].gpio < 0) 
        {
            printk("can't get key %d.\r\n", i);
        }
    }

    for (i = 0; i < KEY_NUM; i++)
    {
        memset(imx6uirq.irqkeydesc[i].name, 0, 
               sizeof(imx6uirq.irqkeydesc[i].name));
        sprintf(imx6uirq.irqkeydesc[i].name, "key %d", i);
        gpio_request(imx)
    }

    
}