/**
 * @file imx6irq.c
 * @author Jack (jackhuang021@gmail.com)
 * @brief linux中断实验
 * @version 0.1
 * @date 2022-01-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

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

/**
 * @brief 中断IO描述结构体
 * 
 */
struct irq_keydesc 
{
    int gpio;                               /** gpio value */
    int irqnum;                             /** 中断号 */
    unsigned char value;                    /** 按键对应键值 */
    char name[10];                          
    irqreturn_t (*handler)(int, void *);    /** 中断服务函数 */    
};

/**
 * @brief 中断设备结构体
 * 
 */
struct imx6uirq_dev
{
    dev_t devid;                            /** 设备号 */
    struct cdev cdev;                       /** cdev */
    struct class *class;                    /** 类 */
    struct device *device;                  /** 设备 */
    int major;                              /** 主节点号 */
    int minor;                              /** 次节点号 */
    struct device_node *nd;                 /** 设备节点 */
    atomic_t keyvalue;                     /** 按键值 */
    atomic_t releasekey;                   /** 标记是否完成一次按键 */
    struct timer_list timer;                /** 定义一个定时器 */    
    struct irq_keydesc irqkeydesc[KEY_NUM]; /** 按键描述数组 */
    unsigned char cur_keynum;               /** 当前的按键号 */
};

struct imx6uirq_dev imx6uirq;


/**
 * @brief key irq handler
 * 
 * @param irqnum 
 * @param dev_id 
 * @return irqreturn_t 
 */
static irqreturn_t key0_handler(int irqnum, void *dev_id)
{
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)dev_id;
    dev->cur_keynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10));
    return IRQ_RETVAL(IRQ_HANDLED);
}

/**
 * @brief timer handler function
 * 
 * @param arg irq device pointer
 */
void timer_function(unsigned long arg)
{
    unsigned char value;
    unsigned char num;
    struct irq_keydesc *keydesc;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)arg;

    num = imx6uirq.cur_keynum;
    keydesc = &dev->irqkeydesc[num];
    value = gpio_get_value(keydesc->gpio);
    if (value == 0) 
    {
        atomic_set(&dev->keyvalue, keydesc->value);
    }
    else
    {
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);
    }
}

/**
 * @brief key gpio and irq init 
 * 
 * @return int 
 */
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

    /* init key io, set irq mode */
    for (i = 0; i < KEY_NUM; i++)
    {
        memset(imx6uirq.irqkeydesc[i].name, 0, 
               sizeof(imx6uirq.irqkeydesc[i].name));
        sprintf(imx6uirq.irqkeydesc[i].name, "KEY%d", i);
        /* request a gpio pin and set gpio name */
        gpio_request(imx6uirq.irqkeydesc[i].gpio,
                     imx6uirq.irqkeydesc[i].name);
        gpio_direction_input(imx6uirq.irqkeydesc[i].gpio);
        imx6uirq.irqkeydesc[i].irqnum = irq_of_parse_and_map(imx6uirq.nd, i);
        printk("key %d : gpio %d, irqnum = %d\r\n", 
               i,
               imx6uirq.irqkeydesc[i].gpio,
               imx6uirq.irqkeydesc[i].irqnum);
    }

    /* request a irq */
    imx6uirq.irqkeydesc[0].handler = key0_handler;
    imx6uirq.irqkeydesc[0].value = KEY0VALUE;
    for (i = 0; i < KEY_NUM; i++)
    {
        ret = request_irq(imx6uirq.irqkeydesc[i].irqnum,
                          imx6uirq.irqkeydesc[i].handler,
                          IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                          imx6uirq.irqkeydesc[i].name, &imx6uirq);
        if (ret < 0)
        {
            printk("irq %s request failed.\r\n", imx6uirq.irqkeydesc[i].name);
            return -EFAULT;
        }
    }

    /* create timer */
    init_timer(&imx6uirq.timer);
    imx6uirq.timer.function = timer_function;
    return 0;
}

/**
 * @brief imx6ulirq device file operation: open 
 * 
 * @param inode device inode
 * @param filp set filp private data to device object
 * @return int 
 */
static int imx6uirq_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &imx6uirq;
    return 0;
}

static ssize_t imx6uirq_read(struct file *filp, char __user *buf, 
                             size_t cnt, loff_t *offt)
{
    int ret = 0;
    unsigned char keyvalue = 0;
    unsigned char releasekey = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)filp->private_data;
    keyvalue = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);

    if (releasekey)
    {
        if (keyvalue & 0x80)
        {
            keyvalue &= ~0x80;
            ret = copy_to_user(buf, &keyvalue, sizeof(keyvalue));
        }
        else
        {
            goto data_error;
        }
        atomic_set(&dev->releasekey, 0);
    }
    else
    {
        goto data_error;
    }

    return 0;

data_error:
    return -EINVAL;
}


static struct file_operations imx6uirq_fops =
{
    .owner = THIS_MODULE,
    .open = imx6uirq_open,
    .read = imx6uirq_read,
};


static int __init imx6uirq_init(void)
{
    if (imx6uirq.major)
    {
        imx6uirq.devid = MKDEV(imx6uirq.major, 0);
        register_chrdev_region(imx6uirq.devid, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
    }
    else
    {
        alloc_chrdev_region(&imx6uirq.devid, 0, IMX6UIRQ_CNT, IMX6UIRQ_NAME);
        imx6uirq.major = MAJOR(imx6uirq.devid);
        imx6uirq.minor = MINOR(imx6uirq.devid);
    }

    cdev_init(&imx6uirq.cdev, &imx6uirq_fops);
    cdev_add(&imx6uirq.cdev, imx6uirq.devid, IMX6UIRQ_CNT);
    
    imx6uirq.class = class_create(THIS_MODULE, IMX6UIRQ_NAME);
    if (IS_ERR(imx6uirq.class)) return PTR_ERR(imx6uirq.class);

    imx6uirq.device = device_create(imx6uirq.class, NULL, imx6uirq.devid,
                                    NULL, IMX6UIRQ_NAME);
    if (IS_ERR(imx6uirq.device)) return PTR_ERR(imx6uirq.device);

    atomic_set(&imx6uirq.keyvalue, INVAKEY);
    atomic_set(&imx6uirq.releasekey, 0);

    keyio_init();
    return 0;
}

static void __exit imx6uirq_exit(void)
{
    unsigned int i = 0;
    del_timer_sync(&imx6uirq.timer);

    for (i = 0; i < KEY_NUM; i++)
    {
        free_irq(imx6uirq.irqkeydesc[i].irqnum, &imx6uirq);
    }

    cdev_del(&imx6uirq.cdev);
    unregister_chrdev_region(imx6uirq.devid, IMX6UIRQ_CNT);
    device_destroy(imx6uirq.class, imx6uirq.devid);
    class_destroy(imx6uirq.class);
}

module_init(imx6uirq_init);
module_exit(imx6uirq_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jack Huang");
