/**
 * @file dtsled.c
 * @author Jack (jackhuang021@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-31
 * 
 * @copyright Copyright (c) 2021
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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define DTSLED_CNT          1
#define DTSLED_NAME         "dtsled"
#define LED_OFF             0
#define LED_ON              1

static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

struct dtsled_drv
{
    dev_t devid;            /** device id */
    struct cdev cdev;       /** cdev */
    struct class *class;    /** dev class */
    struct device *device;
    int major;              /** major device number */
    int minor;              /** minor device number */
    struct device_node *nd; /** device node */        
};

struct dtsled_drv dtsled;

void led_switch(u8 sta)
{
    u32 val = 0;
    if (sta == LED_ON)
    {
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);
        writel(val, GPIO1_DR);
    }
    else if (sta == LED_OFF)
    {
        val = readl(GPIO1_DR);
        val |= (1 << 3);
        writel(val, GPIO1_DR);
    }
}

static int led_open(struct inode *inode, struct file *flip)
{
    flip->private_data = &dtsled;
    return 0;
}

static ssize_t led_read(struct file *flip,
                       char __user *buf,
                       size_t cnt,
                       loff_t *offt)
{
    return 0;
}            

static ssize_t led_write(struct file *flip,
                         const char __user *buf,
                         size_t cnt,
                         loff_t *offt)
{
    int retval = 0;
    unsigned char databuf[1];
    unsigned char ledState;

    retval = copy_from_user(databuf, buf, cnt);
    if (retval < 0)
    {
        printk("kernel write failed.\r\n");
        return -EFAULT;
    }
    
    ledState = databuf[0];
    if (ledState == LED_ON)
    {
        led_switch(LED_ON);
    }
    else if (ledState == LED_OFF)
    {
        led_switch(LED_OFF);
    }
    return 0;
}

static int led_release(struct inode *inode, struct file *flip)
{
    return 0;
}

static struct file_operations dtsled_fops = 
{
    .owner = THIS_MODULE,
    .open = led_open,
    .read = led_read,
    .write = led_write,
    .release = led_release,
};


static int __init led_init(void)
{
    u32 val = 0;
    int ret = 0;
    u32 regdata[14];
    const char *str;
    struct property *proper;

    /* find device node */
    dtsled.nd = of_find_node_by_path("/alphaled");
    if (dtsled.nd == NULL)
    {
        printk("alphaled node can not found.\r\n");
        return -EINVAL;
    }
    else
    {
        printk("alphaled node has been found.\r\n");
    }

    /* get node compatible property info */
    proper = of_find_property(dtsled.nd, "compatible", NULL);
    if (proper == NULL)
    {
        printk("compatible property find failed.\r\n");
    }
    else
    {
        printk("compatible = %s\r\n", (char *)proper->value);
    }

    /* get node status property info */
    ret = of_property_read_string(dtsled.nd, "status", &str);
    if (ret < 0)
    {
        printk("status read failed.\r\n");
    }
    else
    {
        printk("status = %s\r\n", str);
    }

    /* get reg info */
    ret = of_property_read_u32_array(dtsled.nd, "reg", regdata, 10);
    if (ret < 0)
    {
        printk("reg property read failed.\r\n");
    }
    else
    {
        u8 i = 0;
        printk("reg data:\r\n");
        for (i = 0; i < 10; i++)
        {
            printk("%#X ", regdata[i]);
        }
        printk("\r\n");
    }

    IMX6U_CCM_CCGR1 = of_iomap(dtsled.nd, 0);
    SW_MUX_GPIO1_IO03 = of_iomap(dtsled.nd, 1);
    SW_PAD_GPIO1_IO03 = of_iomap(dtsled.nd, 2);
    GPIO1_DR = of_iomap(dtsled.nd, 3);
    GPIO1_GDIR = of_iomap(dtsled.nd, 4);

    /* enable GPIO1 clock */
    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);
    val |= (3 << 26);
    writel(val, IMX6U_CCM_CCGR1);

    /* set GPIO1 MUX */
    writel(5, SW_MUX_GPIO1_IO03);

    /* set GPIO property */
    writel(0x10B0, SW_PAD_GPIO1_IO03);

    /* set output direction */
    val = readl(GPIO1_GDIR);
    val &= ~(1 << 3);
    val |= (1 << 3);
    writel(val, GPIO1_GDIR);

    /* set goio low default */
    val = readl(GPIO1_DR);
    val |= (1 << 3);
    writel(val, GPIO1_DR);

    if (dtsled.major)
    {
        dtsled.devid = MKDEV(dtsled.major, 0);
        register_chrdev_region(dtsled.devid, DTSLED_CNT, DTSLED_NAME);
    }
    {
        alloc_chrdev_region(&dtsled.devid, 0, DTSLED_CNT, DTSLED_NAME);
        dtsled.major = MAJOR(dtsled.devid);
        dtsled.minor = MINOR(dtsled.devid);
    }

    printk("dtsled major = %d, minor = %d\r\n", dtsled.major, dtsled.minor);

    /* init cdev */
    dtsled.cdev.owner = THIS_MODULE;
    cdev_init(&dtsled.cdev, &dtsled_fops);

    /* add a cdev */
    cdev_add(&dtsled.cdev, dtsled.devid, DTSLED_CNT);

    /* create class */
    dtsled.class = class_create(THIS_MODULE, DTSLED_NAME);
    if (IS_ERR(dtsled.class))
    {
        return PTR_ERR(dtsled.class);
    }

    /* create device */
    dtsled.device = device_create(dtsled.class, NULL, dtsled.devid,
                                  NULL, DTSLED_NAME);
    if (IS_ERR(dtsled.device))
    {
        return PTR_ERR(dtsled.device);
    }                                     
    
    return 0;
}

static void __exit led_exit(void)
{
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    cdev_del(&dtsled.cdev);
    unregister_chrdev_region(dtsled.devid, DTSLED_CNT);
    device_destroy(dtsled.class, dtsled.devid);
    class_destroy(dtsled.class);
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jack Huang");






