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
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define LEDDEV_CNT      1
#define LEDDEV_NAME     "paltled"
#define LEDOFF          0
#define LEDON           1

static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

struct leddev_dev
{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    int major;
};

struct leddev_dev leddev;

void led0_switch(uint8_t sta)
{
    uint32_t val = 0;
    if (sta == LEDON)
    {
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);
        writel(val, GPIO1_DR);
    }
    else
    {
        val = readl(GPIO1_DR);
        val |= (1 << 3);
        writel(val, GPIO1_DR);
    }
}

static int led_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &leddev;
    return 0;
}

static ssize_t led_write(struct file *filp, const char __user *buf,
                         size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    unsigned char databuf[1];
    unsigned char led_state;
    
    retvalue = copy_from_user(databuf, buf, cnt);
    if (retvalue < 0)
    {
        return -EFAULT;
    }

    led_state = databuf[0];
    if (led_state == LEDON) 
    {
        led0_switch(LEDON);
    }
    else if (led_state == LEDOFF) 
    {
        led0_switch(LEDOFF);
    }

    return 0;
}

static struct file_operations led_fops = 
{
    .owner = THIS_MODULE,
    .open = led_open,
    .write = led_write,
};

static int led_probe(struct platform_device *dev)
{
    int i = 0;
    int ressize[5];
    uint32_t val = 0;
    struct resource *ledsource[5];
    
    printk("led driver and device has matched.\r\n");
    for (i = 0; i < 5; i++)
    {
        ledsource[i] = platform_get_resource(dev, IORESOURCE_MEM, i);
        if (!ledsource[i])
        {
            dev_err(&dev->dev, "no mem resource for always on\n");
            return -ENXIO;
        }
        ressize[i] = resource_size(ledsource[i]);
    }

    IMX6U_CCM_CCGR1 = ioremap(ledsource[0]->start, ressize[0]);
    SW_MUX_GPIO1_IO03 = ioremap(ledsource[1]->start, ressize[1]);
    SW_PAD_GPIO1_IO03 = ioremap(ledsource[2]->start, ressize[2]);
    GPIO1_DR = ioremap(ledsource[3]->start, ressize[3]);
    GPIO1_GDIR = ioremap(ledsource[4]->start, ressize[4]);

    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);
    val |= (3 << 26);
    writel(val, IMX6U_CCM_CCGR1);

    writel(5, SW_MUX_GPIO1_IO03);
    writel(0x10B0, SW_PAD_GPIO1_IO03);

    val = readl(GPIO1_GDIR);
    val &= ~(1 << 3);
    val |= (1 << 3);
    writel(val, GPIO1_GDIR);

    val = readl(GPIO1_DR);
    val |= (1 << 3);
    writel(val, GPIO1_DR);

    if (leddev.major)
    {
        leddev.devid = MKDEV(leddev.major, 0);
        register_chrdev_region(leddev.devid, LEDDEV_CNT, LEDDEV_NAME);
    }
    else
    {
        alloc_chrdev_region(&leddev.devid, 0, LEDDEV_CNT, LEDDEV_NAME);
        leddev.major = MAJOR(leddev.devid);
    }

    leddev.cdev.owner = THIS_MODULE;
    cdev_init(&leddev.cdev, &led_fops);
    
    cdev_add(&leddev.cdev, leddev.devid, LEDDEV_CNT);

    leddev.class = class_create(THIS_MODULE, LEDDEV_NAME);
    if (IS_ERR(leddev.class)) return PTR_ERR(leddev.class);

    leddev.device = device_create(leddev.class, NULL, leddev.devid,
                                  NULL, LEDDEV_NAME);
    if (IS_ERR(leddev.device)) return PTR_ERR(leddev.device);

    return 0;
}

static int led_remove(struct platform_device *dev)
{
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_GDIR);
    iounmap(GPIO1_DR);

    cdev_del(&leddev.cdev);
    unregister_chrdev_region(leddev.devid, LEDDEV_CNT);
    device_destroy(leddev.class, leddev.devid);
    class_destroy(leddev.class);
    return 0;
}

static struct platform_driver led_driver = 
{
    .driver = 
    {
        .name = "imx6ul-led",
    },
    .probe = led_probe,
    .remove = led_remove,
};


static int __init leddriver_init(void)
{
    return platform_driver_register(&led_driver);
}

static void __exit leddriver_exit(void)
{
    platform_driver_unregister(&led_driver);
}

module_init(leddriver_init);
module_exit(leddriver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jack Huang");


