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
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "ap3216creg.h"


#define AP3216C_CNT         1
#define AP3216C_NAME        "ap3216c"


struct ap3216c_dev 
{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int major;
    void *private_data;
    unsigned short ir;
    unsigned short als;
    unsigned short ps;
};

static struct ap3216c_dev ap3216cdev;

static int ap3216c_read_regs(struct ap3216c_dev *dev, uint8_t reg, 
                             void *val, int len)
{
    int ret = 0;
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client *)dev->private_data;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = val;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret == 2) 
    {
        ret = 0;
    } 
    else 
    {
        printk("i2c read failed %d, reg = %06x len = %d\r\n", ret, reg, len);
        return -EREMOTEIO;
    }

    return ret;
}

static int ap3216c_write_regs(struct ap3216c_dev *dev, uint8_t reg, 
                              uint8_t *buf, uint8_t len)
{
    uint8_t buff[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client *)dev->private_data;
    
    buff[0] = reg;
    memcpy(&buff[1], buf, len);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.buf = buff;
    msg.len = len + 1;

    return i2c_transfer(client->adapter, &msg, 1);
}

static uint8_t ap3216c_read_reg(struct ap3216c_dev *dev, uint8_t reg)
{
    uint8_t data = 0;

    ap3216c_read_regs(dev, reg, &data, 1);
    return data;
}

static void ap3216c_write_reg(struct ap3216c_dev *dev, uint8_t reg, uint8_t data)
{
    uint8_t buf = 0;
    buf = data;
    ap3216c_write_regs(dev, reg, &buf, 1);
}

void ap3216c_readdata(struct ap3216c_dev *dev)
{
    int i = 0;
    uint8_t buff[6];

    for (i = 0; i < 6; i++)
    {
        buff[i] = ap3216c_read_reg(dev, AP3216C_IRDATALOW + i);
    }

    if (buff[0] & 0x80)
    {
        dev->ir = 0;
    }
    else
    {
        dev->ir = ((uint16_t)buff[1] << 2) | (buff[0] & 0x03);
    }

    dev->als = ((uint16_t)buff[3] << 8) | buff[2];

    if (buff[4] & 0x40)
    {
        dev->ps = 0;
    }
    else
    {
        dev->ps = ((uint16_t)(buff[5] & 0x3F) << 4) | (buff[4] & 0x0F);
    }
}


static int ap3216c_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &ap3216cdev;

    ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0x03);
    mdelay(50);
    ap3216c_write_reg(&ap3216cdev, AP3216C_SYSTEMCONG, 0x03);
    return 0;
}

static ssize_t ap3216c_read(struct file *filp, char __user *buf, 
                            size_t snt, loff_t *offt)
{
    uint16_t data[3];
    long err = 0;

    struct ap3216c_dev *dev = (struct ap3216c_dev *)filp->private_data;
    ap3216c_readdata(dev);
    data[0] = dev->ir;
    data[1] = dev->als;
    data[2] = dev->ps;
    err = copy_to_user(buf, data, sizeof(data));
    return 0;
}

static int ap3216c_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static const struct file_operations ap3216c_fops = 
{
    .owner = THIS_MODULE,
    .open = ap3216c_open,
    .read = ap3216c_read,
    .release = ap3216c_release,
};

static int ap3216c_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    printk("ap3216c probe\r\n");
    if (ap3216cdev.major)
    {
        ap3216cdev.devid = MKDEV(ap3216cdev.major, 0);
        register_chrdev_region(ap3216cdev.devid, AP3216C_CNT, AP3216C_NAME);
    }
    else
    {
        alloc_chrdev_region(&ap3216cdev.devid, 0, AP3216C_CNT, AP3216C_NAME);
        ap3216cdev.major = MAJOR(ap3216cdev.devid);
    }

    cdev_init(&ap3216cdev.cdev, &ap3216c_fops);
    cdev_add(&ap3216cdev.cdev, ap3216cdev.devid, AP3216C_CNT);
    
    ap3216cdev.class = class_create(THIS_MODULE, AP3216C_NAME);
    if (IS_ERR(ap3216cdev.class)) return PTR_ERR(ap3216cdev.class);

    ap3216cdev.device = device_create(ap3216cdev.class, NULL,
                                      ap3216cdev.devid, NULL, AP3216C_NAME);
    if (IS_ERR(ap3216cdev.device)) return PTR_ERR(ap3216cdev.device);

    ap3216cdev.private_data = client;

    return 0;
}

static int ap3216c_remove(struct i2c_client *client)
{
    cdev_del(&ap3216cdev.cdev);
    unregister_chrdev_region(ap3216cdev.devid, AP3216C_CNT);

    device_destroy(ap3216cdev.class, ap3216cdev.devid);
    class_destroy(ap3216cdev.class);
    return 0;
}

static const struct i2c_device_id ap3216c_id[] = 
{
    {"alientek,ap3216c", 0},
    {}
};

static const struct of_device_id ap3216c_of_match[] = 
{
    { .compatible = "alientek,ap3216c" },
    { }
};

static struct i2c_driver ap3216c_driver = 
{
    .probe = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = 
    {
        .owner = THIS_MODULE,
        .name = "ap3216c",
        .of_match_table = ap3216c_of_match,
    },
    .id_table = ap3216c_id,
};

static int __init ap3216c_init(void)
{
    int ret = 0;
    ret = i2c_add_driver(&ap3216c_driver);
    return ret;
}

static void __exit ap3216c_exit(void)
{
    i2c_del_driver(&ap3216c_driver);
}


module_init(ap3216c_init);
module_exit(ap3216c_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jack Huang");
