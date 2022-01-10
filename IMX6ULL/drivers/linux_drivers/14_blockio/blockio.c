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
#define IMX6UIRQ_NAME       "blockio"
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
    atomic_t keyvalue;
    atomic_t releasekey;
    struct timer_list timer;
    struct irq_keydesc irqkeydesc[KEY_NUM];
    unsigned char cur_keynum;

    wait_queue_head_t r_wait;       /* 读等待队列头 */
};

struct imx6uirq_dev imx6uirq;

static irqreturn_t key0_handler(int irqnum, void *dev_id)
{
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)dev_id;
    dev->cur_keynum = 0;
    dev->timer.data = (volatile long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10));
    return IRQ_RETVAL(IRQ_HANDLED);
}

void timer_function(unsigned long arg)
{
    unsigned char value;
    unsigned char num;
    struct irq_keydesc *keydesc;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev *)arg;

    num = dev->cur_keynum;
    keydesc = &dev->irqkeydesc[num];
    value = gpio_get_value(keydesc->gpio);
    if (value == 0) atomic_set(&dev->keyvalue, keydesc->value);
    else
    {
        atomic_set(&dev->keyvalue, 0x80 | keydesc->value);
        atomic_set(&dev->releasekey, 1);
    }

    /* 唤醒进程 */
    if (atomic_read(&dev->releasekey))
    {
        wake_up_interruptible(&dev->r_wait);
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
        gpio_request(imx6uirq.irqkeydesc[i].gpio, imx6uirq.irqkeydesc[i].name);
        gpio_direction_input(imx6uirq.irqkeydesc[i].gpio);
        imx6uirq.irqkeydesc[i].irqnum = irq_of_parse_and_map(imx6uirq.nd, i);
        printk("key%d: gpio = %d, irqnum = %d\r\n", i, 
               imx6uirq.irqkeydesc[i].gpio, imx6uirq.irqkeydesc[i].irqnum);
    }

    /* request irq */
    imx6uirq.irqkeydesc[0].handler = key0_handler;
    imx6uirq.irqkeydesc[0].value = KEY0VALUE;

    for (i = 0; i < KEY_NUM; i++)
    {
        ret = request_irq(imx6uirq.irqkeydesc[i].irqnum,
                          imx6uirq.irqkeydesc[i].handler,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          imx6uirq.irqkeydesc[i].name, &imx6uirq);
        if (ret < 0)
        {
            printk("irq %d request failed.\r\n", imx6uirq.irqkeydesc[i].irqnum);
            return -EFAULT;
        }
    }

    /* create timer */
    init_timer(&imx6uirq.timer);
    imx6uirq.timer.function = timer_function;

    /* init wait queue */
    init_waitqueue_head(&imx6uirq.r_wait);

    return 0;
}

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

    /* add wait queue to wait key press */
    DECLARE_WAITQUEUE(wait, current);           /* define a wait queue */
    if (atomic_read(&dev->releasekey) == 0)
    {
        add_wait_queue(&dev->r_wait, &wait);    /* add to wait queue head */
        __set_current_state(TASK_INTERRUPTIBLE);/* set pending state */
        schedule();                             /* schedule task */
        if (signal_pending(current) == 0)       /* is signal wakeup return error */
        {
            ret = -ERESTARTSYS;
            goto wait_error;
        }

        __set_current_state(TASK_RUNNING);      /* set running state */
        remove_wait_queue(&dev->r_wait, &wait); /* remove wait queue from head */
    }
    
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

wait_error:
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&dev->r_wait, &wait);
    return ret;

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

    /* register char device */
    cdev_init(&imx6uirq.cdev, &imx6uirq_fops);
    cdev_add(&imx6uirq.cdev, imx6uirq.devid, IMX6UIRQ_CNT);

    /* create class */
    imx6uirq.class = class_create(THIS_MODULE, IMX6UIRQ_NAME);
    if (IS_ERR(imx6uirq.class)) return PTR_ERR(imx6uirq.class);

    /* create device */
    imx6uirq.device = device_create(imx6uirq.class, NULL,
                                    imx6uirq.devid, NULL, IMX6UIRQ_NAME);
    if (IS_ERR(imx6uirq.device)) return PTR_ERR(imx6uirq.device);

    /* init key */
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


