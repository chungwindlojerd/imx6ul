#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>

#define GT_CTRL_REG             0X8040  /* GT911控制寄存器          */
#define GT_MODSW_REG 	        0X804D  /* GT911模式切换寄存器      */
#define GT_CFGS_REG 	        0X8047  /* GT911配置起始地址寄存器          */
#define GT_CHECK_REG 	        0X80FF  /* GT911校验和寄存器            */
#define GT_PID_REG 		        0X8140  /* GT911产品ID寄存器         */

#define GT_GSTID_REG 	        0X814E  /* GT911当前检测到的触摸情况 */
#define MAX_SUPPORT_POINTS      5       /* 最多5点电容触摸 */

#define GT911_CONTACT_SIZE      8

struct gt911_dev {
	int irq_pin,reset_pin;					/* 中断和复位IO		    */
	int irqnum;								/* 中断号           */
    void *private_data;						/* 私有数据         */
	struct input_dev *input;				/* input结构体 		*/
	struct i2c_client *client;				/* I2C客户端 		*/

};
struct gt911_dev gt911;


const unsigned char GT911_CT[]=
{
	0x48,0xe0,0x01,0x10,0x01,0x05,0x0d,0x00,0x01,0x08,
	0x28,0x05,0x50,0x32,0x03,0x05,0x00,0x00,0xff,0xff,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x89,0x28,0x0a,
	0x17,0x15,0x31,0x0d,0x00,0x00,0x02,0x9b,0x03,0x25,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,
	0x00,0x0f,0x94,0x94,0xc5,0x02,0x07,0x00,0x00,0x04,
	0x8d,0x13,0x00,0x5c,0x1e,0x00,0x3c,0x30,0x00,0x29,
	0x4c,0x00,0x1e,0x78,0x00,0x1e,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x08,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x16,
	0x18,0x1a,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0x00,0x02,0x04,0x05,0x06,0x08,0x0a,0x0c,
	0x0e,0x1d,0x1e,0x1f,0x20,0x22,0x24,0x28,0x29,0xff,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,
};

/*
 * @description     : 复位GT911
 * @param - client 	: 要操作的i2c
 * @param - multidev: 自定义的multitouch设备
 * @return          : 0，成功;其他负值,失败
 */
static int gt911_ts_reset(struct i2c_client *client, struct gt911_dev *dev)
{
	int ret = 0;

    /* 申请复位IO*/
	if (gpio_is_valid(dev->reset_pin)) {  		
		ret = devm_gpio_request_one(&client->dev,	
					dev->reset_pin, GPIOF_OUT_INIT_LOW,
					"gt911 reset");
		if (ret) {
			return ret;
		}
	}

    /* 申请中断IO*/
	if (gpio_is_valid(dev->irq_pin)) {  		
		ret = devm_gpio_request_one(&client->dev,	
					dev->irq_pin, GPIOF_OUT_INIT_LOW,
					"gt911 int");
		if (ret) {
			return ret;
		}
	}

    gpio_set_value(dev->reset_pin, 0); 
    msleep(10);
    gpio_set_value(dev->reset_pin, 1);
    msleep(10);
    gpio_set_value(dev->irq_pin, 0);
    msleep(50);
    gpio_direction_input(dev->irq_pin);

	return 0;
}

/*
 * @description	: 从GT911读取多个寄存器数据
 * @param - dev:  GT911设备
 * @param - reg:  要读取的寄存器首地址
 * @param - buf:  读取到的数据
 * @param - len:  要读取的数据长度
 * @return 		: 操作结果
 */
static int gt911_read_regs(struct gt911_dev *dev, u16 reg, u8 *buf, int len)
{
	int ret;
    u8 regdata[2];
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->client;
    
    /* GT911寄存器长度为2个字节 */
    regdata[0] = reg >> 8;
    regdata[1] = reg & 0xFF;

	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = client->addr;			/* gt911地址 */
	msg[0].flags = !I2C_M_RD;			/* 标记为发送数据 */
	msg[0].buf = &regdata[0];			/* 读取的首地址 */
	msg[0].len = 2;						/* reg长度*/

	/* msg[1]读取数据 */
	msg[1].addr = client->addr;			/* ft5x06地址 */
	msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	msg[1].buf = buf;					/* 读取数据缓冲区 */
	msg[1].len = len;					/* 要读取的数据长度*/

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		ret = -EREMOTEIO;
	}
	return ret;
}

/*
 * @description	: 向GT911多个寄存器写入数据
 * @param - dev:  GT911设备
 * @param - reg:  要写入的寄存器首地址
 * @param - val:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   操作结果
 */
static s32 gt911_write_regs(struct gt911_dev *dev, u16 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->client;
	
	b[0] = reg >> 8;			/* 寄存器首地址低8位 */
    b[1] = reg & 0XFF;			/* 寄存器首地址高8位 */
	memcpy(&b[2],buf,len);		/* 将要写入的数据拷贝到数组b里面 */

	msg.addr = client->addr;	/* gt911地址 */
	msg.flags = 0;				/* 标记为写数据 */

	msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 2;			/* 要写入的数据长度 */

	return i2c_transfer(client->adapter, &msg, 1);
}

/**
 * @brief gt911 写入单个寄存器值
 * 
 * @param dev 
 * @param reg 
 * @param data 
 */
static void gt911_write_reg(struct gt911_dev *dev, uint16_t reg, uint8_t data)
{
    uint8_t buf = 0;
    buf = data;
    gt911_write_regs(dev, reg, &buf, 1);
}

/**
 * @brief 获取gt911配置信息
 * 
 * @param dev 
 */
static void gt911_get_info(struct gt911_dev *dev)
{
    uint8_t rdbuff[11];
    int ret = 0;
    
    memset(rdbuff, 0, sizeof(rdbuff));

    ret = gt911_read_regs(dev, 0x8140, rdbuff, 11);
    if (ret != 0)
    {
        dev_err(&dev->client->dev, "I2C transfer error: %d\n", ret);
    }

    printk("Touch ID: GT%.3s\r\n", rdbuff);
    printk("Hardware Version: 0x%4x\r\n", rdbuff[4] + (rdbuff[5] << 8));
    printk("Touch Resolution: %dx%d \r\n", rdbuff[6] + (rdbuff[7] << 8),
           rdbuff[8] + (rdbuff[9] << 8));
    printk("\r\n");
}


/**
 * @brief gt911读取触摸点坐标并上报
 * 
 * @param dev 
 */
static void gt911_read_points(struct gt911_dev *dev)
{
    uint8_t rdbuff[1 + MAX_SUPPORT_POINTS * GT911_CONTACT_SIZE];
    int i = 0;
    int ret = 0;
    int cnt = 0;
    uint8_t touch_num = 0;
    
    memset(rdbuff, 0, sizeof(rdbuff));

    ret = gt911_read_regs(dev, GT_GSTID_REG, rdbuff,
                        1 + GT911_CONTACT_SIZE);
    if (ret)
    {
        dev_err(&dev->client->dev, "I2C transfer error: %d\n", ret);
        return;
    }
    
    cnt = 0;
    while (!(rdbuff[0] & 0x80))
    {
        mdelay(1);
        ret = gt911_read_regs(dev, GT_GSTID_REG, rdbuff,
                    1 + GT911_CONTACT_SIZE);
        cnt ++;
        if (cnt > 20) 
        {
            for (i = 0; i < MAX_SUPPORT_POINTS; i++)
            {
                input_mt_slot(dev->input, i);
                input_mt_report_slot_state(dev->input, MT_TOOL_FINGER, false);
            }
            return;
        }
    }

    /* 上报触摸事件 */
    touch_num = rdbuff[0] & 0x0f;
    

    if (touch_num > 1)
    {
        ret = gt911_read_regs(dev, GT_GSTID_REG + 1 + GT911_CONTACT_SIZE,
                              rdbuff + GT911_CONTACT_SIZE + 1, 
                              GT911_CONTACT_SIZE * (touch_num - 1));
        if (ret)
        {
            dev_err(&dev->client->dev, "I2C transfer error: %d\n", ret);
            return;
        }
    }

    /* 上报触摸事件 */
	for (i = 0; i < touch_num; i++)
	{
        int index = i * GT911_CONTACT_SIZE + 1;
        int id = rdbuff[index] & 0x0F;
        int input_x = get_unaligned_le16(&rdbuff[index + 1]);
        int input_y = get_unaligned_le16(&rdbuff[index + 3]);
        int input_w = get_unaligned_le16(&rdbuff[index + 5]);

        //printk("id%d: x = %d, y = %d\r\n", id, input_x, input_y);

        input_mt_slot(dev->input, id);
        input_mt_report_slot_state(dev->input, MT_TOOL_FINGER, true);
        input_report_abs(dev->input, ABS_MT_POSITION_X, input_x);
        input_report_abs(dev->input, ABS_MT_POSITION_Y, input_y);
        input_report_abs(dev->input, ABS_MT_TOUCH_MAJOR, input_w);
	    input_report_abs(dev->input, ABS_MT_WIDTH_MAJOR, input_w);
    }

    for (i = touch_num; i < MAX_SUPPORT_POINTS; i++)
    {
        //if (i == 0) printk("id0 release.\r\n");
        input_mt_slot(dev->input, i);
        input_mt_report_slot_state(dev->input, MT_TOOL_FINGER, false);
    }

	input_mt_sync_frame(dev->input);
	input_sync(dev->input);

    gt911_write_reg(dev, GT_GSTID_REG, 0);
}



static irqreturn_t gt911_irq_handler(int irq, void *dev_id)
{
    struct gt911_dev *dev = dev_id;

    disable_irq_nosync(dev->client->irq);
    gt911_read_points(dev);
    enable_irq(dev->client->irq);

	return IRQ_HANDLED;
}


/*
 * @description     : GT911中断初始化
 * @param - client 	: 要操作的i2c
 * @param - multidev: 自定义的multitouch设备
 * @return          : 0，成功;其他负值,失败
 */
static int gt911_ts_irq(struct i2c_client *client, struct gt911_dev *dev)
{
	int ret = 0;

	/* 2，申请中断,client->irq就是IO中断， */
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					gt911_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					client->name, &gt911);
	if (ret) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		return ret;
	}

	return 0;
}

/*
 * @description	: 发送GT911配置参数
 * @param - client: i2c_client
 * @param - mode: 0,参数不保存到flash
 *                1,参数保存到flash
 * @return 		: 无
 */
void gt911_send_cfg(struct gt911_dev *dev, unsigned char mode)
{
	unsigned char buf[2];
	unsigned int i = 0;

	buf[0] = 0;
	buf[1] = mode;	/* 是否写入到GT911 FLASH?  即是否掉电保存 */
	for(i = 0; i < (sizeof(GT911_CT)); i++) /* 计算校验和 */
        buf[0] += GT911_CT[i];            
    buf[0] = (~buf[0]) + 1;

    /* 发送寄存器配置 */
    gt911_write_regs(dev, GT_CFGS_REG, (u8 *)GT911_CT, sizeof(GT911_CT));
    gt911_write_regs(dev, GT_CHECK_REG, buf, 2);/* 写入校验和,配置更新标记 */
} 

int gt911_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    u8 data, ret;

    gt911.client = client;

 	/* 1，获取设备树中的中断和复位引脚 */
	gt911.irq_pin = of_get_named_gpio(client->dev.of_node, "interrupt-gpios", 0);
	gt911.reset_pin = of_get_named_gpio(client->dev.of_node, "reset-gpios", 0);

	/* 2，复位GT911 */
	ret = gt911_ts_reset(client, &gt911);
	if(ret < 0) {
		goto fail;
    }

    /* 3，初始化GT911 */
    data = 0x02;
    gt911_write_regs(&gt911, GT_CTRL_REG, &data, 1); /* 软复位 */
    mdelay(100);
    data = 0x0;
    gt911_write_regs(&gt911, GT_CTRL_REG, &data, 1); /* 停止软复位 */
    mdelay(100);

    /* 4,初始化GT911，烧写固件 
    gt911_read_regs(&gt911, GT_CFGS_REG, &data, 1);
    printk("GT911 ID =%#X\r\n", data);
    if(data <  GT911_CT[0]) {
       gt911_send_cfg(&gt911, 0);   芯片内置固件已能够使用，无需下载固件
    } */

    /* 5，input设备注册 */
	gt911.input = devm_input_allocate_device(&client->dev);
	if (!gt911.input) {
		ret = -ENOMEM;
		goto fail;
	}
	gt911.input->name = client->name;
	gt911.input->id.bustype = BUS_I2C;
	gt911.input->dev.parent = &client->dev;

	__set_bit(EV_KEY, gt911.input->evbit);
	__set_bit(EV_ABS, gt911.input->evbit);
	__set_bit(BTN_TOUCH, gt911.input->keybit);

	input_set_abs_params(gt911.input, ABS_X, 0, 480, 0, 0);
	input_set_abs_params(gt911.input, ABS_Y, 0, 272, 0, 0);
	input_set_abs_params(gt911.input, ABS_MT_POSITION_X,0, 480, 0, 0);
	input_set_abs_params(gt911.input, ABS_MT_POSITION_Y,0, 272, 0, 0);	     
	ret = input_mt_init_slots(gt911.input, MAX_SUPPORT_POINTS, 0);
	if (ret) {
		goto fail;
	}

	ret = input_register_device(gt911.input);
	if (ret)
		goto fail;

    /* 6，最后初始化中断 */
	ret = gt911_ts_irq(client, &gt911);
	if(ret < 0) {
		goto fail;
	}

    gt911_get_info(&gt911);


    return 0;

fail:
	return ret;
}

/*
 * @description     : i2c驱动的remove函数，移除i2c驱动的时候此函数会执行
 * @param - client 	: i2c设备
 * @return          : 0，成功;其他负值,失败
 */
int gt911_remove(struct i2c_client *client)
{
    input_unregister_device(gt911.input);
    return 0;
}

/*
 *  传统驱动匹配表
 */ 
const struct i2c_device_id gt911_id_table[] = {
	{ "alientek,gt911", 0, },
    { /* sentinel */ }
};

/*
 * 设备树匹配表 
 */
const struct of_device_id gt911_of_match_table[] = {
    {.compatible = "alientek,gt911" },
    { /* sentinel */ }
};

/* i2c驱动结构体 */	
struct i2c_driver gt911_i2c_driver = {
    .driver = {
        .name  = "gt911",
        .owner = THIS_MODULE,
        .of_match_table = gt911_of_match_table,
    },
    .id_table = gt911_id_table,
    .probe  = gt911_probe,
    .remove = gt911_remove,
};

module_i2c_driver(gt911_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");

