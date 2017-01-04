/*
 * Copyright (C) 2006-2015 ILITEK TECHNOLOGY CORP.
 *
 * Description: ILITEK I2C touchscreen driver for linux platform.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Author: Steward Fu
 * Maintain: Luca Hsu, Tigers Huang
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,8)
    #include <linux/input/mt.h>
#endif
#include <linux/input/ilitek.h>

//module information
MODULE_AUTHOR("Steward_Fu");
MODULE_DESCRIPTION("ILITEK I2C touchscreen driver for linux platform");
MODULE_LICENSE("GPL");

int driver_information[] = {DERVER_VERSION_MAJOR, DERVER_VERSION_MINOR, RELEASE_VERSION};

int touch_key_hold_press = 0;
#ifdef VIRTUAL_KEY_PAD
int touch_key_code[] = {KEY_MENU, KEY_HOME/*KEY_HOMEPAGE*/, KEY_BACK, KEY_VOLUMEDOWN, KEY_VOLUMEUP};
int touch_key_press[] = {0, 0, 0, 0, 0};
#endif
unsigned long touch_time = 0;

static int ilitek_i2c_register_device(void);
static void ilitek_set_input_param(struct input_dev*, int, int, int);
static int ilitek_i2c_read_tp_info(void);
static int ilitek_init(void);
static void ilitek_exit(void);

//i2c functions
static int ilitek_i2c_transfer(struct i2c_client*, struct i2c_msg*, int);
static int ilitek_i2c_write(struct i2c_client*, uint8_t, int);
static int ilitek_i2c_read(struct i2c_client*, uint8_t*, int);
static int ilitek_i2c_write_read(struct i2c_client*, uint8_t, int, uint8_t*, int);
static int ilitek_i2c_process_and_report(void);
static int ilitek_i2c_suspend(struct i2c_client*, pm_message_t);
static int ilitek_i2c_resume(struct i2c_client*);
static void ilitek_i2c_shutdown(struct i2c_client*);
static int ilitek_i2c_probe(struct i2c_client*, const struct i2c_device_id*);
static int ilitek_i2c_remove(struct i2c_client*);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ilitek_i2c_early_suspend(struct early_suspend *h);
static void ilitek_i2c_late_resume(struct early_suspend *h);
#endif

static int ilitek_i2c_polling_thread(void*);
static irqreturn_t ilitek_i2c_isr(int, void*);
static void ilitek_i2c_irq_work_queue_func(struct work_struct*);

//file operation functions
static int ilitek_file_open(struct inode*, struct file*);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static ssize_t ilitek_file_write(struct file*, const char*, size_t, loff_t*);
static ssize_t ilitek_file_read(struct file*, char*, size_t, loff_t*);
static int ilitek_file_close(struct inode*, struct file*);

static void ilitek_i2c_irq_enable(void);
static void ilitek_i2c_irq_disable(void);

static int ilitek_i2c_reset(void);

//global variables
static struct i2c_data i2c;
static struct dev_data dev;
static char DBG_FLAG;
static char Report_Flag;
volatile static char int_Flag;
volatile static char update_Flag;
static int update_timeout;
int gesture_flag, gesture_count, getstatus;
static int get_time_diff(struct timeval *past_time);//

//i2c id table
static const struct i2c_device_id ilitek_i2c_id[] = {
    {ILITEK_I2C_DRIVER_NAME, 0}, {}
};
MODULE_DEVICE_TABLE(i2c, ilitek_i2c_id);

//declare i2c function table
static struct i2c_driver ilitek_i2c_driver = {
    .id_table = ilitek_i2c_id,
    .driver = {.name = ILITEK_I2C_DRIVER_NAME},
    //	.resume = ilitek_i2c_resume,
    //    .suspend  = ilitek_i2c_suspend,
    .shutdown = ilitek_i2c_shutdown,
    .probe = ilitek_i2c_probe,
    .remove = ilitek_i2c_remove,
};

//declare file operations
struct file_operations ilitek_fops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
    .unlocked_ioctl = ilitek_file_ioctl,
#else
    .ioctl = ilitek_file_ioctl,
#endif
    .read = ilitek_file_read,
    .write = ilitek_file_write,
    .open = ilitek_file_open,
    .release = ilitek_file_close,
};

/*
description
    open function for character device driver
prarmeters
    inode
        inode
    filp
        file pointer
return
    status
*/
static int ilitek_file_open(struct inode *inode, struct file *filp)
{
    DBG("%s\n", __func__);
    return 0;
}

/*
description
    write function for character device driver
prarmeters
    filp
        file pointer
    buf
        buffer
    count
        buffer length
    f_pos
        offset
return
    status
*/
static ssize_t ilitek_file_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    int ret;

    unsigned char buffer[128] = {0};

    //before sending data to touch device, we need to check whether the device is working or not
    if(i2c.valid_i2c_register == 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, i2c device driver doesn't be registered\n", __func__);
        return -1;
    }

    //check the buffer size whether it exceeds the local buffer size or not
    if(count > 128)
    {
        printk(ILITEK_ERROR_LEVEL "%s, buffer exceed 128 bytes\n", __func__);
        return -1;
    }

    //copy data from user space
    ret = copy_from_user(buffer, buf, count - 1);
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed", __func__);
        return -1;
    }

    //parsing command
    if(strcmp(buffer, "dbg") == 0)
    {
        DBG_FLAG = !DBG_FLAG;
        printk("%s, %s message(%X).\n", __func__, DBG_FLAG ? "Enabled" : "Disabled", DBG_FLAG);
    }
    else if(strcmp(buffer, "info") == 0)
    {
        ilitek_i2c_read_tp_info();
    }
    else if(strcmp(buffer, "report") == 0)
    {
        Report_Flag =! Report_Flag;
    }
    else if(strcmp(buffer, "stop_report") == 0)
    {
        i2c.report_status = 0;
        printk("The report point function is disable.\n");
    }
    else if(strcmp(buffer, "start_report") == 0)
    {
        i2c.report_status = 1;
        printk("The report point function is enable.\n");
    }
    else if(strcmp(buffer, "update_flag") == 0)
    {
        printk("update_Flag=%d\n", update_Flag);
    }
    else if(strcmp(buffer, "irq_status") == 0)
    {
        printk("i2c.irq_status = %d\n", i2c.irq_status);
    }
    else if(strcmp(buffer, "disable_irq") == 0)
    {
        ilitek_i2c_irq_disable();
        printk("i2c.irq_status = %d\n", i2c.irq_status);
    }
    else if(strcmp(buffer, "enable_irq") == 0)
    {
        ilitek_i2c_irq_enable();
        printk("i2c.irq_status=%d\n", i2c.irq_status);
    }
    else if(strcmp(buffer, "reset") == 0)
    {
        printk("start reset\n");
        ilitek_i2c_reset();
        printk("end reset\n");
    }
    else if(strcmp(buffer, "gesture") == 0)
    {
        printk("start gesture\n");
        gesture_flag =! gesture_flag;
        printk("%s, %s gesture_flag(%X).\n", __func__, gesture_flag ? "Enabled" : "Disabled", gesture_flag);
        printk("end gesture\n");
    }
#ifdef GESTURE_FUN
#if GESTURE_FUN == GESTURE_FUN1
    else if(strcmp(buffer, "readgesturelist") == 0)
    {
        printk("read gesture.txt start\n");
        readgesturelist();
        printk("read gesture.txt end\n");
    }
#endif
#endif
    return -1;
}

/*
description
    ioctl function for character device driver
prarmeters
    inode
        file node
    filp
        file pointer
    cmd
        command
    arg
        arguments
return
    status
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    static unsigned char buffer[64] = {0};
    static int len = 0, i;
    int ret;
    struct i2c_msg msgs[] = {
        {.addr = i2c.client->addr, .flags = 0, .len = len, .buf = buffer,}
    };

    //parsing ioctl command
    switch(cmd)
    {
        case ILITEK_IOCTL_I2C_WRITE_DATA:
            ret = copy_from_user(buffer, (unsigned char*)arg, len);
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
                return -1;
            }
        #ifdef	SET_RESET
            if(buffer[0] == 0x60)
            {
                ilitek_i2c_reset();
            }
        #endif
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, i2c write, failed\n", __func__);
                return -1;
            }
            break;
        case ILITEK_IOCTL_I2C_READ_DATA:
            msgs[0].flags = I2C_M_RD;
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, i2c read, failed\n", __func__);
                return -1;
            }
            ret = copy_to_user((unsigned char*)arg, buffer, len);

            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
                return -1;
            }
            break;
        case ILITEK_IOCTL_I2C_WRITE_LENGTH:
        case ILITEK_IOCTL_I2C_READ_LENGTH:
            len = arg;
            break;
        case ILITEK_IOCTL_DRIVER_INFORMATION:
            for(i = 0; i < 3; i++)
            {
                buffer[i] = driver_information[i];
            }
            ret = copy_to_user((unsigned char*)arg, buffer, 3);
            break;
        case ILITEK_IOCTL_I2C_UPDATE:
            break;
        case ILITEK_IOCTL_I2C_INT_FLAG:
            if(update_timeout == 1)
            {
                buffer[0] = int_Flag;
                ret = copy_to_user((unsigned char*)arg, buffer, 1);
                if(ret < 0)
                {
                    printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
                    return -1;
                }
            }
            else
            {
                update_timeout = 1;
            }
            break;
        case ILITEK_IOCTL_START_READ_DATA:
            i2c.stop_polling = 0;
            if(i2c.client->irq != 0)
            {
                ilitek_i2c_irq_enable();
            }
            i2c.report_status = 1;
            printk("The report point function is enable.\n");
            break;
        case ILITEK_IOCTL_STOP_READ_DATA:
            i2c.stop_polling = 1;
            if(i2c.client->irq != 0)
            {
                ilitek_i2c_irq_disable();
            }
            i2c.report_status = 0;
            printk("The report point function is disable.\n");
            break;
        case ILITEK_IOCTL_I2C_SWITCH_IRQ:
            ret = copy_from_user(buffer, (unsigned char*)arg, 1);
            if(buffer[0] == 0)
            {
                if(i2c.client->irq != 0)
                {
                    ilitek_i2c_irq_disable();
                }
            }
            else
            {
                if(i2c.client->irq != 0)
                {
                    ilitek_i2c_irq_enable();
                }
            }
            break;
        case ILITEK_IOCTL_UPDATE_FLAG:
            update_timeout = 1;
            update_Flag = arg;
            DBG("%s, update_Flag = %d\n", __func__, update_Flag);
            break;
        case ILITEK_IOCTL_I2C_UPDATE_FW:
            ret = copy_from_user(buffer, (unsigned char*)arg, 35);
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
                return -1;
            }
            msgs[0].len = buffer[34];
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
        #ifndef CLOCK_INTERRUPT
            ilitek_i2c_irq_enable();
        #endif
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, i2c write, failed\n", __func__);
                return -1;
            }

            int_Flag = 0;
            update_timeout = 0;

            break;
        case ILITEK_IOCTL_I2C_GESTURE_FLAG:
            gesture_flag = arg;
            printk("%s, gesture_flag = %d\n", __func__, gesture_flag);
            break;
        case ILITEK_IOCTL_I2C_GESTURE_RETURN:
            buffer[0] = getstatus;
            printk("%s, getstatus = %d\n", __func__, getstatus);
            ret = copy_to_user((unsigned char*)arg, buffer, 1);
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
                return -1;
            }
            //getstatus = 0;
            break;
    #ifdef GESTURE_FUN
    #if GESTURE_FUN == GESTURE_FUN_1
        case ILITEK_IOCTL_I2C_GET_GESTURE_MODEL:
            for(i = 0; i < 32; i = i + 2)
            {
                buffer[i] = gesture_model_value_x(i / 2);
                buffer[i + 1]= gesture_model_value_y(i / 2);
                printk("x[%d] = %d, y[%d] = %d\n", i / 2, buffer[i], i / 2, buffer[i + 1]);
            }
            ret = copy_to_user((unsigned char*)arg, buffer, 32);
            if(ret < 0)
            {
                printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
                return -1;
            }
            //getstatus = 0;
            break;
        case ILITEK_IOCTL_I2C_LOAD_GESTURE_LIST:
            printk("start\n");
            readgesturelist();
            printk("end--------------\n");
            break;
    #endif
    #endif
        default:
            return -1;
    }
    return 0;
}

/*
description
    read function for character device driver
prarmeters
    filp
        file pointer
    buf
        buffer
    count
        buffer length
    f_pos
        offset
return
    status
*/
static ssize_t ilitek_file_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

/*
description
    close function
prarmeters
    inode
        inode
    filp
        file pointer
return
    status
*/
static int ilitek_file_close(struct inode *inode, struct file *filp)
{
    DBG("%s\n", __func__);
    return 0;
}

/*
description
    set input device's parameter
prarmeters
    input
        input device data
    max_tp
        single touch or multi touch
    max_x
        maximum	x value
    max_y
        maximum y value
return
    nothing
*/
static void ilitek_set_input_param(struct input_dev *input, int max_tp, int max_x, int max_y)
{
    int key;
    input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#ifndef ROTATE_FLAG
    input_set_abs_params(input, ABS_MT_POSITION_X, 0, max_x + 2, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, max_y + 2, 0, 0);
#else
    input_set_abs_params(input, ABS_MT_POSITION_X, 0, max_y + 2, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, max_x + 2, 0, 0);
#endif
    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
#ifdef TOUCH_PRESSURE
    input_set_abs_params(input, ABS_PRESSURE , 0, 255, 0, 0);
#endif
#ifdef TOUCH_PROTOCOL_B
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
    input_mt_init_slots(input,max_tp,INPUT_MT_DIRECT);
    #else
    input_mt_init_slots(input,max_tp);
    #endif
#else
    input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, max_tp, 0, 0);
#endif
    set_bit(INPUT_PROP_DIRECT, input->propbit);
    if((i2c.protocol_ver & 0x300) == 0x300){
        for(key = 0; key < i2c.keycount; key++)
        {
            if(i2c.keyinfo[key].id <= 0)
            {
                continue;
            }

            set_bit(i2c.keyinfo[key].id & KEY_MAX, input->keybit);
        }
    }
    else if((i2c.protocol_ver & 0x200) == 0x200){
    #ifdef touch_key_code
        for(key = 0; key < sizeof(touch_key_code); key++)
        {
            if(touch_key_code[key] <= 0)
            {
                continue;
            }

            set_bit(touch_key_code[key] & KEY_MAX, input->keybit);
        }
    #endif
    }
    input->name = ILITEK_I2C_DRIVER_NAME;
    input->id.bustype = BUS_I2C;
    input->dev.parent = &(i2c.client)->dev;
}

/*
description
    send message to i2c adaptor
parameter
    client
        i2c client
    msgs
        i2c message
    cnt
        i2c message count
return
    >= 0 if success
    others if error
*/
static int ilitek_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs, int cnt)
{
    int ret, count = ILITEK_I2C_RETRY_COUNT;
    while(count >= 0)
    {
        count -= 1;
        ret = down_interruptible(&i2c.wr_sem);
        ret = i2c_transfer(client->adapter, msgs, cnt);
        up(&i2c.wr_sem);
        if(ret < 0)
        {
            msleep(500);
            continue;
        }
        break;
    }
    return ret;
}

/*
description
    write data to i2c device
parameter
    client
        i2c client data
    cmd
        data for write
    length
        data length
return
    status
*/
static int ilitek_i2c_write(struct i2c_client *client, uint8_t cmd, int length)
{
    int ret;
    struct i2c_msg msgs[] = {
        {.addr = client->addr, .flags = 0, .len = length, .buf = &cmd,}
    };

    ret = ilitek_i2c_transfer(client, msgs, 1);
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, i2c write error, ret %d\n", __func__, ret);
    }
    return ret;
}

/*
description
    read data from i2c device
parameter
    client
        i2c client data
    data
        data for transmission
    length
        data length
return
    status
*/
static int ilitek_i2c_read(struct i2c_client *client, uint8_t *data, int length)
{
    int ret;
    struct i2c_msg msgs[] = {
        {.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
    };

    ret = ilitek_i2c_transfer(client, msgs, 1);
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
    }
    return ret;
}

/*
description
    process i2c data and then report to kernel
parameters
    none
return
    status
*/
static int ilitek_i2c_process_and_report(void)
{
#ifdef ROTATE_FLAG
    int org_x = 0, org_y = 0;
#endif
    int i, len = 0, ret, x = 0, y = 0, key, mult_tp_id, packet = 0, tp_status = 0, j, release_flag[10] = {0}, pressure = 0;
#ifdef TRANSFER_LIMIT
    int cmd_flag = 0, buffer_flag[10] = {0};
#endif
#ifdef GESTURE
    int gesture_mode = 0;
#endif
#ifdef VIRTUAL_KEY_PAD
    unsigned char key_id = 0, key_flag = 1;
#endif
#ifdef _DOUBLE_CLICK_
    //int event_spacing=0;
    //char gesture_tmp[16];
#endif
    static unsigned char last_id = 0;
    struct input_dev *input = i2c.input_dev;
    unsigned char buf[64] = {0};
    unsigned char tp_id, max_point = 6;
    unsigned char release_counter = 0 ;
    if(i2c.report_status == 0)
    {
        return 1;
    }

    //mutli-touch for protocol 3.1
    if((i2c.protocol_ver & 0x300) == 0x300)
    {
    #ifdef TRANSFER_LIMIT
        ret = ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_TOUCH_STATUS, 0, buf, 2);
        cmd_flag = 1;
        //read touch information
        for(i = 0; i < 8; i++)
        {
            release_flag[i] = (buf[0] & (0x1 << i)) >> i;
        }
        release_flag[8] = buf[1] & 0x1;
        release_flag[9] = (buf[1] & 0x2) >> 1;
        for(i = 0; i < 10; i++)
        {
            DBG("release_flag[%d] = %d, buffer[%d] = %d", i, release_flag[i], i, buffer_flag[i]);
        }
        DBG("\n");

        for(i = 0; i < 10; i++)
        {
            if(release_flag[i] == 1)
            {
                if(i < 6)
                {
                    j = (1 + i * 5) / 8;
                }
                else
                {
                    j = ((1 + i * 5) + 1) / 8;
                }
                DBG("i=%d, j=%d, cmd_flag=%d, command[%d]=%d\n", i, j, cmd_flag, i, buffer_flag[j]);
                if((((j + 1) * 8 > 2 + i * 5) || i == 6) && buffer_flag[j] == 0)
                {
                    if(buffer_flag[j-1] == 1 || j == 0)
                    {
                        cmd_flag = 1;
                    }
                    if(cmd_flag == 1)
                    {
                        ret = ilitek_i2c_read(i2c.client, buf + (j * 8), 8);
                        buffer_flag[j] = 1;
                    }
                    else
                    {
                        ret = ilitek_i2c_write(i2c.client, ILITEK_TP_CMD_READ_DATA + j, 1);
                        udelay(10);
                        ret = ilitek_i2c_read(i2c.client, buf + (j * 8), 8);
                        buffer_flag[j] = 1;
                        cmd_flag = 1;
                    }
                }
                if(buffer_flag[j] == 0)
                {
                    cmd_flag = 0;
                }
                j++;
                //msleep(1);
                if((j * 8 < 6 + i * 5) && buffer_flag[j] == 0)
                {
                    ret = ilitek_i2c_read(i2c.client, buf + (j * 8), 8);
                    buffer_flag[j] = 1;
                    cmd_flag = 1;
                }
                max_point = i + 1;
            }
            else
            {
                cmd_flag = 0;
            }
        }
        //buf[31] is reserved so the data is moved forward.
        for(i = 31; i < 53; i++)
        {
            buf[i] = buf[i + 1];
            //printk("buf[%d]=0x%x\n",i,buf[i]);
        }
        packet = buf[0] + buf[1];
    #else
        ret = ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_READ_DATA, 0, buf, 31);
        if(ret < 0)
        {
            return ret;
        }
        packet = buf[0];
        ret = 1;
        if(packet == 2)
        {
            ret = ilitek_i2c_read(i2c.client, buf + 31, 20);
            if(ret < 0)
            {
                return ret;
            }
            max_point = 10;
        }
    #endif
        DBG("max_point=%d\n", max_point);
        //read touch point
        for(i = 0; i < max_point; i++)
        {
            tp_status = buf[i * 5 + 1] >> 7;
            i2c.touchinfo[i].status = tp_status;
        #ifndef ROTATE_FLAG
            x = (((buf[i * 5 + 1] & 0x3F) << 8) + buf[i * 5 + 2]);
            y = (buf[i * 5 + 3] << 8) + buf[i * 5 + 4];
        #else
            org_x = (((buf[i * 5 + 1] & 0x3F) << 8) + buf[i * 5 + 2]);
            org_y = (buf[i * 5 + 3] << 8) + buf[i * 5 + 4];
            x = i2c.max_y - org_y + 1;
            y = org_x + 1;
        #endif
            pressure = buf[i * 5 + 5];
            if(tp_status)
            {
            #ifdef GESTURE
                finger_state = double_click_touch(x, y, finger_state, i);
                if(gesture_flag && i == 0)
                {
                    //gesture_mode = GestureMatchProcess(1,gesture_count,x,y);
                #if GESTURE_FUN == GESTURE_FUN_1
                    if(gesture_flag)
                    {
                        GestureMatchProcess(1, 0, x, y);
                    }
                #endif
                #if GESTURE_FUN == GESTURE_FUN_2
                    gesture_ili(x, y, gesture_count);
                #endif
                //finger_state = double_click_touch(x,y,finger_state);
                    //printk("gesture_mode=%d\n",gesture_mode);
                    if(gesture_count == 0)
                    {
                        gesture_count++;
                    }
                }
            #endif
                #ifdef VIRTUAL_KEY_PAD
                if(i2c.keyflag == 0)
                {
                    for(j = 0; j <= i2c.keycount; j++)
                    {
                        if((x >= i2c.keyinfo[j].x && x <= i2c.keyinfo[j].x + i2c.key_xlen) && (y >= i2c.keyinfo[j].y && y <= i2c.keyinfo[j].y + i2c.key_ylen))
                        {
                            input_report_key(input,  i2c.keyinfo[j].id, 1);
                            i2c.keyinfo[j].status = 1;
                            touch_key_hold_press = 1;
                            release_flag[0] = 1;
                            DBG("Key, Keydown ID=%d, X=%d, Y=%d, key_status=%d,keyflag=%d\n", i2c.keyinfo[j].id ,x ,y , i2c.keyinfo[j].status,i2c.keyflag);
                            break;
                        }
                    }
                }
                if(touch_key_hold_press == 0)
                #endif
                {
                    #ifdef TOUCH_PROTOCOL_B
                        input_mt_slot(i2c.input_dev, i);
                        input_mt_report_slot_state(i2c.input_dev, MT_TOOL_FINGER,true);
                        i2c.touchinfo[i].flag = 1;
                    #endif
                    input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_X, x);
                    input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_Y, y);
                    input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
                    #ifdef TOUCH_PRESSURE
                        input_event(i2c.input_dev, EV_ABS, ABS_PRESSURE, pressure);
                    #endif
                    #ifndef TOUCH_PROTOCOL_B
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_TRACKING_ID, i);
                        input_report_key(i2c.input_dev, BTN_TOUCH,  1);
                        input_mt_sync(i2c.input_dev);
                    #endif
                    release_flag[i] = 1;
                    i2c.keyflag = 1;
                    DBG("Point, ID=%02X, X=%04d, Y=%04d,release_flag[%d]=%d,tp_status=%d,keyflag=%d\n",i, x,y,i,release_flag[i],tp_status,i2c.keyflag);
                }
                #ifdef VIRTUAL_KEY_PAD
                if(touch_key_hold_press == 1)
                {
                    for(j = 0; j <= i2c.keycount; j++)
                    {
                        if((i2c.keyinfo[j].status == 1) && (x < i2c.keyinfo[j].x || x > i2c.keyinfo[j].x + i2c.key_xlen || y < i2c.keyinfo[j].y || y > i2c.keyinfo[j].y + i2c.key_ylen))
                        {
                            input_report_key(input,  i2c.keyinfo[j].id, 0);
                            i2c.keyinfo[j].status = 0;
                            touch_key_hold_press = 0;
                            DBG("Key, Keyout ID=%d, X=%d, Y=%d, key_status=%d\n", i2c.keyinfo[j].id ,x ,y , i2c.keyinfo[j].status);
                            break;
                        }
                    }
                }
                #endif
                ret = 0;
            }
            else
            {
                release_flag[i] = 0;
                DBG("Point, ID=%02X, X=%04d, Y=%04d,release_flag[%d]=%d,tp_status=%d\n",i, x,y,i,release_flag[i],tp_status);
                #ifdef TOUCH_PROTOCOL_B
                    if(i2c.touchinfo[i].flag == 1)
                    {
                        input_mt_slot(i2c.input_dev, i);
                        input_mt_report_slot_state(input, MT_TOOL_FINGER,false);
                        i2c.touchinfo[i].flag = 0;
                    }
                #else
                input_mt_sync(i2c.input_dev);
                #endif
            }

        }
        if(packet == 0 )
        {
            DBG("packet=%d\n", packet);
        #ifdef GESTURE
            if(gesture_flag)
            {
                gesture_count = 0;
                getstatus = 0;
            #if GESTURE_FUN == GESTURE_FUN_1
                gesture_mode = GestureMatchProcess(0, gesture_count, x, y);
                getstatus = GetGesture();
            #endif
            #if GESTURE_FUN == GESTURE_FUN_2
                getstatus = gesture_ili(x, y,2);
            #endif
                printk("driver getstatus=%d, %c\n", getstatus, getstatus);
                finger_state = double_click_release(x, y, finger_state);
                if(finger_state == 5)
                {
                    //add double click action
                }
            }
        #endif
            i2c.keyflag = 0;
            #ifdef TOUCH_PROTOCOL_B
                for(i = 0; i < i2c.max_tp; i++){
                    if(i2c.touchinfo[i].flag == 1){
                        input_mt_slot(i2c.input_dev, i);
                        input_mt_report_slot_state(input, MT_TOOL_FINGER,false);
                    }
                    i2c.touchinfo[i].flag = 0;
                }
            #else
                input_report_key(i2c.input_dev, BTN_TOUCH, 0);
                input_mt_sync(i2c.input_dev);
            #endif
        }
        else
        {
            for(i = 0; i < max_point; i++)
            {
                if(release_flag[i] == 0)
                {
                    release_counter++;
                }
            }
            if(release_counter == max_point)
            {
            #ifdef TOUCH_PROTOCOL_B
                for(i = 0; i < i2c.max_tp; i++){
                    if(i2c.touchinfo[i].flag == 1){
                        input_mt_slot(i2c.input_dev, i);
                        input_mt_report_slot_state(input, MT_TOOL_FINGER,false);
                    }
                    i2c.touchinfo[i].flag = 0;
                }
            #else
                input_report_key(i2c.input_dev, BTN_TOUCH, 0);
                input_mt_sync(i2c.input_dev);
            #endif
                i2c.keyflag = 0;
                #ifdef VIRTUAL_KEY_PAD
                if(touch_key_hold_press == 1)
                {
                    for(i = 0; i < i2c.keycount; i++)
                    {
                        if(i2c.keyinfo[i].status)
                        {
                            input_report_key(input, i2c.keyinfo[i].id, 0);
                            i2c.keyinfo[i].status = 0;
                            touch_key_hold_press = 0;
                            DBG("Key, Keyup ID=%d, X=%d, Y=%d, key_status=%d, touch_key_hold_press=%d\n", i2c.keyinfo[i].id ,x ,y , i2c.keyinfo[i].status, touch_key_hold_press);
                        }
                    }
                }
                #endif
            }
            DBG("release_counter=%d,packet=%d\n", release_counter, packet);
        }
    }
    //multipoint process
    else if((i2c.protocol_ver & 0x200) == 0x200)
    {
        //read i2c data from device
        ret = ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_READ_DATA, 0, buf, 1);
        if(ret < 0)
        {
            return ret;
        }
        len = buf[0];
        ret = 1;
        if(len>20)
        {
            return ret;
        }
        //read touch point
        for(i = 0; i < len; i++)
        {
            //parse point
            if(ilitek_i2c_write(i2c.client, ILITEK_TP_CMD_READ_SUB_DATA, 1))
            {
                udelay(100);
                ilitek_i2c_read(i2c.client, buf, 5);
            #ifndef ROTATE_FLAG
                x = (((int)buf[1]) << 8) + buf[2];
                y = (((int)buf[3]) << 8) + buf[4];
            #else
                org_x = (((int)buf[1]) << 8) + buf[2];
                org_y = (((int)buf[3]) << 8) + buf[4];
                x = i2c.max_y - org_y + 1;
                y = org_x + 1;
            #endif
                mult_tp_id = buf[0];
                switch ((mult_tp_id & 0xC0))
                {
            #ifdef VIRTUAL_KEY_PAD
                    case RELEASE_KEY:
                        //release key
                        #ifdef VIRTUAL_KEY_PAD
                        DBG("Key: Release\n");
                        for(key = 0; key < sizeof(touch_key_code) / sizeof(touch_key_code[0]); key++)
                        {
                            if(touch_key_press[key])
                            {
                                input_report_key(input, touch_key_code[key], 0);
                                touch_key_press[key] = 0;
                                DBG("Key:%d ID:%d release\n", touch_key_code[key], key);
                                DBG(ILITEK_DEBUG_LEVEL "%s key release, %X, %d, %d\n", __func__, buf[0], x, y);
                            }
                            touch_key_hold_press = 0;
                            //ret = 1;// stop timer interrupt
                        }
                        #endif
                        break;
                    case TOUCH_KEY:
                        //touch key
                    #if VIRTUAL_FUN == VIRTUAL_FUN_1
                        key_id = buf[1] - 1;
                    #endif
                    #if VIRTUAL_FUN == VIRTUAL_FUN_2
                        if(abs(jiffies-touch_time) < msecs_to_jiffies(BTN_DELAY_TIME))
                        {
                            break;
                        }
                        //DBG("Key: Enter\n");
                        x = (((int)buf[4]) << 8) + buf[3];
                        //printk("%s,x=%d\n",__func__,x);
                        if(x > KEYPAD01_X1 && x < KEYPAD01_X2)		// btn 1
                            key_id = 0;
                        else if(x > KEYPAD02_X1 && x < KEYPAD02_X2)	// btn 2
                            key_id = 1;
                        else if (x > KEYPAD03_X1 && x < KEYPAD03_X2)	// btn 3
                            key_id = 2;
                        else if (x > KEYPAD04_X1 && x < KEYPAD04_X2)	// btn 4
                            key_id = 3;
                        else
                            key_flag = 0;
                    #endif
                        if((touch_key_press[key_id] == 0) && (touch_key_hold_press == 0 && key_flag))
                        {
                            input_report_key(input, touch_key_code[key_id], 1);
                            touch_key_press[key_id] = 1;
                            touch_key_hold_press = 1;
                            DBG("Key:%d ID:%d press x=%d,touch_key_hold_press=%d,key_flag=%d\n", touch_key_code[key_id], key_id,x,touch_key_hold_press,key_flag);
                        }
                        break;
                #endif
                    case TOUCH_POINT:
                #ifdef VIRTUAL_KEY_PAD
                    #if VIRTUAL_FUN == VIRTUAL_FUN_3
                        if((buf[0] & 0x80) != 0 && ( y > KEYPAD_Y) && (i == 0))
                        {
                            DBG("%s, touch key\n", __func__);
                            if((x > KEYPAD01_X1) && (x < KEYPAD01_X2))
                            {
                                input_report_key(input,  touch_key_code[0], 1);
                                touch_key_press[0] = 1;
                                touch_key_hold_press = 1;
                                DBG("%s,touch key=0 ,touch_key_hold_press=%d\n",__func__,touch_key_hold_press);
                            }
                            else if((x > KEYPAD02_X1) && (x < KEYPAD02_X2))
                            {
                                input_report_key(input, touch_key_code[1], 1);
                                touch_key_press[1] = 1;
                                touch_key_hold_press = 1;
                                DBG("%s,touch key=1 ,touch_key_hold_press=%d\n", __func__, touch_key_hold_press);
                            }
                            else if((x > KEYPAD03_X1) && (x < KEYPAD03_X2))
                            {
                                input_report_key(input, touch_key_code[2], 1);
                                touch_key_press[2] = 1;
                                touch_key_hold_press = 1;
                                DBG("%s,touch key=2 ,touch_key_hold_press=%d\n", __func__, touch_key_hold_press);
                            }
                            else
                            {
                                input_report_key(input, touch_key_code[3], 1);
                                touch_key_press[3] = 1;
                                touch_key_hold_press = 1;
                                DBG("%s,touch key=3 ,touch_key_hold_press=%d\n", __func__, touch_key_hold_press);
                            }
                        }
                        if((buf[0] & 0x80) != 0 && y <= KEYPAD_Y)
                        {
                            touch_key_hold_press = 0;
                        }
                        if((buf[0] & 0x80) != 0 && y <= KEYPAD_Y)
                    #endif
                #endif
                        {
                            //report to android system
                            DBG("Point, ID=%02X, X=%04d, Y=%04d\n",buf[0]  & 0x3F, x,y);
                            i2c.touch_flag = 1;
                            input_report_key(input, BTN_TOUCH,  1);
                            input_event(input, EV_ABS, ABS_MT_TRACKING_ID, (buf[0] & 0x3F)-1);
                            input_event(input, EV_ABS, ABS_MT_POSITION_X, x+1);
                            input_event(input, EV_ABS, ABS_MT_POSITION_Y, y+1);
                            input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
                            input_mt_sync(input);
                        #ifdef GESTURE
                            finger_state = double_click_touch(x, y, finger_state, (buf[0] & 0x3F) - 1);
                            if(gesture_flag && i == 0)
                            {
                            #if GESTURE_FUN == GESTURE_FUN_1
                                gesture_mode = GestureMatchProcess(1, gesture_count, x, y);
                                gesture_count++;
                            #endif
                            #if GESTURE_FUN == GESTURE_FUN_2
                                gesture_ili(x, y, gesture_count);
                            #endif
                                if(gesture_count == 0)
                                {
                                    gesture_count++;
                                }
                            }
                        #endif
                            ret = 0;
                        }
                        break;
                    case RELEASE_POINT:
                        if(touch_key_hold_press != 0 && i == 0)
                        {
                            #ifdef VIRTUAL_KEY_PAD
                            for(key = 0; key < sizeof(touch_key_code) / sizeof(touch_key_code[0]); key++)
                            {
                                if(touch_key_press[key])
                                {
                                    input_report_key(input, touch_key_code[key], 0);
                                    touch_key_press[key] = 0;
                                    DBG("Key:%d ID:%d release\n", touch_key_code[key], key);
                                    DBG(ILITEK_DEBUG_LEVEL "%s key release, %X, %d, %d,touch_key_hold_press=%d\n", __func__, buf[0], x, y,touch_key_hold_press);
                                }
                                touch_key_hold_press = 0;
                                //ret = 1;// stop timer interrupt
                            }
                            #endif
                        }
                        // release point
                    #ifdef CLOCK_INTERRUPT
                        release_counter++;
                        if(release_counter == len)
                        {
                        #ifdef GESTURE
                            if(gesture_flag)
                            {
                                gesture_count = 0;
                                getstatus = 0;

                            #if GESTURE_FUN == GESTURE_FUN_1
                                gesture_mode = GestureMatchProcess(0, gesture_count, x, y);
                                getstatus = GetGesture();
                            #endif
                            #if GESTURE_FUN == GESTURE_FUN_2
                                getstatus = gesture_ili(x, y, 2);
                            #endif
                                printk("driver getstatus=%d, %c\n", getstatus, getstatus);
                                finger_state = double_click_release(x, y, finger_state);
                                if(finger_state == 5)
                                {
                                    //add double click action
                                }
                                //getstatus is gesture ruturn value
                            }
                        #endif
                            input_report_key(input, BTN_TOUCH, 0);
                            input_mt_sync(input);
                            i2c.touch_flag = 0;
                        }
                    #endif
                        //ret=1;
                        break;
                    default:
                        break;
                }
            }
        }
        //release point
        if(len == 0 && i2c.touch_flag == 1)
        {
            i2c.touch_flag = 0;
            DBG("Release3, ID=%02X, X=%04d, Y=%04d\n", buf[0] & 0x3F, x, y);
        #ifdef GESTURE
            if(gesture_flag)
            {
                gesture_count = 0;
                getstatus = 0;
            #if GESTURE_FUN == GESTURE_FUN_1
                gesture_mode = GestureMatchProcess(0, gesture_count, x, y);
                getstatus = GetGesture();
            #endif
            #if GESTURE_FUN == GESTURE_FUN_2
                getstatus = gesture_ili(x, y, 2);
            #endif
                finger_state = double_click_release(x, y, finger_state);
                if(finger_state == 5)
                {
                    //add double click action
                }
                //getstatus is gesture ruturn value
            }
        #endif
            input_report_key(input, BTN_TOUCH, 0);
            //input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
            input_mt_sync(input);
            //ret = 1;
            #ifdef VIRTUAL_KEY_PAD
            if(touch_key_hold_press !=0)
            {
                for(key = 0; key < sizeof(touch_key_code) / sizeof(touch_key_code[0]); key++)
                {
                    if(touch_key_press[key])
                    {
                        input_report_key(input, touch_key_code[key], 0);
                        touch_key_press[key] = 0;
                        DBG("Key:%d ID:%d release\n", touch_key_code[key], key);
                        DBG(ILITEK_DEBUG_LEVEL "%s key release, %X, %d, %d\n", __func__, buf[0], x, y);
                    }
                    touch_key_hold_press = 0;
                    //ret = 1;// stop timer interrupt
                }
            }
            #endif
        }
        DBG("%s,ret=%d\n",__func__,ret);
    }
    else
    {
        //read i2c data from device
        ret = ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_READ_DATA, 0, buf, 9);
        if(ret < 0)
        {
            return ret;
        }
        if(buf[0] > 20)
        {
            ret = 1;
            return ret;
        }

        //parse point
        ret = 0;
        tp_id = buf[0];
        if(Report_Flag != 0)
        {
            printk("%s(%d):", __func__, __LINE__);
            for(i = 0; i < 9; i++)
            {
                DBG("%02X,", buf[i]);
            }
            DBG("\n");
        }
        switch(tp_id)
        {
            case 0://release point
            #ifdef VIRTUAL_KEY_PAD
                if(touch_key_hold_press != 0)
                {
                    for(key = 0; key < sizeof(touch_key_code) / sizeof(touch_key_code[0]); key++)
                    {
                        if(touch_key_press[key])
                        {
                            //input_report_key(input, touch_key_code[key], 0);
                            touch_key_press[key] = 0;
                            DBG("Key:%d ID:%d release\n", touch_key_code[key], key);
                        }
                    }
                    touch_key_hold_press = 0;
                }
                else
            #endif
                {
                    for(i = 0; i < i2c.max_tp; i++)
                    {
                        //check
                        if(!(last_id & (1<<i)))
                        {
                            continue;
                        }
                    #ifndef ROTATE_FLAG
                        x = (int)buf[1 + (i * 4)] + ((int)buf[2 + (i * 4)] * 256);
                        y = (int)buf[3 + (i * 4)] + ((int)buf[4 + (i * 4)] * 256);
                    #else
                        org_x = (int)buf[1 + (i * 4)] + ((int)buf[2 + (i * 4)] * 256);
                        org_y = (int)buf[3 + (i * 4)] + ((int)buf[4 + (i * 4)] * 256);
                        x = i2c.max_y - org_y + 1;
                        y = org_x + 1;
                    #endif
                        touch_key_hold_press=2; //2: into available area
                        input_report_key(input, BTN_TOUCH,  1);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_TRACKING_ID, i);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_X, x+1);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_Y, y+1);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
                        input_mt_sync(i2c.input_dev);
                        DBG("Last Point[%d]= %d, %d\n", buf[0]&0x3F, x, y);
                        last_id=0;
                    }
                    input_sync(i2c.input_dev);
                    input_report_key(input, BTN_TOUCH,  0);
                    input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
                    input_mt_sync(i2c.input_dev);
                    ret = 1; // stop timer interrupt
                }
                break;
        #ifdef VIRTUAL_KEY_PAD
            case 0x81:
                if(abs(jiffies-touch_time) < msecs_to_jiffies(BTN_DELAY_TIME))
                {
                    break;
                }
                DBG("Key: Enter\n");
            #if VIRTUAL_FUN == VIRTUAL_FUN_1
                key_id = buf[1] - 1;
            #endif
            #if VIRTUAL_FUN == VIRTUAL_FUN_2
                x = (int)buf[1] + ((int)buf[2] * 256);
                if(x > KEYPAD01_X1 && x < KEYPAD01_X2)		// btn 1
                    key_id = 0;
                else if (x > KEYPAD02_X1 && x < KEYPAD02_X2)	// btn 2
                    key_id = 1;
                else if (x > KEYPAD03_X1 && x < KEYPAD03_X2)	// btn 3
                    key_id = 2;
                else if (x > KEYPAD04_X1 && x < KEYPAD04_X2)	// btn 4
                    key_id = 3;
                else
                    key_flag = 0;
            #endif
                input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
                if((touch_key_press[key_id] == 0) && (touch_key_hold_press == 0 && key_flag))
                {
                    input_report_key(input, touch_key_code[key_id], 1);
                    touch_key_press[key_id] = 1;
                    touch_key_hold_press = 1;
                    DBG("Key:%d ID:%d press\n", touch_key_code[key_id], key_id);
                }
                break;
            case 0x80:
                DBG("Key: Release\n");
                for(key = 0; key < sizeof(touch_key_code) / sizeof(touch_key_code[0]); key++)
                {
                    if(touch_key_press[key])
                    {
                        input_report_key(input, touch_key_code[key], 0);
                        touch_key_press[key] = 0;
                        DBG("Key:%d ID:%d release\n", touch_key_code[key], key);
                    }
                }
                touch_key_hold_press = 0;
                ret = 1;// stop timer interrupt
                break;
        #endif
            default:
                last_id = buf[0];
                for(i = 0; i < i2c.max_tp; i++)
                {
                    //check
                    if(!(buf[0] & (1<<i)))
                    {
                        continue;
                    }
                #ifndef ROTATE_FLAG
                    x = (int)buf[1 + (i * 4)] + ((int)buf[2 + (i * 4)] * 256);
                    y = (int)buf[3 + (i * 4)] + ((int)buf[4 + (i * 4)] * 256);
                #else
                    org_x = (int)buf[1 + (i * 4)] + ((int)buf[2 + (i * 4)] * 256);
                    org_y = (int)buf[3 + (i * 4)] + ((int)buf[4 + (i * 4)] * 256);
                    x = i2c.max_y - org_y + 1;
                    y = org_x + 1;
                #endif
                #ifdef VIRTUAL_KEY_PAD
                #if VIRTUAL_FUN == VIRTUAL_FUN_3
                    if(y > KEYPAD_Y)
                    {
                        if(abs(jiffies-touch_time) < msecs_to_jiffies(BTN_DELAY_TIME))
                        {
                            break;
                        }
                        x = (int)buf[1] + ((int)buf[2] * 256);
                        if(x > KEYPAD01_X1 && x < KEYPAD01_X2)		// btn 1
                            key_id = 0;
                        else if (x > KEYPAD02_X1 && x < KEYPAD02_X2)	// btn 2
                            key_id = 1;
                        else if (x > KEYPAD03_X1 && x < KEYPAD03_X2)	// btn 3
                            key_id = 2;
                        else if (x > KEYPAD04_X1 && x < KEYPAD04_X2)	// btn 4
                            key_id = 3;
                        else
                            key_flag = 0;
                        if(touch_key_hold_press == 2)
                        {
                            input_report_key(input, BTN_TOUCH, 0);
                            input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
                            input_mt_sync(i2c.input_dev);
                            touch_key_hold_press=0;
                        }
                        if((touch_key_press[key_id] == 0) && (touch_key_hold_press == 0 && key_flag))
                        {
                            //input_report_key(input, touch_key_code[key_id], 1);
                            touch_key_press[key_id] = 1;
                            touch_key_hold_press = 1;
                            DBG("Key:%d ID:%d press\n", touch_key_code[key_id], key_id);
                        }
                    }
                    else if (touch_key_hold_press)
                    {
                        for(key = 0; key < sizeof(touch_key_code) / sizeof(touch_key_code[0]); key++)
                        {
                            if(touch_key_press[key])
                            {
                                //input_report_key(input, touch_key_code[key], 0);
                                touch_key_press[key] = 0;
                                DBG("Key:%d ID:%d release\n", touch_key_code[key], key);
                            }
                        }
                        touch_key_hold_press = 0;
                    }
                    else
                #endif
                    touch_time = jiffies + msecs_to_jiffies(BTN_DELAY_TIME);
                #endif
                    {
                        touch_key_hold_press = 2; //2: into available area
                        input_report_key(input, BTN_TOUCH,  1);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_TRACKING_ID, i);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_X, x + 1);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_Y, y + 1);
                        input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
                        input_mt_sync(i2c.input_dev);
                        DBG("Point[%d]= %d, %d\n", buf[0]&0x3F, x, y);
                    }
                }
                break;
        }
    }
    #ifdef TOUCH_PROTOCOL_B
    input_mt_report_pointer_emulation(i2c.input_dev, true);
    #endif
    input_sync(i2c.input_dev);
    return ret;
}

static void ilitek_i2c_timer(unsigned long handle)
{
    struct i2c_data *priv = (void *)handle;
    schedule_work(&priv->irq_work);
}

/*
description
    work queue function for irq use
parameter
    work
        work queue
return
    none
*/
static void ilitek_i2c_irq_work_queue_func(struct work_struct *work)
{
    int ret;
#ifndef CLOCK_INTERRUPT
    struct i2c_data *priv = container_of(work, struct i2c_data, irq_work);
#endif
    ret = ilitek_i2c_process_and_report();
    DBG("%s,enter\n",__func__);
#ifdef CLOCK_INTERRUPT
    ilitek_i2c_irq_enable();
#else
    if(ret == 0)
    {
        if(!i2c.stop_polling)
        {
            mod_timer(&priv->timer, jiffies + msecs_to_jiffies(0));
        }
    }
    else if(ret == 1)
    {
        if(!i2c.stop_polling)
        {
            ilitek_i2c_irq_enable();
        }
        DBG("stop_polling\n");
    }
    else if(ret < 0)
    {
        msleep(100);
        DBG(ILITEK_ERROR_LEVEL "%s, process error\n", __func__);
        ilitek_i2c_irq_enable();
    }
#endif
}

/*
description
    i2c interrupt service routine
parameters
    irq
        interrupt number
    dev_id
        device parameter
return
    status
*/
static irqreturn_t ilitek_i2c_isr(int irq, void *dev_id)
{
#ifndef CLOCK_INTERRUPT
    if(i2c.irq_status == 1)
    {
        disable_irq_nosync(i2c.client->irq);
        DBG("disable nosync\n");
        i2c.irq_status = 0;
    }
#endif
    if(update_Flag == 1)
    {
        int_Flag = 1;
    }
    else
    {
        queue_work(i2c.irq_work_queue, &i2c.irq_work);
    }
    return IRQ_HANDLED;
}

/*
description
    i2c polling thread
parameters
    arg
        arguments
return
    status
*/
static int ilitek_i2c_polling_thread(void *arg)
{
    int ret = 0;
    DBG(ILITEK_DEBUG_LEVEL "%s, enter\n", __func__);
    //mainloop
    while(1)
    {
        //check whether we should exit or not
        if(kthread_should_stop())
        {
            printk(ILITEK_DEBUG_LEVEL "%s, stop\n", __func__);
            break;
        }

        //this delay will influence the CPU usage and response latency
        msleep(10);

        //when i2c is in suspend or shutdown mode, we do nothing
        if(i2c.stop_polling)
        {
            msleep(1000);
            continue;
        }

        //read i2c data
        if(ilitek_i2c_process_and_report() < 0)
        {
            msleep(3000);
            printk(ILITEK_ERROR_LEVEL "%s, process error\n", __func__);
        }
    }
    return ret;
}

/*
description
    i2c early suspend function
parameters
    h
        early suspend pointer
return
    none
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ilitek_i2c_early_suspend(struct early_suspend *h)
{
    ilitek_i2c_suspend(i2c.client, PMSG_SUSPEND);
    printk("%s\n", __func__);
}
#endif

/*
description
    i2c later resume function
parameters
    h
        early suspend pointer
return
    none
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ilitek_i2c_late_resume(struct early_suspend *h)
{
    ilitek_i2c_resume(i2c.client);
    printk("%s\n", __func__);
}
#endif

/*
description
    i2c irq enable function
*/
static void ilitek_i2c_irq_enable(void)
{
    if (i2c.irq_status == 0)
    {
        i2c.irq_status = 1;
        enable_irq(i2c.client->irq);
        DBG("enable\n");
    }
    else
    {
        DBG("no enable\n");
    }
}

/*
description
    i2c irq disable function
*/
static void ilitek_i2c_irq_disable(void)
{
    if (i2c.irq_status == 1)
    {
        i2c.irq_status = 0;
        disable_irq(i2c.client->irq);
        DBG("disable\n");
    }
    else
    {
        DBG("no disable\n");
    }
}

/*
description
    i2c suspend function
parameters
    client
        i2c client data
    mesg
        suspend data
return
    status
*/
static int ilitek_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
    ilitek_i2c_write(i2c.client, ILITEK_TP_CMD_SLEEP, 1);

#ifdef GESTURE
    gesture_flag = 1;
#else
    if(i2c.valid_irq_request != 0)
    {
        ilitek_i2c_irq_disable();
    }
    else
    {
        i2c.stop_polling = 1;
        printk(ILITEK_DEBUG_LEVEL "%s, stop i2c thread polling\n", __func__);
    }
#endif
    return 0;
}

/*
description
    i2c resume function
parameters
    client
        i2c client data
return
    status
*/
static int ilitek_i2c_resume(struct i2c_client *client)
{
#ifdef GESTURE
    gesture_flag = 0;
#else
    if(i2c.valid_irq_request != 0)
    {
        ilitek_i2c_irq_enable();
    }
    else
    {
        i2c.stop_polling = 0;
        printk(ILITEK_DEBUG_LEVEL "%s, start i2c thread polling\n", __func__);
    }
#endif
    return 0;
}

/*
description
    reset touch ic
prarmeters
    reset_pin
        reset pin
return
    status
*/
static int ilitek_i2c_reset(void)
{
    int ret = 0;
#ifndef SET_RESET
    static unsigned char buffer[64]={0};
    struct i2c_msg msgs[] = {
        {.addr = i2c.client->addr, .flags = 0, .len = 1, .buf = buffer,}
    };
    buffer[0] = 0x60;
    ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
#else
    /*

    ____         ___________
        |_______|
           1ms      100ms
    */
#endif
    msleep(100);
    return ret;
}

/*
description
    i2c shutdown function
parameters
    client
        i2c client data
return
    nothing
*/
static void ilitek_i2c_shutdown(struct i2c_client *client)
{
    printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
    i2c.stop_polling = 1;
}

/*
description
    when adapter detects the i2c device, this function will be invoked.
parameters
    client
        i2c client data
    id
        i2c data
return
    status
*/
static int ilitek_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    //register i2c device
    int ret = 0;
    //allocate character device driver buffer
    ret = alloc_chrdev_region(&dev.devno, 0, 1, ILITEK_FILE_DRIVER_NAME);
    if(ret)
    {
        printk(ILITEK_ERROR_LEVEL "%s, can't allocate chrdev\n", __func__);
        return ret;
    }
    printk(ILITEK_DEBUG_LEVEL "%s, register chrdev(%d, %d)\n", __func__, MAJOR(dev.devno), MINOR(dev.devno));

    //initialize character device driver
    cdev_init(&dev.cdev, &ilitek_fops);
    dev.cdev.owner = THIS_MODULE;
    ret = cdev_add(&dev.cdev, dev.devno, 1);
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, add character device error, ret %d\n", __func__, ret);
        return ret;
    }
    dev.class = class_create(THIS_MODULE, ILITEK_FILE_DRIVER_NAME);
    if(IS_ERR(dev.class))
    {
        printk(ILITEK_ERROR_LEVEL "%s, create class, error\n", __func__);
        return ret;
    }
    device_create(dev.class, NULL, dev.devno, NULL, "ilitek_ctrl");

    proc_create("ilitek_ctrl", 0666, NULL, &ilitek_fops);

    Report_Flag = 0;

    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(ILITEK_ERROR_LEVEL "%s, I2C_FUNC_I2C not support\n", __func__);
        return -1;
    }
    i2c.client = client;
    printk(ILITEK_DEBUG_LEVEL "%s, i2c new style format\n", __func__);
    printk("%s, IRQ: 0x%X\n", __func__, client->irq);

    ilitek_i2c_register_device();
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, register i2c device, error\n", __func__);
        return ret;
    }
    return 0;
}

/*
description
    when the i2c device want to detach from adapter, this function will be invoked.
parameters
    client
        i2c client data
return
    status
*/
static int ilitek_i2c_remove(struct i2c_client *client)
{
    printk( "%s\n", __func__);
    i2c.stop_polling = 1;
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&i2c.early_suspend);
#endif
    //delete i2c driver
    if(i2c.client->irq != 0)
    {
        if(i2c.valid_irq_request != 0)
        {
            free_irq(i2c.client->irq, &i2c);
            printk(ILITEK_DEBUG_LEVEL "%s, free irq\n", __func__);
            if(i2c.irq_work_queue)
            {
                destroy_workqueue(i2c.irq_work_queue);
                printk(ILITEK_DEBUG_LEVEL "%s, destory work queue\n", __func__);
            }
        }
    }
    else
    {
        if(i2c.thread != NULL)
        {
            kthread_stop(i2c.thread);
            printk(ILITEK_DEBUG_LEVEL "%s, stop i2c thread\n", __func__);
        }
    }
    if(i2c.valid_input_register != 0)
    {
        input_unregister_device(i2c.input_dev);
        printk(ILITEK_DEBUG_LEVEL "%s, unregister i2c input device\n", __func__);
    }

    //delete character device driver
    cdev_del(&dev.cdev);
    unregister_chrdev_region(dev.devno, 1);
    device_destroy(dev.class, dev.devno);
    class_destroy(dev.class);
    printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
    return 0;
}

/*
description
    write data to i2c device and read data from i2c device
parameter
    client
        i2c client data
    cmd
        data for write
    delay
        delay time(ms) after write
    data
        data for read
    length
        data length
return
    status
*/
static int ilitek_i2c_write_read(struct i2c_client *client, uint8_t cmd, int delay, uint8_t *data, int length)
{
    int ret;
    struct i2c_msg msgs_send[] = {
        {.addr = client->addr, .flags = 0, .len = 1, .buf = &cmd,},
    };
    struct i2c_msg msgs_receive[] = {
        {.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
    };
    ret = ilitek_i2c_transfer(client, msgs_send, 1);
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, i2c write error, ret = %d\n", __func__, ret);
    }
    if(delay != 0)
        msleep(delay);
    ret = ilitek_i2c_transfer(client, msgs_receive, 1);
    if(ret < 0)
    {
        printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret = %d\n", __func__, ret);
    }
    return ret;
}

/*
description
    read touch information
parameters
    none
return
    status
*/
static int ilitek_i2c_read_tp_info(void)
{
    int i;
    unsigned char buf[64] = {0};
#ifdef TRANSFER_LIMIT
    int j;
#endif
    //read driver version
    printk(ILITEK_DEBUG_LEVEL "%s, driver version:%d.%d.%d\n", __func__, driver_information[0], driver_information[1], driver_information[2]);

    //read firmware version
    if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_FIRMWARE_VERSION, 10, buf, 4) < 0)
    {
        return -1;
    }
#ifdef UPGRADE_FIRMWARE_ON_BOOT
    for(i = 0; i < 4; i++)
    {
        i2c.firmware_ver[i] = buf[i];
    }
#endif
    printk(ILITEK_DEBUG_LEVEL "%s, firmware version:%d.%d.%d.%d\n", __func__, buf[0], buf[1], buf[2], buf[3]);

    //read protocol version
    if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_PROTOCOL_VERSION, 10, buf, 2) < 0)
    {
        return -1;
    }
    i2c.protocol_ver = (((int)buf[0]) << 8) + buf[1];
    printk(ILITEK_DEBUG_LEVEL "%s, protocol version:%d.%d\n", __func__, buf[0], buf[1]);

    //read touch resolution
    i2c.max_tp = 2;
#ifdef TRANSFER_LIMIT
    if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_RESOLUTION, 10, buf, 8) < 0)
    {
        return -1;
    }
    if(ilitek_i2c_read(i2c.client, buf+8, 2) < 0)
    {
        return -1;
    }
#else
    if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_RESOLUTION, 10, buf, 10) < 0)
    {
        return -1;
    }
#endif

    //calculate the resolution for x and y direction
    i2c.max_x = buf[0];
    i2c.max_x+= ((int)buf[1]) * 256;
    i2c.max_y = buf[2];
    i2c.max_y+= ((int)buf[3]) * 256;
    i2c.x_ch = buf[4];
    i2c.y_ch = buf[5];
    //maximum touch point
    i2c.max_tp = buf[6];
    //maximum button number
    i2c.max_btn = buf[7];
    //key count
    i2c.keycount = buf[8];

    printk(ILITEK_DEBUG_LEVEL "%s, max_x: %d, max_y: %d, ch_x: %d, ch_y: %d, max_tp: %d, max_btn: %d, key_count: %d\n", __func__, i2c.max_x, i2c.max_y, i2c.x_ch, i2c.y_ch, i2c.max_tp, i2c.max_btn, i2c.keycount);

    if((i2c.protocol_ver & 0x300) == 0x300)
    {
        //get key infotmation
    #ifdef TRANSFER_LIMIT
        if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_KEY_INFORMATION, 0, buf, 8) < 0)
        {
            return -1;
        }
        for(i = 1, j = 1; j < i2c.keycount ; i++)
        {
            if(i2c.keycount > j)
            {
                if(ilitek_i2c_read(i2c.client, buf+i*8, 8) < 0)
                {
                    return -1;
                }
                j = (4 + 8 * i) / 5;
            }
        }
        for(j = 29; j < (i + 1) * 8; j++)
        {
            buf[j] = buf[j + 3];
        }
    #else
        if(i2c.keycount)
        {
            if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_KEY_INFORMATION, 0, buf, 29) < 0)
            {
                return -1;
            }
            if(i2c.keycount > 5)
            {
                if(ilitek_i2c_read(i2c.client, buf+29, 25) < 0)
                {
                    return -1;
                }
            }
        }
    #endif
        i2c.key_xlen = (buf[0] << 8) + buf[1];
        i2c.key_ylen = (buf[2] << 8) + buf[3];
        printk(ILITEK_DEBUG_LEVEL "%s, key_xlen: %d, key_ylen: %d\n", __func__, i2c.key_xlen, i2c.key_ylen);

        //print key information
        for(i = 0; i < i2c.keycount; i++)
        {
            i2c.keyinfo[i].id = buf[i*5+4];
            i2c.keyinfo[i].x = (buf[i*5+5] << 8) + buf[i*5+6];
            i2c.keyinfo[i].y = (buf[i*5+7] << 8) + buf[i*5+8];
            i2c.keyinfo[i].status = 0;
            printk(ILITEK_DEBUG_LEVEL "%s, key_id: %d, key_x: %d, key_y: %d, key_status: %d\n", __func__, i2c.keyinfo[i].id, i2c.keyinfo[i].x, i2c.keyinfo[i].y, i2c.keyinfo[i].status);
        }
    }

    return 0;
}

#ifdef UPGRADE_FIRMWARE_ON_BOOT
static int inwrite(unsigned int address)
{
    char outbuff[64];
    int data, ret;
    outbuff[0] = 0x25;
    outbuff[1] = (char)((address & 0x000000FF) >> 0);
    outbuff[2] = (char)((address & 0x0000FF00) >> 8);
    outbuff[3] = (char)((address & 0x00FF0000) >> 16);
    ret = ilitek_i2c_write(i2c.client, outbuff, 4);
    udelay(10);
    ret = ilitek_i2c_read(i2c.client, outbuff, 4);
    data = (outbuff[0] + outbuff[1] * 256 + outbuff[2] * 256 * 256 + outbuff[3] * 256 * 256 * 256);
    //printk("%s, data=0x%x, outbuff[0]=%x, outbuff[1]=%x, outbuff[2]=%x, outbuff[3]=%x\n", __func__, data, outbuff[0], outbuff[1], outbuff[2], outbuff[3]);
    return data;
}

static int outwrite(unsigned int address, unsigned int data, int size)
{
    int ret, i;
    char outbuff[64];
    outbuff[0] = 0x25;
    outbuff[1] = (char)((address & 0x000000FF) >> 0);
    outbuff[2] = (char)((address & 0x0000FF00) >> 8);
    outbuff[3] = (char)((address & 0x00FF0000) >> 16);
    for(i = 0; i < size; i++)
    {
        outbuff[i + 4] = (char)(data >> (8 * i));
    }
    ret = ilitek_i2c_write(i2c.client, outbuff, size + 4);
    return ret;
}

static void set_program_key(void)
{
    int ret;
    char buf[64];
    ret = outwrite(0x41014, 0x7EAE, 2);
    printk("%s, outwrite, ret = %d\n", __func__, ret);
}

static void clear_program_key(void)
{
    int ret;
    char buf[64];
    ret = outwrite(0x41014, 0x0, 2);
    printk("%s, outwrite, ret = %d\n", __func__, ret);
}

static void set_standby_key(unsigned int chip_id_h, unsigned int chip_id_l)
{
    printk("%s, chip id: 0x%x%x\n", __func__, chip_id_h, chip_id_l);

    int ret;
    char buf[64];
    ret = outwrite(0x40010, (chip_id_h << 8) + chip_id_l, 2);
    printk("%s, outwrite, ret = %d\n", __func__, ret);
}

static void mtp_control_reg(int delay)
{
    int i, ret, temp = 0;
    for(i = 0; i < 500 ; i++)
    {
        temp = inwrite(0x04102C);
        //printk("%s, temp = %d, i=%d, 0x%X\n", __func__, temp, i, temp);
        if((temp & 0x00000100) == 0x00000000)
        {
            break;
        }
        udelay(delay);
    }
    if(i != 0)
    {
        //printk("%s, delay time = %dus\n", __func__, i * delay);
    }
    if(i == 500)
    {
        printk("%s, MTP is busy\n", __func__);
    }
}

static void set_standby(void)
{
    int ret;
    char buf[64];
    ret = outwrite(0x40024, 0x01, 1);
    printk("%s, outwrite, ret = %d\n", __func__, ret);
    mtp_control_reg(10000);
}

static int set_sector_erase(unsigned int usSector, unsigned int chip_id_h, unsigned int chip_id_l)
{
    printk("%s, chip id: 0x%x%x\n", __func__, chip_id_h, chip_id_l);

    int ret;
    char buf[64];

    //a.Setting sector erase key (0x41012)
    ret = outwrite(0x41012, 0xA512, 2);
    printk("%s, a.Setting sector erase key (0x41012), outwrite, ret = %d\n", __func__, ret);

    //b.Setting standby key (0x40010)
    set_standby_key(chip_id_h, chip_id_l);
    printk("%s, b.Setting standby key (0x40010)\n", __func__);

    //c.Setting sector number (0x41018)
    buf[0] = 0X25;
    buf[1] = 0X18;
    buf[2] = 0X10;
    buf[3] = 0X04;
    buf[4] = (char)((usSector & 0x00FF) >> 0);
    buf[5] = (char)((usSector & 0xFF00) >> 8);
    buf[6] = 0x00;
    buf[7] = 0x00;
    ret = ilitek_i2c_write(i2c.client, buf, 8);
    printk("%s, c.Setting sector number (0x41018), ilitek_i2c_write, ret = %d\n", __func__, ret);

    //d.Enable sector erase (0x41031)
    ret = outwrite(0x41031, 0xA5, 1);
    printk("%s, d.Enable sector erase (0x41031), outwrite, ret = %d\n", __func__, ret);

    //e.Enable standby (0x40024)
    //f.Wait chip erase (94ms) or check MTP_busy (0x4102c)
    set_standby();
    printk("%s, e.Enable standby\n", __func__);
}

static int set_chip_erase(unsigned short usLCKN, unsigned int chip_id_h, unsigned int chip_id_l)
{
    printk("%s, chip id: 0x%x%x\n", __func__, chip_id_h, chip_id_l);

    int ret;
    char buf[64];
    unsigned char ucCount;

    //a.Setting lock page (0x41024)
    for(ucCount = 0; ucCount < 3; ucCount++)
    {
        ret = outwrite(0x41024, usLCKN, 2);
        printk("%s, a.Setting lock page (0x41024), outwrite, ret = %d\n", __func__, ret);

        if((inwrite(0x41024) | 0x00FF) == usLCKN)
        {
            break;
        }
    }
    if((inwrite(0x41024) | 0x00FF) != usLCKN)
    {
        return -1;
    }

    //b.Setting chip erase key (0x41010)
    ret = outwrite(0x41010, 0xCEAE, 4);
    printk("%s, b.Setting chip erase key (0x41010), outwrite, ret = %d\n", __func__, ret);

    //c.Setting standby key (0x40010)
    set_standby_key(chip_id_h, chip_id_l);
    printk("%s, c.Setting standby key (0x40010)\n", __func__);

    //d.Enable chip erase (0x41030)
    ret = outwrite(0x41030, 0x7E, 1);
    printk("%s, d.Enable chip erase (0x41030), outwrite, ret = %d\n", __func__, ret);

    //e.Enable standby (0x40024)
    //f.Wait chip erase (94ms) or check MTP_busy (0x4102c)
    set_standby();
    printk("%s, e.Enable standby\n", __func__);

    //g.Clear lock page (0x41024)
    ret = outwrite(0x41024, 0xFFFF, 2);
    printk("%s, g.Clear lock page (0x41024), outwrite, ret = %d\n", __func__, ret);

    return 0;
}

static void set_preprogram(unsigned int usStart, unsigned int usEnd, unsigned int chip_id_h, unsigned int chip_id_l)
{
    printk("%s, chip id: 0x%x%x\n", __func__, chip_id_h, chip_id_l);

    int ret;
    char buf[64];
    unsigned int usSize;

    //a.Setting program key
    set_program_key();
    printk("%s, a.Setting program key\n", __func__);

    //b.Setting standby key
    set_standby_key(chip_id_h, chip_id_l);
    printk("%s, b.Setting standby key (0x40010)\n", __func__);

    //c.Setting program start address and size (0x41018)
    usStart = usStart / 16;
    usEnd = usEnd / 16;
    usSize = usEnd - usStart;
    ret = outwrite(0x41018, usStart + ((usSize - 1) << 16), 4);
    printk("%s, c.Setting program start address and size (0x41018), outwrite, ret = %d\n", __func__, ret);

    //d.Setting pre-program data (0x4101c)
    ret = outwrite(0x4101C, 0xFFFFFFFF, 4);
    printk("%s, d.Setting pre-program data (0x4101c), outwrite, ret = %d\n", __func__, ret);

    //e.Enable pre-program (0x41032)
    ret = outwrite(0x41032, 0xB6, 1);
    printk("%s, e.Enable pre-program (0x41032), outwrite, ret = %d\n", __func__, ret);

    //f.Enable standby (0x40024)
    //g.Wait program time (260us*size) or check MTP_busy (0x4102c)
    set_standby();
    printk("%s, f.Enable standby\n", __func__);

    //h.Clear program key (0x41014)
    clear_program_key();
    printk("%s, h.Clear program key (0x41014)\n", __func__);
}

static void clear_standby_key(void)
{
    int ret;
    char buf[64];
    ret = outwrite(0x40010, 0x0000, 2);
    printk("%s, outwrite, ret = %d\n", __func__, ret);
}

static void set_program_and_erase_time(void)
{
    int ret;
    char buf[64];
    //a.setting Program time 400us
    ret = outwrite(0x41004, 0x9F000960, 4);
    printk("%s, a.setting Program time 400us, outwrite, ret = %d\n", __func__, ret);
    //b.setting Erase time 50ms
    ret = outwrite(0x41008, 0x24, 1);
    printk("%s, b.setting Erase time 50ms, outwrite, ret = %d\n", __func__, ret);
}

static void set_48MHz(unsigned int chip_id_h, unsigned int chip_id_l)
{
    printk("%s, chip id: 0x%x%x\n", __func__, chip_id_h, chip_id_l);

    int ret;
    char buf[64];
    unsigned int uiData;

    //a.Setting OSC key
    ret = outwrite(0x42000, 0x27,1);
    printk("%s, a.Setting OSC key, outwrite, ret = %d\n", __func__, ret);

    //b.Setting standby key
    set_standby_key(chip_id_h, chip_id_l);
    printk("%s, b.Setting standby key (0x40010)\n", __func__);

    //c.Disable OSC48DIV2
    uiData = inwrite(0x42008);

    buf[0] = 0x25;
    buf[1] = 0x08;
    buf[2] = 0x20;
    buf[3] = 0x04;
    buf[4] = ((unsigned char)((uiData & 0x000000FF) >> 0));
    buf[5] = ((unsigned char)((uiData & 0x0000FF00) >> 8));
    buf[6] = ((unsigned char)((uiData & 0x00FF0000) >> 16));
    buf[7] = ((unsigned char)((uiData & 0xFF000000) >> 24) | 4);
    ret = ilitek_i2c_write(i2c.client, buf, 8);
    printk("%s, c.Disable OSC48DIV2, ilitek_i2c_write, ret = %d\n", __func__, ret);

    //d.Enable standby
    ret = outwrite(0x40024, 0x01, 1);
    printk("%s, d.Enable standby, outwrite, ret = %d\n", __func__, ret);

    //e.Clear OSC key
    ret = outwrite(0x42000, 0x0, 1);
    printk("%s, e.Clear OSC key, outwrite, ret = %d\n", __func__, ret);

    //f.Clear standby key
    clear_standby_key();
    printk("%s, f.Clear standby key\n", __func__);
}

static void reset_register(int data)
{
    unsigned char buf[16];
    int ret, i;

    //RESET CPU
    //1.Software Reset
    //1-1 Clear Reset Done Flag
    outwrite(0x40048, 0x00000000, 4);
    //1-2 Set CDC/Core Reset
    outwrite(0x40040, data, 4);
    for(i = 0; i < 5000; i++)
    {
        udelay(1000);
        if((inwrite(0x040048) & 0x00010000) == 0x00010000)
        {
            break;
        }
    }
    msleep(100);
    outwrite(0x40040, 0x00000000, 4);
}

static void exit_ice_mode(void)
{
    unsigned char buf[16];
    int ret, i;

    reset_register(0x033E00AE);
    //Exit ICE
    buf[0] = 0x1B;
    buf[1] = 0x62;
    buf[2] = 0x10;
    buf[3] = 0x18;
    ret = ilitek_i2c_write(i2c.client, buf, 4);
    printk("%s, ilitek_i2c_write, ret = %d\n", __func__, ret);
    msleep(1000);
}

/*
description
    upgrade firmware
prarmeters

return
    status
*/
static int ilitek_upgrade_firmware(void)
{
    int ret = 0, upgrade_status = 0, i , j, k = 0, ap_len = 0, df_len = 0, ic_model = 0, ucCount = 0, ice_flag = 0;
    unsigned char buffer[128] = {0}, chip_id_H = 0, chip_id_L = 0;
    unsigned char buf[10] = {0};
    unsigned long ap_startaddr, df_startaddr, ap_endaddr, df_endaddr, ap_checksum = 0, df_checksum = 0, ice_checksum = 0, temp = 0;
    unsigned char firmware_ver[4];
    unsigned int  bl_ver = 0, flow_flag = 0, uiData, usStart, usEnd, usSize;
    struct i2c_msg msgs[] = {
        {.addr = i2c.client->addr, .flags = 0, .len = 0, .buf = buffer,}
    };
    ap_startaddr = ( CTPM_FW[0] << 16 ) + ( CTPM_FW[1] << 8 ) + CTPM_FW[2];
    ap_endaddr = ( CTPM_FW[3] << 16 ) + ( CTPM_FW[4] << 8 ) + CTPM_FW[5];
    ap_checksum = ( CTPM_FW[6] << 16 ) + ( CTPM_FW[7] << 8 ) + CTPM_FW[8];
    df_startaddr = ( CTPM_FW[9] << 16 ) + ( CTPM_FW[10] << 8 ) + CTPM_FW[11];
    df_endaddr = ( CTPM_FW[12] << 16 ) + ( CTPM_FW[13] << 8 ) + CTPM_FW[14];
    df_checksum = ( CTPM_FW[15] << 16 ) + ( CTPM_FW[16] << 8 ) + CTPM_FW[17];
    firmware_ver[0] = CTPM_FW[18];
    firmware_ver[1] = CTPM_FW[19];
    firmware_ver[2] = CTPM_FW[20];
    firmware_ver[3] = CTPM_FW[21];
    df_len = ( CTPM_FW[22] << 16 ) + ( CTPM_FW[23] << 8 ) + CTPM_FW[24];
    ap_len = ( CTPM_FW[25] << 16 ) + ( CTPM_FW[26] << 8 ) + CTPM_FW[27];
    printk("ap_startaddr=0x%d, ap_endaddr=0x%d, ap_checksum=0x%d\n", ap_startaddr, ap_endaddr, ap_checksum);
    printk("df_startaddr=0x%d, df_endaddr=0x%d, df_checksum=0x%d\n", df_startaddr, df_endaddr, df_checksum);

    //reset
    ilitek_i2c_reset();

    //set into test mode
    buffer[0] = 0xF2;
    buffer[1] = 0x01;
    msgs[0].len = 2;
    ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
    msleep(100);

    //check ic type
    ilitek_i2c_write_read(i2c.client, 0x61, 0, buffer, 5);
    printk("buffer[0] = 0x%X\n", buffer[0]);
    if(buffer[0] == 0x07 && buffer[1] == 0x00)
    {
        ice_flag = 1;
        chip_id_H = 0x21;
        chip_id_L = 0x15;
        ic_model = (chip_id_H << 8) + chip_id_L;
        printk("IC is 0x%4X\n", ic_model);
    }
    if(buffer[0] == 0x08 && buffer[1] == 0x00)
    {
        ice_flag = 1;
        chip_id_H = 0x21;
        chip_id_L = 0x16;
        ic_model = (chip_id_H << 8) + chip_id_L;
        printk("IC is 0x%4X\n", ic_model);
    }
    if((buffer[0] == 0xFF && buffer[1] == 0xFF) || (buffer[0] == 0x00 && buffer[1] == 0x00))
    {
        ice_flag = 1;
        printk("IC is NULL\n");
    }

    if(ice_flag)
    {
        printk("%s, entry ice mode start\n", __func__);
        ret = outwrite(0x181062, 0x0, 0);
        printk("%s, outwrite, ret = %d\n", __func__, ret);

        //Device clock ON
        ret = outwrite(0x40020, 0x00000000, 4);
        printk("%s, Device clock ON, outwrite, ret = %d\n", __func__, ret);

        //Read PID
        buffer[0] = inwrite(0x4009b);
        printk("%s, chip id:%x\n", __func__, buffer[0]);
        if(buffer[0] == 0x01 || buffer[0] == 0x02 || buffer[0] == 0x03 || buffer[0] == 0x04 || buffer[0] == 0x11 || buffer[0] == 0x12 || buffer[0] == 0x13 || buffer[0] == 0x14)
        {
            printk("%s, ic is 2115\n", __func__);
            chip_id_H = 0x21;
            chip_id_L = 0x15;
            ic_model = (chip_id_H << 8) + chip_id_L;
        }
        if(buffer[0] == 0x81 || buffer[0] == 0x82 || buffer[0] == 0x83 || buffer[0] == 0x84 || buffer[0] == 0x91 || buffer[0] == 0x92 || buffer[0] == 0x93 || buffer[0] == 0x94)
        {
            printk("%s, ic is 2116\n", __func__);
            chip_id_H = 0x21;
            chip_id_L = 0x16;
            ic_model = (chip_id_H << 8) + chip_id_L;
        }
        printk("ic = 0x%04x\n", ic_model);

        printk("%s, set_48MHz\n", __func__);
        set_48MHz(chip_id_H, chip_id_L);

        printk("%s, set_program_and_erase_time\n",__func__);
        set_program_and_erase_time();

        printk("%s, set_preprogram, chip id: 0x%x%x\n", __func__, chip_id_H, chip_id_L);
        set_preprogram(0x0000, 0x7F00, chip_id_H, chip_id_L);

        printk("%s, set_chip_erase, chip id: 0x%x%x\n", __func__, chip_id_H, chip_id_L);
        set_chip_erase(0x7FFF, chip_id_H, chip_id_L);

        printk("%s, set_sector_erase, chip id: 0x%x%x\n", __func__, chip_id_H, chip_id_L);
        for(i = 0x78; i <= 0x7E; i++)
        {
            set_sector_erase(i, chip_id_H, chip_id_L);
        }

        //write data
        if(((ap_endaddr + 1) % 4) != 0)
        {
            ap_endaddr += 4;
        }

        //Setting program key
        set_program_key();
        printk("%s, Setting program key\n", __func__);

        j = 0;
        for(i = ap_startaddr; i < ap_endaddr; i += 16)
        {
            buffer[0] = 0x25;
            buffer[3] = (char)((i  & 0x00FF0000) >> 16);
            buffer[2] = (char)((i  & 0x0000FF00) >> 8);
            buffer[1] = (char)((i  & 0x000000FF));
            for(k = 0; k < 16; k++)
            {
                buffer[4 + k] = CTPM_FW[i + 32 + k];
            }
            msgs[0].len = 20;
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);

            upgrade_status = ((i * 100)) / ap_endaddr;
            if(upgrade_status > j)
            {
                printk("%cILITEK: Firmware Upgrade(AP), %02d%c. \n", 0x0D, upgrade_status, '%');
                j = j + 10;
            }
            mtp_control_reg(100);
        }
        printk("%s, upgrade end(program)\n", __func__);

        //clear program key
        printk("%s, Clear program key (0x41014)\n", __func__);
        clear_program_key();

        printk("%s, after write, chip id: 0x%x%x\n", __func__, chip_id_H, chip_id_L);
        set_standby_key(chip_id_H, chip_id_L);

        ret = outwrite(0x41020, ap_startaddr + (ap_endaddr << 16), 4);
        printk("%s, outwrite, ret = %d", __func__, ret);

        ret = outwrite(0x41038, 0x01, 1);
        printk("%s, outwrite, ret = %d", __func__, ret);

        ret = outwrite(0x41033, 0xCD, 1);
        printk("%s, outwrite, ret = %d", __func__, ret);

        set_standby();

        ice_checksum = inwrite(0x41028);
        if(ap_checksum != ice_checksum)
        {
            printk("checksum error, hex ap_checksum = 0x%6x, ic checksum = 0x%6x\n", ap_checksum, ice_checksum);
        }
        else
        {
            printk("checksum equal, hex ap_checksum = 0x%6x, ic checksum = 0x%6x\n", ap_checksum, ice_checksum);
        }

        exit_ice_mode();

        if(ap_checksum == ice_checksum)
        {
            //check system busy
            for(i = 0; i < 50 ; i++)
            {
                ilitek_i2c_write_read(i2c.client, 0x80, 0, buffer, 1);
                if(buffer[0] == 0x50)
                {
                    printk("%s, system is not busy, i = %d\n", __func__, i);
                    break;
                }
            }
        }
        if(i == 50 && buf[0] != 0x50)
        {
            printk("%s, system is busy, i = %d\n", __func__, i);
        }
    }
    else
    {
        buffer[0] = 0xc0;
        msgs[0].len = 1;
        ret = ilitek_i2c_write_read(i2c.client, 0xc0, 0, buffer, 1);
        if(ret < 0)
        {
            return 3;
        }
        msleep(30);
        printk("ic. mode = %d\n", buffer[0]);

        if(buffer[0] != 0x55)
        {
            buffer[0] = 0xc4;
            buffer[1] = 0x5A;
            buffer[2] = 0xA5;
            msgs[0].len = 3;
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
            if(ret < 0)
            {
                return 3;
            }
            msleep(30);
            buffer[0] = 0xc2;
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
            if(ret < 0)
            {
                return 3;
            }
            msleep(100);
        }

        if(ilitek_i2c_write_read(i2c.client, ILITEK_TP_CMD_GET_FIRMWARE_VERSION, 10, buf, 4) < 0)
        {
            return 3;
        }

        printk(ILITEK_DEBUG_LEVEL "%s, bl version %d.%d.%d.%d\n", __func__, buf[0], buf[1], buf[2], buf[3]);
        msleep(100);
        bl_ver = buf[3] + (buf[2] << 8) + (buf[1] << 16) + (buf[0] << 24);
        if(bl_ver)
        {
            if(bl_ver < 0x01000100)
            {
                flow_flag = 1;
            }
            else
            {
                flow_flag = 2;
            }
        }
        else
        {
            return  3;
        }

        buffer[0] = 0xc0;
        msgs[0].len = 1;
        ret = ilitek_i2c_write_read(i2c.client, 0xc0, 0, buffer, 1);
        if(ret < 0)
        {
            return 3;
        }

        msleep(30);
        printk("ILITEK:%s, upgrade firmware...\n", __func__);
        buffer[0] = 0xc4;
        msgs[0].len = 10;
        buffer[1] = 0x5A;
        buffer[2] = 0xA5;
        buffer[3] = 0;
        buffer[4] = CTPM_FW[3];
        buffer[5] = CTPM_FW[4];
        buffer[6] = CTPM_FW[5];
        buffer[7] = CTPM_FW[6];
        buffer[8] = CTPM_FW[7];
        buffer[9] = CTPM_FW[8];
        ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
        if(ret < 0)
        {
            return 3;
        }

        msleep(30);

        buffer[0] = 0xc4;
        msgs[0].len = 10;
        buffer[1] = 0x5A;
        buffer[2] = 0xA5;
        buffer[3] = 1;
        buffer[4] = 0;
        buffer[5] = 0;
        buffer[6] = 0;
        buffer[7] = 0;
        buffer[8] = 0;
        buffer[9] = 0;
        ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
        if(ret < 0)
        {
            return 3;
        }

        msleep(30);

        j = 0;
        for(i = 0; i < df_len; i += 32)
        {
            j += 1;
            if(flow_flag == 1)
            {
                if((j % 16) == 1)
                {
                    msleep(60);
                }
            }
            else if(flow_flag == 2)
            {
                if((j % 8) == 2)
                {
                    msleep(40);
                }
            }

            for(k = 0; k < 32; k++)
            {
                buffer[1 + k] = CTPM_FW[i + 32 + k];
            }

            buffer[0] = 0xc3;
            msgs[0].len = 33;
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
            if(ret < 0)
            {
                return 3;
            }
            upgrade_status = (i * 100) / df_len;
            if(flow_flag == 1)
            {
                msleep(10);
            }
            else if(flow_flag == 2)
            {
                msleep(20);
            }
            printk("%cILITEK: Firmware Upgrade(Data flash), %02d%c. ",0x0D,upgrade_status,'%');
        }

        buffer[0] = 0xc4;
        msgs[0].len = 10;
        buffer[1] = 0x5A;
        buffer[2] = 0xA5;
        buffer[3] = 0;
        buffer[4] = CTPM_FW[3];
        buffer[5] = CTPM_FW[4];
        buffer[6] = CTPM_FW[5];
        buffer[7] = CTPM_FW[6];
        buffer[8] = CTPM_FW[7];
        buffer[9] = CTPM_FW[8];
        ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
        if(ret < 0)
        {
            return 3;
        }
        msleep(30);

        j = 0;
        for(i = 0; i < ap_len; i += 32)
        {
            j += 1;
            if(flow_flag == 1)
            {
                if((j % 16) == 1)
                {
                    msleep(60);
                }
            }
            else if(flow_flag == 2)
            {
                if((j % 8) == 2)
                {
                    msleep(40);
                }
            }

            for(k = 0; k < 32; k++)
            {
                buffer[1 + k] = CTPM_FW[i + df_len + 32 + k];
            }

            buffer[0] = 0xc3;
            msgs[0].len = 33;
            ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
            if(ret < 0)
            {
                return 3;
            }
            upgrade_status = (i * 100) / ap_len;
            if(flow_flag == 1)
            {
                msleep(10);
            }
            else if(flow_flag == 2)
            {
                msleep(20);
            }
            printk("%cILITEK: Firmware Upgrade(AP), %02d%c. ", 0x0D, upgrade_status, '%');
        }

        printk("ILITEK:%s, upgrade firmware completed\n", __func__);

        buffer[0] = 0xc4;
        buffer[1] = 0x5A;
        buffer[2] = 0xA5;
        msgs[0].len = 3;
        ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
        if(ret < 0)
        {
            return 3;
        }

        msleep(30);
        buffer[0] = 0xc1;
        ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
        if(ret < 0)
        {
            return 3;
        }

        buffer[0] = 0xc0;
        msgs[0].len = 1;
        ret = ilitek_i2c_write_read(i2c.client, 0xc0, 0, buffer, 1);
        if(ret < 0)
        {
            return 3;
        }
        msleep(30);
        printk("ic. mode =%d, it's  %s \n", buffer[0], ((buffer[0] == 0x5A)?"AP MODE":"BL MODE"));
    }

    msleep(100);
    return 2;
}
#endif

/*
description
    register i2c device and its input device
parameters
    none
return
    status
*/
static int ilitek_i2c_register_device(void)
{
    int ret = 0;
    printk(ILITEK_DEBUG_LEVEL "%s, client.addr: 0x%X\n", __func__, (unsigned int)i2c.client->addr);
    printk(ILITEK_DEBUG_LEVEL "%s, client.adapter: 0x%X\n", __func__, (unsigned int)i2c.client->adapter);
    //	printk(ILITEK_DEBUG_LEVEL "%s, client.driver: 0x%X\n", __func__, (unsigned int)i2c.client->driver);
    if((i2c.client->addr == 0) || (i2c.client->adapter == 0)) // || (i2c.client->driver == 0))
    {
        printk(ILITEK_ERROR_LEVEL "%s, invalid register\n", __func__);
        return ret;
    }
    //read touch parameter
    ilitek_i2c_read_tp_info();

#ifdef UPGRADE_FIRMWARE_ON_BOOT
    //check firmware version
    int i = 0;
    for(i = 0; i < 4; i++)
    {
        printk("i2c.firmware_ver[%d] = %d, firmware_ver[%d] = %d\n", i, i2c.firmware_ver[i], i, CTPM_FW[i + 18]);
        if((i2c.firmware_ver[i] > CTPM_FW[i + 18]) || ((i == 3) && (i2c.firmware_ver[3] == CTPM_FW[3 + 18])))
        {
            ret = 1;
            break;
        }
        else if(i2c.firmware_ver[i] < CTPM_FW[i + 18])
        {
            break;
        }
    }

    if(ret == 1)
    {
        printk("Do not need update\n");
    }
    else
    {
        //upgrade fw
        ret = ilitek_upgrade_firmware();
        if(ret == 2)
        {
            printk("update end\n");
        }
        else if(ret == 3)
        {
            printk("i2c communication error\n");
        }

        //reset
        ilitek_i2c_reset();

        //read touch parameter
        ilitek_i2c_read_tp_info();
    }
#endif

    //register input device
    i2c.input_dev = input_allocate_device();
    if(i2c.input_dev == NULL)
    {
        printk(ILITEK_ERROR_LEVEL "%s, allocate input device, error\n", __func__);
        return -1;
    }
    ilitek_set_input_param(i2c.input_dev, i2c.max_tp, i2c.max_x, i2c.max_y);
    ret = input_register_device(i2c.input_dev);
    if(ret)
    {
        printk(ILITEK_ERROR_LEVEL "%s, register input device, error\n", __func__);
        return ret;
    }
    printk(ILITEK_ERROR_LEVEL "%s, register input device, success\n", __func__);
    i2c.valid_input_register = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
    i2c.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    i2c.early_suspend.suspend = ilitek_i2c_early_suspend;
    i2c.early_suspend.resume = ilitek_i2c_late_resume;
    register_early_suspend(&i2c.early_suspend);
#endif

    if(i2c.client->irq != 0)
    {
        i2c.irq_work_queue = create_singlethread_workqueue("ilitek_i2c_irq_queue");
        if(i2c.irq_work_queue)
        {
            INIT_WORK(&i2c.irq_work, ilitek_i2c_irq_work_queue_func);
        #ifdef CLOCK_INTERRUPT
            if(request_irq(i2c.client->irq, ilitek_i2c_isr, IRQF_TRIGGER_FALLING , "ilitek_i2c_irq", &i2c))
            {
                printk(ILITEK_ERROR_LEVEL "%s, request irq, error\n", __func__);
            }
            else
            {
                i2c.valid_irq_request = 1;
                i2c.irq_status = 1;
                printk(ILITEK_ERROR_LEVEL "%s, request irq(Trigger Falling), success\n", __func__);
            }
        #else
            init_timer(&i2c.timer);
            i2c.timer.data = (unsigned long)&i2c;
            i2c.timer.function = ilitek_i2c_timer;
            if(request_irq(i2c.client->irq, ilitek_i2c_isr, IRQF_TRIGGER_LOW, "ilitek_i2c_irq", &i2c))
            {
                printk(ILITEK_ERROR_LEVEL "%s, request irq, error\n", __func__);
            }
            else
            {
                i2c.valid_irq_request = 1;
                i2c.irq_status = 1;
                printk(ILITEK_ERROR_LEVEL "%s, request irq(Trigger Low), success\n", __func__);
            }
        #endif
        }
    }
    else
    {
        i2c.stop_polling = 0;
        i2c.thread = kthread_create(ilitek_i2c_polling_thread, NULL, "ilitek_i2c_thread");
        printk(ILITEK_ERROR_LEVEL "%s, polling mode\n", __func__);
        if(i2c.thread == (struct task_struct*)ERR_PTR)
        {
            i2c.thread = NULL;
            printk(ILITEK_ERROR_LEVEL "%s, kthread create, error\n", __func__);
        }
        else
        {
            wake_up_process(i2c.thread);
        }
    }

    return 0;
}

/*
description
    initiali function for driver to invoke.
parameters
    none
return
    status
*/
static int __init ilitek_init(void)
{
    int ret = 0;
    //initialize global variable
    memset(&dev, 0, sizeof(struct dev_data));
    memset(&i2c, 0, sizeof(struct i2c_data));

    //initialize mutex object
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
    init_MUTEX(&i2c.wr_sem);
#else
    sema_init(&i2c.wr_sem, 1);
#endif
    i2c.wr_sem.count = 1;
    i2c.report_status = 1;
    ret = i2c_add_driver(&ilitek_i2c_driver);
    if(ret == 0)
    {
        i2c.valid_i2c_register = 1;
        printk(ILITEK_DEBUG_LEVEL "%s, add i2c device, success\n", __func__);
        if(i2c.client == NULL)
        {
            printk(ILITEK_ERROR_LEVEL "%s, no i2c board information\n", __func__);
        }
    }
    else
    {
        printk(ILITEK_ERROR_LEVEL "%s, add i2c device, error\n", __func__);
    }
    return ret;
}

/*
description
    driver exit function
parameters
    none
return
    nothing
*/
static void __exit ilitek_exit(void)
{
    printk("%s, enter\n", __func__);
    if(i2c.valid_i2c_register != 0)
    {
        printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver\n", __func__);
        i2c_del_driver(&ilitek_i2c_driver);
        printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver\n", __func__);
    }
    else
    {
        printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver Fail\n", __func__);
    }

    remove_proc_entry("ilitek_ctrl", NULL);
}

//set init and exit function for this module
module_init(ilitek_init);
module_exit(ilitek_exit);
