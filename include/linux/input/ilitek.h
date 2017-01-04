//driver information
#define DERVER_VERSION_MAJOR	3
#define DERVER_VERSION_MINOR	0
#define RELEASE_VERSION			5

//hardware setting

//i2c board info

//debug message
#define ILITEK_DEBUG_LEVEL	KERN_ALERT
#define ILITEK_ERROR_LEVEL	KERN_ALERT
#define DBG(fmt, args...)	if (DBG_FLAG)printk("%s(%d): " fmt, __func__, __LINE__, ## args)
#define DBG_CO(fmt, args...)	if (DBG_FLAG || DBG_COR)printk("%s: " fmt, "ilitek",  ## args)

//upgrade firmware on boot
//#define UPGRADE_FIRMWARE_ON_BOOT
#ifdef UPGRADE_FIRMWARE_ON_BOOT
#include "ilitek_fw.h"
#endif

//gesture resume function
//#define GESTURE
#ifdef GESTURE
#define GESTURE_FUN_1	1 //model learning function
#define GESTURE_FUN_2	2 //algorithm function
#define GESTURE_FUN		GESTURE_FUN_2
#define _DOUBLE_CLICK_
#define GESTURE_H
#include "gesture.h"
#endif

//leather function

//big area suspend

//defing key pad function for protocol 2.X
//#define VIRTUAL_KEY_PAD
#define VIRTUAL_FUN_1	1	//0X81 with key_id
#define VIRTUAL_FUN_2	2	//0x81 with x position
#define VIRTUAL_FUN_3	3	//Judge x & y position
//#define VIRTUAL_FUN	VIRTUAL_FUN_2
#define BTN_DELAY_TIME	500 //ms

//define key pad range for protocol 2.X
#define KEYPAD01_X1	0
#define KEYPAD01_X2	1000
#define KEYPAD02_X1	1000
#define KEYPAD02_X2	2000
#define KEYPAD03_X1	2000
#define KEYPAD03_X2	3000
#define KEYPAD04_X1	3000
#define KEYPAD04_X2	3968
#define KEYPAD_Y	2100

//define the application command
#define ILITEK_IOCTL_BASE                       100
#define ILITEK_IOCTL_I2C_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 0, unsigned char*)
#define ILITEK_IOCTL_I2C_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 2, unsigned char*)
#define ILITEK_IOCTL_I2C_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 3, int)
#define ILITEK_IOCTL_USB_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 4, unsigned char*)
#define ILITEK_IOCTL_USB_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 5, int)
#define ILITEK_IOCTL_USB_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 6, unsigned char*)
#define ILITEK_IOCTL_USB_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 7, int)
#define ILITEK_IOCTL_DRIVER_INFORMATION		    _IOWR(ILITEK_IOCTL_BASE, 8, int)
#define ILITEK_IOCTL_USB_UPDATE_RESOLUTION      _IOWR(ILITEK_IOCTL_BASE, 9, int)
#define ILITEK_IOCTL_I2C_INT_FLAG	            _IOWR(ILITEK_IOCTL_BASE, 10, int)
#define ILITEK_IOCTL_I2C_UPDATE                 _IOWR(ILITEK_IOCTL_BASE, 11, int)
#define ILITEK_IOCTL_STOP_READ_DATA             _IOWR(ILITEK_IOCTL_BASE, 12, int)
#define ILITEK_IOCTL_START_READ_DATA            _IOWR(ILITEK_IOCTL_BASE, 13, int)
#define ILITEK_IOCTL_GET_INTERFANCE				_IOWR(ILITEK_IOCTL_BASE, 14, int)//default setting is i2c interface
#define ILITEK_IOCTL_I2C_SWITCH_IRQ				_IOWR(ILITEK_IOCTL_BASE, 15, int)
#define ILITEK_IOCTL_UPDATE_FLAG				_IOWR(ILITEK_IOCTL_BASE, 16, int)
#define ILITEK_IOCTL_I2C_UPDATE_FW				_IOWR(ILITEK_IOCTL_BASE, 18, int)
#define ILITEK_IOCTL_I2C_GESTURE_FLAG			_IOWR(ILITEK_IOCTL_BASE, 26, int)
#define ILITEK_IOCTL_I2C_GESTURE_RETURN			_IOWR(ILITEK_IOCTL_BASE, 27, int)
#define ILITEK_IOCTL_I2C_GET_GESTURE_MODEL		_IOWR(ILITEK_IOCTL_BASE, 28, int)
#define ILITEK_IOCTL_I2C_LOAD_GESTURE_LIST		_IOWR(ILITEK_IOCTL_BASE, 29, int)

//i2c command for ilitek touch screen
#define ILITEK_TP_CMD_READ_DATA				0x10
#define ILITEK_TP_CMD_READ_SUB_DATA			0x11
#define ILITEK_TP_CMD_GET_RESOLUTION		0x20
#define ILITEK_TP_CMD_GET_KEY_INFORMATION	0x22
#define ILITEK_TP_CMD_SLEEP                 0x30
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION	0x40
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION	0x42
#define	ILITEK_TP_CMD_CALIBRATION			0xCC
#define	ILITEK_TP_CMD_CALIBRATION_STATUS	0xCD
#define ILITEK_TP_CMD_ERASE_BACKGROUND		0xCE

//i2c command for Protocol 3.1
#define ILITEK_TP_CMD_TOUCH_STATUS			0x0F

#define TOUCH_POINT		0x80
#define TOUCH_KEY		0xC0
#define RELEASE_KEY		0x40
#define RELEASE_POINT	0x00
//----TP function define-------
//#define ROTATE_FLAG
//#define TRANSFER_LIMIT
#define CLOCK_INTERRUPT
//#define SET_RESET
//#define TOUCH_PROTOCOL_B
#define TOUCH_PRESSURE
#define ILITEK_I2C_RETRY_COUNT		3
//-----------------------------
#define ILITEK_I2C_DRIVER_NAME		"ilitek_i2c"
#define ILITEK_FILE_DRIVER_NAME		"ilitek_file"

//key
struct key_info {
	int id;
	int x;
	int y;
	int status;
	int flag;
};
//touch
struct touch_info {
	int id;
	int x;
	int y;
	int status;
	int flag;
};
// declare i2c data member
struct i2c_data {
	// input device
	struct input_dev *input_dev;
	// i2c client
	struct i2c_client *client;
	// polling thread
	struct task_struct *thread;
	// maximum x
	int max_x;
	// maximum y
	int max_y;
	// maximum touch point
	int max_tp;
	// maximum key button
	int max_btn;
	// the total number of x channel
	int x_ch;
	// the total number of y channel
	int y_ch;
	// check whether i2c driver is registered success
	int valid_i2c_register;
	// check whether input driver is registered success
	int valid_input_register;
	// check whether the i2c enter suspend or not
	int stop_polling;
	// read semaphore
	struct semaphore wr_sem;
	// protocol version
	int protocol_ver;
	//firmware version
    unsigned char firmware_ver[4];
	// valid irq request
	int valid_irq_request;
	// work queue for interrupt use only
	struct workqueue_struct *irq_work_queue;
	// work struct for work queue
	struct work_struct irq_work;
	
    struct timer_list timer;
	
	int report_status;
	
	int irq_status;
	//irq_status enable:1 disable:0
	struct completion complete;
	// touch_flag release:0 touch:1 
 	int touch_flag ;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	int keyflag;
	int keycount;
	int key_xlen;
	int key_ylen;
	struct key_info keyinfo[10];
	struct touch_info touchinfo[10];
};

//device data
struct dev_data {
	//device number
	dev_t devno;
	//character device
	struct cdev cdev;
	//class device
	struct class *class;
};
