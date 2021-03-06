/* drivers/input/touchscreen/silabs_f760.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
//#include <linux/melfas_ts.h>

#include <linux/firmware.h>
#if defined(CONFIG_MACH_TOTORO_CTC)
#include <mach/vreg.h>
#endif

#define MAX_X	240 
#define MAX_Y	320
#if defined(CONFIG_MACH_TOTORO_CTC)
#define TSP_INT 19
#else
#define TSP_INT 30
#endif

#define F760_MAX_TOUCH		2
#define ESCAPE_ADDR 	    0xAA
#define TS_READ_START_ADDR 	    0x10
#define TS_READ_VERSION_ADDR	0x1F
#define TS_READ_ESD_ADDR	0x1E
#define TS_READ_REGS_LEN 		13
#define SILABS_MAX_TOUCH		F760_MAX_TOUCH
#define MTSI_VERSION		    0x05

#define __TOUCH_DEBUG__ 1

#define I2C_RETRY_CNT			3

#define TOUCH_ON 1
#define TOUCH_OFF 0

#define PRESS_KEY				1
#define RELEASE_KEY				0
#define MAX_KEYS	     2

#define SET_DOWNLOAD_BY_GPIO	1

#define SILABS_TS_NAME "silabs-f760"

#if defined(CONFIG_MACH_TOTORO_CTC)
#define TSP_LDO "ldo8"
#endif

#define YTE_MODULE_VER   0x04
#define SMAC_MODULE_VER   0x05
#define SMAC_MODULE_VER_NEW   0x07
#define FW_VER  0x14
//#define FW_VER_NEW  0x0D

#define NUM_TX_CHANNEL 12
#define NUM_RX_CHANNEL 9
#define JIG_MODE_COMMAND 0xA0
#define RAWDATA_ADDRESS          0x003A
#define BASELINE_ADDRESS         0x0112
#define  QUICKSENSE_OVERHEAD     6 
#define I2CMAP_BUTTON_ADDRESS    (0x0212 + 0x22)
#define  NUM_MTRBUTTONS             2

#define DEBUG  0
#if DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do {} while (0)
#endif

static int prev_wdog_val = -1;
//static int check_ic_counter = 3;
static unsigned int touch_present = 0;
#if defined(CONFIG_MACH_TOTORO_CTC)
static unsigned int touch_resume = 1;
static unsigned int touch_reset = 1;
#endif

static struct workqueue_struct *check_ic_wq;

static struct regulator *touch_regulator=NULL;
//add by brcm

int touch_id[2], posX[2], posY[2], strength[2];

static int firmware_ret_val = -1;
static int pre_ta_stat = 0;
int touch_check=0;
int tsp_irq;
int tsp_chheck=0;
int TSP_MODULE_ID;
#if defined (CONFIG_MACH_TOTORO_CTC)
int TSP_FIRMWARE_ID;
#endif
EXPORT_SYMBOL(TSP_MODULE_ID);

uint8_t buf_firmware[3];
int firm_update( void );
int set_tsp_for_ta_detect(int);
static int testmode = 1;
int PHONE_VER;
int tsp_status=0;
unsigned long check_node;

int Tx_Channel = NUM_TX_CHANNEL;
int Rx_Channel = NUM_RX_CHANNEL;
uint16_t baseline_node[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};

#if SET_DOWNLOAD_BY_GPIO
#include "silabs_bootloader.h"
#endif // SET_DOWNLOAD_BY_GPIO

enum
{
	TOUCH_SCREEN=0,
	TOUCH_KEY
};

struct key_info
{
	int key_value;
	int key_press;	
};

struct muti_touch_info
{
	int strength;
	int width;	
	int posX;
	int posY;
};

struct silabs_ts_data {
	uint16_t addr;
	struct i2c_client *client; 
	struct input_dev *input_dev;
    	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
    	struct work_struct  work_timer;
	struct early_suspend early_suspend;
};

struct silabs_ts_data *ts_global;

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR | S_IWGRP, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret, S_IRUGO | S_IWUSR | S_IWGRP, firmware_ret_show, firmware_ret_store);

static ssize_t rawdata_show_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_enable_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_disable_silabs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t baseline_show_silabs(struct device *dev, struct device_attribute *attr, char *buf1);
static ssize_t diff_show_silabs(struct device *dev, struct device_attribute *attr, char *buf);
//static ssize_t fimware_show_versname(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t rawdata_pass_fail_silabs(struct device *dev, struct device_attribute *attr, char *buf);
//static ssize_t read_node(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_node(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tkey_rawcounter_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tkey_rawcounter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(reference, S_IRUGO, baseline_show_silabs, NULL) ;
static DEVICE_ATTR(raw, S_IRUGO, rawdata_show_silabs, NULL) ;
static DEVICE_ATTR(raw_enable, S_IRUGO, raw_enable_silabs, NULL) ;
static DEVICE_ATTR(raw_disable, S_IRUGO, raw_disable_silabs, NULL) ;
static DEVICE_ATTR(diff, S_IRUGO, diff_show_silabs, NULL) ;
//static DEVICE_ATTR(versname, S_IRUGO, fimware_show_versname, NULL) ;
static DEVICE_ATTR(raw_value, S_IRUGO, rawdata_pass_fail_silabs, NULL) ;
//static DEVICE_ATTR(node_read, S_IRUGO, read_node, NULL) ;
static DEVICE_ATTR(node_read, S_IRUGO, show_node, NULL) ;
static DEVICE_ATTR(tkey_rawcounter, S_IRUGO | S_IWUSR | S_IWGRP, tkey_rawcounter_show, tkey_rawcounter_store);

/* sys fs */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void silabs_ts_early_suspend(struct early_suspend *h);
static void silabs_ts_late_resume(struct early_suspend *h);
#endif

extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);
extern int tsp_charger_type_status;
void TSP_forced_release_forkey(void);

static struct muti_touch_info g_Mtouch_info[SILABS_MAX_TOUCH];
static struct key_info touchkey_status[MAX_KEYS];

int touch_ctrl_regulator(int on_off)
{
#if defined(CONFIG_MACH_TOTORO_CTC)
        struct vreg *vreg_touch;
        int vreg_ret=0;

	vreg_touch = vreg_get(NULL, TSP_LDO);
		
	if(on_off==TOUCH_ON)
	{
		vreg_ret = vreg_set_level(vreg_touch, OUT2800mV);
		if (vreg_ret) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, vreg_ret);
		}
		
		vreg_ret = vreg_enable(vreg_touch);
		if (vreg_ret)
			printk(KERN_ERR "%s: vreg enable failed (%d)\n", __func__, vreg_ret);
	}
	else
	{
		vreg_ret = vreg_disable(vreg_touch);
		if (vreg_ret)
			printk(KERN_ERR "%s: vreg disable failed (%d)\n", __func__, vreg_ret);
	}
	return vreg_ret;
#else
	if(on_off==TOUCH_ON)
	{
			regulator_set_voltage(touch_regulator,2900000,2900000);
			regulator_enable(touch_regulator);
	}
	else
	{
			regulator_disable(touch_regulator);
	}
#endif
}
EXPORT_SYMBOL(touch_ctrl_regulator);
int tsp_reset( void )
{
	int ret=1;
	int vreg_ret=0;

      if(touch_reset == 0) // prevent reset duplication
		return ret;
	  	
	printk("[TSP] %s+\n", __func__ );

#if defined(CONFIG_MACH_TOTORO_CTC)
	touch_reset = 0;
#endif
	vreg_ret = touch_ctrl_regulator(0);

	if (vreg_ret) {
		printk(KERN_ERR "%s: vreg disable failed (%d)\n", __func__, vreg_ret);
		goto tsp_reset_out;
	}

	gpio_direction_output(30, 0);
      	gpio_direction_output(23, 0);
	gpio_direction_output(19, 0);
            
	msleep(200);


	// for TSK

      TSP_forced_release_forkey();

	gpio_direction_output(30, 1);
      	gpio_direction_output(23, 1);
	gpio_direction_output(19, 1);

	gpio_direction_input(30);
      	gpio_direction_input(23);
	gpio_direction_input(19);

	touch_ctrl_regulator(1);
		
	msleep(200);
	
tsp_reset_out:
#if defined(CONFIG_MACH_TOTORO_CTC)
	touch_reset = 1;
#endif

	printk("[TSP] %s-\n", __func__ );

	return ret;
}


int tsp_i2c_write (unsigned char *rbuf, int num)
{
    int ret;
    ret = i2c_master_send(ts_global->client, rbuf, num);
    
       if(ret<0) {
		printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
	}

    return ret;
}
EXPORT_SYMBOL(tsp_i2c_write);

int tsp_i2c_read(unsigned char *rbuf, int len)
{
    int ret;
    
	ret = i2c_master_recv(ts_global->client, rbuf, len);

       if(ret<0) {
		printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
	}

       return ret;
}
EXPORT_SYMBOL(tsp_i2c_read);


void TSP_forced_release_forkey(void)
{
	int i, key;
	int temp_value=0;
    
	for(i=0; i<SILABS_MAX_TOUCH; i++)
	{
		if(g_Mtouch_info[i].strength== -1)
			continue;

		input_report_abs(ts_global->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0 );
		input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);      				
		input_mt_sync(ts_global->input_dev);   

		printk("[TSP] force release\n");

		if(g_Mtouch_info[i].strength == 0)
			g_Mtouch_info[i].strength = -1;

		temp_value++;
	}

	if(temp_value>0)
		input_sync(ts_global->input_dev);

    
	for(key = 0; key < MAX_KEYS ; key++)
	{
		touchkey_status[key].key_press = RELEASE_KEY;
		input_report_key(ts_global->input_dev, touchkey_status[key].key_value, touchkey_status[key].key_press);	
	}
	
}
EXPORT_SYMBOL(TSP_forced_release_forkey);

static irqreturn_t  silabs_ts_work_func(int irq, void *dev_id)
{
	int ret=0, i=0, key=0;
	uint8_t buf[TS_READ_REGS_LEN];
	int touch_num=0, button_num =0, button_status=0, button_check=0;

	struct silabs_ts_data *ts = dev_id;

#if 0 //def __TOUCH_DEBUG__
	printk("[TSP] %s, %d\n", __func__, __LINE__ );
#endif

	if(ts ==NULL)
	{
       printk("[TSP] silabs_ts_work_func : TS NULL\n");
	   return IRQ_HANDLED;
	}
	
	buf[0] = ESCAPE_ADDR;
	buf[1] = 0x02;

	for(i=0; i<I2C_RETRY_CNT; i++)
	{	
		ret = i2c_master_send(ts->client, buf, 2);

		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);

			if(ret >=0)
			{
				break; // i2c success
			}
		}
	}
        
	if (ret < 0)
	{
		printk("[TSP] silabs_ts_work_func: i2c failed %d\n", ret);
		tsp_reset();
		return IRQ_HANDLED;
	}
	else 
	{

		touch_num  = buf[0]&0x0F;
		button_num = ((buf[0]&0xC0)>>6);
		button_status=((buf[1]&0x10)>>4);
		button_check=buf[1]&0x0F;

#if 0 //def __TOUCH_DEBUG__ // for google security auth
		printk("[TSP] button_num : %d, touch_num : %d, button_check:%d, buf[1] : %d\n", button_num, touch_num, button_check, buf[1]);
#endif
     
        	if(button_check == 0)
		{
		   //printk("[TSP] status=0x%x, strength1=0x%x, strength2=0x%x\n", buf[0], buf[7], buf[12]);
                   if(touch_num >0) 
                   {
			  touch_id[0] = (buf[2]&0xf0)>>4;
			  posX[0] = (( buf[3]<< (8) ) +  buf[4]);
			  posY[0] = ( buf[5]<< (8) ) +  buf[6];

			  strength[0] = buf[7]; 

			  touch_id[1] = (buf[2]&0x0f);
			  posX[1] =  (( buf[8]<< (8) ) +  buf[9]);
			  posY[1] = ( buf[10]<< (8) ) +  buf[11];

			  strength[1] = buf[12]; 
                   }
                   
                    if(touch_num==0)
                    {
                          touch_id[0]=0;
                          touch_id[1]=0;
                          strength[0]=0;
                          strength[1]=0;
                    }

			for(i=0; i<2; i++)
			{
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_id[i]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, posX[i]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  posY[i]);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, touch_id[i] );
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, strength[i]);      				
				input_mt_sync(ts->input_dev); 

				g_Mtouch_info[i].posX = posX[i];
				g_Mtouch_info[i].posY = posY[i];
				g_Mtouch_info[i].width = strength[i];
				g_Mtouch_info[i].strength = strength[i];
#if 0 //def __TOUCH_DEBUG__ // for google security auth
				printk("[TSP] id : %d, x: %d, y: %d, str: %d\n", touch_id[i], posX[i], posY[i], strength[i]);
#endif			
			}
		}
            	else
		{   
			if (buf[1] & 0x1)
	                        key=0;
			if (buf[1] & 0x2)
	                        key=1; 
            
			touchkey_status[0].key_value= KEY_MENU;	
			touchkey_status[1].key_value= KEY_BACK;	
			touchkey_status[key].key_press=(button_status ? PRESS_KEY : RELEASE_KEY);
                   
            		input_report_key(ts->input_dev, touchkey_status[key].key_value, touchkey_status[key].key_press);
                    
#if 0 //def __TOUCH_DEBUG__
			printk(KERN_ERR "[TSP] silabs_ts_work_func: buf[%d] : %d, button_status: %d\n", key, touchkey_status[key].key_value, touchkey_status[key].key_press);
#else
			printk(KERN_ERR "[TSP] button : %d, button_status: %d\n", key, touchkey_status[key].key_press);
#endif		
		}
   	input_sync(ts->input_dev);              
	}

    return IRQ_HANDLED;
}

static irqreturn_t silabs_ts_irq_handler(int irq, void *dev_id)
{
	struct silabs_ts_data *ts = dev_id;
	
#if defined(CONFIG_MACH_TOTORO_CTC)
	if((touch_present == 0) || (touch_resume == 0) || (touch_reset == 0)) // if initializing is fail or resume is working
		return IRQ_HANDLED;
#endif

	touch_check=1;

       #if 0 //def __TOUCH_DEBUG__
	printk("[TSP] %s, %d\n", __func__, __LINE__ );
       #endif
       
	return IRQ_WAKE_THREAD;
}

int set_tsp_for_ta_detect(int state)
{
	
	int ret=0;
	uint8_t wdog_val[7] = {0x80, 0x05, 0x00, 0x02, 0x12, 0x20, 0x00};
	
	if((tsp_status==0)&&(testmode==1)) // tsp active status
	{
		if(state)
		{
			printk("[TSP] [1] set_tsp_for_ta_detect!!! state=1\n");

			wdog_val[5] = 0x20;
			ret = i2c_master_send(ts_global->client, &wdog_val, 7);

			if(ret<0) {
				printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
			}

			pre_ta_stat = 1;
		}
		else
		{
			printk("[TSP] [2] set_tsp_for_ta_detect!!! state=0\n");

			wdog_val[5] = 0x00;
			ret = i2c_master_send(ts_global->client, &wdog_val, 7);

			if(ret<0) {
				printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
			}
			pre_ta_stat = 0;
		}
	}
} 
EXPORT_SYMBOL(set_tsp_for_ta_detect);

static void check_ic_work_func(struct work_struct *work_timer)
{
	int ret=0;
	uint8_t buf_esd[2];
	uint8_t i2c_addr = 0x1F;
	uint8_t wdog_val[1];

	struct silabs_ts_data *ts = container_of(work_timer, struct silabs_ts_data, work_timer);

	//printk("[TSP] %s, %d\n", __func__, __LINE__ );

    	buf_esd[0] = ESCAPE_ADDR;
	buf_esd[1] = TS_READ_ESD_ADDR;

	wdog_val[0] = 1;
	touch_check=0;

	if(testmode==1)
	{
		if( pre_ta_stat != tsp_charger_type_status )
		{
			set_tsp_for_ta_detect(tsp_charger_type_status);
		}

		ret = i2c_master_send(ts->client, &buf_esd, 2);

		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, wdog_val, 1);
		}
		
		if (touch_check==0)
		{
			if(ret < 0)
			{
				tsp_reset();
				printk(KERN_ERR "check_ic_work_func : i2c_master_send [%d]\n", ret);			
			}
			else if(wdog_val[0] == (uint8_t)prev_wdog_val)
			{
				printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
				tsp_reset();
				prev_wdog_val = -1;
			}
			else
			{
				prev_wdog_val = wdog_val[0];
			}
		}
	}
}

static enum hrtimer_restart silabs_watchdog_timer_func(struct hrtimer *timer)
{
	queue_work(check_ic_wq, &ts_global->work_timer);
	hrtimer_start(&ts_global->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static int silabs_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct silabs_ts_data *ts;
	int ret = 0; 
       int i=0;
#if defined(CONFIG_MACH_TOTORO_CTC)
       struct vreg *vreg_touch;
       int vreg_ret=0;

       vreg_touch = vreg_get(NULL, TSP_LDO);

       vreg_ret = vreg_set_level(vreg_touch, OUT2800mV);
       if (vreg_ret) {
           printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, ret);
           //return -EIO;
	}

	vreg_ret = vreg_enable(vreg_touch);
	if (vreg_ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		//return -EIO;
	}
	mdelay(200);
#else
	touch_ctrl_regulator(TOUCH_ON);
	mdelay(200);
#endif

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

    	INIT_WORK(&ts->work_timer, check_ic_work_func );
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	tsp_irq=client->irq;

#if 0
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = silabs_watchdog_timer_func;
#endif

	/* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
      if (device_create_file(firmware_dev, &dev_attr_raw) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_tkey_rawcounter) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_rawcounter.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_reference) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_reference.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_diff) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_diff.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_value) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_value.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_node_read) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_node_read.attr.name);
     if (device_create_file(firmware_dev, &dev_attr_raw_enable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_enable.attr.name);
     if (device_create_file(firmware_dev, &dev_attr_raw_disable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_disable.attr.name);
	/* sys fs */

        ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
      		touch_present = 0;
		printk(KERN_ERR "silabs_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "sec_touchscreen";
    
	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	
	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);		
	ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);	

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS,  ts->input_dev->evbit);
	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	set_bit(EV_SYN, ts->input_dev->evbit); 
	set_bit(EV_KEY, ts->input_dev->evbit);	
     
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
        	touch_present = 0;
		printk(KERN_ERR "silabs_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    	printk("[TSP] %s, irq=%d\n", __func__, tsp_irq );

#if defined(CONFIG_MACH_TOTORO_CTC)
	gpio_tlmm_config(GPIO_CFG(TSP_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#else
    	gpio_request(TSP_INT, "ts_irq");
	gpio_direction_input(TSP_INT);
	bcm_gpio_pull_up(TSP_INT, true);
	bcm_gpio_pull_up_down_enable(TSP_INT, true);
	set_irq_type(GPIO_TO_IRQ(TSP_INT), IRQF_TRIGGER_FALLING);
#endif
    

    if (tsp_irq) {
		ret = request_threaded_irq(tsp_irq, silabs_ts_irq_handler,  silabs_ts_work_func, IRQF_TRIGGER_FALLING|IRQF_ONESHOT, client->name, ts);

		if (ret < 0)
			dev_err(&client->dev, "request_irq failed\n");
	}

#if 1
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = silabs_ts_early_suspend;
	ts->early_suspend.resume = silabs_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#endif

	if(ts ==NULL)
	{
       printk("[TSP] silabs_ts_init_read : TS NULL\n");
	   goto err_input_dev_alloc_failed;
	}

	touch_present = 1;

	buf_firmware[0] = ESCAPE_ADDR;
	buf_firmware[1] = TS_READ_VERSION_ADDR;

	for(i=0; i < 10; i++)
	{
		ret = i2c_master_send(ts->client, &buf_firmware, 2);
		if(ret < 0)
		{
			printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
		}

		ret = i2c_master_recv(ts->client, &buf_firmware, 3);
		if(ret < 0)
		{
			printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);
		}

		if(ret >= 0)
		{
			tsp_chheck = 1;
			break;
		}
		//else
			//goto err_tsp_i2c_failed;
	}
	printk("[TSP] silabs_ts_probe %d, %d, %d\n", buf_firmware[0], buf_firmware[1], buf_firmware[2]);

#if defined (CONFIG_MACH_TOTORO_CTC)
	TSP_FIRMWARE_ID =  buf_firmware[0];
#endif
	TSP_MODULE_ID =  buf_firmware[2];

#if 0 // SET_DOWNLOAD_BY_GPIO
#if defined (CONFIG_MACH_TOTORO_CTC)
	if ((( buf_firmware[2] == SMAC_MODULE_VER)  || ( buf_firmware[2] == SMAC_MODULE_VER_NEW)) && (buf_firmware[0] < FW_VER))
#else
	if ((( buf_firmware[2] == YTE_MODULE_VER)||( buf_firmware[2] == SMAC_MODULE_VER))&&(buf_firmware[0] < FW_VER))
#endif
	{ 
		//TSP_MODULE_ID =  buf_firmware[2];
		local_irq_disable();
		ret = Firmware_Download();	
		printk("[TSP] enable_irq : %d\n", __LINE__ );
		local_irq_enable();

		if(ret == 0)
		{
			printk(KERN_ERR "SET Download Fail - error code [%d]\n", ret);		
		}
#if defined(CONFIG_MACH_TOTORO_CTC)
		else
		{
			buf_firmware[0] = ESCAPE_ADDR;
			buf_firmware[1] = TS_READ_VERSION_ADDR;

			for(i=0; i < I2C_RETRY_CNT; i++)
			{
				ret = i2c_master_send(ts->client, &buf_firmware, 2);
				if(ret < 0)
				{
					printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
				}

				ret = i2c_master_recv(ts->client, &buf_firmware, 3);
				if(ret < 0)
				{
					printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);
				}

				if(ret >= 0)
				{
				        TSP_FIRMWARE_ID = buf_firmware[0];
					TSP_MODULE_ID = buf_firmware[2];
					break;
				}
			}
		}
#endif
	}	
#endif // SET_DOWNLOAD_BY_GPIO

	if(tsp_chheck == 1)
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = silabs_watchdog_timer_func;
		hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);
	}

	return 0;

err_input_register_device_failed:
	printk(KERN_ERR "silabs-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
//err_tsp_i2c_failed:
	//printk(KERN_ERR "silabs-ts: err_tsp_i2c failed\n");
err_input_dev_alloc_failed:
	printk(KERN_ERR "silabs-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "silabs-ts: err_alloc_data failed_\n");	
	//vreg_ret = vreg_disable(vreg_touch);
	//if (vreg_ret) {
		//printk(KERN_ERR "%s: vreg disable failed (%d)\n", __func__, ret);
	//}
	return 1;
    }

static int silabs_ts_remove(struct i2c_client *client)
{
	struct silabs_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (tsp_irq)
		free_irq(tsp_irq, ts);
	//else
	//	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int silabs_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    	int ret;
	struct silabs_ts_data *ts = i2c_get_clientdata(client);

	DBG("[TSP] %s, %d\n", __func__, __LINE__ );
        tsp_status=1;
	if( touch_present )
	{
		if (tsp_irq)
		{
			disable_irq(tsp_irq);
		}
		ret = cancel_work_sync(&ts->work_timer);

		ret = cancel_work_sync(&ts->work);
		if (ret && tsp_irq) /* if work was pending disable-count is now 2 */
		{
			enable_irq(tsp_irq);
		}

		if(tsp_chheck==1)
			hrtimer_cancel(&ts->timer);

		touch_ctrl_regulator(TOUCH_OFF);

		gpio_direction_output(30, 0);
		gpio_direction_output(23, 0);
		gpio_direction_output(19, 0);

		msleep(400);    
	}
    	else
		DBG("[TSP] TSP isn't present.\n", __func__ );

        TSP_forced_release_forkey();

	DBG("[TSP] %s-\n", __func__ );
	return 0;
}

static int silabs_ts_resume(struct i2c_client *client)
{
	int retry_count, key;
	struct silabs_ts_data *ts = i2c_get_clientdata(client);

	DBG("[TSP] %s, %d\n", __func__, __LINE__ );

      //enable_irq(client->irq); // scl wave

	DBG("[TSP] %s+\n", __func__ );

#if defined(CONFIG_MACH_TOTORO_CTC)
	touch_resume = 0;
#endif
	
	if( touch_present )
	{
		gpio_direction_output(30, 1);
		gpio_direction_output(23, 1);
		gpio_direction_output(19, 1);

		gpio_direction_input(30);
		gpio_direction_input(23);
		gpio_direction_input(19);

		touch_ctrl_regulator(TOUCH_ON);
		msleep(80);

	// for TSK
	for(key = 0; key < MAX_KEYS; key++)
		touchkey_status[key].key_press = RELEASE_KEY;
    
	prev_wdog_val = -1;

	if(tsp_charger_type_status == 1)
	{
		set_tsp_for_ta_detect(tsp_charger_type_status);
	}
    
    #if 0	
	if( tsp_proximity_irq_status == 1)
	{
		set_tsp_for_prox_enable(tsp_proximity_irq_status);
	}
    #endif

	if(tsp_chheck==1)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	enable_irq(tsp_irq);
	}
	else
		DBG("[TSP] TSP isn't present.\n", __func__ );

#if defined(CONFIG_MACH_TOTORO_CTC)
        touch_resume = 1;
#endif
		
	DBG("[TSP] %s-\n", __func__ );
        tsp_status=0; 
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void silabs_ts_early_suspend(struct early_suspend *h)
{
	struct silabs_ts_data *ts;
	ts = container_of(h, struct silabs_ts_data, early_suspend);
	silabs_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void silabs_ts_late_resume(struct early_suspend *h)
{
	struct silabs_ts_data *ts;
	ts = container_of(h, struct silabs_ts_data, early_suspend);
	silabs_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id silabs_ts_id[] = {
	{ SILABS_TS_NAME, 0 },
	{ }
};

static struct i2c_driver silabs_ts_driver = {
	.probe		= silabs_ts_probe,
	.remove		= silabs_ts_remove,
#if 1
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= silabs_ts_suspend,
	.resume		= silabs_ts_resume,
#endif
#endif
	.id_table	= silabs_ts_id,
	.driver = {
		.name	= SILABS_TS_NAME,
	},
};

static int __devinit silabs_ts_init(void)
{

    	check_ic_wq = create_singlethread_workqueue("check_ic_wq");	
	if (!check_ic_wq)
		return -ENOMEM;

#if !defined(CONFIG_MACH_TOTORO_CTC)
	touch_regulator = regulator_get(NULL,"touch_vcc");

	board_sysconfig(SYSCFG_TOUCH, SYSCFG_INIT);
#endif

	return i2c_add_driver(&silabs_ts_driver);
}

static void __exit silabs_ts_exit(void)
{
	if (touch_regulator) 
	{
       	 regulator_put(touch_regulator);
		 touch_regulator = NULL;
    	}
	
	i2c_del_driver(&silabs_ts_driver);

	if (check_ic_wq)
		destroy_workqueue(check_ic_wq);
}


/* Touch Reference ************************************************************/
static ssize_t raw_enable_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{

		DBG("[TSP] %s stop. line : %d, \n", __func__,__LINE__);

            testmode=0;

    return sprintf(buf, "1") ;
}

static ssize_t raw_disable_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	DBG("[TSP] %s stop. line : %d, \n", __func__,__LINE__);

	testmode=1;
#if defined (CONFIG_MACH_TOTORO_CTC)
	tsp_reset();
#else
	touch_ctrl_regulator(0);  
	mdelay(2);
	touch_ctrl_regulator(1);  
	mdelay(300);
#endif

	return sprintf(buf, "1") ;
}

int silabs_quicksense ( int address, int size, char* buff )
{
	uint8_t buf1[7]={0x78, 0x05, 0x00, 0, 0, 0, 0};
      int ret;
      
	buf1[3] = (address >> 8) & 0xFF; // Address High Byte
	buf1[4] = address & 0xFF; // Address Low Byte
	buf1[5] = size;

	ret = i2c_master_send(ts_global->client, buf1, 7);
	if (ret < 0)
	{
		DBG("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}
	
	ret = i2c_master_recv(ts_global->client, buff, size + QUICKSENSE_OVERHEAD);

	if (ret < 0)
	{
		DBG("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	return 0;
}


static ssize_t rawdata_pass_fail_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
	uint8_t buffer1[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
    uint8_t buffer2[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t RAWDATA_MAX[108] = {14350,14510,14560,14610,14710,14680,14780,14780,14440,13720,13990,14080,14150,14190,14190,14260,14250,13910,13650,13940,14040,14110,14150,14150,14210,14200,13870,13610,13890,14000,14080,14120,14120,14200,14170,13840,13570,13870,13960,14040,14080,14080,14160,14150,13820,13560,13830,13920,13990,14020,14020,14110,14120,13790,13540,13800,13900,13960,14000,14000,14090,14100,13770,13400,13760,13900,13980,14030,14040,14150,14200,14270,13380,13750,13890,13980,14020,14040,14140,14160,14050,13370,13730,13870,13960,14010,14010,14100,14130,13980,13440,13720,13860,13960,14000,14000,14100,14130,14180,14150,14560,14730,14830,14870,14870,14960,15000,15010};
	uint16_t RAWDATA_MIN[108] = {10610,10720,10770,10800,10870,10850,10930,10920,10680,10140,10340,10410,10460,10490,10480,10540,10530,10280,10090,10300,10370,10430,10460,10460,10500,10500,10250,10060,10270,10340,10410,10440,10440,10490,10470,10230,10030,10250,10320,10380,10400,10410,10460,10460,10210,10020,10220,10290,10340,10360,10360,10430,10440,10190,10010,10200,10270,10320,10350,10350,10410,10420,10180,9900,10170,10270,10330,10370,10380,10460,10500,10540,9890,10160,10270,10330,10360,10370,10450,10470,10390,9880,10150,10250,10320,10350,10360,10420,10440,10330,9930,10140,10250,10310,10350,10350,10420,10450,10480,10460,10760,10880,10960,10990,10990,11060,11090,11100};
	uint8_t buf_firmware_show[3];

    int i, j, ret;

    DBG("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	if(testmode==1) return sprintf(buf, "-1");   

	mdelay(300); 

	buffer1[0] = ESCAPE_ADDR;
	buffer1[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer1, 2);
	if (ret < 0)
	{
		DBG("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return sprintf(buf, "-1");
	}

	ret = i2c_master_recv(ts_global->client,buffer2, 1);

	if (ret < 0)
	{
		DBG("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return sprintf(buf, "-1");
	}

	//
	//	quicksense format for reading rawdata
	//
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_RX_CHANNEL*2, buffer2);
	if (ret != 0)
	{
		DBG("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return sprintf(buf, "-1");
	}

	for (i = 0; i < 1; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
        {
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer2[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			if( RAWDATA_MAX[i*Rx_Channel+j] < rawdata[i][j]) return sprintf(buf, "0"); // fail
			if( RAWDATA_MIN[i*Rx_Channel+j] > rawdata[i][j]) return sprintf(buf, "0"); // fail
		}
    }


	buf_firmware_show[0] = ESCAPE_ADDR;
	buf_firmware_show[1] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts_global->client, &buf_firmware_show, 2);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
	}

	ret = i2c_master_recv(ts_global->client, &buf_firmware_show, 3);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);			
	}
	DBG("[TSP] ver tsp=%x, HW=%x, SW=%x\n", buf_firmware_show[1], buf_firmware_show[2], buf_firmware_show[0]);

        if (buf_firmware_show[2] == SMAC_MODULE_VER_NEW)
	{
		PHONE_VER = FW_VER;
		sprintf(buf, "10%x%x%x\n", buf_firmware_show[2], buf_firmware_show[0], PHONE_VER);
		DBG("[TSP] %s\n", buf);

		return sprintf(buf, "%s", buf );
	}

	if(buf_firmware_show[0]!=PHONE_VER)
		return sprintf(buf, "0");

    return sprintf(buf, "1"); // success
 }

static ssize_t rawdata_show_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
      int  written_bytes = 0 ;  /* & error check */
	uint8_t buffer1[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
    	uint8_t buffer2[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_rawdata[NUM_MTRBUTTONS]={0,};
	uint16_t baseline[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
      int i, j, ret;

      DBG("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	if(testmode==1) return 0;   

	mdelay(300); 

	buffer1[0] = ESCAPE_ADDR;
	buffer1[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer1, 2);
	if (ret < 0)
	{
		DBG("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer2, 1);

	if (ret < 0)
	{
		DBG("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	//
	//	quicksense format for reading baseline
	//
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer1);
	if (ret != 0)
	{
		DBG("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}	
	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
			baseline[i][j] = (buffer1[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer1[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
		}
	}

	//
	//	quicksense format for reading rawdata
	//
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer2);
	if (ret != 0)
	{
		DBG("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}
	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
        {
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer2[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			written_bytes += sprintf(buf+written_bytes, "%d %d\n", rawdata[i][j], baseline[i][j]-rawdata[i][j]) ;
        }
    }

    if (written_bytes > 0)
        return written_bytes ;

    return sprintf(buf, "-1") ;
 }

static ssize_t baseline_show_silabs(struct device *dev, struct device_attribute *attr, char *buf1)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
	int written_bytes = 0 ;  /* & error check */
	uint8_t buffer[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t baseline[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
    int i, j, ret;


	if(testmode==1) return 0;
    
    DBG("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

   mdelay(300); 

	//
	//	Entering JIG_MODE
	//
	buffer[0] = ESCAPE_ADDR;
	buffer[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer, 2);
	if (ret < 0)
	{
		DBG("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer, 1);

	if (ret < 0)
	{
		DBG("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	//
	//	quicksense format for reading baseline
	//
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer);
	if (ret != 0)
	{
		DBG("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}	

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
			baseline[i][j] = (buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			printk(" %5d", baseline[i][j]);
			
			written_bytes += sprintf(buf1+written_bytes, "%d\n", baseline[i][j]) ;
		}
		printk("\n");
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf1, "-1") ;
}

static ssize_t diff_show_silabs(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Tx_Channel = NUM_TX_CHANNEL;
	int Rx_Channel = NUM_RX_CHANNEL;
	uint8_t buffer1[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
    uint8_t buffer2[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_rawdata[NUM_MTRBUTTONS]={0,};
	uint16_t baseline[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
      int i, j, ret;

      DBG("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	if(testmode==1) return 0;   

	mdelay(300); 

	buffer1[0] = ESCAPE_ADDR;
	buffer1[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer1, 2);
	if (ret < 0)
	{
		DBG("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client, buffer1, 1);

	if (ret < 0)
	{
		DBG("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	//
	//	quicksense format for reading baseline
	//
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer1);
	if (ret != 0)
	{
		DBG("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}	

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
			baseline[i][j] = (buffer1[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer1[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			//printk(" %5d", baseline[i][j]);
		}
	}

	//
	//	quicksense format for reading rawdata
	//
	ret = silabs_quicksense(RAWDATA_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer2);
	if (ret != 0)
	{
		DBG("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
        {
			rawdata[i][j] = (buffer2[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer2[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			printk(" %5d", baseline[i][j]-rawdata[i][j]);
        }
	printk("\n");
    }

    return 0;
 }


static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
       uint8_t buf_firmware_show[3];
       int ret;
       
	DBG("[TSP] %s\n",__func__);

#if defined (CONFIG_MACH_TOTORO_CTC)
	if(TSP_FIRMWARE_ID >= 0x0A)
	{
		sprintf(buf, "F0%d F0%d M00%x\n", FW_VER, TSP_FIRMWARE_ID, TSP_MODULE_ID);
	}
	else
	{
		sprintf(buf, "F0%d F00%x M00%x\n", FW_VER, TSP_FIRMWARE_ID, TSP_MODULE_ID);
	}
#else
	buf_firmware_show[0] = ESCAPE_ADDR;
	buf_firmware_show[1] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts_global->client, &buf_firmware_show, 2);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
	}

	ret = i2c_master_recv(ts_global->client, &buf_firmware_show, 3);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);			
	}
	DBG("[TSP] ver tsp=%x, HW=%x, SW=%x\n", buf_firmware_show[1], buf_firmware_show[2], buf_firmware_show[0]);

    	sprintf(buf, "10%x0%x0%x\n", buf_firmware_show[1], buf_firmware_show[2], buf_firmware_show[0]);
       DBG("[TSP] %s\n", buf);
#endif
	
	return sprintf(buf, "%s", buf );
}

static ssize_t fimware_show_versname(struct device *dev, struct device_attribute *attr, char *buf)
{
       uint8_t buf_firmware_ver[3];
       int ret;
       
	DBG("[TSP] %s\n",__func__);

	buf_firmware_ver[0] = ESCAPE_ADDR;
	buf_firmware_ver[1] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts_global->client, &buf_firmware_ver, 2);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
	}

	ret = i2c_master_recv(ts_global->client, &buf_firmware_ver, 3);
	if(ret < 0)
	{
		printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);			
	}
	DBG("[TSP] ver tsp=%x, HW=%x, SW=%x\n", buf_firmware_ver[1], buf_firmware_ver[2], buf_firmware_ver[0]);

    	sprintf(buf, "%x\n", buf_firmware_ver[0]);
       DBG("[TSP] %s\n", buf);
       
	return sprintf(buf, "%s", buf );
}
static ssize_t read_node(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *after;


	int written_bytes = 0 ;  /* & error check */
	uint8_t buffer[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	uint16_t button_baseline[NUM_MTRBUTTONS]={0,};
       int i, j, ret;
       
	check_node = simple_strtoul(buf, &after, 10);	
	DBG(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

       mdelay(300); 
  
	//
	//	Entering JIG_MODE
	//
	buffer[0] = ESCAPE_ADDR;
	buffer[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer, 2);
	if (ret < 0)
	{
		DBG("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer, 1);

	if (ret < 0)
	{
		DBG("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	//
	//	quicksense format for reading baseline
	//
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}	

	for (i = 0; i < Tx_Channel; i++)
    {
		for(j = 0 ; j < Rx_Channel; j++)
	{
			baseline_node[i][j] = (buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			printk(" %5d", baseline_node[i][j]);
		}
		printk("\n");
	}


    for (j = 0; j < Rx_Channel; j++)
    {
    	for(i = 0 ; i < Tx_Channel; i++)
		{
			written_bytes += sprintf(buf+written_bytes, ",%d", baseline_node[i][j]) ;
    	}
	}

	 printk("[TSP] %s\n", buf);
	
	touch_ctrl_regulator(0);  
	mdelay(2);
	touch_ctrl_regulator(1);  
	mdelay(300);
	
	return written_bytes;

}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	DBG("[TSP] %s, %d\n", __func__, __LINE__);
	firmware_ret_val = -1;
	printk("[TSP] firmware_store  valuie : %d\n",value);
	if ( value == 1 )
	{
		printk("[TSP] Firmware update start!!\n" );

		firm_update( );
		return size;
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	DBG("[TSP] %s!\n", __func__);

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	DBG("[TSP] %s, operate nothing!\n", __func__);

	return size;
}

int firm_update( void )
{
#if defined (CONFIG_MACH_TOTORO_CTC)
        uint8_t buf_firmware[3];
	int i, ret;
#endif
	
	DBG("[TSP] %s, %d\n", __func__, __LINE__);

	disable_irq(tsp_irq);

       ret = cancel_work_sync(&ts_global->work_timer);

       ret = cancel_work_sync(&ts_global->work);
       
	hrtimer_cancel(&ts_global->timer);
       
	touch_ctrl_regulator(TOUCH_OFF);
       mdelay(200);      
	touch_ctrl_regulator(TOUCH_ON);
       mdelay(200);

       TSP_MODULE_ID =  buf_firmware[2];
	 local_irq_disable();
       firmware_ret_val = Firmware_Download();	
	local_irq_enable();

	msleep(1000);
	if( firmware_ret_val )
		DBG( "[TSP] %s success, %d\n", __func__, __LINE__);
	else	
		DBG("[TSP] %s fail, %d\n", __func__, __LINE__);

	local_irq_enable();

	enable_irq(tsp_irq);

hrtimer_start(&ts_global->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#if defined (CONFIG_MACH_TOTORO_CTC)
	buf_firmware[0] = ESCAPE_ADDR;
	buf_firmware[1] = TS_READ_VERSION_ADDR;

	for(i=0; i < I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts_global->client, &buf_firmware, 2);
		if(ret < 0)
		{
			printk(KERN_ERR "silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
		}

		ret = i2c_master_recv(ts_global->client, &buf_firmware, 3);
		if(ret < 0)
		{
			printk(KERN_ERR "silabs_ts_work_func : i2c_master_recv [%d]\n", ret);
		}

		if(ret >= 0)
		{
			TSP_FIRMWARE_ID = buf_firmware[0];
			TSP_MODULE_ID = buf_firmware[2];
			break;
		}
	}
#endif

	return 0;
} 

static ssize_t tkey_rawcounter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		//int written_bytes = 0 ;  /* & error check */
		uint8_t buffer[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
		//uint16_t rawdata[NUM_TX_CHANNEL][NUM_RX_CHANNEL]={{0,},};
		uint16_t button_rawdata[NUM_MTRBUTTONS]={0,};
		int i, j, ret;

		//            Entering JIG_MODE
                buffer[0] = ESCAPE_ADDR;
                buffer[1] = JIG_MODE_COMMAND;
                ret = i2c_master_send(ts_global->client, buffer, 2);
                if (ret < 0)
                {
                                printk("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
                                return -1;
                }
		ret = i2c_master_recv(ts_global->client,buffer, 1);

                //            quicksense format for reading baseline & rawdata of buttons
                ret = silabs_quicksense(I2CMAP_BUTTON_ADDRESS,NUM_MTRBUTTONS*4, buffer);
                if (ret != 0)
                {
                                printk("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
                                return -1;
                }              

                //            reading rawdata of buttons
                for (i = 0; i < NUM_MTRBUTTONS; i++)
                {
                                button_rawdata[i] = (buffer[i*4 + QUICKSENSE_OVERHEAD +1] <<8) + buffer[i*4+QUICKSENSE_OVERHEAD + 2];
                                //printk(" button[%d] = %5d\n", i, button_rawdata[i]);
                }

		return sprintf(buf, "%d %d", button_rawdata[0], button_rawdata[1]) ;
}

static ssize_t tkey_rawcounter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	if(value == 0)
	{
	        hrtimer_cancel(&ts_global->timer);
		tsp_reset();
		prev_wdog_val = -1;
		hrtimer_start(&ts_global->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	return size;
}

static ssize_t show_node(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int written_bytes = 0 ;  /* & error check */
	uint8_t buffer[NUM_TX_CHANNEL*NUM_RX_CHANNEL*2+QUICKSENSE_OVERHEAD]={0,};
	int i, j, ret;
	char report_buf[216];
	int k=0;
	//int z=0;
       
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	mdelay(300); 

	//
	//	Entering JIG_MODE
	//
	buffer[0] = ESCAPE_ADDR;
	buffer[1] = JIG_MODE_COMMAND;
	ret = i2c_master_send(ts_global->client, buffer, 2);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_send fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	ret = i2c_master_recv(ts_global->client,buffer, 1);
	if (ret < 0)
	{
		printk("[TSP] i2c_master_recv fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}

	//
	//	quicksense format for reading baseline
	//
	ret = silabs_quicksense(BASELINE_ADDRESS,NUM_TX_CHANNEL*NUM_RX_CHANNEL*2, buffer);
	if (ret != 0)
	{
		printk("[TSP] silabs_quicksense fail! %s : %d, \n", __func__,__LINE__);
		return -1;
	}	

	for (i = 0; i < Tx_Channel; i++)
	{
		for(j = 0 ; j < Rx_Channel; j++)
		{
#if 0
			written_bytes += sprintf(buf+written_bytes, "%x", buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1]) ;
			written_bytes += sprintf(buf+written_bytes, "%x", buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD]) ;
#else
			report_buf[k++] = buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1];
			report_buf[k++] = buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD];

			memcpy(buf, report_buf, 216);
#endif
				
#if 0 // for debug
			baseline_node[i][j] = (buffer[(i*Rx_Channel+j)*2 + QUICKSENSE_OVERHEAD -1] <<8) + buffer[(i*Rx_Channel+j)*2+QUICKSENSE_OVERHEAD];
			printk(" %5d", baseline_node[i][j]);
#else
                        //for(z=0; z < 216; z++)
				//printk(" raw_data[%d] = 0x%x\n", z, report_buf[z], 0);
#endif
		}
		//printk("\n");
	}

	tsp_reset();
	
	return 216; //written_bytes
}

module_init(silabs_ts_init);
module_exit(silabs_ts_exit);

MODULE_DESCRIPTION("silabs Touchscreen Driver");
MODULE_LICENSE("GPL");
