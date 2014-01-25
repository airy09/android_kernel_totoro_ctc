/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

//PGH TEST

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "sr200pc10.h"

#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>


#define SENSOR_DEBUG 0

// Gopeace LeeSangmin CAM GPIO define
#define GPIO_CAM_IO 3
#define GPIO_CAM_PWR 2

#define GPIO_CAM_MCLK 15

#define GPIO_CAM_RESET 0
#define GPIO_CAM_STBY 1

#define GPIO_CAM_SCL 60
#define GPIO_CAM_SDA 61

#if 1//PCAM ROUGH CODE
//#define CONFIG_LOAD_FILE 1

//kk0704.par TEMP :: #define SR200PC10_BURST_WRITE_LIST(A)	sr200pc10_sensor_burst_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#define SR200PC10_WRITE_LIST(A)			sr200pc10_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#define SR200PC10_WRITE_LIST2(A)			sr200pc10_sensor_write_list2(A,(sizeof(A) / sizeof(A[0])),#A);

struct samsung_short_t sr200pc10_init0_test[1500];

#endif

static char first_start_camera = 1;//  1 is not init a sensor

static char mEffect = 0;
static char mBrightness = 0;
static char mContrast = 0;
static char mSaturation = 0;
static char mSharpness = 0;
static char mWhiteBalance = 0;
static char mAutoExposure = 0;
static char mScene = 0;
static char mAfMode = 0;
static char mDTP = 0;
static char mInit = 0;
static char mMode = 0;


#define	MAX_RETRY_COUNT	3


struct sr200pc10_work {
	struct work_struct work;
};

static struct  sr200pc10_work *sr200pc10_sensorw;
static struct  i2c_client *sr200pc10_client;

struct sr200pc10_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct sr200pc10_ctrl *sr200pc10_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(sr200pc10_wait_queue);
DECLARE_MUTEX(sr200pc10_sem);
static int16_t sr200pc10_effect = CAMERA_EFFECT_OFF;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct sr200pc10_reg sr200pc10_regs;
extern int cpufreq_direct_set_policy(unsigned int cpu, const char *buf);
extern void pcam_msm_i2c_pwr_mgmt(struct i2c_adapter *adap, int on);
extern int* get_i2c_clock_addr( struct i2c_adapter *adap);
/*=============================================================*/


static int cam_hw_init(void);

#ifdef CONFIG_LOAD_FILE
//static int sr200pc10_regs_table_write(char *name);
int sr200pc10_regs_table_write(char *name);
int sr200pc10_regs_table_write2(char *name);
#endif


static int sr200pc10_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[1] = {0};
	struct i2c_msg msg = { sr200pc10_client->addr, 0, 1, buf };
	
	buf[0] = subaddr;
//	buf[1] = 0x0;

	ret = i2c_transfer(sr200pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(sr200pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

//	*data = ((buf[0] << 8) | buf[1]);
	*data = buf[0];

error:
	return ret;
}

static int sr200pc10_sensor_write(unsigned short subaddr, unsigned short val)
{
	int retry_count = 5;
	int err = 0;
	unsigned char buf[2] = {0};
	struct i2c_msg msg = { sr200pc10_client->addr, 0, 2, buf };

	//printk("[PGH] on write func subaddr: 0x%x, val:0x%x\n", subaddr, val);

	buf[0] = subaddr;
	buf[1] = val;

	while( retry_count--)
	{
		err = i2c_transfer(sr200pc10_client->adapter, &msg, 1);

		if( likely( err == 1))
			break;
		msleep(1);
	}

	return (err == 1) == 1 ? 0 : -EIO;
}



static int sr200pc10_sensor_write_list2(struct samsung_short_t *list,int size, char *name)
{
	int ret = 0;

	printk("[sr200pc10] sr200pc10_sensor_write_list2 : %s\n", name);

#ifdef CONFIG_LOAD_FILE
	ret = sr200pc10_regs_table_write2(name);
#endif
}


static int sr200pc10_sensor_write_list(struct samsung_short_t *list,int size, char *name)
{

	int ret = 0;
#ifdef CONFIG_LOAD_FILE
	ret = sr200pc10_regs_table_write(name);
#else
	int i;

	for (i = 0; i < size; i++)
	{
//kk0704.park TEMP ::		printk("[HAPPYROOT] %x      %x\n", list[i].subaddr, list[i].value);

		if(list[i].subaddr == 0xff)
		{
			printk("<=HAPPYROOT=> now %d ms SLEEP!!!!\n", list[i].value*10);
			msleep(list[i].value*10);
		}
		else
		{
		    if(sr200pc10_sensor_write(list[i].subaddr, list[i].value) < 0)
		    {
			    printk("<=PCAM=> sensor_write_list fail...-_-\n");
			    return -1;
		    }
		}
	}
#endif
	return ret;
}



#if 1//PCAM REMOVE OR FIX ME ROUGH CODE

void static night_shot_control(int flag)
{
	int Exptime;
	int Expmax;
	unsigned short read_1, read_2, read_3;	

	//PCAM_DEBUG("night_shot_control start\n");
	
	if(flag == 1) //ON
	{
	    PCAM_DEBUG("PCAM_NIGHT_SHOT ON");

	    sr200pc10_sensor_write(0x03, 0x20);
	    sr200pc10_sensor_write(0x10, 0x1C); //AE off for 50Hz

	    sr200pc10_sensor_read(0x80, &read_1);
	    sr200pc10_sensor_read(0x81, &read_2);
	    sr200pc10_sensor_read(0x82, &read_3);

	    Exptime = ((read_1 << 16) | (read_2 << 8) | (read_3));
	    PCAM_DEBUG(" read first : %d %d %d\n", read_1, read_2, read_3);

	    sr200pc10_sensor_read(0x88, &read_1);
	    sr200pc10_sensor_read(0x89, &read_2);
	    sr200pc10_sensor_read(0x8A, &read_3);

	    Expmax = ((read_1 << 16) | (read_2 << 8) | (read_3));

	    PCAM_DEBUG(" read second : %d %d %d\n", read_1, read_2, read_3);
	    PCAM_DEBUG("Night Shot, Exptime : %d, Expmax : %d \n", Exptime, Expmax);

	    if(Exptime < Expmax)
	    {
//		printk("<=PCAM=> night 1 set\n");
			SR200PC10_WRITE_LIST(sr200pc_cfg_scene_nightshot1);		
	    }
	    else
	    {
//		printk("<=PCAM=> night 2 set\n");
			SR200PC10_WRITE_LIST(sr200pc_cfg_scene_nightshot2);		
	    }
	}
	else
	{
		PCAM_DEBUG("PCAM_NIGHT_SHOT OFF");

		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal); 
		//msleep(200);
		//SR200PC10_WRITE_LIST(sr200pc10_preview_table); // preview start
	}
}

void sensor_effect_control(char value)
{

	//int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!
	//i2c_clk_addr = 0xd500c004;

	switch(value)
	{
		case PCAM_EFFECT_NORMAL :{
		PCAM_DEBUG("PCAM_EFFECT_NORMAL");
              SR200PC10_WRITE_LIST(sr200pc10_effect_normal);
		}
		break;		

		case PCAM_EFFECT_NEGATIVE :{
		PCAM_DEBUG("PCAM_EFFECT_NEGATIVE");
		SR200PC10_WRITE_LIST(sr200pc10_effect_negative);
		}
		break;	
		
		case PCAM_EFFECT_MONO :{
		PCAM_DEBUG("PCAM_EFFECT_MONO");
		SR200PC10_WRITE_LIST(sr200pc10_effect_mono);
		}
		break;	

		case PCAM_EFFECT_SEPIA :{
		PCAM_DEBUG("PCAM_EFFECT_SEPIA");
		SR200PC10_WRITE_LIST(sr200pc10_effect_sepia);
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected Effect mode : %d\n",  value);
		}
		break;
				
	}

}


void sensor_whitebalance_control(char value)
{
	switch(value)
	{
		case PCAM_WB_AUTO :{
		PCAM_DEBUG("PCAM_WB_AUTO");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_wb_auto);
		}
		break;	

		case PCAM_WB_DAYLIGHT :{
		PCAM_DEBUG("PCAM_WB_DAYLIGHT");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_wb_daylight);
		}
		break;	

		case PCAM_WB_CLOUDY :{
		PCAM_DEBUG("PCAM_WB_CLOUDY");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_wb_cloudy);
		}
		break;	

		case PCAM_WB_FLUORESCENT :{
		PCAM_DEBUG("PCAM_WB_FLUORESCENT");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_wb_fluorescent);
		}
		break;	
		
		case PCAM_WB_INCANDESCENT :{
		PCAM_DEBUG("PCAM_WB_INCANDESCENT");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_wb_incandescent);
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected WB mode : %d\n",  value);
		}
		break;
		
	}// end of switch

}


void sensor_brightness_control(char value)
{
	switch(value)
	{
	       case PCAM_BR_STEP_P_4 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_4");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_8);
		}
		break;
		
		case PCAM_BR_STEP_P_3 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_3");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_7);
		}
		break;

		case PCAM_BR_STEP_P_2 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_2");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_6);
		}
		break;

		case PCAM_BR_STEP_P_1 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_1");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_5);
		}
		break;

		case PCAM_BR_STEP_0 :{
		PCAM_DEBUG("PCAM_BR_STEP_0");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_4);
		}
		break;

		case PCAM_BR_STEP_M_1 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_1");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_3);
		}
		break;

		case PCAM_BR_STEP_M_2 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_2");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_2);
		}
		break;

		case PCAM_BR_STEP_M_3 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_3");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_1);
		}
		break;

              case PCAM_BR_STEP_M_4 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_4");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_brightness_0);
		}
		break;
		
		default :{
			printk("<=PCAM=> Unexpected BR mode : %d\n",  value);
		}
		break;

	}
	
}

/*
void sensor_iso_control(char value)
{
	switch(value)
	{
		case PCAM_ISO_AUTO :{
		PCAM_DEBUG("PCAM_ISO_AUTO");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_iso_auto);
		}
		break;

		case PCAM_ISO_100 :{
		PCAM_DEBUG("PCAM_ISO_100");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_iso_100);
		}
		break;

		case PCAM_ISO_200 :{
		PCAM_DEBUG("PCAM_ISO_200");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_iso_200);
		}
		break;

		case PCAM_ISO_400 :{
		PCAM_DEBUG("PCAM_ISO_400");
		SR200PC10_WRITE_LIST(sr200pc10_cfg_iso_400);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected ISO mode : %d\n",  value);
		}
		break;
		
	}

}
*/

void sensor_scene_control(char value)
{
	switch(value)
	{
		case PCAM_SCENE_OFF :
		{
			PCAM_DEBUG("PCAM_SCENE_OFF");
			night_shot_control(0);
			//SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);
		}
		break;
/*
		case PCAM_SCENE_PORTRAIT :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);
		PCAM_DEBUG("PCAM_SCENE_PORTRAIT");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_portrait);
		}
		break;

		case PCAM_SCENE_LANDSCAPE :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);
		PCAM_DEBUG("PCAM_SCENE_LANDSCAPE");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_landscape);
		}
		break;

		case PCAM_SCENE_SPORTS :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);
		PCAM_DEBUG("PCAM_SCENE_SPORTS");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_sports);
		}
		break;

		case PCAM_SCENE_PARTY :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);		
		PCAM_DEBUG("PCAM_SCENE_PARTY");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_party);			
		}
		break;

		case PCAM_SCENE_BEACH :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);		
		PCAM_DEBUG("PCAM_SCENE_BEACH");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_beach);
		}
		break;

		case PCAM_SCENE_SUNSET :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);	
		PCAM_DEBUG("PCAM_SCENE_SUNSET");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_sunset);
		}
		break;
		
		case PCAM_SCENE_DAWN :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);			
		PCAM_DEBUG("PCAM_SCENE_DAWN");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_dawn);	
		}
		break;

		case PCAM_SCENE_FALL :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);						
		PCAM_DEBUG("PCAM_SCENE_FALL");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_fall);		
		}
		break;
*/
		case PCAM_SCENE_NIGHTSHOT :
		{
			PCAM_DEBUG("PCAM_SCENE_NIGHTSHOT");

			if( PCAM_EFFECT_NORMAL != mEffect)
			{
				sensor_effect_control(PCAM_EFFECT_NORMAL);
			}
			if( PCAM_WB_AUTO != mWhiteBalance)
			{
				sensor_whitebalance_control(PCAM_WB_AUTO);
			}
			night_shot_control(1);
		}
		break;
/*
		case PCAM_SCENE_BACKLIGHT :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);	
		PCAM_DEBUG("PCAM_SCENE_BACKLIGHT");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_backlight);
		}
		break;

		case PCAM_SCENE_FIREWORK :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);		
		PCAM_DEBUG("PCAM_SCENE_FIREWORK");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_firework);				
		}
		break;

		case PCAM_SCENE_TEXT :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);					
		PCAM_DEBUG("PCAM_SCENE_TEXT");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_text);
		}
		break;

		case PCAM_SCENE_CANDLE :{
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_normal);							
		PCAM_DEBUG("PCAM_SCENE_CANDLE");
		SR200PC10_WRITE_LIST(sr200pc_cfg_scene_candle);	
		}
		break;
*/
		default :{
			printk("<=PCAM=> Unexpected SCENE mode : %d\n",  value);
		}
		break;				
	}
}


void sensor_contrast_control(char value)
{
/* kk0704.park TEMP
	switch(value)
	{
		case PCAM_CR_STEP_M_2 :{
		PCAM_DEBUG("PCAM_CR_STEP_M_2");
		SR200PC10_WRITE_LIST(sr200pc10_contrast_m_2);
		}
		break;

		case PCAM_CR_STEP_M_1 :{
		PCAM_DEBUG("PCAM_CR_STEP_M_1");
		SR200PC10_WRITE_LIST(sr200pc10_contrast_m_1);	
		}
		break;

		case PCAM_CR_STEP_0 :{
		PCAM_DEBUG("PCAM_CR_STEP_0");
		SR200PC10_WRITE_LIST(sr200pc10_contrast_0);
		}
		break;

		case PCAM_CR_STEP_P_1 :{
		PCAM_DEBUG("PCAM_CR_STEP_P_1");
		SR200PC10_WRITE_LIST(sr200pc10_contrast_p_1);
		}
		break;

		case PCAM_CR_STEP_P_2 :{
		PCAM_DEBUG("PCAM_CR_STEP_P_2");
		SR200PC10_WRITE_LIST(sr200pc10_contrast_p_2);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_CR_CONTROL mode : %d\n",  value);
		}
		break;												
	}
*/ //kk0704.park TEMP
}


void sensor_saturation_control(char value)
{
/* kk0704.park TEMP
	switch(value)
	{
		case PCAM_SA_STEP_M_2 :{
		PCAM_DEBUG("PCAM_SA_STEP_M_2");
		SR200PC10_WRITE_LIST(sr200pc10_saturation_m_2);
		}
		break;

		case PCAM_SA_STEP_M_1 :{
		PCAM_DEBUG("PCAM_SA_STEP_M_1");
		SR200PC10_WRITE_LIST(sr200pc10_saturation_m_1);		
		}
		break;

		case PCAM_SA_STEP_0 :{
		PCAM_DEBUG("PCAM_SA_STEP_0");
		SR200PC10_WRITE_LIST(sr200pc10_saturation_0);
		}
		break;

		case PCAM_SA_STEP_P_1 :{
		PCAM_DEBUG("PCAM_SA_STEP_P_1");
		SR200PC10_WRITE_LIST(sr200pc10_saturation_p_1);
		}
		break;

		case PCAM_SA_STEP_P_2 :{
		PCAM_DEBUG("PCAM_SA_STEP_P_2");
		SR200PC10_WRITE_LIST(sr200pc10_saturation_p_2);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_SA_CONTROL mode : %d\n",  value);
		}
		break;					
	}
*/ //kk0704.park TEMP
}


void sensor_sharpness_control(char value)
{
/* kk0704.park TEMP
	switch(value)
	{
		case PCAM_SP_STEP_M_2 :{
		PCAM_DEBUG("PCAM_SP_STEP_M_2");
		SR200PC10_WRITE_LIST(sr200pc10_sharpness_m_2);
		}
		break;

		case PCAM_SP_STEP_M_1 :{
		PCAM_DEBUG("PCAM_SP_STEP_M_1");
		SR200PC10_WRITE_LIST(sr200pc10_sharpness_m_1);		
		}
		break;

		case PCAM_SP_STEP_0 :{
		PCAM_DEBUG("PCAM_SP_STEP_0");
		SR200PC10_WRITE_LIST(sr200pc10_sharpness_0);	
		}
		break;

		case PCAM_SP_STEP_P_1 :{
		PCAM_DEBUG("PCAM_SP_STEP_P_1");
		SR200PC10_WRITE_LIST(sr200pc10_sharpness_p_1);			
		}
		break;

		case PCAM_SP_STEP_P_2 :{
		PCAM_DEBUG("PCAM_SP_STEP_P_2");
		SR200PC10_WRITE_LIST(sr200pc10_sharpness_p_2);			
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_SP_CONTROL mode : %d\n",  value);
		}
		break;					
	}
*/ //kk0704.park TEMP
}

void sensor_metering_control(char value)
{
	switch(value)
	{
		case PCAM_METERING_NORMAL :
		{
			PCAM_DEBUG("PCAM_METERING_NORMAL");
			SR200PC10_WRITE_LIST(sr200pc10_exposure_matrix);
		}
		break;
		
		case PCAM_METERING_SPOT :
		{
			PCAM_DEBUG("PCAM_METERING_SPOT");
			SR200PC10_WRITE_LIST(sr200pc10_exposure_spot);
		}
		break;

		case PCAM_METERING_CENTER :
		{
			PCAM_DEBUG("PCAM_METERING_CENTER");
			SR200PC10_WRITE_LIST(sr200pc10_exposure_centerweighted);
		}
		break;

		default :
		{
			printk("<=PCAM=> Unexpected METERING mode : %d\n",  value);
		}
		break;
	}
}

void sensor_DTP_control(char value)
{
	switch( value)
	{
		case PCAM_DTP_OFF:
		{
			PCAM_DEBUG("DTP OFF");
			mDTP = 0;
			SR200PC10_WRITE_LIST(sr200pc10_cfg_dtp_off);
		}
		break;

		case PCAM_DTP_ON:
		{
			PCAM_DEBUG("DTP ON");
			mDTP = 1;
			SR200PC10_WRITE_LIST(sr200pc10_cfg_dtp_on);
		}
		break;

		default:
		{
			printk("<=PCAM=> unexpected DTP control on PCAM\n");
		}
		break;
	}
}

void sensor_rough_control(void __user *arg)
{
	ioctl_pcam_info_8bit		ctrl_info;
                    
	PCAM_DEBUG("START");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail copy_from_user!\n", __func__);
	}
	printk("<=PCAM=>HAPPYROOT TEST %d %d %d %d %d \n", ctrl_info.mode, ctrl_info.address, ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);


	switch(ctrl_info.mode)
	{
		case PCAM_AUTO_TUNNING:
			break;

		
		case PCAM_SDCARD_DETECT:
			break;

		case PCAM_GET_INFO:
		{
		      unsigned short read_data1, read_data2,read_data3,iso;

		      //iso
		      sr200pc10_sensor_write(0x03, 0x20);	
                    sr200pc10_sensor_read(0xb0, (unsigned short*)&iso);

                    //exposure time
                    sr200pc10_sensor_write(0x03, 0x20);	
                    sr200pc10_sensor_read(0x80, (unsigned short*)&read_data1);
                    sr200pc10_sensor_read(0x81, (unsigned short*)&read_data2);
                    sr200pc10_sensor_read(0x82, (unsigned short*)&read_data3);
                    
                    ctrl_info.address= iso;		
                    ctrl_info.value_1 = read_data1;
                    ctrl_info.value_2 = read_data2;
                    ctrl_info.value_3 = read_data3;

                    PCAM_DEBUG("exposure %x %x %x , iso %d \n", read_data1, read_data2, read_data3, iso);
		}
		break;

		case PCAM_FRAME_CONTROL:
		{
			switch(ctrl_info.value_1)
			{

				case PCAM_FRAME_AUTO :{
				PCAM_DEBUG("PCAM_FRAME_AUTO");		
				}					
				break;
				
				case PCAM_FRAME_FIX_15 :{
				PCAM_DEBUG("PCAM_FRAME_FIX_15");
				SR200PC10_WRITE_LIST(sr200pc10_cfg_fps_15);		
				}
				break;

				default :{
					printk("<=PCAM=> Unexpected PCAM_FRAME_CONTROL mode : %d\n", ctrl_info.value_1);
				}
				break;				
			
			}
		}
		break;


		case PCAM_AF_CONTROL:
		{
			//kk0704.park TEMP
		}
		break;

		
		case PCAM_EFFECT_CONTROL:
		{
			mEffect = ctrl_info.value_1;
			sensor_effect_control(mEffect);
				
			
		}// end of PCAM_EFFECT_CONTROL
		break;


		case PCAM_WB_CONTROL:
		{
			mWhiteBalance = ctrl_info.value_1;
			sensor_whitebalance_control(mWhiteBalance);
			

		}//end of PCAM_WB_CONTROL
		break;


		case PCAM_BR_CONTROL:
		{
			mBrightness = ctrl_info.value_1;
			if(mInit)
				sensor_brightness_control(mBrightness);
			
		}//end of PCAM_BR_CONTROL
		break;
/*
		case PCAM_ISO_CONTROL:
		{
			mISO = ctrl_info.value_1;
			sensor_iso_control(mISO);
	
		}
		break;

*/
		case PCAM_METERING_CONTROL:
		{
			mAutoExposure = ctrl_info.value_1;
			sensor_metering_control(mAutoExposure);
		}
		break;
              
		case PCAM_SCENE_CONTROL:
		{
			mScene = ctrl_info.value_1;
			sensor_scene_control(mScene);
		}
		break;


		case PCAM_AWB_AE_CONTROL:
		{
			printk("<=PCAM=> PCAM_AWB_AE_CONTROL skip~~~\n");
		}
		break;

		case PCAM_CR_CONTROL:
		{
			mContrast = ctrl_info.value_1;
			if(mInit)
				sensor_contrast_control(mContrast);

		}
		break;
			

		case PCAM_SA_CONTROL:
		{
			mSaturation = ctrl_info.value_1;
			if(mInit)
				sensor_saturation_control(mSaturation);
			
		}
		break;

		

		case PCAM_SP_CONTROL:
		{
			mSharpness = ctrl_info.value_1;
			if(mInit)
				sensor_sharpness_control(mSharpness);
			
		}
		break;

		case PCAM_CPU_CONTROL:
		{

			switch(ctrl_info.value_1)
			{
				case PCAM_CPU_CONSERVATIVE:{
				PCAM_DEBUG("now conservative");
				cpufreq_direct_set_policy(0, "conservative");
				}
				break;

				case PCAM_CPU_ONDEMAND:{
				PCAM_DEBUG("now ondemand");
				cpufreq_direct_set_policy(0, "ondemand");
				}
				break;	

				case PCAM_CPU_PERFORMANCE:{
				PCAM_DEBUG("now performance");
				cpufreq_direct_set_policy(0, "performance");
				}
				break;
				
				default:{
					printk("<=PCAM=> unexpected CPU control on PCAM\n");
				}
				break;
			}
		}
		break;

              case PCAM_DTP_CONTROL:
              {
                  if(ctrl_info.value_1 == 0)
                      ctrl_info.value_3 = 2;
  
                  else if(ctrl_info.value_1 == 1)
                      ctrl_info.value_3 = 3;
  
                  sensor_DTP_control(ctrl_info.value_1);                
              }
              
		default :
			printk("<=PCAM=> Unexpected mode on sensor_rough_control!!! ctrl_info.mode = %d \n", ctrl_info.mode);
			break;
	}


	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail on copy_to_user!\n", __func__);
	}
	
}
#endif//PCAM


void cam_pw(int status)
{

	printk("<=PCAM=> cam_pw start\n");

	if(status == 1)
	{
		printk("[SR200PC10]Camera Sensor Power ON\n");

		gpio_set_value( GPIO_CAM_RESET, 0);

		gpio_tlmm_config(GPIO_CFG(GPIO_CAM_IO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
		gpio_set_value( GPIO_CAM_IO, 1); //CAM_IO 2.8V
#if defined (CONFIG_MACH_TOTORO_CTC)
		//msleep(1);
#else
		msleep(1);
#endif

		gpio_tlmm_config(GPIO_CFG(GPIO_CAM_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
		gpio_set_value( GPIO_CAM_PWR, 1); // CAM_A 2.8V , CAM_D 1.8V
		msleep(1);

		gpio_tlmm_config(GPIO_CFG(GPIO_CAM_STBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
		gpio_set_value( GPIO_CAM_STBY, 1); // CAM_STBY
		
//kk0704.park TEMP::		msleep(1);
		
	}
	else
	{
		printk("[SR200PC10]Camera Sensor Power OFF\n");

		gpio_set_value( GPIO_CAM_STBY, 0);
		udelay(1);
		gpio_set_value( GPIO_CAM_IO, 0);
		udelay(1);
		gpio_set_value( GPIO_CAM_PWR, 0);
		udelay(1);
		gpio_set_value( GPIO_CAM_RESET, 0);
		mdelay(1); // to enter a shutdown mode
	
	}

}

static int cam_hw_init()
{

	int rc = 0;
	unsigned short	id = 0; //PGH FOR TEST
	int retry = 0;

	PCAM_DEBUG("next cam_hw_init");

	//msm_camio_enable( dev);
	cam_pw(1); //kk0704.park
	
	/* Input MCLK = 24MHz */
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_rate_set(24000000);
	msm_camio_camif_pad_reg_reset();
	mdelay(10);

//kk0704.park TEMP ::  	gpio_tlmm_config(GPIO_CFG(1,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//kk0704.park TEMP ::  	gpio_set_value(1, 1); //CAM_STBY
//kk0704.park TEMP ::  	udelay(100);
	gpio_tlmm_config(GPIO_CFG( GPIO_CAM_RESET,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	gpio_set_value( GPIO_CAM_RESET, 1); //RESET UP
	mdelay(1);

#if defined (CONFIG_MACH_TOTORO_CTC)
    if (gpio_request(60, "i2c_pri_clk"))
        pr_err("failed to request gpio i2c_pri_clk\n");
    if (gpio_request(61, "i2c_pri_dat"))
        pr_err("failed to request gpio i2c_pri_dat\n");
#endif

	do
	{
		sr200pc10_sensor_read(0x04, &id);		
	} while((++retry < MAX_RETRY_COUNT) && (id == 0));

	if(id == 0x84)
		printk("<=PCAM=> RIGHT SENSOR FW => id 0x%x \n", id);
	else
	{
		printk("<=PCAM=> WRONG SENSOR FW => id 0x%x \n", id);
		rc = -1;
	}



	return rc;
}





static long sr200pc10_set_effect(int mode, int effect)
{
	long rc = 0;


	PCAM_DEBUG("mode : %d,   effect : %d", mode, effect);
	

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		PCAM_DEBUG("SENSOR_PREVIEW_MODE");
		break;

	case SENSOR_SNAPSHOT_MODE:
		PCAM_DEBUG("SENSOR_SNAPSHOT_MODE");
		break;

	default:
		printk("<=PCAM=> %s default\n", __func__);
		break;
	}

	switch (effect) {
	case CAMERA_EFFECT_OFF: {
	PCAM_DEBUG("CAMERA_EFFECT_OFF");
	SR200PC10_WRITE_LIST(sr200pc10_effect_normal); 

	}
			break;

	case CAMERA_EFFECT_MONO: {

	PCAM_DEBUG("CAMERA_EFFECT_MONO");
	SR200PC10_WRITE_LIST(sr200pc10_effect_mono); 

	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {

	PCAM_DEBUG("CAMERA_EFFECT_NEGATIVE");
	SR200PC10_WRITE_LIST(sr200pc10_effect_negative); 

	}
		break;

	case CAMERA_EFFECT_SOLARIZE: {

	printk("<=PCAM=> CAMERA_EFFECT_SOLARIZE NOT SUPPORT\n");

	}
		break;

	case CAMERA_EFFECT_SEPIA: {
	PCAM_DEBUG("CAMERA_EFFECT_SEPIA");
	SR200PC10_WRITE_LIST(sr200pc10_effect_sepia); 

	}
		break;

	default: {

	printk("<=PCAM=> unexpeceted effect  %s/%d\n", __func__, __LINE__);

		return -EINVAL;
	}
	}

	sr200pc10_effect = effect;

	return rc;
}

static long sr200pc10_set_sensor_mode(int mode)
{
	int *i2c_clk_addr;
	//i2c_clk_addr = 0xd580c004;
	i2c_clk_addr = get_i2c_clock_addr( sr200pc10_client->adapter);

	PCAM_DEBUG("START");

	if(first_start_camera)
	{
		pcam_msm_i2c_pwr_mgmt( sr200pc10_client->adapter, 1);
		printk( "[sr200pc10] i2c clk set forcely 0x316\n");
		writel( 0x316, i2c_clk_addr);
		printk( "[sr200pc10] re-check i2c clk : %x\n", readl( i2c_clk_addr));
		pcam_msm_i2c_pwr_mgmt( sr200pc10_client->adapter, 0);

#if 0//def CONFIG_LOAD_FILE
		SR200PC10_WRITE_LIST2(sr200pc10_init0);
#else
		SR200PC10_WRITE_LIST(sr200pc10_init0);
#endif

		PCAM_DEBUG("first_start_camera, sr200pc10_init0");
		first_start_camera = 0;
		mInit = 1;
	}

	switch (mode) 
	{
		case SENSOR_PREVIEW_MODE:
		{
			PCAM_DEBUG("Preview");

			SR200PC10_WRITE_LIST(sr200pc10_preview_table);

			if(PCAM_SCENE_NIGHTSHOT == mScene)
			{
				night_shot_control(1);
				PCAM_DEBUG("night preview start");
			}
			else
			{
				PCAM_DEBUG("normal preview start");
			}
		}
			break;

		case SENSOR_SNAPSHOT_MODE:
		{
			PCAM_DEBUG("Capture");				
			SR200PC10_WRITE_LIST(sr200pc10_capture_table);
		}
			break;

		case SENSOR_RAW_SNAPSHOT_MODE:
			PCAM_DEBUG("RAW_SNAPSHOT NOT SUPPORT!!");
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int sr200pc10_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	return rc;

}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>


static char *sr200pc10_regs_table = NULL;

static int sr200pc10_regs_table_size;

void sr200pc10_regs_table_init(void)
{
	struct file	*filp;
	char		*dp;
	long		l;
	loff_t		pos;
//	int		i;
	int		ret;

	mm_segment_t fs	= get_fs();

	set_fs(get_ds());

	filp = filp_open("/mnt/sdcard/sr200pc10.h", O_RDONLY, 0);

	if(IS_ERR(filp))
	{
		printk("<=PCAM=> file open error\n");
		return;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	printk("<=PCAM=> l = %ld\n", l);

	dp = kmalloc(l, GFP_KERNEL);
	if(dp == NULL)
	{
		printk("<=PCAM=> Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);

	if(ret != l)
	{
		printk("<=PCAM=> Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sr200pc10_regs_table		= dp;
	sr200pc10_regs_table_size	= l;

	*((sr200pc10_regs_table + sr200pc10_regs_table_size) - 1) = '\0';

//	printk("<=PCAM=> sr200pc10_regs_table 0x%x, %ld\n", (unsigned int)dp, l);

}

void sr200pc10_regs_table_exit(void)
{
	PCAM_DEBUG("START");

	if(sr200pc10_regs_table)
	{
		kfree(sr200pc10_regs_table);
		sr200pc10_regs_table = NULL;
	}

}


//static int sr200pc10_regs_table_write(char *name)
int sr200pc10_regs_table_write(char *name)
{
	char *start, *end, *reg;// *data;
	unsigned short addr = 0;
	unsigned short value = 0;
	char reg_buf[5], data_buf[5];

	*(reg_buf + 4) = '\0';
	*(data_buf + 4) = '\0';

	start = strstr(sr200pc10_regs_table, name);
	end = strstr(start, "};");

	while(1)
	{
		/* Find Address */
		reg = strstr(start, "{0x");
		if(reg)
			start = (reg + 12);
		if((reg == NULL) || (reg > end))
			break;

		/* Write Value to Address */
		if(reg != NULL)
		{
			memcpy(reg_buf, (reg+1), 4);
			memcpy(data_buf, (reg + 7), 4);
			
			PCAM_DEBUG("reg_buf : %s,  data_buf : %s\n", reg_buf, data_buf);

			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16);
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16);
			
			PCAM_DEBUG("addr : 0x%x,  value : 0x%x\n", addr, value);


			//Can Added Special Handle
				/* now NONE */


			if(addr == 0xff)
			{
				msleep(value*10);
			}
			else
			{
			    if( sr200pc10_sensor_write(addr, value) < 0)
			    {
				    printk("<=PCAM=> %s fail on sensor_write\n", __func__);
				    return -1;
			    }
			}
		}
		else
			printk("<=PCAM=> EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
	}
	return 0;
}

int sr200pc10_regs_table_write2(char *name)
{
	char *start, *end, *reg;// *data;

	int sr200pc10_count = 0;
	unsigned short addr = 0;
	unsigned short value = 0;
	char reg_buf[5], data_buf[5];

	int write_count = 0;
	unsigned short write_addr = 0;
	unsigned short write_value = 0;

	*(reg_buf + 4) = '\0';
	*(data_buf + 4) = '\0';

	start = strstr(sr200pc10_regs_table, name);

	end = strstr(start, "};");

	PCAM_DEBUG("address copy start\n");
	while(1)
	{
		/* Find Address */
		reg = strstr(start, "{0x");
		if(reg)
			start = (reg + 12);
		if((reg == NULL) || (reg > end))
			break;

		/* Write Value to Address */
		if(reg != NULL)
		{
			memcpy(reg_buf, (reg+1), 4);
			memcpy(data_buf, (reg + 7), 4);
			//PCAM_DEBUG("sr200pc10_regs_table_write2 , reg_buf : %s,  data_buf : %s\n", reg_buf, data_buf);
			
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16);
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16);
			//PCAM_DEBUG("sr200pc10_regs_table_write2 , addr : 0x%x,  value : 0x%x\n", addr, value);

			sr200pc10_init0_test[ sr200pc10_count].subaddr = addr;
			sr200pc10_init0_test[ sr200pc10_count].value = value;
			sr200pc10_count++;
		}
		else
			printk("<=PCAM=> EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
	}
	PCAM_DEBUG("address copy end\n");
	
	printk("<<<HAPPYROOT>>> count == %d \n", sr200pc10_count);

	while( write_count != sr200pc10_count)
	{
		write_addr = sr200pc10_init0_test[write_count].subaddr;
		write_value = sr200pc10_init0_test[write_count].value;

		PCAM_DEBUG("sr200pc10_regs_table_write2[count : %d] , write_addr : 0x%x,  write_value : 0x%x\n", write_count, write_addr, write_value);

		if(write_addr == 0xff)
		{
			msleep(write_value*10);
		}
		else
		{
			PCAM_DEBUG("i2c write start\n");
			if( sr200pc10_sensor_write(write_addr, write_value) < 0)
			{
				printk("<=PCAM=> %s fail on sensor_write\n", __func__);
				return -1;
			}
			PCAM_DEBUG("i2c write end\n");
		}
		write_count++;
	}

	//SR200PC10_WRITE_LIST( sr200pc10_init0_test);

	return 0;

}
#endif //CONFIG_LOAD_FILE

int tsp_cam_irq_status=0;
EXPORT_SYMBOL(tsp_cam_irq_status);

int sr200pc10_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	tsp_cam_irq_status = 1;

	sr200pc10_ctrl = kzalloc(sizeof(struct sr200pc10_ctrl), GFP_KERNEL);
	if (!sr200pc10_ctrl) {
		CDBG("sr200pc10_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		sr200pc10_ctrl->sensordata = data;

	rc = cam_hw_init();
	if (rc < 0) 
	{
		printk("<=PCAM=> cam_hw_init failed!\n");
		goto init_fail;
	}


#ifdef CONFIG_LOAD_FILE
	sr200pc10_regs_table_init();
#endif


	rc = sr200pc10_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("sr200pc10_sensor_init failed!\n");
		goto init_fail;
	}


init_done:
	return rc;

init_fail:
	kfree(sr200pc10_ctrl);
	return rc;
}

static int sr200pc10_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&sr200pc10_wait_queue);
	return 0;
}

int sr200pc10_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&sr200pc10_sem); */

	CDBG("sr200pc10_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = sr200pc10_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = 0;//sr200pc10_set_effect(cfg_data.mode, cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&sr200pc10_sem); */

	return rc;
}

int sr200pc10_sensor_release(void)
{
	int rc = 0;

	first_start_camera = 1;

	//If did not init below that, it can keep the previous status. it depend on concept by PCAM
	mEffect = 0;
	mBrightness = 0;
	mContrast = 0;
	mSaturation = 0;
	mSharpness = 0;
	mWhiteBalance = 0;
//	mISO = 0;
	mAutoExposure = 0;
	mScene = 0;
	//mAfMode = 0;
	mDTP = 0;
	mInit = 0;

	/* down(&sr200pc10_sem); */

	PCAM_DEBUG("sensor release");	

#if defined (CONFIG_MACH_TOTORO_CTC)
	//SR200PC10_WRITE_LIST(sr200pc_sleep_command); 
#else
	SR200PC10_WRITE_LIST(sr200pc_sleep_command); 
#endif

	kfree(sr200pc10_ctrl);
	/* up(&sr200pc10_sem); */

	tsp_cam_irq_status = 0;

#if defined (CONFIG_MACH_TOTORO_CTC)
	gpio_tlmm_config( GPIO_CFG( GPIO_CAM_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), GPIO_CFG_ENABLE); /* SCL */
	gpio_tlmm_config( GPIO_CFG( GPIO_CAM_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), GPIO_CFG_ENABLE); /* SDA */

	gpio_set_value( GPIO_CAM_SCL,0); //CAM_SCL
	gpio_set_value( GPIO_CAM_SDA,0); //CAM_SDA

	cam_pw( 0);
#endif

#ifdef CONFIG_LOAD_FILE
	sr200pc10_regs_table_exit();
#endif


	return rc;
}

static int sr200pc10_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	sr200pc10_sensorw =
		kzalloc(sizeof(struct sr200pc10_work), GFP_KERNEL);

	if (!sr200pc10_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, sr200pc10_sensorw);
	sr200pc10_init_client(client);
	sr200pc10_client = client;


	CDBG("sr200pc10_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(sr200pc10_sensorw);
	sr200pc10_sensorw = NULL;
	CDBG("sr200pc10_probe failed!\n");
	return rc;
}

static const struct i2c_device_id sr200pc10_i2c_id[] = {
	{ "sr200pc10", 0},
	{ },
};

static struct i2c_driver sr200pc10_i2c_driver = {
	.id_table = sr200pc10_i2c_id,
	.probe  = sr200pc10_i2c_probe,
	.remove = __exit_p(sr200pc10_i2c_remove),
	.driver = {
		.name = "sr200pc10",
	},
};


static int sr200pc10_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
#if defined (CONFIG_MACH_TOTORO_CTC)
	int rc = 0;

	gpio_tlmm_config( GPIO_CFG( GPIO_CAM_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE); /* MCLK */
	gpio_set_value( GPIO_CAM_MCLK, 0); //MCLK

	gpio_tlmm_config( GPIO_CFG( GPIO_CAM_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), GPIO_CFG_ENABLE); /* SCL */
	gpio_tlmm_config( GPIO_CFG( GPIO_CAM_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), GPIO_CFG_ENABLE); /* SDA */
	gpio_set_value( GPIO_CAM_SCL, 0); //CAM_SCL
	gpio_set_value( GPIO_CAM_SDA, 0); //CAM_SDA

	cam_pw( 1);
	msleep( 1);

	/* Input MCLK = 24MHz */
	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_rate_set( 24000000);

	gpio_tlmm_config( GPIO_CFG( GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE); /* MCLK */
	mdelay( 10);

	//msm_camio_camif_pad_reg_reset();

	gpio_tlmm_config(GPIO_CFG( GPIO_CAM_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	gpio_set_value( GPIO_CAM_RESET, 1); //RESET UP  	
	mdelay( 1);
#else
	int rc = i2c_add_driver(&sr200pc10_i2c_driver);
	if (rc < 0 || sr200pc10_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	cam_pw(1); //TEMP

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);
#endif

#if 0//bestiq
	rc = sr200pc10_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;
#endif

#if defined (CONFIG_MACH_TOTORO_CTC)
	rc = i2c_add_driver(&sr200pc10_i2c_driver);
	if (rc < 0 || sr200pc10_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");

	sr200pc10_sensor_write(0x03, 0x00);
	sr200pc10_sensor_write(0x01, 0xf8); //sleep off
	sr200pc10_sensor_write(0x0e, 0x73);

	mdelay( 10);
#endif

	gpio_set_value( GPIO_CAM_RESET, 0);//RESET
	gpio_set_value( GPIO_CAM_STBY, 0);//STBY
	cam_pw( 0); //TEMP

	s->s_init = sr200pc10_sensor_init;
	s->s_release = sr200pc10_sensor_release;
	s->s_config  = sr200pc10_sensor_config;
       s->s_mount_angle = 90; 
probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __sr200pc10_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, sr200pc10_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sr200pc10_probe,
	.driver = {
		.name = "msm_camera_sr200pc10",
		.owner = THIS_MODULE,
	},
};

static int __init sr200pc10_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(sr200pc10_init);
