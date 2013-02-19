/*
 *  headset/ear-jack device detection driver.
 *
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/sec_jack.h>
//[[#ifdef FEATURE_SEC_JACK_ADC_TEST
#include <linux/hrtimer.h>
//#endif]]

//shkang_ef18
#define MODULE_NAME "sec_jack:"
#define MAX_ZONE_LIMIT		10
#define SEND_KEY_CHECK_TIME_MS	70		/* 50ms */
#define DET_CHECK_TIME_MS	200		/* 200ms */
#define WAKE_LOCK_TIME		(HZ * 5)	/* 5 sec */

#ifdef CONFIG_MACH_COOPER
#undef FEATURE_HSSD
#endif

#define DEBUG  0
#if DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do {} while (0)
#endif

#define SUPPORT_PBA

#ifdef SUPPORT_PBA
struct class *jack_class;
EXPORT_SYMBOL(jack_class);

/* Sysfs device, this is used for communication with Cal App. */
static struct device *jack_selector_fs;     
EXPORT_SYMBOL(jack_selector_fs);
#endif

struct sec_jack_info {
	struct sec_jack_platform_data *pdata;
	struct delayed_work jack_detect_work;
	struct input_dev *input;
	struct wake_lock det_wake_lock;
	struct sec_jack_zone *zone;

	bool send_key_pressed;
	bool send_key_irq_enabled;
	unsigned int cur_jack_type;
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	bool volume_up_key_pressed;
	bool volume_down_key_pressed;
	unsigned int sec_jack_pole_type;
	bool check_jack_key;
	struct hrtimer ear_key_timer;
	struct work_struct  work_timer;
#endif
};

#ifdef FEATURE_SEC_JACK_VOLUME_KEY
struct sec_jack_info *hi_global;
static struct workqueue_struct *check_ear_key_wq;
static enum hrtimer_restart ear_key_timer_func(struct hrtimer *timer);
static void check_ear_key_work_func(struct work_struct *work_timer);
#endif

/* sysfs name HeadsetObserver.java looks for to track headset state
 */
struct switch_dev switch_jack_detection = {
	.name = "h2w",
};

/* To support samsung factory test */
struct switch_dev switch_sendend = {
	.name = "sec_earbutton",
};


#ifdef FEATURE_SEC_JACK_VOLUME_KEY
static void set_ear_key_state(struct sec_jack_info *hi, int key_value, int state)
{
	pr_info("%s :  key_value:: %d state::%d \n", __func__,key_value,state);

	// volume send_end key
	if( key_value == EAR_SEND_END_KEY )
	{
		input_report_key(hi->input, KEY_MEDIA, state);
		input_sync(hi->input);
		switch_set_state(&switch_sendend, state);
		hi->send_key_pressed = state;
		hi->volume_up_key_pressed = !state;
		hi->volume_down_key_pressed = !state;
	}
	// volume up key
	else if( key_value == EAR_VOLUME_UP_KEY )
	{
		input_report_key(hi->input, KEY_VOLUMEUP, state);
		input_sync(hi->input);
		hi->send_key_pressed = !state;
		hi->volume_up_key_pressed = state;
		hi->volume_down_key_pressed = !state;
	}
	// volume down key
	else if( key_value == EAR_VOLUME_DOWN_KEY )
	{
		input_report_key(hi->input, KEY_VOLUMEDOWN, state);
		input_sync(hi->input);
		hi->send_key_pressed = !state;
		hi->volume_up_key_pressed = !state;
		hi->volume_down_key_pressed = state;
	}
	else
	{
		pr_info(":  send:: %d up::%d down::%d \n", hi->send_key_pressed,hi->volume_up_key_pressed,hi->volume_down_key_pressed);
		if(hi->send_key_pressed == true ){
			input_report_key(hi->input, KEY_MEDIA, state);
			input_sync(hi->input);
			switch_set_state(&switch_sendend, state);
		}
		if(hi->volume_up_key_pressed == true ){
			input_report_key(hi->input, KEY_VOLUMEUP, state);
			input_sync(hi->input);
		}
		if(hi->volume_down_key_pressed == true ){
			input_report_key(hi->input, KEY_VOLUMEDOWN, state);
			input_sync(hi->input);
		}
		
		hi->send_key_pressed = 0;//state;
		hi->volume_up_key_pressed = 0;//state;
		hi->volume_down_key_pressed = 0;//state;
	}
	
	return;

}

static void get_jack_pole_type(struct sec_jack_info *hi)
{
	int adc;
	adc = hi->pdata->get_adc_value();
	
	pr_info(MODULE_NAME "!!!! %s::: EAR connetc adc %d !!!! \n", __func__,adc);

#if 1 // #if 1 :: because EAR_1_35K, EAR_25K, EAR_DEFAULT is 4poly
	hi->sec_jack_pole_type = EAR_1_35K;
	pr_info(MODULE_NAME "!!!! %s::: EAR_1_35K connetc !!!! \n", __func__);
#else
	if( 1100 <= adc && adc <= 1200 )
	{
		hi->sec_jack_pole_type = EAR_1_35K;
		pr_info(MODULE_NAME "!!!! %s::: EAR_1_35K connetc !!!! \n", __func__);
	}
	else if( 3000 <= adc){
		hi->sec_jack_pole_type = EAR_25K;
		pr_info(MODULE_NAME "!!!! %s::: EAR_25K connetc !!!! \n", __func__);
	}
	else{
		hi->sec_jack_pole_type = EAR_DEFAULT;
		pr_info(MODULE_NAME "!!!! %s::: EAR_DEFAULT connetc !!!! \n", __func__);
	}
#endif

	return;
}
#else
static void set_send_key_state(struct sec_jack_info *hi, int state)
{
	pr_info("%s : state::%d  \n", __func__,state);

	input_report_key(hi->input, KEY_MEDIA, state);
	input_sync(hi->input);
	switch_set_state(&switch_sendend, state);
	hi->send_key_pressed = state;
}
#endif

static void sec_jack_set_type(struct sec_jack_info *hi, int jack_type)
{
	struct sec_jack_platform_data *pdata = hi->pdata;

	/* this can happen during slow inserts where we think we identified
	 * the type but then we get another interrupt and do it again
	 */

	pr_info(MODULE_NAME "%s::: cur_jack_type=0x%x jack_type=0x%x irq_enable=%d\n", __func__, hi->cur_jack_type,jack_type,hi->send_key_irq_enabled);
	
	if (jack_type == hi->cur_jack_type)
		return;

	if (jack_type == SEC_HEADSET_4POLE) {
		/* for a 4 pole headset, enable irq
		   for detecting send/end key presses */
		if (!hi->send_key_irq_enabled) {
			enable_irq(pdata->send_int);
			enable_irq_wake(pdata->send_int);
			hi->send_key_irq_enabled = 1;
		}
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
		get_jack_pole_type(hi);
#endif
		
	} else {
		/* for all other jacks, disable send/end irq */
		if (hi->send_key_irq_enabled) {
			disable_irq(pdata->send_int);
			disable_irq_wake(pdata->send_int);
			hi->send_key_irq_enabled = 0;
		}
		
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
		hi->sec_jack_pole_type = EAR_NORMAL;
		set_ear_key_state(hi, -1, 0);
		pr_info(MODULE_NAME "%s : BTN set released by jack switch to %d\n",
			__func__, jack_type);
#else
		if (hi->send_key_pressed) {
			set_send_key_state(hi, 0);
			pr_info(MODULE_NAME "%s : BTN set released by jack switch to %d\n",
				__func__, jack_type);
		}
#endif
	}

	/* micbias is left enabled for 4pole and disabled otherwise */
	pdata->set_micbias_state(hi->send_key_irq_enabled);
	/* because of ESD, micbias always remain high. for only 7x27 device*/
	//pdata->set_micbias_state(true); 


	hi->cur_jack_type = jack_type;
	pr_info(MODULE_NAME "%s : jack_type = %d\n", __func__, jack_type);

	/* prevent suspend to allow user space to respond to switch */
	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);

	switch_set_state(&switch_jack_detection, jack_type);
}

static void handle_jack_not_inserted(struct sec_jack_info *hi)
{
	pr_info(MODULE_NAME "%s ::: \n", __func__);
	sec_jack_set_type(hi, SEC_JACK_NO_DEVICE);
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	hi->sec_jack_pole_type = EAR_NORMAL;
#endif
	/* because of ESD, micbias always remain high. for only 7x27 device*/
	hi->pdata->set_micbias_state(false);
}

static void determine_jack_type(struct sec_jack_info *hi)
{
	struct sec_jack_zone *zones = hi->pdata->zones;
	int size = hi->pdata->num_zones;
	int count[MAX_ZONE_LIMIT] = {0};
	int adc;
	int i;

#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	while (hi->pdata->get_det_jack_state()) {
		adc = hi->pdata->get_adc_value();
		pr_debug(MODULE_NAME "adc = %d\n", adc);

		/* determine the type of headset based on the adc value.  
		 * I509(TOTORO_CTC) headset New Concept,
		 * ADC <= 1000 :: 3POLE
		 * 1050 <= ADC :: 4POLE
		*/
		for (i = 0; i < size; i++) {
			switch(i){
				case 0:
					// ADC <= 1000 :: 3POLE
					if ( (0 <= adc) && (adc <= zones[i].adc_high) ) 
					{
						if (++count[i] > zones[i].check_count) {
							pr_info(MODULE_NAME " %s ::: i::%d :: adc %d\n", __func__,i,hi->pdata->get_adc_value());
							sec_jack_set_type(hi, zones[i].jack_type);
								return;
						}
						msleep(zones[i].delay_ms);
						break;
					}
					break;

				case 1:
					// 1050 <= ADC :: 4POLE
					if(zones[i].adc_high <= adc){
						if (++count[i] > zones[i].check_count) {
							pr_info(MODULE_NAME "%s ::: i::%d :: adc %d\n", __func__,i,hi->pdata->get_adc_value());
							sec_jack_set_type(hi, zones[i].jack_type);
								return;
						}
						msleep(zones[i].delay_ms);
						break;
					}
					break;
			}
		}
	}
#else
	while (hi->pdata->get_det_jack_state()) {
		adc = hi->pdata->get_adc_value();
		pr_debug(MODULE_NAME "adc = %d\n", adc);

		/* determine the type of headset based on the
		 * adc value.  An adc value can fall in various
		 * ranges or zones.  Within some ranges, the type
		 * can be returned immediately.  Within others, the
		 * value is considered unstable and we need to sample
		 * a few more types (up to the limit determined by
		 * the range) before we return the type for that range.
		 */
		for (i = 0; i < size; i++) {
			if (adc <= zones[i].adc_high) {
				if (++count[i] > zones[i].check_count) {
					sec_jack_set_type(hi,
							  zones[i].jack_type);
						return;
				}
				msleep(zones[i].delay_ms);
				break;
			}
		}
	}
#endif
	/* jack removed before detection complete */
	handle_jack_not_inserted(hi);
}

/* thread run whenever the headset detect state changes (either insertion
 * or removal).
 */
static irqreturn_t sec_jack_detect_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = DET_CHECK_TIME_MS;

	//pr_info("%s : irq ~~~~~~  \n", __func__);

	/* debounce headset jack.  don't try to determine the type of
	 * headset until the detect state is true for a while.
	 */
	while (time_left_ms > 0) {
		if (!pdata->get_det_jack_state()) {
			/* jack not detected. */
			handle_jack_not_inserted(hi);
			return IRQ_HANDLED;
		}
		msleep(10);
		time_left_ms -= 10;
	}

	/* set mic bias to enable adc */
	pdata->set_micbias_state(true);
	/* jack presence was detected the whole time, figure out which type */
	determine_jack_type(hi);
	return IRQ_HANDLED;
}

#ifdef SUPPORT_PBA
static ssize_t select_jack_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);

	return 0;
}       
static ssize_t select_jack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	int value = 0;


	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);
	if (value == SEC_HEADSET_4POLE) {
		pdata->set_micbias_state(true);
		msleep(100);
	}

	sec_jack_set_type(hi, value);

	return size;
}
static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IWGRP , select_jack_show, select_jack_store);
#endif

#ifdef FEATURE_SEC_JACK_VOLUME_KEY
static int get_ear_key(struct sec_jack_info *hi)
{
	int return_val = -1;
	int adc = hi->pdata->get_adc_value();
	
	pr_err("%s ::: adc_value (%d) sec_jack_pole_type:%d \n",__func__,adc,hi->sec_jack_pole_type);
	
	switch (hi->sec_jack_pole_type){
		case EAR_NORMAL:
			break;
			
		case EAR_1_35K:
		case EAR_25K:
			if(0 <= adc && adc <= 150){
				return_val = EAR_SEND_END_KEY;
			}else if(151 <= adc && adc <= 341){
				return_val = EAR_VOLUME_UP_KEY;
			}else if(342 <= adc && adc <= 730){
				return_val = EAR_VOLUME_DOWN_KEY;
			}else
				pr_err("%s ::: get_adc_value (%d) :: NULL key !! \n","EAR_xxxK",adc);
			break;
			
		case EAR_DEFAULT:
			if(0 <= adc && adc <= 150){
				return_val = EAR_SEND_END_KEY;
			}else
				pr_err("%s ::: get_adc_value (%d) :: NULL key !! \n","EAR_DEFAULT",adc);

			break;
/*
		case EAR_25K:
			if(0 <= adc && adc <= 10){
				pr_err("%s ::: get_adc_value (%d) :: SEND_END key !! \n","EAR_25K",adc);
				return_val = EAR_SEND_END_KEY;
			}else if(10 <= adc && adc <= 22){
				pr_err("%s ::: get_adc_value (%d) :: +++++ key !! \n","EAR_25K",adc);
				return_val = EAR_VOLUME_UP_KEY;
			}else if(23 <= adc && adc <= 46){
				pr_err("%s ::: get_adc_value (%d) :: ----- key !! \n","EAR_25K",adc);
				return_val = EAR_VOLUME_DOWN_KEY;
			}else
				pr_err("%s ::: get_adc_value (%d) :: NULL key !! \n","EAR_25K",adc);

			break;
*/
	}

	return return_val;
}
#endif

/* thread run whenever the send/end key state changes. irq thread
 * handles don't need wake locks and since this one reports using
 * input_dev, input_dev guarantees that user space gets event
 * without needing a wake_lock.
 */
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
static irqreturn_t sec_jack_send_key_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	
	DBG("%s check_jack_key(%d)\n",__func__,hi->check_jack_key);

	if(hi->check_jack_key == true){
		DBG("%s return IRQ_HANDLED \n",__func__);
		return IRQ_HANDLED;
	}
	else{
		hi->check_jack_key = true;
		hrtimer_cancel(&hi->ear_key_timer);
		hrtimer_start(&hi->ear_key_timer, ktime_set(0, 300000000), HRTIMER_MODE_REL);
	}
	return IRQ_HANDLED;
}
#else
static irqreturn_t sec_jack_send_key_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = SEND_KEY_CHECK_TIME_MS;
	int send_key_state=0;
	//pr_err("%s ::: get_adc_value (%d) (0x%x)\n",__func__,pdata->get_adc_value(),pdata->get_adc_value());

    /* rukai1013.wei[issue:P110601-2439]. the mic bias was set to low when 
     * deal with the event led by touch "MUTE" of fm when the EP was output. 
     * and that led an send_end_key irq. 
     */
    if(pdata->get_send_key_state() == 1)
    {   
        pdata->set_micbias_state(true);
    }
    /* end of rukai1013.wei */
	
	/* debounce send/end key */
	while (time_left_ms > 0 && !hi->send_key_pressed) {
		send_key_state = pdata->get_send_key_state();
		
		if (!send_key_state || !pdata->get_det_jack_state() ||
		    hi->cur_jack_type != SEC_HEADSET_4POLE) {
			/* button released or jack removed or more
			 * strangely a non-4pole headset
			 */
			pr_info(MODULE_NAME "%s : ignored button (%d %d %d)\n", __func__,
				!send_key_state, !pdata->get_det_jack_state(),
				hi->cur_jack_type != SEC_HEADSET_4POLE );
			return IRQ_HANDLED;
		}

		msleep(10);
		time_left_ms -= 10;
	}

	/* report state change of the send_end_key */
	if (hi->send_key_pressed != send_key_state) {
		set_send_key_state(hi, send_key_state);
		pr_info(MODULE_NAME "%s : BTN is %s.\n",
			__func__, send_key_state ? "pressed" : "released");
	}

	return IRQ_HANDLED;
}
#endif

static int sec_jack_probe(struct platform_device *pdev)
{
	struct sec_jack_info *hi;
	struct sec_jack_platform_data *pdata = pdev->dev.platform_data;
	int ret;
#if defined(CONFIG_MACH_TOTORO_CTC) && (CONFIG_BOARD_REVISION >= 0x03)
    int time_left_ms = DET_CHECK_TIME_MS;
#endif

	pr_info(MODULE_NAME "%s : Registering jack driver\n", __func__);
	if (!pdata) {
		pr_err("%s : pdata is NULL.\n", __func__);
		return -ENODEV;
	}

	if (!pdata->get_adc_value || !pdata->get_det_jack_state	||
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	    !pdata->get_ear_key_state || !pdata->zones ||
#else
	    !pdata->get_send_key_state || !pdata->zones ||
#endif
	    !pdata->set_micbias_state || pdata->num_zones > MAX_ZONE_LIMIT) {
		pr_err("%s : need to check pdata\n", __func__);
		return -ENODEV;
	}

	hi = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (hi == NULL) {
		pr_err("%s : Failed to allocate memory.\n", __func__);
		return -ENOMEM;
	}

	hi->pdata = pdata;
#ifndef FEATURE_HSSD
	hi->input = input_allocate_device();
	if (hi->input == NULL) {
		ret = -ENOMEM;
		pr_err("%s : Failed to allocate input device.\n", __func__);
		goto err_request_input_dev;
	}
	hi->input->name = "sec_jack";
	input_set_capability(hi->input, EV_KEY, KEY_MEDIA);
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	input_set_capability(hi->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(hi->input, EV_KEY, KEY_VOLUMEDOWN);
#endif
	ret = input_register_device(hi->input);
	if (ret) {
		pr_err("%s : Failed to register driver\n", __func__);
		goto err_register_input_dev;
	}
#endif

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}

#ifndef FEATURE_HSSD
	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}
#endif

	wake_lock_init(&hi->det_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack_det");

#ifdef SUPPORT_PBA
	/* Create JACK Device file in Sysfs */
	jack_class = class_create(THIS_MODULE, "jack");
	if(IS_ERR(jack_class))
	{
		printk(KERN_ERR "Failed to create class(sec_jack)\n");
	}

	jack_selector_fs = device_create(jack_class, NULL, 0, hi, "jack_selector");
	if (IS_ERR(jack_selector_fs))
		printk(KERN_ERR "Failed to create device(sec_jack)!= %ld\n", IS_ERR(jack_selector_fs));

	if (device_create_file(jack_selector_fs, &dev_attr_select_jack) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_select_jack.attr.name);
#endif

#if defined(CONFIG_MACH_TOTORO_CTC) && (CONFIG_BOARD_REVISION >= 0x03)
   while (time_left_ms > 0) {
         if (pdata->get_det_jack_state()) {
			pdata->set_micbias_state(true);
            /* jack presence was detected the whole time, figure out which type */
            determine_jack_type(hi);
            break;
         }
         msleep(10);
         time_left_ms -= 10;
   }
#endif

	ret = request_threaded_irq(pdata->det_int, NULL,
				   sec_jack_detect_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT, "sec_headset_detect", hi);


	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);
		goto err_request_detect_irq;
	}

	/* to handle insert/removal when we're sleeping in a call */
	ret = enable_irq_wake(pdata->det_int);
	if (ret) {
		pr_err("%s : Failed to enable_irq_wake.\n", __func__);
		goto err_enable_irq_wake;
	}
#ifndef FEATURE_HSSD
	ret = request_threaded_irq(pdata->send_int, NULL,
				   sec_jack_send_key_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT,
				   "sec_headset_send_key", hi);
	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);

		goto err_request_send_key_irq;
	}

	/* start with send/end interrupt disable. we only enable it
	 * when we detect a 4 pole headset
	 */
	if (!pdata->get_det_jack_state())
		disable_irq(pdata->send_int);
#endif
	dev_set_drvdata(&pdev->dev, hi);

#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	INIT_WORK(&hi->work_timer, check_ear_key_work_func );
    hi_global = hi;
	hi->check_jack_key = false;
	hrtimer_init(&hi->ear_key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->ear_key_timer.function = ear_key_timer_func;
#endif
	return 0;

err_request_send_key_irq:
	disable_irq_wake(pdata->det_int);
err_enable_irq_wake:
	free_irq(pdata->det_int, hi);
err_request_detect_irq:
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_sendend);
err_switch_dev_register:
	input_unregister_device(hi->input);
	goto err_request_input_dev;
err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
	kfree(hi);

	return ret;
}

#ifdef FEATURE_SEC_JACK_VOLUME_KEY
static void check_ear_key_work_func(struct work_struct *work_timer)
{
	struct sec_jack_info *hi = hi_global;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int ear_key_state=0;
	int key_value;
	
	pr_err("%s ::: get_ear_key_state (%d)\n",__func__,pdata->get_ear_key_state());

    if(pdata->get_ear_key_state() == 1){
	    pdata->set_micbias_state(true);
	}
	
    //pdata->set_micbias_state(true);
	ear_key_state = pdata->get_ear_key_state();

	key_value = get_ear_key(hi);

	if(key_value == EAR_SEND_END_KEY){
		if(hi->send_key_pressed != ear_key_state){
			set_ear_key_state(hi, key_value, ear_key_state);
		}
	}
	else if(key_value == EAR_VOLUME_UP_KEY){
		if(hi->volume_up_key_pressed != ear_key_state){
			set_ear_key_state(hi, key_value, ear_key_state);
		}

	}
	else if(key_value == EAR_VOLUME_DOWN_KEY){
		if(hi->volume_down_key_pressed != ear_key_state){
			set_ear_key_state(hi, key_value, ear_key_state);
		}
	}else{
			set_ear_key_state(hi, key_value, ear_key_state);
	}

}

static enum hrtimer_restart ear_key_timer_func(struct hrtimer *timer)
{
	pr_info(MODULE_NAME "%s :\n", __func__);
	hi_global->check_jack_key = false;
	queue_work(check_ear_key_wq, &hi_global->work_timer);
	
	return HRTIMER_NORESTART;
}
#endif

static int sec_jack_remove(struct platform_device *pdev)
{

	struct sec_jack_info *hi = dev_get_drvdata(&pdev->dev);

	pr_info(MODULE_NAME "%s :\n", __func__);
	/* rebalance before free */
	if (hi->send_key_irq_enabled)
		disable_irq_wake(hi->pdata->send_int);
	else
		enable_irq(hi->pdata->send_int);
	free_irq(hi->pdata->send_int, hi);
	disable_irq_wake(hi->pdata->det_int);
	free_irq(hi->pdata->det_int, hi);
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_sendend);
	input_unregister_device(hi->input);
	kfree(hi);

	return 0;
}

static struct platform_driver sec_jack_driver = {
	.probe = sec_jack_probe,
	.remove = sec_jack_remove,
	.driver = {
			.name = "sec_jack",
			.owner = THIS_MODULE,
		   },
};
static int __init sec_jack_init(void)
{
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	check_ear_key_wq = create_singlethread_workqueue("check_ear_key_wq");	
	if (!check_ear_key_wq)
		return -ENOMEM;
#endif	
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
#ifdef FEATURE_SEC_JACK_VOLUME_KEY
	if (check_ear_key_wq)
		destroy_workqueue(check_ear_key_wq);
#endif
	platform_driver_unregister(&sec_jack_driver);
}

module_init(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("ms17.kim@samsung.com");
MODULE_DESCRIPTION("Samsung Electronics Corp Ear-Jack detection driver");
MODULE_LICENSE("GPL");
