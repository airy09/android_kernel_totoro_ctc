/*
 * =====================================================================================
 *
 *       Filename:  lcdc_ktd259_backlight.c
 *
 *    Description:  LCD Backlight control Function
 *
 *        Version:  1.0
 *        Created:  2010??11??25??15??21�?38�?
 *       Revision:  1.0
 *       Compiler:  arm-linux-gcc
 *
 *         Author:  File (System S/W Group Kernel Part), 
 *        Company:  Samsung Electronics
 *
 * =====================================================================================
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "lcdc_ktd259_backlight.h"
#include "msm_fb.h"

/* Panel Type */
#define PANEL_SHARP_VA		0x511810
#define PANEL_GP_TN			0x61ad10

#define DEBUG  0
#if DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do {} while (0)
#endif



extern unsigned int lcd_panel_type;
static int lcd_brightness = -1;
#if defined(CONFIG_MACH_COOPER_CTC)
struct brt_value brt_table_ktd[] = {
	  { 0,  5 }, // Off
	  { 20, 7 }, // Dimming pulse 26(33-7) by HW
	  { MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse 27(33-6) by HW
	  { 38,  2 }, 
	  { 45,  3 }, 
	  { 53,  3 }, 
	  { 60,  5 }, 
	  { 68,  6 }, 
	  { 75,  7 },  
	  { 83,  8 }, 
	  { 90,  9 }, 
	  { 98,  10 }, 
	  { 105,  11 }, 
	  { 113,  12 }, 
	  { 120,  13 }, 
	  { 128,  14 },
	  { 135,  15 },
	  { 145,  16 }, // default pulse 17(33-16) by HW
	  { 156,  17 },
	  { 167,  18 },
	  { 178,  19 }, 
	  { 189,  20 }, 
	  { 200,  21 }, 
	  { 211,  22 }, 
	  { 222,  23 }, 
	  { 233,  24  },
	  { 244,  25  },
	  { MAX_BRIGHTNESS_VALUE,  26 }, // Max pulse 7(33-26) by HW
};
#elif defined(CONFIG_MACH_COOPER)
struct brt_value brt_table_ktd[] = {
	  { 0,  5 }, // Off
	  { 20, 7 }, // Dimming pulse 26(33-7) by HW
	  { MIN_BRIGHTNESS_VALUE,  6 }, // Min pulse 27(33-6) by HW
	  { 41,  7 }, 
	  { 53,  8 },  
	  { 64,  9 }, 
	  { 76,  10 }, 
	  { 87,  11 }, 
	  { 99,  12 }, 
	  { 110,  13 },
	  { 122,  14 },
	  { 133,  15 },
	  { 145,  16 }, // default pulse 17(33-16) by HW
	  { 156,  17 },
	  { 167,  18 },
	  { 178,  19 }, 
	  { 189,  20 }, 
	  { 200,  21 }, 
	  { 211,  22 }, 
	  { 222,  23 }, 
	  { 233,  24  },
	  { 244,  25  },
	  { MAX_BRIGHTNESS_VALUE,  26 }, // Max pulse 7(33-26) by HW
};
#elif defined(CONFIG_MACH_BENI)
struct brt_value brt_table_ktd[] = {
		{ 0, 	1	}, // Off
		{ 20, 	1	}, // Dimming
		{ MIN_BRIGHTNESS_VALUE, 	1	}, // Min
		{ 40, 	2	}, 
		{ 50, 	3	}, 
		{ 60, 	4	}, 
		{ 70, 	5	}, 
		{ 80, 	6	}, 
		{ 90, 	7	}, 
		{ 100, 	8	}, 	
		{ 110, 	9	}, 
		{ 120, 	10	}, 
		{ 130, 	11	}, 
		{ 145, 	12	}, // default
		{ 155, 	13	}, 
		{ 165, 	14	}, 
		{ 175, 	15	}, 
		{ 185, 	16	}, 
		{ 195, 	17	},	
		{ 205, 	18	}, 
		{ 215, 	19	}, 
		{ 225, 	20	}, 
		{ 235,  21  },
		{ 245,  22  },
		{ MAX_BRIGHTNESS_VALUE, 	23	}, // Max
};
#elif defined(CONFIG_MACH_LUCAS)
struct brt_value brt_table_ktd[] = {
		{ 0, 	1	}, // Off
		{ 15, 	2	}, // Test value 1.
		{ 20, 	3	}, // Dimming
		{ 25,	4 	}, // Test value 2.
		{ MIN_BRIGHTNESS_VALUE, 	5	}, // Min
		{ 49, 	6	},
	    { 55,   6   },	
		{ 65, 	7	},
	    { 75,   7   },
		{ 81, 	8	},
	    { 90,   8   },	
		{ 97, 	9	},
	    { 105,  9   },	
		{ 113, 	10	},
	    { 120,  10  },	
		{ 129, 	11	}, 
		{ 145, 	12	}, // default
		{ 156, 	13	}, 
		{ 167, 	14	}, 
		{ 178, 	15	}, 
		{ 189, 	16	}, 
		{ 200, 	17	},	
		{ 211, 	18	}, 
		{ 222, 	19	}, 
		{ 233, 	20	}, 
		{ 244,  21  },
		{ MAX_BRIGHTNESS_VALUE,  22  }, // Max
};

struct brt_value brt_table_ktd_tn[] = {
		{ 0, 	1	}, // Off
		{ 20, 	1	}, // Dimming
		{ MIN_BRIGHTNESS_VALUE, 	1	}, // Min
		{ 38, 	2	}, 
		{ 40, 	3	}, 
		{ 48, 	4	}, 
		{ 58, 	5	}, 
		{ 68, 	6	}, 
		{ 78, 	7	}, 
		{ 90, 	8	}, 	
		{ 104, 	9	}, 
		{ 114, 	10	}, 
		{ 126, 	11	}, 
		{ 136, 	12	}, 
		{ 145, 	13	}, // default
		{ 154, 	14	}, 
		{ 162, 	15	}, 
		{ 171, 	16	}, 
		{ 179, 	17	},	
		{ 187, 	18	}, 
		{ 196, 	19	}, 
		{ 204, 	20	}, 
		{ 213,  21  },
		{ 222,  22  },
		{ 231,  23  }, 
		{ 240, 	24	}, 
		{ MAX_BRIGHTNESS_VALUE, 	25	}, // Max

};

#elif defined(CONFIG_MACH_TASS)
struct brt_value brt_table_ktd[] = {
#if 0
		{ 0, 	1	}, // Off
		{ 20, 	1	}, // Dimming
		{ MIN_BRIGHTNESS_VALUE, 	1	}, // Min
		{ 38, 	2	}, 
		{ 40, 	3	}, 
		{ 48, 	4	}, 
		{ 58, 	5	}, 
		{ 68, 	6	}, 
		{ 78, 	7	}, 
		{ 90, 	8	}, 	
		{ 104, 	9	}, 
		{ 114, 	10	}, 
		{ 126, 	11	}, 
		{ 136, 	12	}, 
		{ 145, 	13	}, // default
		{ 154, 	14	}, 
		{ 162, 	15	}, 
		{ 171, 	16	}, 
		{ 179, 	17	},	
		{ 187, 	18	}, 
		{ 196, 	19	}, 
		{ 204, 	20	}, 
		{ 213,  21  },
		{ 222,  22  },
		{ 231,  23  }, 
		{ 240, 	24	}, 
		{ MAX_BRIGHTNESS_VALUE, 	25	}, // Max
#else
		{ 0,          1      }, // off
		{ 20,         1      }, // dimming
		{ MIN_BRIGHTNESS_VALUE,         1      }, // min
		{ 31, 	2	},
		{ 45, 	3	}, 
		{ 59, 	4	}, 
		{ 73, 	5	}, 
		{ 87, 	6	}, 
		{ 101, 	7	}, 
		{ 115, 	8	}, 
		{ 129, 	9	}, 	
		{ 143, 	10	}, 
		{ 157, 	11	},  // default
		{ 171, 	12	}, 
		{ 185, 	13	}, 
		{ 199, 	14	},
		{ 213, 	15	}, 
		{ 227, 	16	}, 
		{ 241, 	17	}, 
		{ MAX_BRIGHTNESS_VALUE, 	18	}, // max
#endif
};
#else // Default value is based on TASS Setting
struct brt_value brt_table_ktd[] = {
		{ 0, 	1	}, // Off
		{ 20, 	3	}, // Dimming
		{ MIN_BRIGHTNESS_VALUE, 	5	}, // Min
		{ 39, 	6	}, 
		{ 56, 	7	}, 
		{ 72, 	8	}, 	
		{ 88, 	9	}, 
		{ 104, 	10	}, 
		{ 120, 	11	}, 
		{ 136, 	12	}, 
		{ 145, 	13	}, // default
		{ 154, 	14	}, 
		{ 162, 	15	}, 
		{ 171, 	16	}, 
		{ 179, 	17	},	
		{ 187, 	18	}, 
		{ 196, 	19	}, 
		{ 204, 	20	}, 
		{ 213,  21  },
		{ 222,  22  },
		{ 231,  23  }, 
		{ 240, 	24	}, 
		{ MAX_BRIGHTNESS_VALUE, 	25	}, // Max

};

#endif

#if defined(CONFIG_MACH_LUCAS)
struct brt_value brt_table_aat[] = {
		{ 255, 	9	}, // Max
		{ 244, 	10	}, 
		{ 233, 	11	}, 
		{ 222, 	12	}, 
		{ 211, 	13	}, 
		{ 200, 	14	}, 	
		{ 189, 	15	}, 
		{ 178, 	16	}, 
		{ 166, 	17	}, 
		{ 154, 	18	}, 
		{ 141, 	19	}, // Bennet Default	
		{ 127, 	20	}, 
		{ 113, 	21	}, 
		{ 99, 	22	}, 
		{ 85, 	23	},	
		{ 71, 	24	}, 
		{ 57, 	25	}, 
		{ 43, 	26	},  
		{ 30,   27  }, // Min
		{ 20, 	29	}, // Dimming
		{ 0, 	30	}, // Off
};
#else
struct brt_value brt_table_aat[] = {
		{ 255, 	9	}, // Max
		{ 240, 	10	}, 
		{ 230, 	11	}, 
		{ 220, 	12	}, 
		{ 210, 	13	}, 
		{ 200, 	14	}, 	
		{ 190, 	15	}, 
		{ 180, 	16	}, 
		{ 170, 	17	}, 
		{ 160, 	18	}, 
		{ 150, 	19	},	
		{ 141, 	20	}, // default
		{ 125, 	21	}, 
		{ 110, 	22	}, 
		{ 95, 	23	},	
		{ 80, 	24	}, 
		{ 65, 	25	}, 
		{ 50, 	26	},  
		{ 30,   27  }, // Min
		{ 20, 	29	}, // Dimming
		{ 0, 	30	}, // Off
};

#endif

#define MAX_BRT_STAGE_KTD (int)(sizeof(brt_table_ktd)/sizeof(struct brt_value))
#define MAX_BRT_STAGE_AAT (int)(sizeof(brt_table_aat)/sizeof(struct brt_value))

static DEFINE_SPINLOCK(bl_ctrl_lock);

void lcdc_s6d_set_brightness_by_aat1401(int level)
{
	int tune_level = 0;
	int i;

	spin_lock(&bl_ctrl_lock);
	if(level > 0) {
		if(level < MIN_BRIGHTNESS_VALUE) {
			tune_level = AAT_DIMMING_VALUE; //DIMMING
			} else {
				for(i = 0; i < MAX_BRT_STAGE_AAT; i++) {
					if(level <= brt_table_aat[i].level && level > brt_table_aat[i+1].level) {
						tune_level = brt_table_aat[i].tune_level;
						break;
					}
				}
			}
		} /*  BACKLIGHT is KTD model */

	if(!tune_level) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			mdelay(3);
	} else {
		for(;tune_level>0;tune_level--) 
		{
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(3);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3);
		}
		udelay(600); 		
	}

	spin_unlock(&bl_ctrl_lock);

}

void lcdc_s6d_set_brightness_by_ktd259(int level)
{
		int pulse;
		int tune_level = 0;
		int i;
		unsigned long flag;

#if defined(CONFIG_MACH_LUCAS) 
		static int selected_value = 0;

		if(!selected_value) {
			if(lcd_panel_type == PANEL_GP_TN) {
				for(i = 0; i < MAX_BRT_STAGE_KTD; i++) {
					brt_table_ktd[i] = brt_table_ktd_tn[i];
				}
			}
			selected_value = 1;
		}
#endif
		if(level > 0) {
			if(level < MIN_BRIGHTNESS_VALUE) {
				tune_level = KTD_DIMMING_VALUE; //DIMMING
			} else if (level == MAX_BRIGHTNESS_VALUE) {
				tune_level = brt_table_ktd[MAX_BRT_STAGE_KTD-1].tune_level;
			} else {
				for(i = 0; i < MAX_BRT_STAGE_KTD; i++) {
					if(level <= brt_table_ktd[i].level ) {
						tune_level = brt_table_ktd[i].tune_level;
						break;
					}
				}
			}
		}
		DBG("Platform V:%d, Find V:%d\n",level, tune_level);

		if(tune_level <= 0) {
				gpio_set_value(GPIO_BL_CTRL, 0);
				mdelay(3);
				lcd_brightness = tune_level;
				DBG("LCD Backlight OFF. tune:%d,lcd:%d\n",tune_level, lcd_brightness);
		} else {
				if(unlikely(lcd_brightness < 0)) {
						// first time
						int val = gpio_get_value(GPIO_BL_CTRL);
						if(val)
						{
								lcd_brightness = 0;
								gpio_set_value(GPIO_BL_CTRL, 0);
								mdelay(3); // guaranteed for shutdown
								DBG("LCD Baklight init - first boot time\n");
						}
				} 
				if(!lcd_brightness)
				{
						gpio_set_value(GPIO_BL_CTRL, 1);
						udelay(3);
						lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
						DBG("LCD Backlight re-init - wakeup time tune:%d, lcd:%d\n",tune_level,lcd_brightness);
				}
				pulse = (lcd_brightness - tune_level + MAX_BRIGHTNESS_IN_BLU) % MAX_BRIGHTNESS_IN_BLU;
	
				spin_lock_irqsave(&bl_ctrl_lock,flag);
				for(;pulse>0;pulse--) 
				{
						gpio_set_value(GPIO_BL_CTRL, 0);
						udelay(2);
						gpio_set_value(GPIO_BL_CTRL, 1);
						udelay(2);
				}
				spin_unlock_irqrestore(&bl_ctrl_lock,flag);
		}
		lcd_brightness = tune_level;

}

