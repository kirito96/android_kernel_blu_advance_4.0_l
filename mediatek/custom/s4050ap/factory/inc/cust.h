#ifndef FTM_CUST_H
#define FTM_CUST_H

#define FEATURE_FTM_MAIN_CAMERA
//#define FEATURE_FTM_MAIN2_CAMERA
#define FEATURE_FTM_SUB_CAMERA
#define FEATURE_FTM_TOUCH
#define FEATURE_FTM_KEYS
#define FEATURE_FTM_LCD
#define FEATURE_FTM_LCM
#define FEATURE_FTM_BT
#define FEATURE_FTM_GPS
#define FEATURE_FTM_FM
#if defined(MTK_WLAN_SUPPORT)
#define FEATURE_FTM_WIFI
#endif
#define FEATURE_FTM_TOUCH
#define FEATURE_FTM_KEYS
#ifdef MTK_EMMC_SUPPORT
#define FEATURE_FTM_EMMC
#define FEATURE_FTM_CLEAREMMC
#else
#define FEATURE_FTM_FLASH
#define FEATURE_FTM_CLEARFLASH
#endif

#define FEATURE_FTM_AUDIO
#define FEATURE_FTM_VIBRATOR
#define FEATURE_FTM_HEADSET
#define FEATURE_FTM_SPK_OC
#define FEATURE_FTM_WAVE_PLAYBACK

#define FEATURE_FTM_BATTERY
#define FEATURE_FTM_PMIC_632X
#define BATTERY_TYPE_Z3
#define FEATURE_FTM_LED
/*
//#define FEATURE_DUMMY_AUDIO

//#define FEATURE_FTM_FMTX
#define FEATURE_FTM_LED
*/
#define FEATURE_FTM_MEMCARD
//#define FEATURE_FTM_RTC
#define FEATURE_FTM_IDLE
#define FEATURE_FTM_BT
//#define FEATURE_FTM_TPUPGRADE
#define FTM_PCBA_ITEMS
#define FTM_ESSENTIAL_ITEMS
#define FEATURE_FTM_USB
/*
//#define FEATURE_FTM_CLEARFLASH
#define FEATURE_FTM_ACCDET

#define HEADSET_BUTTON_DETECTION
#ifdef HAVE_MATV_FEATURE
#define FEATURE_FTM_MATV
#endif
//#define FEATURE_FTM_FONT_10x18
//#define FEATURE_FTM_OTG
*/
#define HEADSET_BUTTON_DETECTION

//#define ALS_ENABLE//lxz add for 5200auto test

#define FEATURE_FTM_SIM

#define FEATURE_TP_LEFT_BACK //4050 tp swaped MENU and BACK key

#if !defined(CUSTOM_KERNEL_PS)
#define FEATURE_FTM_PS_CALIBRATION //yixuhong 20140522 add ps calibration
#endif

#define FEATURE_FTM_MSENSOR

#define FEATURE_FTM_REPORT_IN_VERSION

//Add SMT RF Test
#define FEATURE_FTM_RF

#include "cust_font.h"		/* common part */
#include "cust_keys.h"		/* custom part */
#include "cust_lcd.h"		/* custom part */
#include "cust_led.h"		/* custom part */
#include "cust_touch.h"         /* custom part */

#endif /* FTM_CUST_H */
