#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
//#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_DBG_FUNC printk

#define DEBUG_CAMERA_HW_K
#ifdef  DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_XLOG_INFO(fmt, args...) \
        do {    \
                    xlog_printk(ANDROID_LOG_INFO, "kd_camera_hw", fmt, ##args); \
        } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif
static u32 subCamPDNStatus = 0;
static u32 mainCamPDNStatus = 0;


int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
    u32 pinSetIdx = 0;  //default main sensor
    u32 pinSetIdxTmp = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

    u32 pinSet[2][8] =
    {
        //for main sensor
        {
            GPIO_CAMERA_CMRST_PIN,
            GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
            GPIO_OUT_ONE,                   /* ON state */
            GPIO_OUT_ZERO,                  /* OFF state */
            GPIO_CAMERA_CMPDN_PIN,
            GPIO_CAMERA_CMPDN_PIN_M_GPIO,
            GPIO_OUT_ZERO,    //GPIO_OUT_ONE,
            GPIO_OUT_ONE, //GPIO_OUT_ZERO,
        },
        //for sub sensor
        {
            GPIO_CAMERA_CMRST1_PIN,
            GPIO_CAMERA_CMRST1_PIN_M_GPIO,
            GPIO_OUT_ONE,
            GPIO_OUT_ZERO,
            GPIO_CAMERA_CMPDN1_PIN,
            GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
            GPIO_OUT_ZERO, //GPIO_OUT_ONE,
            GPIO_OUT_ONE, //GPIO_OUT_ZERO,
        }
    };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
    {
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
    {
        pinSetIdx = 1;
    }

    if(pinSetIdx == 0 && ((0 == strcmp(SENSOR_DRVNAME_SP0A19_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_HI704_YUV,currSensorName))))
        return 0;
    if(pinSetIdx == 1 && ((0 == strcmp(SENSOR_DRVNAME_HI253_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_HI257_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_SP2519_YUV,currSensorName))))
        return 0;
    //power ON
    if (On)
    {

        PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
        PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);

        if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI257_YUV,currSensorName)))
        {
		   PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_HI257_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);

			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio 				mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio 				failed!! \n");}
       	    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA 				SENSOR] set gpio mode failed!! \n");}
        	if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed\n");}
        	if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio 				failed!! \n");}
        	mdelay(2);
					
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");//2.8 ->1.8
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
		   
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)) //LINE: <VOL_1500> <20130704> <modify the DVDD> panzaoyan
			{
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
			}	   

	   
		   	mdelay(2);

        	//enable active sensor
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio 				mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio 				failed!! \n");}
				PK_DBG("kdCISModulePowerOn enable active sensor set GPIO_CAMERA_CMPDN_PIN(GPIO%d) on\n",pinSet[pinSetIdx][IDX_PS_CMPDN]);

				mdelay(10);
				
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA 				SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed\n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio 				failed!! \n");}
				mdelay(2);
				PK_DBG("kdCISModulePowerOn enable active sensor set GPIO_CAMERA_CMRST_PIN(GPIO%d) on\n",pinSet[pinSetIdx][IDX_PS_CMRST]);
            	    //PDN pin

        	}  
		   
		   //disable inactive sensor
		   if(pinSetIdx == 0) {//disable sub
			   pinSetIdxTmp = 1;
		   }
		   else{
			   pinSetIdxTmp = 0;
		   }
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} 
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} 

			PK_DBG("kdCISModulePowerOn disable inactive sensor --> set CMRST(GPIO%d) and CMPDN(GPIO%d) off\n",
				pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMPDN]);
		   }
	   }	   
           else if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_SP0A19_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_SP2519_YUV,currSensorName))))
           {  
	      PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_SP0A19_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);

           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
           if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
           if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
           msleep(10);
			 
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }

          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }

          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
          /*
          if(mainCamAFPowerFlag==1) {
			   if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			   {
				   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				   //return -EIO;
				   goto _kdCISModulePowerOn_exit_;
			   }
			   mainCamAFPowerFlag = 0;
          }*/
          /*if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }*/

		  msleep(10);	
          //PDN/STBY pin
          if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
          {
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(5);
       
             //RST pin
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(5);
          }	   
             //disable inactive sensor
             if(pinSetIdx == 0) {//disable sub
                 pinSetIdxTmp = 1;
             }
             else{
                 pinSetIdxTmp = 0;
             }
             if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
               	if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA 		    SENSOR] set gpio mode failed!! \n");}
             	if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio 			    mode failed!! \n");}
             	if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             	if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             	if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio 			    failed!! \n");} 
             //	if(pinSetIdx == 0){//when scan sp0a19 for main
			         if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio 			failed!! \n");}
			 //    }else if(pinSetIdx == 1){////when scan sp0a19 for sub
			//	     if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio 			failed!! \n");}
		     //    } 
		PK_DBG("kdCISModulePowerOn disable inactive sensor --> set CMRST(GPIO%d) and CMPDN(GPIO%d) off\n",
				pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMPDN]);
             }          
       }     
       else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI704_YUV,currSensorName)))
       {  
	  PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_HI704_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);
           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
           if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
           if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
           msleep(10);
           
       	  if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }

          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }

          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name)) //VOL_1200
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }


          //PDN/STBY pin
          if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
          {
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(5);
       
             //RST pin
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(5);
          }
          
          //disable inactive sensor
          if(pinSetIdx == 0) {//disable sub
             pinSetIdxTmp = 1;
          }
          else{
             pinSetIdxTmp = 0;
          }
          if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio 			mode failed!! \n");}
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio 			mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio 			failed!! \n");} 
         //    if(pinSetIdx == 0){//when hi704 scan for main
			     if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio 			failed!! \n");}
		//	 }else if(pinSetIdx == 1){////when hi704 scan for sub
		//		 if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio 			failed!! \n");}
		//     }
             
	     PK_DBG("kdCISModulePowerOn disable inactive sensor --> set CMRST(GPIO%d) and CMPDN(GPIO%d) off\n",
				pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMPDN]);
          }          
       }
    }
    else {//power OFF

       PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);
       if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
           if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
           if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor

	   		mdelay(5);
	   #if 0	
           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
           if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			//if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI704_YUV,currSensorName)))
			if((pinSetIdx == 1) && (subCamPDNStatus == 1))
            {
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] HI253 set gpio failed!! \n");} //high == power down lens module
			}
			else
			{
 				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module			
			}
           PK_DBG("kdCISModulePowerOn -off sensor --> set CMRST(GPIO%d) and CMPDN(GPIO%d) off\n",
				pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMPDN]);
           #endif
       }
       /* 
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
           PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }

       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
           PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
       {
           PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
       {
           PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }*/
       
       //  if(mainCamPDNStatus == 1)
        {
            PK_DBG("[CAMERA SENSOR] mainCamPDNStatus==1, pdn zero\n");
        	mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO);
        	mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT);
//        	mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO);
                mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ONE);
        }

        //if(subCamPDNStatus == 1)
        {
            PK_DBG("[CAMERA SENSOR] subCamPDNStatus==1, pdn zero\n");
        	mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
        	mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
        	//mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO);
        	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE);
        }
        
  }

  return 0;
_kdCISModulePowerOn_exit_:
    return -EIO;
}
EXPORT_SYMBOL(kdCISModulePowerOn);



void MainCameraDigtalPDNCtrl(u32 onoff){
    PK_DBG("[CAMERA SENSOR] MainCameraDigtalPDNCtrl, %d\n", onoff);
    mainCamPDNStatus = onoff;
}
EXPORT_SYMBOL(MainCameraDigtalPDNCtrl);


void SubCameraDigtalPDNCtrl(u32 onoff){
    PK_DBG("[CAMERA SENSOR] SubCameraDigtalPDNCtrl, %d\n", onoff);
    subCamPDNStatus = onoff;
}
EXPORT_SYMBOL(SubCameraDigtalPDNCtrl);
//!--
//


//!--
//
