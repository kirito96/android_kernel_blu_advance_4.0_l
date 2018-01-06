/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
 
/* MediaTek Inc. (C) 2010. All rights reserved.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *============================================================================
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>	 
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "ov5646mipiyuv_Sensor.h"
#include "ov5646mipiyuv_Camera_Sensor_para.h"
#include "ov5646mipiyuv_CameraCustomized.h" 
#define OV5646MIPIYUV_DEBUG
#ifdef OV5646MIPIYUV_DEBUG
#define OV5646MIPISENSORDB printk
#else
#define OV5646MIPISENSORDB(x,...)
#endif
static DEFINE_SPINLOCK(ov5646mipi_drv_lock);
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;

#define OV5646MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV5646MIPI_WRITE_ID)
#define mDELAY(ms)  mdelay(ms)

kal_uint8 OV5646MIPI_sensor_socket = DUAL_CAMERA_NONE_SENSOR;
typedef enum
{
    OV5646MIPI_PRV_W=1280,
    OV5646MIPI_PRV_H=960
}OV5646MIPI_PREVIEW_VIEW_SIZE;
kal_uint16 OV5646MIPIYUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV5646MIPI_WRITE_ID);
    return get_byte;
}
extern void afPowerOn(void);

#define OV5646MIPI_MAX_AXD_GAIN (32) //max gain = 32
#define OV5646MIPI_MAX_EXPOSURE_TIME (1968) // preview:984,capture 984*2
static struct
{
	//kal_uint8   Banding;
	kal_bool	  NightMode;
	kal_bool	  VideoMode;
	kal_uint16  Fps;
	kal_uint16  ShutterStep;
	kal_uint8   IsPVmode;
	kal_uint32  PreviewDummyPixels;
	kal_uint32  PreviewDummyLines;
	kal_uint32  CaptureDummyPixels;
	kal_uint32  CaptureDummyLines;
	kal_uint32  PreviewPclk;
	kal_uint32  CapturePclk;
	kal_uint32  ZsdturePclk;
	kal_uint32  PreviewShutter;
	kal_uint32  PreviewExtraShutter;
	kal_uint32  SensorGain;
	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;
	kal_uint32      currentExposureTime;
    kal_uint32      currentShutter;
	kal_uint32      currentextshutter;
    kal_uint32      currentAxDGain;
	kal_uint32  	sceneMode;
    unsigned char isoSpeed;
	unsigned char zsd_flag;
	kal_uint16 af_xcoordinate;
	kal_uint16 af_ycoordinate;
	unsigned char   awbMode;
	UINT16 iWB;
	OV5646MIPI_SENSOR_MODE SensorMode;
} OV5646MIPISensor;
/* Global Valuable */
static kal_uint32 zoom_factor = 0; 
static kal_int8 OV5646MIPI_DELAY_AFTER_PREVIEW = -1;
static kal_uint8 OV5646MIPI_Banding_setting = AE_FLICKER_MODE_50HZ; 
static kal_bool OV5646MIPI_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV5646MIPI_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT OV5646MIPISensorConfigData;
#define OV5646_TEST_PATTERN_CHECKSUM (0x7ba87eae)
kal_bool OV5646MIPI_run_test_potten=0;
void OV5646MIPI_set_scene_mode(UINT16 para);
BOOL OV5646MIPI_set_param_wb(UINT16 para);


typedef enum
{
    OV5646MIPI_AE_SECTION_INDEX_BEGIN=0, 
    OV5646MIPI_AE_SECTION_INDEX_1=OV5646MIPI_AE_SECTION_INDEX_BEGIN, 
    OV5646MIPI_AE_SECTION_INDEX_2, 
    OV5646MIPI_AE_SECTION_INDEX_3, 
    OV5646MIPI_AE_SECTION_INDEX_4, 
    OV5646MIPI_AE_SECTION_INDEX_5, 
    OV5646MIPI_AE_SECTION_INDEX_6, 
    OV5646MIPI_AE_SECTION_INDEX_7, 
    OV5646MIPI_AE_SECTION_INDEX_8, 
    OV5646MIPI_AE_SECTION_INDEX_9, 
    OV5646MIPI_AE_SECTION_INDEX_10, 
    OV5646MIPI_AE_SECTION_INDEX_11, 
    OV5646MIPI_AE_SECTION_INDEX_12, 
    OV5646MIPI_AE_SECTION_INDEX_13, 
    OV5646MIPI_AE_SECTION_INDEX_14, 
    OV5646MIPI_AE_SECTION_INDEX_15, 
    OV5646MIPI_AE_SECTION_INDEX_16,  
    OV5646MIPI_AE_SECTION_INDEX_MAX
}OV5646MIPI_AE_SECTION_INDEX;
typedef enum
{
    OV5646MIPI_AE_VERTICAL_BLOCKS=4,
    OV5646MIPI_AE_VERTICAL_BLOCKS_MAX,
    OV5646MIPI_AE_HORIZONTAL_BLOCKS=4,
    OV5646MIPI_AE_HORIZONTAL_BLOCKS_MAX
}OV5646MIPI_AE_VERTICAL_HORIZONTAL_BLOCKS;
static UINT32 line_coordinate[OV5646MIPI_AE_VERTICAL_BLOCKS_MAX] = {0};//line[0]=0      line[1]=160     line[2]=320     line[3]=480     line[4]=640
static UINT32 row_coordinate[OV5646MIPI_AE_HORIZONTAL_BLOCKS_MAX] = {0};//line[0]=0       line[1]=120     line[2]=240     line[3]=360     line[4]=480
static BOOL AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_MAX] = {FALSE};
static BOOL AE_2_ARRAY[OV5646MIPI_AE_HORIZONTAL_BLOCKS][OV5646MIPI_AE_VERTICAL_BLOCKS] = {{FALSE},{FALSE},{FALSE},{FALSE}};//how to ....

//=====================For iBurstWriteReg AF regs==========================//
static const kal_uint8 addr_data_pair1[254] =
{
	0x80,
	0x00,
	0x02,
	0x15,
	0x5c,
	0x02,
	0x12,
	0x01,
	0xc2,
	0x01,
	0x22,
	0x00,
	0x00,
	0x02,
	0x15,
	0x18,
	0x78,
	0xc3,
	0xe6,
	0x18,
	0xf6,
	0xe5,
	0x30,
	0xc3,
	0x13,
	0xfc,
	0xe5,
	0x31,
	0x13,
	0xfd,
	0xe5,
	0x2e,
	0xc3,
	0x13,
	0xfe,
	0xe5,
	0x2f,
	0x13,
	0x2d,
	0x78,
	0x9b,
	0xf6,
	0xee,
	0x3c,
	0x18,
	0xf6,
	0x78,
	0xc3,
	0xa6,
	0x4f,
	0xe5,
	0x1e,
	0x70,
	0x6b,
	0xe6,
	0x12,
	0x0f,
	0x25,
	0xff,
	0x33,
	0x95,
	0xe0,
	0xfe,
	0x74,
	0x9d,
	0x2f,
	0xf5,
	0x82,
	0x74,
	0x0e,
	0x3e,
	0xf5,
	0x83,
	0xe4,
	0x93,
	0x78,
	0xc1,
	0xf6,
	0x75,
	0x4e,
	0x02,
	0x12,
	0x0f,
	0x31,
	0x78,
	0x56,
	0x12,
	0x0f,
	0x2c,
	0x78,
	0x96,
	0x12,
	0x0f,
	0x2c,
	0x12,
	0x0f,
	0x91,
	0x78,
	0xc3,
	0xe6,
	0x78,
	0x9e,
	0xf6,
	0x78,
	0xc3,
	0xe6,
	0x78,
	0xbe,
	0xf6,
	0x78,
	0xc3,
	0xe6,
	0x78,
	0xbf,
	0xf6,
	0x08,
	0x76,
	0x02,
	0x78,
	0xc5,
	0x76,
	0x01,
	0x08,
	0x76,
	0x01,
	0x08,
	0x76,
	0x01,
	0xe6,
	0x78,
	0xc9,
	0xf6,
	0xe6,
	0x78,
	0xcb,
	0xf6,
	0x78,
	0xc9,
	0xe6,
	0x78,
	0xcc,
	0xf6,
	0xe4,
	0x08,
	0xf6,
	0x08,
	0xf6,
	0x08,
	0x76,
	0x40,
	0x78,
	0xc3,
	0xe6,
	0x78,
	0xd0,
	0xf6,
	0xe4,
	0x08,
	0xf6,
	0xc2,
	0x38,
	0xe5,
	0x1e,
	0x64,
	0x06,
	0x70,
	0x2e,
	0xd3,
	0x78,
	0xc0,
	0xe6,
	0x64,
	0x80,
	0x94,
	0x80,
	0x40,
	0x02,
	0x16,
	0x22,
	0xa2,
	0x38,
	0xe4,
	0x33,
	0xf5,
	0x41,
	0x90,
	0x30,
	0x28,
	0xf0,
	0xe4,
	0xf5,
	0x1e,
	0xc2,
	0x01,
	0x75,
	0x1d,
	0x10,
	0xd2,
	0x36,
	0x78,
	0x52,
	0xa6,
	0x2e,
	0x08,
	0xa6,
	0x2f,
	0x08,
	0xa6,
	0x30,
	0x08,
	0xa6,
	0x31,
	0x22,
	0x79,
	0xc3,
	0xe7,
	0x78,
	0xc1,
	0x26,
	0x78,
	0xc4,
	0xf6,
	0xc3,
	0x78,
	0xce,
	0xe6,
	0x64,
	0x80,
	0xf8,
	0x09,
	0xe7,
	0x64,
	0x80,
	0x98,
	0x50,
	0x06,
	0x78,
	0xce,
	0xe6,
	0x78,
	0xc4,
	0xf6,
	0xd3,
	0x78,
	0xcf,
	0xe6,
	0x64,
	0x80,
	0xf8,
	0x79,
	0xc4,
	0xe7,
	0x64,
	0x80,
};
static const kal_uint8 addr_data_pair2[255] =
{	
	0x80, 
	0xfc, 
	0x98, 
	0x40, 
	0x06, 
	0x78, 
	0xcf, 
	0xe6, 
	0x78, 
	0xc4, 
	0xf6, 
	0x78, 
	0xc4, 
	0xe6, 
	0xf5, 
	0x4f, 
	0x12, 
	0x12, 
	0xaf, 
	0x78, 
	0xc1, 
	0xe6, 
	0xff, 
	0x33, 
	0x95, 
	0xe0, 
	0xfe, 
	0x12, 
	0x15, 
	0x2e, 
	0x8f, 
	0x0a, 
	0xd3, 
	0xe5, 
	0x0a, 
	0x64, 
	0x80, 
	0x94, 
	0x86, 
	0x40, 
	0x05, 
	0x75, 
	0x1e, 
	0x01, 
	0x80, 
	0x1f, 
	0xd3, 
	0xe5, 
	0x0a, 
	0x64, 
	0x80, 
	0x94, 
	0x83, 
	0x40, 
	0x05, 
	0x75, 
	0x1e, 
	0x02, 
	0x80, 
	0x11, 
	0xd3, 
	0xe5, 
	0x0a, 
	0x64, 
	0x80, 
	0x94, 
	0x81, 
	0x40, 
	0x05, 
	0x75, 
	0x1e, 
	0x03, 
	0x80, 
	0x03, 
	0x75, 
	0x1e, 
	0x04, 
	0xd3, 
	0x78, 
	0xc0, 
	0xe6, 
	0x64, 
	0x80, 
	0x94, 
	0x80, 
	0x40, 
	0x02, 
	0x16, 
	0x22, 
	0x78, 
	0xc6, 
	0xe6, 
	0x18, 
	0xf6, 
	0x08, 
	0x06, 
	0x78, 
	0xc2, 
	0xe6, 
	0xff, 
	0x12, 
	0x0f, 
	0x99, 
	0x12, 
	0x0f, 
	0x2f, 
	0x78, 
	0xc5, 
	0xe6, 
	0x25, 
	0xe0, 
	0x24, 
	0x56, 
	0xf8, 
	0xa6, 
	0x06, 
	0x08, 
	0xa6, 
	0x07, 
	0x79, 
	0xc5, 
	0xe7, 
	0x78, 
	0xc7, 
	0x66, 
	0x70, 
	0x05, 
	0xe6, 
	0x78, 
	0xc9, 
	0xf6, 
	0x22, 
	0x78, 
	0xc5, 
	0xe6, 
	0x78, 
	0x99, 
	0x12, 
	0x0e, 
	0xfa, 
	0x40, 
	0x0d, 
	0x78, 
	0xc5, 
	0xe6, 
	0x12, 
	0x0e, 
	0xdf, 
	0xfe, 
	0x08, 
	0xe6, 
	0xff, 
	0x12, 
	0x0f, 
	0x91, 
	0x78, 
	0xc5, 
	0xe6, 
	0x25, 
	0xe0, 
	0x24, 
	0x57, 
	0xf9, 
	0xc3, 
	0xe7, 
	0x78, 
	0x97, 
	0x96, 
	0x19, 
	0xe7, 
	0x18, 
	0x96, 
	0x50, 
	0x11, 
	0x78, 
	0xc5, 
	0xe6, 
	0x12, 
	0x0e, 
	0xdf, 
	0xfe, 
	0x08, 
	0xe6, 
	0xff, 
	0x78, 
	0x96, 
	0xa6, 
	0x06, 
	0x08, 
	0xa6, 
	0x07, 
	0x78, 
	0xc5, 
	0xe6, 
	0x25, 
	0xe0, 
	0x24, 
	0x57, 
	0xf9, 
	0x78, 
	0xc9, 
	0xe6, 
	0x12, 
	0x0e, 
	0xdf, 
	0xfe, 
	0x08, 
	0xe6, 
	0xc3, 
	0x97, 
	0xee, 
	0x19, 
	0x97, 
	0x50, 
	0x06, 
	0x78, 
	0xc5, 
	0xe6, 
	0x78, 
	0xc9, 
	0xf6, 
	0x78, 
	0xc5, 
	0xe6, 
	0x24, 
	0x9e, 
	0x78, 
	0xbe, 
	0x12, 
	0x0f, 
	0x09, 
	0x40, 
	0x07, 
	0x12, 
	0x0f, 
	0x99, 
	0xe6, 
	0x78, 
	0xbe, 
	0xf6, 
	0x78, 
	0xc5, 
	0xe6, 
	0x24, 
	0x9e, 
	0x78, 
	0xbf, 
	0x12, 
	0x0e, 
	0xe8, 
	0x50, 
	0x07, 
	0x12, 
	0x0f, 
	0x99, 
};
static const kal_uint8 addr_data_pair3[255] =
{
	0x81,
	0xf9,
	0xe6,
	0x78,
	0xbf,
	0xf6,
	0x78,
	0xc5,
	0xe6,
	0x78,
	0xc8,
	0xf6,
	0x12,
	0x10,
	0x7f,
	0x12,
	0x0c,
	0x86,
	0x12,
	0x14,
	0xdd,
	0x78,
	0xcd,
	0xa6,
	0x07,
	0xe6,
	0x24,
	0x02,
	0x70,
	0x03,
	0x02,
	0x02,
	0x9e,
	0x14,
	0x70,
	0x03,
	0x02,
	0x02,
	0x9e,
	0x24,
	0xfe,
	0x60,
	0x03,
	0x02,
	0x03,
	0xb1,
	0xd2,
	0x38,
	0x12,
	0x0f,
	0x16,
	0x40,
	0x16,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9d,
	0x12,
	0x0e,
	0xe6,
	0x50,
	0x20,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9d,
	0xf8,
	0xe6,
	0x78,
	0xce,
	0xf6,
	0x80,
	0x14,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9d,
	0x12,
	0x0f,
	0x07,
	0x40,
	0x0a,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9d,
	0xf8,
	0xe6,
	0x78,
	0xcf,
	0xf6,
	0x78,
	0xca,
	0x12,
	0x0f,
	0x52,
	0x79,
	0xc2,
	0xe7,
	0x78,
	0xc3,
	0x66,
	0x60,
	0x03,
	0x02,
	0x04,
	0x9f,
	0x78,
	0xd1,
	0x06,
	0xe5,
	0x1e,
	0xb4,
	0x01,
	0x07,
	0x12,
	0x0f,
	0x1f,
	0xf6,
	0x09,
	0x80,
	0x1e,
	0xe5,
	0x1e,
	0xb4,
	0x02,
	0x08,
	0x12,
	0x0f,
	0x1f,
	0xf6,
	0x79,
	0xc3,
	0x80,
	0x11,
	0xe5,
	0x1e,
	0xb4,
	0x03,
	0x14,
	0x78,
	0xc1,
	0xe6,
	0xf4,
	0x04,
	0xff,
	0xa2,
	0xe7,
	0x13,
	0xf6,
	0x79,
	0xc3,
	0xe7,
	0x26,
	0x78,
	0xc4,
	0xf6,
	0x02,
	0x04,
	0x8a,
	0x02,
	0x04,
	0x1c,
	0xd2,
	0x38,
	0x78,
	0xd1,
	0x06,
	0xc3,
	0x12,
	0x0f,
	0x17,
	0x50,
	0x25,
	0x79,
	0xc9,
	0xe7,
	0x78,
	0xc5,
	0x66,
	0x78,
	0xc9,
	0x60,
	0x05,
	0xe6,
	0x04,
	0xff,
	0x80,
	0x02,
	0xe6,
	0xff,
	0x8f,
	0x0a,
	0x74,
	0x9e,
	0x2f,
	0x12,
	0x0e,
	0xe6,
	0x50,
	0x2d,
	0x74,
	0x9e,
	0x2f,
	0xf8,
	0xe6,
	0x78,
	0xce,
	0xf6,
	0x80,
	0x23,
	0x79,
	0xc9,
	0xe7,
	0x78,
	0xc5,
	0x66,
	0x78,
	0xc9,
	0x60,
	0x05,
	0xe6,
	0x04,
	0xff,
	0x80,
	0x02,
	0xe6,
	0xff,
	0x8f,
	0x0a,
	0x74,
	0x9e,
	0x2f,
	0x12,
	0x0f,
	0x07,
	0x40,
	0x08,
	0x74,
	0x9e,
	0x2f,
	0xf8,
	0xe6,
	0x78,
	0xcf,
	0xf6,
	0x12,
	0x0f,
	0x50,
	0x78,
	0xc1,
};
static const kal_uint8 addr_data_pair4[255] =
{
	0x82,
	0xf6,
	0xe6,
	0xff,
	0x33,
	0x95,
	0xe0,
	0xfe,
	0xef,
	0x78,
	0x02,
	0xc3,
	0x33,
	0xce,
	0x33,
	0xce,
	0xd8,
	0xf9,
	0xff,
	0x12,
	0x15,
	0x2e,
	0x78,
	0xce,
	0xe6,
	0xfd,
	0x33,
	0x95,
	0xe0,
	0xfc,
	0x08,
	0xe6,
	0xfb,
	0x33,
	0x95,
	0xe0,
	0xfa,
	0xc3,
	0xeb,
	0x9d,
	0xfd,
	0xea,
	0x9c,
	0xfc,
	0xd3,
	0xed,
	0x9f,
	0xee,
	0x64,
	0x80,
	0xf8,
	0xec,
	0x64,
	0x80,
	0x98,
	0x40,
	0x02,
	0x80,
	0x01,
	0xd3,
	0x92,
	0x3a,
	0xe5,
	0x1e,
	0x64,
	0x01,
	0x70,
	0x21,
	0x12,
	0x0f,
	0x72,
	0x30,
	0x3a,
	0x05,
	0xe6,
	0xa2,
	0xe7,
	0x13,
	0xf6,
	0x12,
	0x0f,
	0x16,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x02,
	0x04,
	0x8a,
	0xe5,
	0x1e,
	0x64,
	0x02,
	0x70,
	0x21,
	0x12,
	0x0f,
	0x72,
	0x30,
	0x3a,
	0x05,
	0xe6,
	0xa2,
	0xe7,
	0x13,
	0xf6,
	0x12,
	0x0f,
	0x16,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x02,
	0x04,
	0x8a,
	0xe5,
	0x1e,
	0x64,
	0x03,
	0x70,
	0x21,
	0x12,
	0x0f,
	0x72,
	0x30,
	0x3a,
	0x05,
	0xe6,
	0xa2,
	0xe7,
	0x13,
	0xf6,
	0x12,
	0x0f,
	0x16,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x02,
	0x04,
	0x8a,
	0x78,
	0xc0,
	0x76,
	0x01,
	0x12,
	0x0f,
	0xa0,
	0x02,
	0x04,
	0x9c,
	0x79,
	0xc2,
	0xe7,
	0x78,
	0xc3,
	0x66,
	0x60,
	0x03,
	0x02,
	0x04,
	0x9f,
	0x78,
	0xd1,
	0x06,
	0xc3,
	0x12,
	0x0f,
	0x17,
	0x50,
	0x16,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9f,
	0x12,
	0x0e,
	0xe6,
	0x50,
	0x20,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9f,
	0xf8,
	0xe6,
	0x78,
	0xce,
	0xf6,
	0x80,
	0x14,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9f,
	0x12,
	0x0f,
	0x07,
	0x40,
	0x0a,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9f,
	0xf8,
	0xe6,
	0x78,
	0xcf,
	0xf6,
	0x12,
	0x0f,
	0x50,
	0x20,
};
static const kal_uint8 addr_data_pair5[255] =
{
	0x83,
	0xf3,
	0x38,
	0x2e,
	0xc3,
	0x08,
	0xe6,
	0x64,
	0x80,
	0x94,
	0x82,
	0x50,
	0x1b,
	0x12,
	0x0f,
	0x72,
	0xe6,
	0x64,
	0x80,
	0x94,
	0x80,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x02,
	0x04,
	0x8a,
	0x12,
	0x0f,
	0xa0,
	0x78,
	0xc0,
	0x76,
	0x01,
	0x02,
	0x04,
	0x9c,
	0xe5,
	0x1e,
	0x64,
	0x01,
	0x70,
	0x1d,
	0x78,
	0xc1,
	0xe6,
	0xf4,
	0x04,
	0x12,
	0x0f,
	0x25,
	0x12,
	0x0f,
	0x81,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x80,
	0x44,
	0xe5,
	0x1e,
	0x64,
	0x02,
	0x70,
	0x1d,
	0x78,
	0xc1,
	0xe6,
	0xf4,
	0x04,
	0x12,
	0x0f,
	0x25,
	0x12,
	0x0f,
	0x81,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x80,
	0x21,
	0xe5,
	0x1e,
	0x64,
	0x03,
	0x70,
	0x26,
	0x78,
	0xc1,
	0xe6,
	0xf4,
	0x04,
	0xa2,
	0xe7,
	0x13,
	0x12,
	0x0f,
	0x81,
	0x40,
	0x06,
	0x78,
	0xce,
	0xe6,
	0xff,
	0x80,
	0x04,
	0x78,
	0xcf,
	0xe6,
	0xff,
	0x78,
	0xc4,
	0xa6,
	0x07,
	0x78,
	0xc6,
	0xe6,
	0x08,
	0xf6,
	0x78,
	0xc0,
	0x76,
	0x01,
	0x80,
	0x0a,
	0x12,
	0x0f,
	0xa0,
	0x78,
	0xc0,
	0x76,
	0x01,
	0x75,
	0x1e,
	0x06,
	0x78,
	0xc4,
	0xe6,
	0xf5,
	0x4f,
	0x12,
	0x12,
	0xaf,
	0x22,
	0x30,
	0x01,
	0x03,
	0x02,
	0x08,
	0x5f,
	0x30,
	0x02,
	0x03,
	0x02,
	0x08,
	0x5f,
	0xe5,
	0x1e,
	0x60,
	0x03,
	0x02,
	0x05,
	0x3b,
	0x75,
	0x1d,
	0x20,
	0xd2,
	0x36,
	0xd3,
	0x78,
	0x53,
	0xe6,
	0x94,
	0x00,
	0x18,
	0xe6,
	0x94,
	0x00,
	0x40,
	0x07,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0x80,
	0x0e,
	0x90,
	0x0e,
	0x8d,
	0xe4,
	0x93,
	0x25,
	0xe0,
	0x25,
	0xe0,
	0x24,
	0x2a,
	0x12,
	0x11,
	0x8e,
	0x78,
	0x52,
	0xa6,
	0x06,
	0x08,
	0xa6,
	0x07,
	0xd3,
	0x78,
	0x55,
	0xe6,
	0x94,
	0x00,
	0x18,
	0xe6,

};
static const kal_uint8 addr_data_pair6[255] =
{	
	0x84,
	0xf0,
	0x94,
	0x00,
	0x40,
	0x07,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0x80,
	0x08,
	0x90,
	0x0e,
	0x8d,
	0xe4,
	0x93,
	0x12,
	0x11,
	0x88,
	0x78,
	0x54,
	0xa6,
	0x06,
	0x08,
	0xa6,
	0x07,
	0x12,
	0x11,
	0xd8,
	0x12,
	0x11,
	0xa9,
	0x78,
	0x5a,
	0x12,
	0x11,
	0x82,
	0x78,
	0x5c,
	0xa6,
	0x06,
	0x08,
	0xa6,
	0x07,
	0x12,
	0x11,
	0xd8,
	0x78,
	0xad,
	0xa6,
	0x33,
	0x08,
	0xa6,
	0x33,
	0x08,
	0xa6,
	0x35,
	0x78,
	0xb3,
	0xa6,
	0x33,
	0x08,
	0xa6,
	0x33,
	0x08,
	0xa6,
	0x35,
	0x75,
	0x1e,
	0x01,
	0x78,
	0xaa,
	0x76,
	0x01,
	0x22,
	0xe5,
	0x1e,
	0xb4,
	0x05,
	0x10,
	0xd2,
	0x01,
	0xc2,
	0x02,
	0xe4,
	0xf5,
	0x1e,
	0xf5,
	0x1d,
	0xd2,
	0x36,
	0xd2,
	0x34,
	0xd2,
	0x37,
	0x22,
	0x12,
	0x11,
	0xc7,
	0x24,
	0xb3,
	0x12,
	0x11,
	0xc4,
	0x24,
	0xb4,
	0x12,
	0x11,
	0xc4,
	0x24,
	0xb5,
	0xf8,
	0xa6,
	0x35,
	0x12,
	0x11,
	0xa9,
	0x12,
	0x11,
	0xbc,
	0x24,
	0x5a,
	0xf8,
	0x12,
	0x11,
	0x82,
	0x12,
	0x11,
	0xbc,
	0x24,
	0x5c,
	0xf8,
	0xa6,
	0x06,
	0x08,
	0xa6,
	0x07,
	0x12,
	0x11,
	0xbc,
	0x24,
	0x5e,
	0xf8,
	0xa6,
	0x2a,
	0x08,
	0xa6,
	0x2b,
	0x12,
	0x11,
	0xbc,
	0x24,
	0x60,
	0xf8,
	0xa6,
	0x2c,
	0x08,
	0xa6,
	0x2d,
	0x90,
	0x0e,
	0x99,
	0xe4,
	0x93,
	0x24,
	0xff,
	0xff,
	0xe4,
	0x34,
	0xff,
	0xfe,
	0x78,
	0xaa,
	0xe6,
	0x24,
	0x01,
	0xfd,
	0xe4,
	0x33,
	0xfc,
	0xd3,
	0xed,
	0x9f,
	0xee,
	0x64,
	0x80,
	0xf8,
	0xec,
	0x64,
	0x80,
	0x98,
	0x40,
	0x04,
	0x7f,
	0x00,
	0x80,
	0x05,
	0x78,
	0xaa,
	0xe6,
	0x04,
	0xff,
	0x78,
	0xaa,
	0xa6,
	0x07,
	0xe5,
	0x1e,
	0xb4,
	0x01,
	0x07,
	0xe6,
	0x70,
	0x04,
	0x75,
	0x1e,
	0x02,
	0x22,
	0xe4,
	0x78,
	0xab,
	0xf6,
	0x08,
	0xf6,
	0xf5,
	0x0b,
	0x12,
	0x11,
	0xcf,
	0xf5,
	0x14,
	0x08,
	0xe6,
	0xf5,
	0x15,
	0x12,
	0x11,
	0xcf,
	0xf5,
	0x16,
	0x08,
	0xe6,
	0xf5,
	0x17,
	0x12,
	0x11,
	0xcf,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0x12,
	0x11,
};
static const kal_uint8 addr_data_pair7[255] =
{
	0x85,
	0xed,
	0xf0,
	0x75,
	0x0a,
	0x01,
	0x90,
	0x0e,
	0x99,
	0xe4,
	0x93,
	0xfb,
	0xe5,
	0x0a,
	0xc3,
	0x9b,
	0x50,
	0x67,
	0x12,
	0x11,
	0x73,
	0xf8,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0xe4,
	0xfc,
	0xfd,
	0xe5,
	0x0f,
	0x2f,
	0xf5,
	0x0f,
	0xe5,
	0x0e,
	0x3e,
	0xf5,
	0x0e,
	0xed,
	0x35,
	0x0d,
	0xf5,
	0x0d,
	0xec,
	0x35,
	0x0c,
	0xf5,
	0x0c,
	0xe5,
	0x0a,
	0x75,
	0xf0,
	0x08,
	0xa4,
	0x24,
	0x5b,
	0x12,
	0x11,
	0x7b,
	0xf9,
	0xc3,
	0xe5,
	0x15,
	0x97,
	0xe5,
	0x14,
	0x19,
	0x97,
	0x50,
	0x0b,
	0x12,
	0x11,
	0x73,
	0xf8,
	0xe6,
	0xf5,
	0x14,
	0x08,
	0xe6,
	0xf5,
	0x15,
	0xe5,
	0x0a,
	0x75,
	0xf0,
	0x08,
	0xa4,
	0x24,
	0x5b,
	0x12,
	0x11,
	0x7b,
	0xf9,
	0xd3,
	0xe5,
	0x17,
	0x97,
	0xe5,
	0x16,
	0x19,
	0x97,
	0x40,
	0x0b,
	0x12,
	0x11,
	0x73,
	0xf8,
	0xe6,
	0xf5,
	0x16,
	0x08,
	0xe6,
	0xf5,
	0x17,
	0x05,
	0x0a,
	0x02,
	0x05,
	0xf1,
	0xe4,
	0xfa,
	0xf9,
	0xf8,
	0xaf,
	0x0f,
	0xae,
	0x0e,
	0xad,
	0x0d,
	0xac,
	0x0c,
	0x12,
	0x0b,
	0x7b,
	0x8e,
	0x18,
	0x8f,
	0x19,
	0xc3,
	0xe5,
	0x15,
	0x95,
	0x17,
	0xff,
	0xe5,
	0x14,
	0x95,
	0x16,
	0xfe,
	0xe5,
	0x0b,
	0x25,
	0xe0,
	0x24,
	0x53,
	0xf9,
	0xd3,
	0xe5,
	0x15,
	0x97,
	0xe5,
	0x14,
	0x19,
	0x97,
	0xe5,
	0x0b,
	0x40,
	0x11,
	0x25,
	0xe0,
	0x24,
	0x53,
	0xf8,
	0xc3,
	0xe5,
	0x15,
	0x96,
	0xfd,
	0xe5,
	0x14,
	0x18,
	0x96,
	0xfc,
	0x80,
	0x0f,
	0x25,
	0xe0,
	0x24,
	0x53,
	0xf8,
	0xc3,
	0xe6,
	0x95,
	0x15,
	0xfd,
	0x18,
	0xe6,
	0x95,
	0x14,
	0xfc,
	0x8c,
	0x1a,
	0x8d,
	0x1b,
	0x12,
	0x11,
	0xf0,
	0x12,
	0x11,
	0x6a,
	0x90,
	0x0e,
	0x8e,
	0x12,
	0x11,
	0x95,
	0xe4,
	0x85,
	0x15,
	0x13,
	0x85,
	0x14,
	0x12,
	0xf5,
	0x11,
	0xf5,
	0x10,
	0xaf,
	0x13,
	0xae,
	0x12,
	0x7b,
	0x04,
	0x12,
	0x11,
	0x58,
	0xc3,
	0x12,
	0x0c,
	0x0d,
	0x50,
	0x11,
	0xaf,
	0x0b,
	0x74,
	0x01,
	0xa8,
	0x07,
	0x08,
	0x80,
	0x02,
	0xc3,
	0x33,
};
static const kal_uint8 addr_data_pair8[255] =
{	
	0x86,
	0xea,
	0xd8,
	0xfc,
	0x78,
	0xab,
	0x26,
	0xf6,
	0xe4,
	0x85,
	0x1b,
	0x0f,
	0x85,
	0x1a,
	0x0e,
	0xf5,
	0x0d,
	0xf5,
	0x0c,
	0x12,
	0x11,
	0x6a,
	0x90,
	0x0e,
	0x92,
	0x12,
	0x11,
	0x95,
	0xe5,
	0x0b,
	0x25,
	0xe0,
	0x24,
	0x53,
	0xf9,
	0xd3,
	0xe5,
	0x19,
	0x97,
	0xe5,
	0x18,
	0x19,
	0x97,
	0x40,
	0x0e,
	0xe5,
	0x0b,
	0x25,
	0xe0,
	0x24,
	0x52,
	0xf8,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0x80,
	0x04,
	0xae,
	0x18,
	0xaf,
	0x19,
	0xe4,
	0x8f,
	0x13,
	0x8e,
	0x12,
	0xf5,
	0x11,
	0xf5,
	0x10,
	0x7b,
	0x10,
	0x12,
	0x11,
	0x58,
	0xd3,
	0x12,
	0x0c,
	0x0d,
	0x40,
	0x11,
	0xaf,
	0x0b,
	0x74,
	0x01,
	0xa8,
	0x07,
	0x08,
	0x80,
	0x02,
	0xc3,
	0x33,
	0xd8,
	0xfc,
	0x78,
	0xac,
	0x26,
	0xf6,
	0x05,
	0x0b,
	0xe5,
	0x0b,
	0x64,
	0x04,
	0x60,
	0x03,
	0x02,
	0x05,
	0xd2,
	0xe4,
	0xf5,
	0x0b,
	0x12,
	0x11,
	0xfa,
	0xfb,
	0x12,
	0x11,
	0xfa,
	0xfa,
	0x12,
	0x11,
	0xfa,
	0x75,
	0x18,
	0x00,
	0xf5,
	0x19,
	0x75,
	0x0a,
	0x01,
	0x90,
	0x0e,
	0x99,
	0xe4,
	0x93,
	0xff,
	0xe5,
	0x0a,
	0xc3,
	0x9f,
	0x50,
	0x2a,
	0x12,
	0x11,
	0x4b,
	0x25,
	0x19,
	0xf5,
	0x19,
	0xe4,
	0x35,
	0x18,
	0xf5,
	0x18,
	0x12,
	0x11,
	0x4b,
	0xfe,
	0xeb,
	0xc3,
	0x9e,
	0x50,
	0x04,
	0x12,
	0x11,
	0x4b,
	0xfb,
	0x12,
	0x11,
	0x4b,
	0xfe,
	0xea,
	0xd3,
	0x9e,
	0x40,
	0x04,
	0x12,
	0x11,
	0x4b,
	0xfa,
	0x05,
	0x0a,
	0x80,
	0xca,
	0xef,
	0xfd,
	0x7c,
	0x00,
	0xae,
	0x18,
	0xaf,
	0x19,
	0x12,
	0x0a,
	0x9b,
	0xc3,
	0xeb,
	0x9a,
	0xfe,
	0x74,
	0xad,
	0x25,
	0x0b,
	0xf8,
	0xe6,
	0xfd,
	0xef,
	0xd3,
	0x9d,
	0x74,
	0xad,
	0x40,
	0x0b,
	0x25,
	0x0b,
	0xf8,
	0xe6,
	0xfd,
	0xc3,
	0xef,
	0x9d,
	0xff,
	0x80,
	0x07,
	0x25,
	0x0b,
	0xf8,
	0xc3,
	0xe6,
	0x9f,
	0xff,
	0x8f,
	0x1c,
	0x90,
	0x0e,
	0x96,
	0xe4,
	0x93,
	0xff,
	0xee,
	0xc3,
	0x9f,
	0x50,
	0x0d,
	0x12,
	0x11,
	0xe5,
	0x80,
	0x02,
	0xc3,
	0x33,
	0xd8,
};
static const kal_uint8 addr_data_pair9[255] =
{
	0x87,
	0xe7,
	0xfc,
	0x78,
	0xab,
	0x26,
	0xf6,
	0x90,
	0x0e,
	0x97,
	0xe4,
	0x93,
	0xff,
	0xe5,
	0x1c,
	0xd3,
	0x9f,
	0x40,
	0x0d,
	0x12,
	0x11,
	0xe5,
	0x80,
	0x02,
	0xc3,
	0x33,
	0xd8,
	0xfc,
	0x78,
	0xac,
	0x26,
	0xf6,
	0x74,
	0xb0,
	0x25,
	0x0b,
	0xf8,
	0xa6,
	0x1c,
	0x05,
	0x0b,
	0xe5,
	0x0b,
	0x64,
	0x03,
	0x60,
	0x03,
	0x02,
	0x07,
	0x5a,
	0x78,
	0xb1,
	0xe6,
	0xff,
	0x18,
	0xe6,
	0x2f,
	0xff,
	0xe4,
	0x33,
	0xfe,
	0x78,
	0xb2,
	0xe6,
	0x7c,
	0x00,
	0x2f,
	0xf5,
	0x1b,
	0xec,
	0x3e,
	0xf5,
	0x1a,
	0x90,
	0x0e,
	0x98,
	0xe4,
	0x93,
	0xff,
	0xd3,
	0xe5,
	0x1b,
	0x9f,
	0xe5,
	0x1a,
	0x94,
	0x00,
	0x40,
	0x06,
	0x78,
	0xac,
	0x74,
	0x80,
	0x26,
	0xf6,
	0x78,
	0xac,
	0xe6,
	0x79,
	0xab,
	0x57,
	0xf6,
	0xe5,
	0x1e,
	0xb4,
	0x02,
	0x0f,
	0x18,
	0xe6,
	0xb4,
	0x7f,
	0x0a,
	0x08,
	0xe6,
	0xd3,
	0x94,
	0x00,
	0x40,
	0x03,
	0x75,
	0x1e,
	0x05,
	0x22,
	0x90,
	0x0e,
	0x89,
	0x12,
	0x0c,
	0x44,
	0x8f,
	0x4d,
	0x8e,
	0x4c,
	0x8d,
	0x4b,
	0x8c,
	0x4a,
	0x90,
	0x38,
	0x04,
	0x12,
	0x14,
	0xa8,
	0xfb,
	0xaa,
	0x06,
	0x90,
	0x38,
	0x00,
	0x12,
	0x14,
	0xa8,
	0xff,
	0xc3,
	0xeb,
	0x9f,
	0xfb,
	0xea,
	0x9e,
	0xfa,
	0x90,
	0x38,
	0x10,
	0xe0,
	0xa3,
	0xe0,
	0x75,
	0xf0,
	0x02,
	0xa4,
	0xff,
	0xc3,
	0xeb,
	0x9f,
	0xfb,
	0xea,
	0x95,
	0xf0,
	0xfa,
	0x90,
	0x38,
	0x06,
	0xe0,
	0xfe,
	0xa3,
	0xe0,
	0xfd,
	0xee,
	0xf5,
	0x0c,
	0xed,
	0xf5,
	0x0d,
	0x90,
	0x38,
	0x02,
	0x12,
	0x14,
	0xa8,
	0xff,
	0x12,
	0x14,
	0x91,
	0x90,
	0x38,
	0x12,
	0xe0,
	0xa3,
	0xe0,
	0x75,
	0xf0,
	0x02,
	0xa4,
	0xff,
	0xae,
	0xf0,
	0x12,
	0x14,
	0x91,
	0xa3,
	0xe0,
	0xb4,
	0x31,
	0x07,
	0xea,
	0xc3,
	0x13,
	0xfa,
	0xeb,
	0x13,
	0xfb,
	0x90,
	0x38,
	0x14,
	0xe0,
	0xb4,
	0x71,
	0x0f,
	0xeb,
	0xae,
	0x02,
	0x78,
	0x02,
	0xce,
	0xc3,
	0x13,
	0xce,
	0x13,
	0xd8,
	0xf9,
	0xfb,
	0xaa,
	0x06,
	0x90,
	0x38,
};
static const kal_uint8 addr_data_pair10[255] =
{
	0x88,
	0xe4,
	0x15,
	0xe0,
	0xb4,
	0x31,
	0x0b,
	0xe5,
	0x0c,
	0xc3,
	0x13,
	0xf5,
	0x0c,
	0xe5,
	0x0d,
	0x13,
	0xf5,
	0x0d,
	0x90,
	0x38,
	0x15,
	0xe0,
	0xb4,
	0x71,
	0x11,
	0xe5,
	0x0d,
	0xae,
	0x0c,
	0x78,
	0x02,
	0xce,
	0xc3,
	0x13,
	0xce,
	0x13,
	0xd8,
	0xf9,
	0xf5,
	0x0d,
	0x8e,
	0x0c,
	0xea,
	0xc4,
	0xf8,
	0x54,
	0xf0,
	0xc8,
	0x68,
	0xfa,
	0xeb,
	0xc4,
	0x54,
	0x0f,
	0x48,
	0xfb,
	0xe5,
	0x0c,
	0xc4,
	0xf8,
	0x54,
	0xf0,
	0xc8,
	0x68,
	0xf5,
	0x0c,
	0xe5,
	0x0d,
	0xc4,
	0x54,
	0x0f,
	0x48,
	0xf5,
	0x0d,
	0xe5,
	0x41,
	0x54,
	0x10,
	0xd3,
	0x94,
	0x00,
	0x40,
	0x08,
	0x85,
	0x42,
	0x4a,
	0x85,
	0x43,
	0x4b,
	0x80,
	0x0b,
	0x30,
	0x39,
	0x04,
	0x7f,
	0x16,
	0x80,
	0x02,
	0x7f,
	0x1e,
	0x8f,
	0x4b,
	0xaf,
	0x4a,
	0x12,
	0x14,
	0x76,
	0xaf,
	0x4b,
	0x7e,
	0x00,
	0xac,
	0x0c,
	0xad,
	0x0d,
	0x12,
	0x14,
	0x84,
	0xfd,
	0x7c,
	0x00,
	0xae,
	0x0e,
	0xaf,
	0x0f,
	0x12,
	0x0a,
	0x9b,
	0x8f,
	0x4a,
	0xae,
	0x10,
	0xaf,
	0x11,
	0x7c,
	0x00,
	0x30,
	0x39,
	0x04,
	0x7d,
	0x2d,
	0x80,
	0x02,
	0x7d,
	0x3c,
	0x12,
	0x0a,
	0x9b,
	0x8f,
	0x4b,
	0x8b,
	0x49,
	0x85,
	0x0d,
	0x48,
	0xaf,
	0x4c,
	0x12,
	0x14,
	0x76,
	0xaf,
	0x4d,
	0x7e,
	0x00,
	0x12,
	0x14,
	0x84,
	0xfb,
	0xae,
	0x0e,
	0xaf,
	0x0f,
	0xfd,
	0x7c,
	0x00,
	0x12,
	0x0a,
	0x9b,
	0x8f,
	0x4c,
	0xae,
	0x10,
	0xaf,
	0x11,
	0xad,
	0x03,
	0x7c,
	0x00,
	0x12,
	0x0a,
	0x9b,
	0x8f,
	0x4d,
	0xe5,
	0x4c,
	0x75,
	0xf0,
	0x02,
	0xa4,
	0xad,
	0x49,
	0x7c,
	0x00,
	0xd3,
	0x9d,
	0x74,
	0x80,
	0xf8,
	0x65,
	0xf0,
	0x98,
	0x40,
	0x05,
	0xe5,
	0x49,
	0x13,
	0xf5,
	0x4c,
	0xe5,
	0x4d,
	0x75,
	0xf0,
	0x02,
	0xa4,
	0xd3,
	0x95,
	0x48,
	0x74,
	0x80,
	0xf8,
	0x65,
	0xf0,
	0x98,
	0x40,
	0x05,
	0xe5,
	0x48,
	0x13,
	0xf5,
	0x4d,
	0xe5,
	0x4a,
	0xc3,
	0x95,
	0x4c,
	0x50,
	0x03,
	0x85,
	0x4c,
	0x4a,
	0xe5,
	0x4b,
	0xc3,
	0x95,
	0x4d,

};
static const kal_uint8 addr_data_pair11[255] =
{
	0x89,
	0xe1,
	0x50,
	0x03,
	0x85,
	0x4d,
	0x4b,
	0xe5,
	0x4a,
	0x25,
	0x4c,
	0xff,
	0xe4,
	0x33,
	0xfe,
	0xd3,
	0xef,
	0x9d,
	0xec,
	0x64,
	0x80,
	0xf8,
	0xee,
	0x64,
	0x80,
	0x98,
	0x40,
	0x06,
	0xe5,
	0x49,
	0x95,
	0x4c,
	0xf5,
	0x4a,
	0xe5,
	0x4b,
	0x25,
	0x4d,
	0xff,
	0xe4,
	0x33,
	0xfe,
	0xd3,
	0xef,
	0x95,
	0x48,
	0x74,
	0x80,
	0xf8,
	0x6e,
	0x98,
	0x40,
	0x06,
	0xe5,
	0x48,
	0x95,
	0x4d,
	0xf5,
	0x4b,
	0xc3,
	0xe5,
	0x4a,
	0x95,
	0x4c,
	0xf5,
	0x0a,
	0xc3,
	0xe5,
	0x4b,
	0x95,
	0x4d,
	0xf5,
	0x0b,
	0xe5,
	0x4a,
	0x25,
	0x4c,
	0xf9,
	0xe5,
	0x4b,
	0x25,
	0x4d,
	0xfd,
	0x90,
	0x60,
	0x01,
	0xe4,
	0xf0,
	0xa3,
	0xf0,
	0xa3,
	0xe5,
	0x49,
	0xf0,
	0xa3,
	0xe5,
	0x48,
	0xf0,
	0x7c,
	0x01,
	0xec,
	0x75,
	0xf0,
	0x04,
	0xa4,
	0xff,
	0x24,
	0x01,
	0x12,
	0x14,
	0x9d,
	0xe5,
	0x0a,
	0xf0,
	0xef,
	0x24,
	0x02,
	0xff,
	0xee,
	0x34,
	0x60,
	0x8f,
	0x82,
	0xf5,
	0x83,
	0xe5,
	0x0b,
	0xf0,
	0xec,
	0x75,
	0xf0,
	0x04,
	0xa4,
	0xff,
	0x24,
	0x03,
	0x12,
	0x14,
	0x9d,
	0xe9,
	0xf0,
	0xef,
	0x24,
	0x04,
	0xff,
	0xee,
	0x34,
	0x60,
	0x8f,
	0x82,
	0xf5,
	0x83,
	0xed,
	0xf0,
	0x0c,
	0xbc,
	0x05,
	0xc6,
	0x90,
	0x30,
	0x01,
	0xe0,
	0x44,
	0x40,
	0xf0,
	0xe0,
	0x54,
	0xbf,
	0xf0,
	0x22,
	0xef,
	0x8d,
	0xf0,
	0xa4,
	0xa8,
	0xf0,
	0xcf,
	0x8c,
	0xf0,
	0xa4,
	0x28,
	0xce,
	0x8d,
	0xf0,
	0xa4,
	0x2e,
	0xfe,
	0x22,
	0xbc,
	0x00,
	0x0b,
	0xbe,
	0x00,
	0x29,
	0xef,
	0x8d,
	0xf0,
	0x84,
	0xff,
	0xad,
	0xf0,
	0x22,
	0xe4,
	0xcc,
	0xf8,
	0x75,
	0xf0,
	0x08,
	0xef,
	0x2f,
	0xff,
	0xee,
	0x33,
	0xfe,
	0xec,
	0x33,
	0xfc,
	0xee,
	0x9d,
	0xec,
	0x98,
	0x40,
	0x05,
	0xfc,
	0xee,
	0x9d,
	0xfe,
	0x0f,
	0xd5,
	0xf0,
	0xe9,
	0xe4,
	0xce,
	0xfd,
	0x22,
	0xed,
	0xf8,
	0xf5,
	0xf0,
	0xee,
	0x84,
	0x20,
	0xd2,
	0x1c,
	0xfe,
	0xad,
	0xf0,
	0x75,
	0xf0,
	0x08,
	0xef,
	0x2f,
	0xff,
	0xed,
	0x33,
};
static const kal_uint8 addr_data_pair12[255] =
{
	0x8a,
	0xde,
	0xfd,
	0x40,
	0x07,
	0x98,
	0x50,
	0x06,
	0xd5,
	0xf0,
	0xf2,
	0x22,
	0xc3,
	0x98,
	0xfd,
	0x0f,
	0xd5,
	0xf0,
	0xea,
	0x22,
	0xe8,
	0x8f,
	0xf0,
	0xa4,
	0xcc,
	0x8b,
	0xf0,
	0xa4,
	0x2c,
	0xfc,
	0xe9,
	0x8e,
	0xf0,
	0xa4,
	0x2c,
	0xfc,
	0x8a,
	0xf0,
	0xed,
	0xa4,
	0x2c,
	0xfc,
	0xea,
	0x8e,
	0xf0,
	0xa4,
	0xcd,
	0xa8,
	0xf0,
	0x8b,
	0xf0,
	0xa4,
	0x2d,
	0xcc,
	0x38,
	0x25,
	0xf0,
	0xfd,
	0xe9,
	0x8f,
	0xf0,
	0xa4,
	0x2c,
	0xcd,
	0x35,
	0xf0,
	0xfc,
	0xeb,
	0x8e,
	0xf0,
	0xa4,
	0xfe,
	0xa9,
	0xf0,
	0xeb,
	0x8f,
	0xf0,
	0xa4,
	0xcf,
	0xc5,
	0xf0,
	0x2e,
	0xcd,
	0x39,
	0xfe,
	0xe4,
	0x3c,
	0xfc,
	0xea,
	0xa4,
	0x2d,
	0xce,
	0x35,
	0xf0,
	0xfd,
	0xe4,
	0x3c,
	0xfc,
	0x22,
	0x75,
	0xf0,
	0x08,
	0x75,
	0x82,
	0x00,
	0xef,
	0x2f,
	0xff,
	0xee,
	0x33,
	0xfe,
	0xcd,
	0x33,
	0xcd,
	0xcc,
	0x33,
	0xcc,
	0xc5,
	0x82,
	0x33,
	0xc5,
	0x82,
	0x9b,
	0xed,
	0x9a,
	0xec,
	0x99,
	0xe5,
	0x82,
	0x98,
	0x40,
	0x0c,
	0xf5,
	0x82,
	0xee,
	0x9b,
	0xfe,
	0xed,
	0x9a,
	0xfd,
	0xec,
	0x99,
	0xfc,
	0x0f,
	0xd5,
	0xf0,
	0xd6,
	0xe4,
	0xce,
	0xfb,
	0xe4,
	0xcd,
	0xfa,
	0xe4,
	0xcc,
	0xf9,
	0xa8,
	0x82,
	0x22,
	0xb8,
	0x00,
	0xc1,
	0xb9,
	0x00,
	0x59,
	0xba,
	0x00,
	0x2d,
	0xec,
	0x8b,
	0xf0,
	0x84,
	0xcf,
	0xce,
	0xcd,
	0xfc,
	0xe5,
	0xf0,
	0xcb,
	0xf9,
	0x78,
	0x18,
	0xef,
	0x2f,
	0xff,
	0xee,
	0x33,
	0xfe,
	0xed,
	0x33,
	0xfd,
	0xec,
	0x33,
	0xfc,
	0xeb,
	0x33,
	0xfb,
	0x10,
	0xd7,
	0x03,
	0x99,
	0x40,
	0x04,
	0xeb,
	0x99,
	0xfb,
	0x0f,
	0xd8,
	0xe5,
	0xe4,
	0xf9,
	0xfa,
	0x22,
	0x78,
	0x18,
	0xef,
	0x2f,
	0xff,
	0xee,
	0x33,
	0xfe,
	0xed,
	0x33,
	0xfd,
	0xec,
	0x33,
	0xfc,
	0xc9,
	0x33,
	0xc9,
	0x10,
	0xd7,
	0x05,
	0x9b,
	0xe9,
	0x9a,
	0x40,
	0x07,
	0xec,
	0x9b,
	0xfc,
	0xe9,
	0x9a,
	0xf9,
	0x0f,
	0xd8,
	0xe0,
	0xe4,
	0xc9,
	0xfa,
	0xe4,
	0xcc,
	0xfb,
	0x22,
	0x75,
};
static const kal_uint8 addr_data_pair13[255] =
{
	0x8b,
	0xdb,
	0xf0,
	0x10,
	0xef,
	0x2f,
	0xff,
	0xee,
	0x33,
	0xfe,
	0xed,
	0x33,
	0xfd,
	0xcc,
	0x33,
	0xcc,
	0xc8,
	0x33,
	0xc8,
	0x10,
	0xd7,
	0x07,
	0x9b,
	0xec,
	0x9a,
	0xe8,
	0x99,
	0x40,
	0x0a,
	0xed,
	0x9b,
	0xfd,
	0xec,
	0x9a,
	0xfc,
	0xe8,
	0x99,
	0xf8,
	0x0f,
	0xd5,
	0xf0,
	0xda,
	0xe4,
	0xcd,
	0xfb,
	0xe4,
	0xcc,
	0xfa,
	0xe4,
	0xc8,
	0xf9,
	0x22,
	0xeb,
	0x9f,
	0xf5,
	0xf0,
	0xea,
	0x9e,
	0x42,
	0xf0,
	0xe9,
	0x9d,
	0x42,
	0xf0,
	0xe8,
	0x9c,
	0x45,
	0xf0,
	0x22,
	0xe8,
	0x60,
	0x0f,
	0xec,
	0xc3,
	0x13,
	0xfc,
	0xed,
	0x13,
	0xfd,
	0xee,
	0x13,
	0xfe,
	0xef,
	0x13,
	0xff,
	0xd8,
	0xf1,
	0x22,
	0xe8,
	0x60,
	0x0f,
	0xef,
	0xc3,
	0x33,
	0xff,
	0xee,
	0x33,
	0xfe,
	0xed,
	0x33,
	0xfd,
	0xec,
	0x33,
	0xfc,
	0xd8,
	0xf1,
	0x22,
	0xe4,
	0x93,
	0xfc,
	0x74,
	0x01,
	0x93,
	0xfd,
	0x74,
	0x02,
	0x93,
	0xfe,
	0x74,
	0x03,
	0x93,
	0xff,
	0x22,
	0xa4,
	0x25,
	0x82,
	0xf5,
	0x82,
	0xe5,
	0xf0,
	0x35,
	0x83,
	0xf5,
	0x83,
	0x22,
	0xd0,
	0x83,
	0xd0,
	0x82,
	0xf8,
	0xe4,
	0x93,
	0x70,
	0x12,
	0x74,
	0x01,
	0x93,
	0x70,
	0x0d,
	0xa3,
	0xa3,
	0x93,
	0xf8,
	0x74,
	0x01,
	0x93,
	0xf5,
	0x82,
	0x88,
	0x83,
	0xe4,
	0x73,
	0x74,
	0x02,
	0x93,
	0x68,
	0x60,
	0xef,
	0xa3,
	0xa3,
	0xa3,
	0x80,
	0xdf,
	0x90,
	0x0e,
	0x9a,
	0x12,
	0x0f,
	0x65,
	0x78,
	0x98,
	0xe6,
	0xf5,
	0x0f,
	0x08,
	0xe6,
	0xf5,
	0x10,
	0xe4,
	0xfd,
	0xed,
	0xc3,
	0x94,
	0x08,
	0x50,
	0x18,
	0xe5,
	0x10,
	0x94,
	0x00,
	0xe5,
	0x0f,
	0x94,
	0x78,
	0x50,
	0x0e,
	0xe5,
	0x10,
	0x25,
	0xe0,
	0xf5,
	0x10,
	0xe5,
	0x0f,
	0x33,
	0xf5,
	0x0f,
	0x1d,
	0x80,
	0xe2,
	0xc3,
	0x74,
	0x07,
	0x9d,
	0xfd,
	0xc3,
	0x94,
	0x00,
	0x50,
	0x02,
	0xe4,
	0xfd,
	0x12,
	0x0e,
	0xca,
	0xed,
	0x90,
	0x0d,
	0xd5,
	0x12,
	0x0f,
	0x8a,
	0x12,
	0x0a,
	0xf0,
	0x12,
	0x0e,
	0xc2,
	0xc0,
	0x00,
	0x78,
	0xd0,
	0xe6,
	0x12,
	0x0f,
};
static const kal_uint8 addr_data_pair14[255] =
{
	0x8c,
	0xd8,
	0x25,
	0xff,
	0x33,
	0x95,
	0xe0,
	0xfe,
	0x74,
	0xc4,
	0x2f,
	0xf5,
	0x82,
	0x74,
	0x0d,
	0x12,
	0x0f,
	0x39,
	0xd0,
	0x00,
	0x12,
	0x0a,
	0xf0,
	0x12,
	0x0e,
	0xc2,
	0xc0,
	0x00,
	0xc0,
	0x01,
	0xc3,
	0x79,
	0xbf,
	0xe7,
	0x78,
	0xbe,
	0x96,
	0x12,
	0x0f,
	0x25,
	0xff,
	0x33,
	0x95,
	0xe0,
	0xfe,
	0x74,
	0xb3,
	0x2f,
	0xf5,
	0x82,
	0x74,
	0x0d,
	0x12,
	0x0f,
	0x39,
	0xd0,
	0x01,
	0xd0,
	0x00,
	0x12,
	0x0e,
	0xb2,
	0x90,
	0x0d,
	0xee,
	0x12,
	0x0f,
	0x89,
	0x12,
	0x0e,
	0xb2,
	0xc0,
	0x00,
	0xc0,
	0x01,
	0xc3,
	0x79,
	0xcf,
	0xe7,
	0x78,
	0xce,
	0x96,
	0x12,
	0x0f,
	0x25,
	0xff,
	0x33,
	0x95,
	0xe0,
	0xfe,
	0x74,
	0xdd,
	0x2f,
	0xf5,
	0x82,
	0x74,
	0x0d,
	0x12,
	0x0f,
	0x39,
	0xd0,
	0x01,
	0xd0,
	0x00,
	0x12,
	0x0f,
	0x44,
	0x78,
	0x0e,
	0x12,
	0x0e,
	0xbf,
	0xc0,
	0x00,
	0x78,
	0x98,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0xe4,
	0xfc,
	0xfd,
	0xd0,
	0x00,
	0x12,
	0x0f,
	0x44,
	0x78,
	0x0e,
	0x12,
	0x0c,
	0x1e,
	0x12,
	0x0f,
	0x47,
	0x90,
	0x0e,
	0x9b,
	0x12,
	0x0f,
	0x89,
	0x12,
	0x0e,
	0xca,
	0xc3,
	0x12,
	0x0c,
	0x0d,
	0x50,
	0x06,
	0x90,
	0x0e,
	0x9b,
	0x12,
	0x0f,
	0x65,
	0x78,
	0xc9,
	0xe6,
	0x12,
	0x0e,
	0xdf,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0xe4,
	0xfc,
	0xfd,
	0x12,
	0x0e,
	0xca,
	0xd3,
	0x12,
	0x0c,
	0x0d,
	0x40,
	0x07,
	0xe4,
	0xf5,
	0x0f,
	0xf5,
	0x10,
	0x80,
	0x1a,
	0x85,
	0x0d,
	0x0f,
	0x85,
	0x0e,
	0x10,
	0x78,
	0xc9,
	0xe6,
	0x25,
	0xe0,
	0x24,
	0x57,
	0xf8,
	0xc3,
	0xe6,
	0x95,
	0x10,
	0xf5,
	0x10,
	0x18,
	0xe6,
	0x95,
	0x0f,
	0xf5,
	0x0f,
	0x78,
	0x9c,
	0xa6,
	0x0f,
	0x08,
	0xa6,
	0x10,
	0x22,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
};
static const kal_uint8 addr_data_pair15[255] =
{
	0x8d,
	0xd5,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x10,
	0x18,
	0x20,
	0x28,
	0x30,
	0x38,
	0x40,
	0x48,
	0x50,
	0x58,
	0x60,
	0x68,
	0x70,
	0x78,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x80,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x13,
	0x04,
	0x26,
	0x15,
	0x01,
	0x36,
	0x4f,
	0x56,
	0x54,
	0x20,
	0x20,
	0x20,
	0x20,
	0x20,
	0x43,
	0x01,
	0x10,
	0x00,
	0x56,
	0x45,
	0x1a,
	0x30,
	0x29,
	0x7e,
	0x00,
	0x30,
	0x04,
	0x20,
	0xdf,
	0x30,
	0x05,
	0x40,
	0xbf,
	0x50,
	0x03,
	0x00,
	0xfd,
	0x50,
	0x27,
	0x01,
	0xfe,
	0x60,
	0x00,
	0x13,
	0x00,
	0x36,
	0x06,
	0x07,
	0x00,
	0x3f,
	0x05,
	0x30,
	0x00,
	0x3f,
	0x06,
	0x22,
	0x00,
	0x3f,
	0x08,
	0x00,
	0x00,
	0x3f,
	0x09,
	0x00,
	0x00,
	0x3f,
	0x0a,
	0x00,
	0x00,
	0x3f,
	0x0b,
	0x0f,
	0x00,
	0x3f,
	0x01,
	0x2a,
	0x00,
	0x3f,
	0x02,
	0x00,
	0x00,
	0x30,
	0x01,
	0x40,
	0xbf,
	0x30,
	0x01,
	0x00,
	0xbf,
	0x30,
	0x29,
	0x70,
	0x00,
	0x3a,
	0x00,
	0x00,
	0xff,
	0x3a,
	0x00,
	0x00,
	0xff,
	0x36,
	0x03,
	0x36,
	0x02,
	0x41,
	0x44,
	0x58,
	0x20,
	0x18,
	0x10,
	0x0a,
	0x04,
	0x04,
	0x00,
	0x03,
	0xff,
	0x64,
	0x00,
	0x00,
	0x80,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x02,
	0x04,
	0x06,
	0x06,
	0x00,
	0x02,
	0x65,
	0x00,
	0x7a,
	0x50,
	0x28,
	0x1e,
	0x08,
	0x08,
	0x01,
	0x1e,
	0x1e,
	0x1e,
	0x1e,
	0x68,
	0x68,
	0x68,
	0x68,
	0x03,
	0x05,
	0x0a,
	0x08,
	0x10,
	0x01,
	0x0a,
	0x06,
	0x06,
	0x05,
	0x05,
	0x05,
	0x05,
	0x04,
	0x04,
	0x04,
	0x04,
	0x04,
	0xfc,
	0xfc,
	0xfc,
	0xfc,
	0xfc,
	0xfc,
	0x00,
	0xa5,
	0x5a,
	0x00,
	0x12,
	0x0a,
	0xf0,
	0x8f,
	0x0e,
	0x8e,
	0x0d,
	0x8d,
	0x0c,
	0x8c,
	0x0b,
	0x78,
	0x07,
	0x12,
	0x0c,
	0x1e,
	0x8f,
	0x0e,
	0x8e,
	0x0d,
	0x8d,
	0x0c,
	0x8c,
	0x0b,
	0xab,
	0x0e,
	0xaa,
	0x0d,
	0xa9,
	0x0c,
	0xa8,
	0x0b,
};
static const kal_uint8 addr_data_pair16[255] =
{	
	0x8e,
	0xd2,
	0x22,
	0xef,
	0x25,
	0xe0,
	0x24,
	0x56,
	0xf8,
	0xe6,
	0xfc,
	0x08,
	0xe6,
	0xfd,
	0xee,
	0x25,
	0xe0,
	0x24,
	0x56,
	0xf8,
	0xe6,
	0x22,
	0x78,
	0xce,
	0xf9,
	0xc3,
	0xe7,
	0x64,
	0x80,
	0xf5,
	0xf0,
	0xe6,
	0x64,
	0x80,
	0x95,
	0xf0,
	0x22,
	0x78,
	0xcb,
	0xe6,
	0x78,
	0x9d,
	0x25,
	0xe0,
	0x24,
	0x57,
	0xf9,
	0xd3,
	0xe7,
	0x96,
	0x19,
	0xe7,
	0x18,
	0x96,
	0x22,
	0x78,
	0xcf,
	0xf9,
	0xd3,
	0xe7,
	0x64,
	0x80,
	0xf5,
	0xf0,
	0xe6,
	0x64,
	0x80,
	0x95,
	0xf0,
	0x22,
	0xd3,
	0x78,
	0xc1,
	0xe6,
	0x64,
	0x80,
	0x94,
	0x80,
	0x22,
	0x78,
	0xc1,
	0xe6,
	0xf4,
	0x04,
	0xff,
	0xa2,
	0xe7,
	0x13,
	0xa2,
	0xe7,
	0x13,
	0x22,
	0xa6,
	0x06,
	0x08,
	0xa6,
	0x07,
	0x78,
	0x9a,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0x22,
	0x3e,
	0xf5,
	0x83,
	0xe4,
	0x93,
	0xff,
	0xe4,
	0xfc,
	0xfd,
	0xfe,
	0x22,
	0x12,
	0x0a,
	0xf0,
	0x8f,
	0x0e,
	0x8e,
	0x0d,
	0x8d,
	0x0c,
	0x8c,
	0x0b,
	0x22,
	0x78,
	0xc9,
	0xe6,
	0x24,
	0x9e,
	0xf8,
	0xe6,
	0x78,
	0xd0,
	0xf6,
	0x22,
	0xc3,
	0xe6,
	0x64,
	0x80,
	0xf8,
	0xef,
	0x64,
	0x80,
	0x98,
	0x22,
	0xe4,
	0x93,
	0xff,
	0xe4,
	0x8f,
	0x0e,
	0xf5,
	0x0d,
	0xf5,
	0x0c,
	0xf5,
	0x0b,
	0x22,
	0x78,
	0xc1,
	0xe6,
	0xf4,
	0x04,
	0xf6,
	0x22,
	0xfa,
	0x08,
	0xe6,
	0xc3,
	0x9d,
	0xea,
	0x9c,
	0x22,
	0xf6,
	0xd3,
	0xe6,
	0x64,
	0x80,
	0x94,
	0x80,
	0x22,
	0xe4,
	0x93,
	0xff,
	0xe4,
	0xfc,
	0xfd,
	0xfe,
	0x22,
	0x78,
	0x98,
	0xa6,
	0x06,
	0x08,
	0xa6,
	0x07,
	0x22,
	0x78,
	0xc5,
	0xe6,
	0x24,
	0x9e,
	0xf8,
	0x22,
	0x78,
	0xd0,
	0xe6,
	0x78,
	0xc4,
	0xf6,
	0x22,
	0x85,
	0x28,
	0x46,
	0x90,
	0x30,
	0x24,
	0xe0,
	0xf5,
	0x42,
	0xa3,
	0xe0,
	0xf5,
	0x43,
	0xa3,
	0xe0,
	0xf5,
	0x44,
	0xa3,
	0xe0,
	0xf5,
	0x45,
	0xa3,
	0xe0,
	0xf5,
	0x41,
	0xd2,
	0x35,
	0xe5,
	0x46,
	0x12,
	0x0c,
	0x60,
	0x0f,
	0xf2,
	0x03,
	0x0f,
	0xff,
	0x04,
	0x10,
	0x10,
};
static const kal_uint8 addr_data_pair17[255] =
{
	0x8f,
	0xcf,
	0x05,
	0x10,
	0x13,
	0x06,
	0x10,
	0x63,
	0x07,
	0x10,
	0x1c,
	0x08,
	0x10,
	0x31,
	0x12,
	0x10,
	0x40,
	0x1a,
	0x10,
	0x4b,
	0x1b,
	0x10,
	0x31,
	0x80,
	0x10,
	0x2c,
	0x81,
	0x10,
	0x63,
	0xdc,
	0x10,
	0x53,
	0xec,
	0x00,
	0x00,
	0x10,
	0x7e,
	0x12,
	0x15,
	0x4e,
	0xd2,
	0x37,
	0xd2,
	0x01,
	0xc2,
	0x02,
	0x12,
	0x15,
	0x53,
	0x22,
	0xd2,
	0x34,
	0xd2,
	0x37,
	0xe5,
	0x42,
	0xd3,
	0x94,
	0x00,
	0x40,
	0x03,
	0x12,
	0x15,
	0x4e,
	0xd2,
	0x03,
	0x22,
	0xd2,
	0x03,
	0x22,
	0xc2,
	0x03,
	0x20,
	0x01,
	0x66,
	0x30,
	0x02,
	0x48,
	0x22,
	0xc2,
	0x01,
	0xc2,
	0x02,
	0xc2,
	0x03,
	0x12,
	0x13,
	0xc5,
	0x75,
	0x1d,
	0x70,
	0xd2,
	0x36,
	0x80,
	0x37,
	0x43,
	0x41,
	0x10,
	0x80,
	0x0a,
	0xe5,
	0x41,
	0x70,
	0x03,
	0xc3,
	0x80,
	0x01,
	0xd3,
	0x92,
	0x39,
	0x12,
	0x08,
	0x60,
	0x80,
	0x23,
	0x85,
	0x45,
	0x4e,
	0x85,
	0x41,
	0x4f,
	0x12,
	0x12,
	0xaf,
	0x80,
	0x18,
	0x85,
	0x4e,
	0x45,
	0x85,
	0x4f,
	0x41,
	0x80,
	0x10,
	0xc2,
	0xaf,
	0x85,
	0x2a,
	0x42,
	0x85,
	0x2b,
	0x43,
	0x85,
	0x2c,
	0x44,
	0x85,
	0x2d,
	0x45,
	0xd2,
	0xaf,
	0x90,
	0x30,
	0x24,
	0xe5,
	0x42,
	0xf0,
	0xa3,
	0xe5,
	0x43,
	0xf0,
	0xa3,
	0xe5,
	0x44,
	0xf0,
	0xa3,
	0xe5,
	0x45,
	0xf0,
	0xa3,
	0xe5,
	0x41,
	0xf0,
	0x90,
	0x30,
	0x23,
	0xe4,
	0xf0,
	0x22,
	0x78,
	0xc8,
	0xe6,
	0xf5,
	0x0b,
	0x18,
	0xe6,
	0xf5,
	0x0c,
	0xe4,
	0xf5,
	0x0d,
	0xf5,
	0x0e,
	0xf5,
	0x0f,
	0xf9,
	0x78,
	0xc9,
	0xe6,
	0x08,
	0xf6,
	0x78,
	0xc7,
	0xe6,
	0xff,
	0x04,
	0xfe,
	0x78,
	0xc8,
	0x12,
	0x0f,
	0x5b,
	0x50,
	0x54,
	0x12,
	0x0e,
	0xd3,
	0xfa,
	0x08,
	0xe6,
	0xd3,
	0x9d,
	0xea,
	0x9c,
	0x40,
	0x14,
	0x05,
	0x0f,
	0xd3,
	0xe5,
	0x0e,
	0x64,
	0x80,
	0xf8,
	0xe9,
	0x64,
	0x80,
	0x98,
	0x40,
	0x02,
	0x89,
	0x0e,
	0xe4,
	0xf9,
	0x80,
	0x1b,
	0x12,
	0x0e,
	0xd3,
	0x12,
	0x0f,
	0x79,
	0x50,
	0x13,
	0x09,
	0xe5,
};
static const kal_uint8 addr_data_pair18[255] =
{	
	0x90,
	0xcc,
	0x0d,
	0x64,
	0x80,
	0xf8,
	0xe5,
	0x0f,
	0x64,
	0x80,
	0x98,
	0x40,
	0x03,
	0x85,
	0x0f,
	0x0d,
	0xe4,
	0xf5,
	0x0f,
	0x78,
	0xc9,
	0xe6,
	0x12,
	0x0e,
	0xd4,
	0xfa,
	0x08,
	0xe6,
	0xb5,
	0x05,
	0x08,
	0xea,
	0xb5,
	0x04,
	0x04,
	0x78,
	0xca,
	0xa6,
	0x06,
	0x0f,
	0x0e,
	0x02,
	0x10,
	0x9b,
	0x78,
	0xc7,
	0xe6,
	0xf5,
	0x0c,
	0xe6,
	0x04,
	0xff,
	0x78,
	0xc9,
	0x12,
	0x0f,
	0x5b,
	0x50,
	0x17,
	0xe5,
	0x0c,
	0x12,
	0x0e,
	0xdf,
	0xfc,
	0x08,
	0xe6,
	0xfd,
	0xef,
	0x12,
	0x0e,
	0xdf,
	0x12,
	0x0f,
	0x79,
	0x50,
	0x02,
	0x8f,
	0x0c,
	0x0f,
	0x80,
	0xe2,
	0x78,
	0xc8,
	0xe6,
	0xf5,
	0x0b,
	0xe6,
	0x14,
	0xff,
	0x78,
	0xca,
	0xd3,
	0x12,
	0x0f,
	0x5c,
	0x40,
	0x17,
	0xe5,
	0x0b,
	0x12,
	0x0e,
	0xdf,
	0xfc,
	0x08,
	0xe6,
	0xfd,
	0xef,
	0x12,
	0x0e,
	0xdf,
	0x12,
	0x0f,
	0x79,
	0x50,
	0x02,
	0x8f,
	0x0b,
	0x1f,
	0x80,
	0xe1,
	0x78,
	0xcb,
	0xa6,
	0x0c,
	0x08,
	0xa6,
	0x0b,
	0x22,
	0xe5,
	0x0a,
	0x75,
	0xf0,
	0x03,
	0xa4,
	0x24,
	0xb3,
	0x25,
	0x0b,
	0xf8,
	0xe6,
	0x22,
	0xad,
	0x11,
	0xac,
	0x10,
	0xfa,
	0xf9,
	0xf8,
	0x12,
	0x0a,
	0xf0,
	0x8f,
	0x13,
	0x8e,
	0x12,
	0x8d,
	0x11,
	0x8c,
	0x10,
	0xab,
	0x0f,
	0xaa,
	0x0e,
	0xa9,
	0x0d,
	0xa8,
	0x0c,
	0x22,
	0xe5,
	0x0a,
	0x75,
	0xf0,
	0x08,
	0xa4,
	0x24,
	0x5a,
	0xf8,
	0xe5,
	0x0b,
	0x25,
	0xe0,
	0x28,
	0x22,
	0xa6,
	0x04,
	0x08,
	0xa6,
	0x05,
	0xef,
	0x25,
	0xe0,
	0x25,
	0xe0,
	0x24,
	0x2c,
	0xf8,
	0xe6,
	0xfe,
	0x08,
	0xe6,
	0xff,
	0x22,
	0xe5,
	0x0b,
	0x93,
	0xff,
	0xe4,
	0xfc,
	0xfd,
	0xfe,
	0x12,
	0x0a,
	0xf0,
	0x8f,
	0x0f,
	0x8e,
	0x0e,
	0x8d,
	0x0d,
	0x8c,
	0x0c,
	0x22,
	0x90,
	0x0e,
	0x8d,
	0xe4,
	0x93,
	0xff,
	0x25,
	0xe0,
	0x25,
	0xe0,
	0x24,
	0x2a,
	0xf8,
	0xe6,
	0xfc,
	0x08,
	0xe6,
	0xfd,
	0x22,
	0x78,
	0xaa,
	0xe6,
	0x75,
	0xf0,
	0x08,
	0xa4,
	0x22,
	0xf8,
	0xa6,
	0x33,
	0x78,
	0xaa,
};
static const kal_uint8 addr_data_pair19[255] =
{
	0x91,
	0xc9,
	0xe6,
	0x75,
	0xf0,
	0x03,
	0xa4,
	0x22,
	0xe5,
	0x0b,
	0x25,
	0xe0,
	0x24,
	0x5a,
	0xf8,
	0xe6,
	0x22,
	0x08,
	0xa6,
	0x2a,
	0x08,
	0xa6,
	0x2b,
	0x08,
	0xa6,
	0x2c,
	0x08,
	0xa6,
	0x2d,
	0x22,
	0xe5,
	0x0b,
	0x24,
	0x04,
	0xff,
	0x74,
	0x01,
	0xa8,
	0x07,
	0x08,
	0x22,
	0xe4,
	0x8f,
	0x0f,
	0x8e,
	0x0e,
	0xf5,
	0x0d,
	0xf5,
	0x0c,
	0x22,
	0x74,
	0xb3,
	0x25,
	0x0b,
	0xf8,
	0xe6,
	0x22,
	0xc0,
	0xe0,
	0xc0,
	0x83,
	0xc0,
	0x82,
	0xc0,
	0xd0,
	0x90,
	0x3f,
	0x0c,
	0xe0,
	0xf5,
	0x08,
	0xe5,
	0x08,
	0x30,
	0xe3,
	0x60,
	0x30,
	0x37,
	0x52,
	0x90,
	0x60,
	0x19,
	0xe0,
	0xf5,
	0x2a,
	0xa3,
	0xe0,
	0xf5,
	0x2b,
	0x90,
	0x60,
	0x1d,
	0xe0,
	0xf5,
	0x2c,
	0xa3,
	0xe0,
	0xf5,
	0x2d,
	0x90,
	0x60,
	0x21,
	0xe0,
	0xf5,
	0x2e,
	0xa3,
	0xe0,
	0xf5,
	0x2f,
	0x90,
	0x60,
	0x25,
	0xe0,
	0xf5,
	0x30,
	0xa3,
	0xe0,
	0xf5,
	0x31,
	0x30,
	0x01,
	0x06,
	0x30,
	0x34,
	0x03,
	0xd3,
	0x80,
	0x01,
	0xc3,
	0x92,
	0x09,
	0x30,
	0x02,
	0x06,
	0x30,
	0x34,
	0x03,
	0xd3,
	0x80,
	0x01,
	0xc3,
	0x92,
	0x0a,
	0x30,
	0x34,
	0x0c,
	0x30,
	0x03,
	0x09,
	0x20,
	0x02,
	0x06,
	0x20,
	0x01,
	0x03,
	0xd3,
	0x80,
	0x01,
	0xc3,
	0x92,
	0x0b,
	0x90,
	0x30,
	0x01,
	0xe0,
	0x44,
	0x40,
	0xf0,
	0xe0,
	0x54,
	0xbf,
	0xf0,
	0xe5,
	0x08,
	0x30,
	0xe1,
	0x14,
	0x30,
	0x35,
	0x11,
	0x90,
	0x30,
	0x22,
	0xe0,
	0xf5,
	0x28,
	0xe4,
	0xf0,
	0x30,
	0x00,
	0x03,
	0xd3,
	0x80,
	0x01,
	0xc3,
	0x92,
	0x08,
	0xe5,
	0x08,
	0x30,
	0xe2,
	0x0e,
	0x90,
	0x51,
	0xa5,
	0xe0,
	0xf5,
	0x33,
	0xa3,
	0xe0,
	0xf5,
	0x34,
	0xa3,
	0xe0,
	0xf5,
	0x35,
	0x90,
	0x3f,
	0x0c,
	0xe5,
	0x08,
	0xf0,
	0xd0,
	0xd0,
	0xd0,
	0x82,
	0xd0,
	0x83,
	0xd0,
	0xe0,
	0x32,
	0xe5,
	0x4f,
	0xd3,
	0x94,
	0x40,
	0x40,
	0x04,
	0x7f,
	0x40,
	0x80,
	0x02,
	0xaf,
	0x4f,
	0x8f,
	0x4f,
	0x90,
	0x0e,
	0x86,
	0xe4,
	0x93,
	0xfe,
	0x74,
	0x01,
};
static const kal_uint8 addr_data_pair20[255] =
{
	0x92,
	0xc6,
	0x93,
	0xff,
	0xc3,
	0x90,
	0x0e,
	0x84,
	0x74,
	0x01,
	0x93,
	0x9f,
	0xff,
	0xe4,
	0x93,
	0x9e,
	0xfe,
	0xe4,
	0x8f,
	0x12,
	0x8e,
	0x11,
	0xf5,
	0x10,
	0xf5,
	0x0f,
	0xab,
	0x12,
	0xaa,
	0x11,
	0xa9,
	0x10,
	0xa8,
	0x0f,
	0xaf,
	0x4f,
	0xfc,
	0xfd,
	0xfe,
	0x12,
	0x0a,
	0xf0,
	0x12,
	0x14,
	0xfd,
	0xe4,
	0x7b,
	0x40,
	0xfa,
	0xf9,
	0xf8,
	0x12,
	0x0b,
	0x7b,
	0x12,
	0x14,
	0xfd,
	0x90,
	0x0e,
	0x71,
	0xe4,
	0x12,
	0x15,
	0x12,
	0x12,
	0x14,
	0xfd,
	0xe4,
	0x85,
	0x4e,
	0x0e,
	0xf5,
	0x0d,
	0xf5,
	0x0c,
	0xf5,
	0x0b,
	0xaf,
	0x0e,
	0xae,
	0x0d,
	0xad,
	0x0c,
	0xac,
	0x0b,
	0xa3,
	0x12,
	0x15,
	0x12,
	0x8f,
	0x0e,
	0x8e,
	0x0d,
	0x8d,
	0x0c,
	0x8c,
	0x0b,
	0xe5,
	0x12,
	0x45,
	0x0e,
	0xf5,
	0x12,
	0xe5,
	0x11,
	0x45,
	0x0d,
	0xf5,
	0x11,
	0xe5,
	0x10,
	0x45,
	0x0c,
	0xf5,
	0x10,
	0xe5,
	0x0f,
	0x45,
	0x0b,
	0xf5,
	0x0f,
	0xe4,
	0xf5,
	0x22,
	0xf5,
	0x23,
	0x85,
	0x12,
	0x40,
	0x85,
	0x11,
	0x3f,
	0x85,
	0x10,
	0x3e,
	0x85,
	0x0f,
	0x3d,
	0x02,
	0x14,
	0xaf,
	0x75,
	0x89,
	0x03,
	0x75,
	0xa8,
	0x01,
	0x75,
	0xb8,
	0x04,
	0x75,
	0x0a,
	0xff,
	0x75,
	0x0b,
	0x0e,
	0x75,
	0x0c,
	0x15,
	0x75,
	0x0d,
	0x0f,
	0x12,
	0x14,
	0x36,
	0x12,
	0x08,
	0x60,
	0xc2,
	0x39,
	0x12,
	0x00,
	0x06,
	0xd2,
	0x00,
	0xd2,
	0x35,
	0xd2,
	0xaf,
	0x75,
	0x0a,
	0xff,
	0x75,
	0x0b,
	0x0e,
	0x75,
	0x0c,
	0x51,
	0x75,
	0x0d,
	0x03,
	0x12,
	0x14,
	0x36,
	0x30,
	0x08,
	0x09,
	0xc2,
	0x35,
	0x12,
	0x0f,
	0xa7,
	0xc2,
	0x08,
	0xd2,
	0x35,
	0x30,
	0x0b,
	0x09,
	0xc2,
	0x37,
	0x12,
	0x04,
	0xa8,
	0xc2,
	0x0b,
	0xd2,
	0x37,
	0x30,
	0x09,
	0x09,
	0xc2,
	0x37,
	0x12,
	0x00,
	0x0e,
	0xc2,
	0x09,
	0xd2,
	0x37,
	0x30,
	0x0e,
	0x03,
	0x12,
	0x08,
	0x60,
	0x30,
	0x36,
	0xd3,
	0x90,
	0x30,
	0x29,
	0xe5,
	0x1d,
	0xf0,
	0xb4,
	0x10,
	0x05,
	0x90,
	0x30,
	0x23,
	0xe4,
	0xf0,
	0xc2,
	0x36,
};
static const kal_uint8 addr_data_pair21[255] =
{	
	0x93,
	0xc3,
	0x80,
	0xc1,
	0xe4,
	0xf5,
	0x4f,
	0x90,
	0x0e,
	0x82,
	0x93,
	0xff,
	0xe4,
	0x8f,
	0x0d,
	0xf5,
	0x0c,
	0xf5,
	0x0b,
	0xf5,
	0x0a,
	0xaf,
	0x0d,
	0xae,
	0x0c,
	0xad,
	0x0b,
	0xac,
	0x0a,
	0x90,
	0x0e,
	0x72,
	0x12,
	0x15,
	0x12,
	0x8f,
	0x0d,
	0x8e,
	0x0c,
	0x8d,
	0x0b,
	0x8c,
	0x0a,
	0x90,
	0x0e,
	0x7a,
	0x12,
	0x0c,
	0x44,
	0xef,
	0x45,
	0x0d,
	0xf5,
	0x0d,
	0xee,
	0x45,
	0x0c,
	0xf5,
	0x0c,
	0xed,
	0x45,
	0x0b,
	0xf5,
	0x0b,
	0xec,
	0x45,
	0x0a,
	0xf5,
	0x0a,
	0xe4,
	0xf5,
	0x22,
	0xf5,
	0x23,
	0x85,
	0x0d,
	0x40,
	0x85,
	0x0c,
	0x3f,
	0x85,
	0x0b,
	0x3e,
	0x85,
	0x0a,
	0x3d,
	0x12,
	0x14,
	0xaf,
	0xe4,
	0xf5,
	0x22,
	0xf5,
	0x23,
	0x90,
	0x0e,
	0x7a,
	0x12,
	0x15,
	0x06,
	0x12,
	0x14,
	0xaf,
	0xe4,
	0xf5,
	0x22,
	0xf5,
	0x23,
	0x90,
	0x0e,
	0x76,
	0x12,
	0x15,
	0x06,
	0x02,
	0x14,
	0xaf,
	0xae,
	0x0b,
	0xaf,
	0x0c,
	0xe4,
	0xfd,
	0xed,
	0xc3,
	0x95,
	0x0d,
	0x50,
	0x33,
	0x12,
	0x15,
	0x68,
	0xe4,
	0x93,
	0xf5,
	0x0e,
	0x74,
	0x01,
	0x93,
	0xf5,
	0x0f,
	0x45,
	0x0e,
	0x60,
	0x23,
	0x85,
	0x0f,
	0x82,
	0x85,
	0x0e,
	0x83,
	0xe0,
	0xfc,
	0x12,
	0x15,
	0x68,
	0x74,
	0x03,
	0x93,
	0x52,
	0x04,
	0x12,
	0x15,
	0x68,
	0x74,
	0x02,
	0x93,
	0x42,
	0x04,
	0x85,
	0x0f,
	0x82,
	0x85,
	0x0e,
	0x83,
	0xec,
	0xf0,
	0x0d,
	0x80,
	0xc7,
	0x22,
	0x7e,
	0x00,
	0xad,
	0x03,
	0xac,
	0x02,
	0x12,
	0x0a,
	0x89,
	0x8e,
	0x0e,
	0x8f,
	0x0f,
	0x22,
	0x12,
	0x0a,
	0x89,
	0x8e,
	0x10,
	0x8f,
	0x11,
	0x90,
	0x0e,
	0x88,
	0xe4,
	0x93,
	0x22,
	0xc3,
	0xe5,
	0x0d,
	0x9f,
	0xf5,
	0x0d,
	0xe5,
	0x0c,
	0x9e,
	0xf5,
	0x0c,
	0x22,
	0xae,
	0xf0,
	0xfb,
	0xee,
	0x34,
	0x60,
	0x8b,
	0x82,
	0xf5,
	0x83,
	0x22,
	0xe0,
	0xfe,
	0xa3,
	0xe0,
	0xfd,
	0xed,
	0x22,
	0xa2,
	0xaf,
	0x92,
	0x33,
	0xc2,
	0xaf,
	0xe5,
	0x23,
	0x45,
	0x22,
	0x90,
	0x0e,
	0x65,
	0x60,
	0x0e,
	0x12,
	0x15,
};
static const kal_uint8 addr_data_pair22[181] =
{
	0x94,
	0xc0,
	0x43,
	0xe0,
	0xf5,
	0x3b,
	0x12,
	0x15,
	0x40,
	0xe0,
	0xf5,
	0x3c,
	0x80,
	0x0c,
	0x12,
	0x15,
	0x43,
	0xe5,
	0x3f,
	0xf0,
	0x12,
	0x15,
	0x40,
	0xe5,
	0x40,
	0xf0,
	0xa2,
	0x33,
	0x92,
	0xaf,
	0x22,
	0x78,
	0xcc,
	0x12,
	0x0e,
	0xf7,
	0x40,
	0x0d,
	0x12,
	0x0e,
	0xf5,
	0x40,
	0x04,
	0xe4,
	0xff,
	0x80,
	0x0f,
	0x7f,
	0x01,
	0x80,
	0x0b,
	0x12,
	0x0e,
	0xf5,
	0x40,
	0x04,
	0x7f,
	0xff,
	0x80,
	0x02,
	0x7f,
	0xfe,
	0x22,
	0x8f,
	0x12,
	0x8e,
	0x11,
	0x8d,
	0x10,
	0x8c,
	0x0f,
	0x22,
	0x12,
	0x0c,
	0x44,
	0x8f,
	0x40,
	0x8e,
	0x3f,
	0x8d,
	0x3e,
	0x8c,
	0x3d,
	0x22,
	0x93,
	0xf9,
	0xf8,
	0x02,
	0x0c,
	0x31,
	0xc0,
	0xe0,
	0xc0,
	0x83,
	0xc0,
	0x82,
	0x90,
	0x3f,
	0x0d,
	0xe0,
	0xf5,
	0x09,
	0xe5,
	0x09,
	0xf0,
	0xd0,
	0x82,
	0xd0,
	0x83,
	0xd0,
	0xe0,
	0x32,
	0xc3,
	0xee,
	0x64,
	0x80,
	0x94,
	0x80,
	0x40,
	0x02,
	0x80,
	0x07,
	0xc3,
	0xe4,
	0x9f,
	0xff,
	0xe4,
	0x9e,
	0xfe,
	0x22,
	0x90,
	0x0e,
	0x67,
	0xe4,
	0x93,
	0xfe,
	0x74,
	0x01,
	0x93,
	0xf5,
	0x82,
	0x8e,
	0x83,
	0x22,
	0xd2,
	0x01,
	0xc2,
	0x02,
	0xe4,
	0xf5,
	0x1e,
	0xf5,
	0x1d,
	0xd2,
	0x36,
	0xd2,
	0x34,
	0x22,
	0x78,
	0x7f,
	0xe4,
	0xf6,
	0xd8,
	0xfd,
	0x75,
	0x81,
	0xd1,
	0x02,
	0x13,
	0x51,
	0x8f,
	0x82,
	0x8e,
	0x83,
	0x75,
	0xf0,
	0x04,
	0xed,
	0x02,
	0x0c,
	0x54,
};

//=====================touch AE begin==========================//
void OV5646MIPI_writeAEReg(void)
{	
    UINT8 temp;
    //write 1280X960
     OV5646MIPI_write_cmos_sensor(0x501d,0x10);
     OV5646MIPI_write_cmos_sensor(0x5680,0x00); 
     OV5646MIPI_write_cmos_sensor(0x5681,0x00);
     OV5646MIPI_write_cmos_sensor(0x5682,0x00);  
     OV5646MIPI_write_cmos_sensor(0x5683,0x00);
     OV5646MIPI_write_cmos_sensor(0x5684,0x05); //width=256  
     OV5646MIPI_write_cmos_sensor(0x5685,0x00);
     OV5646MIPI_write_cmos_sensor(0x5686,0x03); //heght=256
     OV5646MIPI_write_cmos_sensor(0x5687,0xc0);
    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_1]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_2]==TRUE)    { temp=temp|0xF0;}
    //write 0x5688
    OV5646MIPI_write_cmos_sensor(0x5688,temp);
    
    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_3]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_4]==TRUE)    { temp=temp|0xF0;}
    //write 0x5689
    OV5646MIPI_write_cmos_sensor(0x5689,temp);

    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_5]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_6]==TRUE)    { temp=temp|0xF0;}
    //write 0x568A
    OV5646MIPI_write_cmos_sensor(0x568A,temp);
    
    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_7]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_8]==TRUE)    { temp=temp|0xF0;}
    //write 0x568B
    OV5646MIPI_write_cmos_sensor(0x568B,temp);

    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_9]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_10]==TRUE)  { temp=temp|0xF0;}
	//write 0x568C
    OV5646MIPI_write_cmos_sensor(0x568C,temp);

    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_11]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_12]==TRUE)    { temp=temp|0xF0;}
    //write 0x568D
    OV5646MIPI_write_cmos_sensor(0x568D,temp);    
    
    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_13]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_14]==TRUE)    { temp=temp|0xF0;}
	//write 0x568E
    OV5646MIPI_write_cmos_sensor(0x568E,temp);

    temp=0x11;
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_15]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[OV5646MIPI_AE_SECTION_INDEX_16]==TRUE)    { temp=temp|0xF0;}
	//write 0x568F
    OV5646MIPI_write_cmos_sensor(0x568F,temp);
}


void OV5646MIPI_printAE_1_ARRAY(void)
{
    UINT32 i;
    for(i=0; i<OV5646MIPI_AE_SECTION_INDEX_MAX; i++)
    {
        OV5646MIPISENSORDB("AE_1_ARRAY[%2d]=%d\n", i, AE_1_ARRAY[i]);
    }
}

void OV5646MIPI_printAE_2_ARRAY(void)
{
    UINT32 i, j;
    OV5646MIPISENSORDB("\t\t");
    for(i=0; i<OV5646MIPI_AE_VERTICAL_BLOCKS; i++)
    {
        OV5646MIPISENSORDB("      line[%2d]", i);
    }
    printk("\n");
    for(j=0; j<OV5646MIPI_AE_HORIZONTAL_BLOCKS; j++)
    {
        OV5646MIPISENSORDB("\trow[%2d]", j);
        for(i=0; i<OV5646MIPI_AE_VERTICAL_BLOCKS; i++)
        {
            //SENSORDB("AE_2_ARRAY[%2d][%2d]=%d\n", j,i,AE_2_ARRAY[j][i]);
            OV5646MIPISENSORDB("  %7d", AE_2_ARRAY[j][i]);
        }
        OV5646MIPISENSORDB("\n");
    }
}

void OV5646MIPI_clearAE_2_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<OV5646MIPI_AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<OV5646MIPI_AE_VERTICAL_BLOCKS; i++)
        {AE_2_ARRAY[j][i]=FALSE;}
    }
}

void OV5646MIPI_mapAE_2_ARRAY_To_AE_1_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<OV5646MIPI_AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<OV5646MIPI_AE_VERTICAL_BLOCKS; i++)
        { AE_1_ARRAY[j*OV5646MIPI_AE_VERTICAL_BLOCKS+i] = AE_2_ARRAY[j][i];}
    }
}

void OV5646MIPI_mapMiddlewaresizePointToPreviewsizePoint(
    UINT32 mx,
    UINT32 my,
    UINT32 mw,
    UINT32 mh,
    UINT32 * pvx,
    UINT32 * pvy,
    UINT32 pvw,
    UINT32 pvh
)
{
    *pvx = pvw * mx / mw;
    *pvy = pvh * my / mh;
    OV5646MIPISENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}


void OV5646MIPI_calcLine(void)
{//line[5]
    UINT32 i;
    UINT32 step = OV5646MIPI_PRV_W / OV5646MIPI_AE_VERTICAL_BLOCKS;
    for(i=0; i<=OV5646MIPI_AE_VERTICAL_BLOCKS; i++)
    {
        *(&line_coordinate[0]+i) = step*i;
        OV5646MIPISENSORDB("line[%d]=%d\t",i, *(&line_coordinate[0]+i));
    }
    OV5646MIPISENSORDB("\n");
}

void OV5646MIPI_calcRow(void)
{//row[5]
    UINT32 i;
    UINT32 step = OV5646MIPI_PRV_H / OV5646MIPI_AE_HORIZONTAL_BLOCKS;
    for(i=0; i<=OV5646MIPI_AE_HORIZONTAL_BLOCKS; i++)
    {
        *(&row_coordinate[0]+i) = step*i;
        OV5646MIPISENSORDB("row[%d]=%d\t",i,*(&row_coordinate[0]+i));
    }
    OV5646MIPISENSORDB("\n");
}

void OV5646MIPI_calcPointsAELineRowCoordinate(UINT32 x, UINT32 y, UINT32 * linenum, UINT32 * rownum)
{
    UINT32 i;
    i = 1;
    while(i<=OV5646MIPI_AE_VERTICAL_BLOCKS)
    {
        if(x<line_coordinate[i])
        {
            *linenum = i;
            break;
        }
        *linenum = i++;
    }
    
    i = 1;
    while(i<=OV5646MIPI_AE_HORIZONTAL_BLOCKS)
    {
        if(y<row_coordinate[i])
        {
            *rownum = i;
            break;
        }
        *rownum = i++;
    }
    OV5646MIPISENSORDB("PV point [%d, %d] to section line coordinate[%d] row[%d]\n",x,y,*linenum,*rownum);
}



MINT32 OV5646MIPI_clampSection(UINT32 x, UINT32 min, UINT32 max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void OV5646MIPI_mapCoordinate(UINT32 linenum, UINT32 rownum, UINT32 * sectionlinenum, UINT32 * sectionrownum)
{
    *sectionlinenum = OV5646MIPI_clampSection(linenum-1,0,OV5646MIPI_AE_VERTICAL_BLOCKS-1);
    *sectionrownum = OV5646MIPI_clampSection(rownum-1,0,OV5646MIPI_AE_HORIZONTAL_BLOCKS-1);	
    OV5646MIPISENSORDB("OV5646MIPI_mapCoordinate from[%d][%d] to[%d][%d]\n",
		linenum, rownum,*sectionlinenum,*sectionrownum);
}

void OV5646MIPI_mapRectToAE_2_ARRAY(UINT32 x0, UINT32 y0, UINT32 x1, UINT32 y1)
{
    UINT32 i, j;
    OV5646MIPISENSORDB("([%d][%d]),([%d][%d])\n", x0,y0,x1,y1);
    OV5646MIPI_clearAE_2_ARRAY();
    x0=OV5646MIPI_clampSection(x0,0,OV5646MIPI_AE_VERTICAL_BLOCKS-1);
    y0=OV5646MIPI_clampSection(y0,0,OV5646MIPI_AE_HORIZONTAL_BLOCKS-1);
    x1=OV5646MIPI_clampSection(x1,0,OV5646MIPI_AE_VERTICAL_BLOCKS-1);
    y1=OV5646MIPI_clampSection(y1,0,OV5646MIPI_AE_HORIZONTAL_BLOCKS-1);

    for(j=y0; j<=y1; j++)
    {
        for(i=x0; i<=x1; i++)
        {
            AE_2_ARRAY[j][i]=TRUE;
        }
    }
}

void OV5646MIPI_resetPVAE_2_ARRAY(void)
{
    OV5646MIPI_mapRectToAE_2_ARRAY(1,1,2,2);
}

//update ae window
//@input zone[] addr
void OV5646_FOCUS_Set_AE_Window(UINT32 zone_addr)
{//update global zone
    OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646_FOCUS_Set_AE_Window function\n");
    //input:
    UINT32 FD_XS;
    UINT32 FD_YS;	
    UINT32 x0, y0, x1, y1;
    UINT32 pvx0, pvy0, pvx1, pvy1;
    UINT32 linenum, rownum;
    UINT32 rightbottomlinenum,rightbottomrownum;
    UINT32 leftuplinenum,leftuprownum;
    UINT32* zone = (UINT32*)zone_addr;
    x0 = *zone;
    y0 = *(zone + 1);
    x1 = *(zone + 2);
    y1 = *(zone + 3);	
    FD_XS = *(zone + 4);
    FD_YS = *(zone + 5);

    OV5646MIPISENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
    x0, y0, x1, y1, FD_XS, FD_YS);	
    
    //print_sensor_ae_section();
    //print_AE_section();	

    //1.transfer points to preview size
    //UINT32 pvx0, pvy0, pvx1, pvy1;
    OV5646MIPI_mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, OV5646MIPI_PRV_W, OV5646MIPI_PRV_H);
    OV5646MIPI_mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, OV5646MIPI_PRV_W, OV5646MIPI_PRV_H);
    
    //2.sensor AE line and row coordinate
    OV5646MIPI_calcLine();
    OV5646MIPI_calcRow();

    //3.calc left up point to section
    //UINT32 linenum, rownum;
    OV5646MIPI_calcPointsAELineRowCoordinate(pvx0,pvy0,&linenum,&rownum);    
    //UINT32 leftuplinenum,leftuprownum;
    OV5646MIPI_mapCoordinate(linenum, rownum, &leftuplinenum, &leftuprownum);
    //SENSORDB("leftuplinenum=%d,leftuprownum=%d\n",leftuplinenum,leftuprownum);

    //4.calc right bottom point to section
    OV5646MIPI_calcPointsAELineRowCoordinate(pvx1,pvy1,&linenum,&rownum);    
    //UINT32 rightbottomlinenum,rightbottomrownum;
    OV5646MIPI_mapCoordinate(linenum, rownum, &rightbottomlinenum, &rightbottomrownum);
    //SENSORDB("rightbottomlinenum=%d,rightbottomrownum=%d\n",rightbottomlinenum,rightbottomrownum);

    //5.update global section array
    OV5646MIPI_mapRectToAE_2_ARRAY(leftuplinenum, leftuprownum, rightbottomlinenum, rightbottomrownum);
    //print_AE_section();

    //6.write to reg
    OV5646MIPI_mapAE_2_ARRAY_To_AE_1_ARRAY();
    //OV5646MIPI_printAE_1_ARRAY();
    OV5646MIPI_printAE_2_ARRAY();
    OV5646MIPI_writeAEReg();
	 OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646_FOCUS_Set_AE_Window function\n");
}
//=====================touch AE end==========================//
/*************************************************************************
* FUNCTION
*	OV5646MIPI_set_dummy
*
* DESCRIPTION
*	This function set the dummy pixels(Horizontal Blanking) & dummy lines(Vertical Blanking), it can be
*	used to adjust the frame rate or gain more time for back-end process.
*	
*	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
*
* PARAMETERS
*	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
*	2. kal_uint32 : Dummy Lines (Vertical Blanking)
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5646MIPIinitalvariable()
{
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.VideoMode = KAL_FALSE;
	OV5646MIPISensor.NightMode = KAL_FALSE;
	OV5646MIPISensor.Fps = 100;
	OV5646MIPISensor.ShutterStep= 0xde;
	OV5646MIPISensor.CaptureDummyPixels = 0;
	OV5646MIPISensor.CaptureDummyLines = 0;
	OV5646MIPISensor.PreviewDummyPixels = 0;
	OV5646MIPISensor.PreviewDummyLines = 0;
	OV5646MIPISensor.SensorMode= SENSOR_MODE_INIT;
	OV5646MIPISensor.IsPVmode= KAL_TRUE;	
	OV5646MIPISensor.PreviewPclk= 560;
	OV5646MIPISensor.CapturePclk= 900;
	OV5646MIPISensor.ZsdturePclk= 900;
	OV5646MIPISensor.PreviewShutter=0x0375; //0375
	OV5646MIPISensor.PreviewExtraShutter=0x00; 
	OV5646MIPISensor.SensorGain=0x10;
	OV5646MIPISensor.manualAEStart=0;
	OV5646MIPISensor.isoSpeed=AE_ISO_100;
	OV5646MIPISensor.userAskAeLock=KAL_FALSE;
    OV5646MIPISensor.userAskAwbLock=KAL_FALSE;
	OV5646MIPISensor.currentExposureTime=0;
    OV5646MIPISensor.currentShutter=0;
	OV5646MIPISensor.zsd_flag=0;
	OV5646MIPISensor.af_xcoordinate=0;
	OV5646MIPISensor.af_ycoordinate=0;
	OV5646MIPISensor.currentextshutter=0;
	OV5646MIPISensor.awbMode = AWB_MODE_AUTO;
	OV5646MIPISensor.iWB=AWB_MODE_AUTO;
	spin_unlock(&ov5646mipi_drv_lock);
}
void OV5646MIPIGetExifInfo(UINT32 exifAddr)
{
	 OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIGetExifInfo function\n");
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 20;
    pExifInfo->AEISOSpeed = OV5646MIPISensor.isoSpeed;
	pExifInfo->AWBMode = OV5646MIPISensor.awbMode;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = OV5646MIPISensor.isoSpeed;
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIGetExifInfo function\n");
}
static void OV5646MIPISetDummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
		OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPISetDummy function:dummy_pixels=%d,dummy_lines=%d\n",dummy_pixels,dummy_lines);
		if (OV5646MIPISensor.IsPVmode)  
        {
            dummy_pixels = dummy_pixels+OV5646MIPI_PV_PERIOD_PIXEL_NUMS; 
            OV5646MIPI_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            OV5646MIPI_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+OV5646MIPI_PV_PERIOD_LINE_NUMS; 
            OV5646MIPI_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            OV5646MIPI_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
        else
        {
            dummy_pixels = dummy_pixels+OV5646MIPI_FULL_PERIOD_PIXEL_NUMS; 
            OV5646MIPI_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            OV5646MIPI_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+OV5646MIPI_FULL_PERIOD_LINE_NUMS; 
            OV5646MIPI_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            OV5646MIPI_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
		OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPISetDummy function:\n ");
}    /* OV5646MIPI_set_dummy */

/*************************************************************************
* FUNCTION
*	OV5646MIPIWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5646MIPIWriteShutter(kal_uint32 shutter)
{
	kal_uint32 extra_exposure_vts = 0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIWriteShutter function:shutter=%d\n ",shutter);
	if (shutter < 1)
	{
		shutter = 1;
	}
	if (shutter > OV5646MIPI_FULL_EXPOSURE_LIMITATION) 
	{
		extra_exposure_vts =shutter+4;
		OV5646MIPI_write_cmos_sensor(0x380f, extra_exposure_vts & 0xFF);          // EXVTS[b7~b0]
		OV5646MIPI_write_cmos_sensor(0x380e, (extra_exposure_vts & 0xFF00) >> 8); // EXVTS[b15~b8]
		OV5646MIPI_write_cmos_sensor(0x350D,0x00); 
		OV5646MIPI_write_cmos_sensor(0x350C,0x00);
	}
	shutter*=16;
	OV5646MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV5646MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV5646MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIWriteShutter function:\n ");
}    /* OV5646MIPI_write_shutter */
/*************************************************************************
* FUNCTION
*	OV5646MIPIExpWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5646MIPIWriteExpShutter(kal_uint32 shutter)
{
	shutter*=16;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIWriteExpShutter function:shutter=%d\n ",shutter);
	OV5646MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV5646MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV5646MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIWriteExpShutter function:\n ");
}    /* OV5646MIPI_write_shutter */

/*************************************************************************
* FUNCTION
*	OV5646MIPIExtraWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5646MIPIWriteExtraShutter(kal_uint32 shutter)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIWriteExtraShutter function:shutter=%d\n ",shutter);
	OV5646MIPI_write_cmos_sensor(0x350D, shutter & 0xFF);          // EXVTS[b7~b0]
	OV5646MIPI_write_cmos_sensor(0x350C, (shutter & 0xFF00) >> 8); // EXVTS[b15~b8]
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIWriteExtraShutter function:\n ");
}    /* OV5646MIPI_write_shutter */

/*************************************************************************
* FUNCTION
*	OV5646MIPIWriteSensorGain
*
* DESCRIPTION
*	This function used to write the sensor gain.
*
* PARAMETERS
*	1. kal_uint32 : The sensor gain want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV5646MIPIWriteSensorGain(kal_uint32 gain)
{
	kal_uint16 temp_reg ;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIWriteSensorGain function:gain=%d\n",gain);
	if(gain > 1024)  ASSERT(0);
	temp_reg = 0;
	temp_reg=gain&0x0FF;	
	OV5646MIPI_write_cmos_sensor(0x350B,temp_reg);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIWriteSensorGain function:\n ");
}  /* OV5646MIPI_write_sensor_gain */

/*************************************************************************
* FUNCTION
*	OV5646MIPIReadShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 OV5646MIPIReadShutter(void)
{
	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIReadShutter function:\n ");
	temp_reg1 = OV5646MIPIYUV_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV5646MIPIYUV_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV5646MIPIYUV_read_cmos_sensor(0x3502);    // AEC[b7~b0]

	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.PreviewShutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIReadShutter function:\n ");	
	return OV5646MIPISensor.PreviewShutter;
} /* OV5646MIPI_read_shutter */

/*************************************************************************
* FUNCTION
*	OV5646MIPIReadExtraShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 OV5646MIPIReadExtraShutter(void)
{
	kal_uint16 temp_reg1, temp_reg2 ;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIReadExtraShutter function:\n ");
	temp_reg1 = OV5646MIPIYUV_read_cmos_sensor(0x350c);    // AEC[b15~b8]
	temp_reg2 = OV5646MIPIYUV_read_cmos_sensor(0x350d);    // AEC[b7~b0]
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.PreviewExtraShutter  = ((temp_reg1<<8)| temp_reg2);
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIReadExtraShutter function:\n ");		
	return OV5646MIPISensor.PreviewExtraShutter;
} /* OV5646MIPI_read_shutter */
/*************************************************************************
* FUNCTION
*	OV5646MIPIReadSensorGain
*
* DESCRIPTION
*	This function read current sensor gain for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current sensor gain value.
*
*************************************************************************/
static kal_uint32 OV5646MIPIReadSensorGain(void)
{
	kal_uint32 sensor_gain = 0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIReadSensorGain function:\n ");
	sensor_gain=(OV5646MIPIYUV_read_cmos_sensor(0x350B)&0xFF); 
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIReadSensorGain function:\n ");
	return sensor_gain;
}  /* OV5646MIPIReadSensorGain */
/*************************************************************************
* FUNCTION
*	OV5646MIPI_set_AE_mode
*
* DESCRIPTION
*	This function OV5646MIPI_set_AE_mode.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void OV5646MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_AE_mode function:\n ");
    AeTemp = OV5646MIPIYUV_read_cmos_sensor(0x3503);
    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        OV5646MIPI_write_cmos_sensor(0x3503,(AeTemp&(~0x07)));
    }
    else
    {
        // turn off AEC/AGC
      OV5646MIPI_write_cmos_sensor(0x3503,(AeTemp|0x07));
    }
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_AE_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*	OV5646MIPI_set_AWB_mode
*
* DESCRIPTION
*	This function OV5646MIPI_set_AWB_mode.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void OV5646MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AwbTemp;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_AWB_mode function:\n ");
	  AwbTemp = OV5646MIPIYUV_read_cmos_sensor(0x3406);   

    if (AWB_enable == KAL_TRUE)
    {
             
		OV5646MIPI_write_cmos_sensor(0x3406,AwbTemp&0xFE); 
		
    }
    else
    {             
		OV5646MIPI_write_cmos_sensor(0x3406,AwbTemp|0x01); 
		
    }
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_AWB_mode function:\n ");
}
static void OV5646MIPI_set_AWB_mode_UNLOCK()
{
    OV5646MIPI_set_AWB_mode(KAL_TRUE);
    if (!((SCENE_MODE_OFF == OV5646MIPISensor.sceneMode) || (SCENE_MODE_NORMAL == 
    OV5646MIPISensor.sceneMode) || (SCENE_MODE_HDR == OV5646MIPISensor.sceneMode)))
    {
      OV5646MIPI_set_scene_mode(OV5646MIPISensor.sceneMode);        
    }
    if (!((AWB_MODE_OFF == OV5646MIPISensor.iWB) || (AWB_MODE_AUTO == OV5646MIPISensor.iWB)))
    {
	   OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_AWB_mode_UNLOCK function:iWB=%d\n ",OV5646MIPISensor.iWB);
	   OV5646MIPI_set_param_wb(OV5646MIPISensor.iWB);
    }
    return;
}

bool OV5646MIPIYUV_read_otp_test(void)
{
	int i;
	kal_uint16 mid, lens_id;
	kal_uint32 address = 0x3d05;
	//return true;//darling af modules. jlhong for 5200 only support daling camera
	OV5646MIPI_write_cmos_sensor(0x3000, 0x00);
	OV5646MIPI_write_cmos_sensor(0x3004, 0xff);
	mDELAY(10);
	OV5646MIPI_write_cmos_sensor(0x3D21, 0x01);
#ifdef OV5646_OTP_DEBUG
	for(i=0; i<32; i++)
	{
            printk("[OV5646]mingji test otp data.res[0x%x]==0x%x\n",0x3D00+i,OV5646MIPIYUV_read_cmos_sensor(0x3D00+i));
	}
#endif
	for(i=0; i<3; i++)
	{
	    address = 0x3d05 + i*9;
	    
	    mid = OV5646MIPIYUV_read_cmos_sensor(address);
	    lens_id = OV5646MIPIYUV_read_cmos_sensor(address+1);
		printk("[OV5646]mingji test address == 0x%x mid == 0x%x len_id == 0x%x\n",address, mid, lens_id);
	    if ((mid == 0x05)&&((lens_id == 0x00)||(lens_id == 0x89)))  break;//darling af modules.
	}

	mDELAY(40);
	OV5646MIPI_write_cmos_sensor(0x3D21, 0x00);

	if (i<3)
	{
	    printk("[OV5646]mingji test. otp read success.");
	    return true;//darling af modules.
	}
	else
	{
	    printk("[OV5646]mingji test. otp read fail.");
	    return false;
	}
}


bool OV5646MIPIYUV_read_otp(void)
{
	int i;
	kal_uint16 mid, lens_id;
	kal_uint32 address = 0x3d05;
	//return true;//darling af modules. jlhong for 5200 only support daling camera
	OV5646MIPI_write_cmos_sensor(0x3000, 0x00);
	OV5646MIPI_write_cmos_sensor(0x3004, 0xff);
	mDELAY(10);
	OV5646MIPI_write_cmos_sensor(0x3D21, 0x01);
	mDELAY(10);
#ifdef OV5646_OTP_DEBUG
	for(i=0; i<32; i++)
	{
            printk("[OV5646]mingji test otp data.res[0x%x]==0x%x\n",0x3D00+i,OV5646MIPIYUV_read_cmos_sensor(0x3D00+i));
	}
#endif
	for(i=0; i<3; i++)
	{
	    address = 0x3d05 + i*9;
	    
	    mid = OV5646MIPIYUV_read_cmos_sensor(address);
	    lens_id = OV5646MIPIYUV_read_cmos_sensor(address+1);
		printk("[OV5646]mingji test address == 0x%x mid == 0x%x len_id == 0x%x\n",address, mid, lens_id);
	    if ((mid == 0x05)&&((lens_id == 0x00)||(lens_id == 0x89)))  break;//darling af modules.
	}

	mDELAY(40);
	OV5646MIPI_write_cmos_sensor(0x3D21, 0x00);

	if (i<3)
	{
	    printk("[OV5646]mingji test. otp read success.");
	    return true;//darling af modules.
	}
	else
	{
	    printk("[OV5646]mingji test. otp read fail.");
	    return false;
	}
}

/*************************************************************************
* FUNCTION
*	OV5646MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//static 
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
kal_uint32 OV5646MIPI_GetSensorID(kal_uint32 *sensorID)
{
    volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint8 temp_sccb_addr = 0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_GetSensorID function:\n ");
	OV5646MIPI_write_cmos_sensor(0x3008,0x82);// Reset sensor
	mDELAY(5);

	//*sensorID=OV5646MIPI_SENSOR_ID;
	//return ERROR_NONE; 
	
	for(i=0;i<3;i++)
	{
		sensor_id = (OV5646MIPIYUV_read_cmos_sensor(0x300A) << 8) | OV5646MIPIYUV_read_cmos_sensor(0x300B);
		OV5646MIPISENSORDB("OV5646MIPI READ ID: %x",sensor_id);
    #if 1
		if(sensor_id != OV5645MIPI_SENSOR_ID)
		{
			*sensorID =0xffffffff;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else if (OV5646MIPIYUV_read_otp()) //read mid, truly af modules mid is 0x02 and lens_id is 0x36.
		{
		        OV5646MIPISENSORDB("[OV5646MIPI]This is darling modules.");
			*sensorID=OV5646MIPI_SENSOR_ID;
			break;
		}
		else
		{
		        OV5646MIPISENSORDB("[OV5646MIPI]This is other modules.");
			*sensorID =0xffffffff;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
    #else
        if (sensor_id == OV5646MIPI_SENSOR_ID)
        {        
            int data[4];        
            int adcVol=0;                
            int res=IMM_GetOneChannelValue(0, data, 0);        
            adcVol=data[0]*1000+data[1]*10;
            OV5646MIPISENSORDB("[OV5646MIPI]res:%d, adcVol:%d mV\n", res, adcVol);
        	if(adcVol < 500) //GND
            {   
                *sensorID=OV5646MIPI_SENSOR_ID;
        	    OV5646MIPISENSORDB("[OV5646MIPI]This is darling modules.\n");
            }
            else
            {
                *sensorID = 0xFFFFFFFF;
                OV5646MIPISENSORDB("[OV5646MIPI]This isn't darling module.\n");                
            }
            break;
        }
    #endif
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_GetSensorID function:\n ");
	return ERROR_NONE;    
}
UINT32 OV5646SetTestPatternMode(kal_bool bEnable)
{
	OV5646MIPISENSORDB("[OV5646MIPI_OV5646SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
		OV5646MIPI_write_cmos_sensor(0x503d,0x80);
		OV5646MIPI_run_test_potten=1;
	}
	else
	{
		OV5646MIPI_write_cmos_sensor(0x503d,0x00);
		OV5646MIPI_run_test_potten=0;
	}
	return ERROR_NONE;
}




/*************************************************************************
* FUNCTION
*    OV5646MIPI_set_AWB_switch
*
*************************************************************************/

static void OV5646MIPI_set_AWB_switch(void)
{
	kal_uint32 sensor_gain,Gth1,Gth2,sensor_level,Yth1,Yth2,Ith1,Ith2,AE_target_value;
	sensor_gain=OV5646MIPIYUV_read_cmos_sensor(0x3507);
	sensor_level=OV5646MIPIYUV_read_cmos_sensor(0x56a1);
	Gth1=OV5646MIPIYUV_read_cmos_sensor(0x81df);
	Gth2=OV5646MIPIYUV_read_cmos_sensor(0x81e0);
	Ith1=OV5646MIPIYUV_read_cmos_sensor(0x81e1);
	Ith2=OV5646MIPIYUV_read_cmos_sensor(0x81e2);
	AE_target_value=OV5646MIPIYUV_read_cmos_sensor(0x3a1b);
	Yth1=(Ith1==0)?AE_target_value-8 : Ith1;
	Yth2=(Ith2==0)?AE_target_value	 : Ith2;
	if(sensor_gain>=Gth2 && sensor_level<=Yth1)
		{
			OV5646MIPI_write_cmos_sensor(0x5183,0x94); 	
			}
	if(sensor_gain<=Gth1 || sensor_level>=Yth2)
		{
			OV5646MIPI_write_cmos_sensor(0x5183,0x14); 	
		} 
	
	
}







/*************************************************************************
* FUNCTION
*    OV5646MIPIInitialSetting
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
//static 
void OV5646MIPIInitialSetting(void)
{
	//;OV5646MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2 Lane
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIInitialSetting function:\n ");
	OV5646MIPI_write_cmos_sensor(0x3103, 0x11);//	; PLL clock selection
	OV5646MIPI_write_cmos_sensor(0x3008, 0x82);//	; software reset	
	mDELAY(5);            										//; delay 2ms                           					
	OV5646MIPI_write_cmos_sensor(0x3008, 0x42);//	; software power down
	OV5646MIPI_write_cmos_sensor(0x3103, 0x03);//	; clock from PLL
	OV5646MIPI_write_cmos_sensor(0x3503, 0x07);//	; AGC manual, AEC manual
	OV5646MIPI_write_cmos_sensor(0x3406, 0x01);//	; awb manual, 	
	OV5646MIPI_write_cmos_sensor(0x3000, 0x30);
	OV5646MIPI_write_cmos_sensor(0x3004, 0xef);
	OV5646MIPI_write_cmos_sensor(0x3002, 0x1c);//	; system reset
	OV5646MIPI_write_cmos_sensor(0x3006, 0xc3);//	; clock enable
	OV5646MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5646MIPI_write_cmos_sensor(0x3017, 0x40);//	; Frex, CSK input, Vsync output
	OV5646MIPI_write_cmos_sensor(0x3018, 0x00);//	; GPIO input
	OV5646MIPI_write_cmos_sensor(0x302c, 0x02);//	; GPIO input
	OV5646MIPI_write_cmos_sensor(0x302e, 0x0b);//
	OV5646MIPI_write_cmos_sensor(0x3031, 0x00);//	zhouliao:from 0x08	//
	OV5646MIPI_write_cmos_sensor(0x3611, 0x06);//   laimingji:from 0x06, low light noise 0xc6
	OV5646MIPI_write_cmos_sensor(0x3612, 0xab);//   laimingji:from 0xab, low light noise 0x2b
	OV5646MIPI_write_cmos_sensor(0x3614, 0x50);//
	OV5646MIPI_write_cmos_sensor(0x3618, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x4800, 0x04);//chage mipi data free/gate
	OV5646MIPI_write_cmos_sensor(0x3034, 0x18);//	; PLL, MIPI 8-bit mode
	OV5646MIPI_write_cmos_sensor(0x3035, 0x21);//	; PLL
	OV5646MIPI_write_cmos_sensor(0x3036, 0x70);//	; PLL
	OV5646MIPI_write_cmos_sensor(0x3037, 0x13); // ; PLL
	OV5646MIPI_write_cmos_sensor(0x3108, 0x01); // ; PLL
	OV5646MIPI_write_cmos_sensor(0x3824, 0x01); // ; PLL
	OV5646MIPI_write_cmos_sensor(0x460c, 0x20); // ; PLL
	OV5646MIPI_write_cmos_sensor(0x3400, 0x05);// r
	OV5646MIPI_write_cmos_sensor(0x3401, 0x42); 
	OV5646MIPI_write_cmos_sensor(0x3402, 0x04);// g
	OV5646MIPI_write_cmos_sensor(0x3403, 0x00); 
	OV5646MIPI_write_cmos_sensor(0x3404, 0x07);// b
	OV5646MIPI_write_cmos_sensor(0x3405, 0x60); 
	OV5646MIPI_write_cmos_sensor(0x3600, 0x09);//
	OV5646MIPI_write_cmos_sensor(0x3601, 0x43);//
	OV5646MIPI_write_cmos_sensor(0x3620, 0x33);//
	OV5646MIPI_write_cmos_sensor(0x3621, 0xe0);//
	OV5646MIPI_write_cmos_sensor(0x3622, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x3630, 0x2d);//
	OV5646MIPI_write_cmos_sensor(0x3631, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x3632, 0x32);//
	OV5646MIPI_write_cmos_sensor(0x3633, 0x52);//
	OV5646MIPI_write_cmos_sensor(0x3634, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x3635, 0x13);//
	OV5646MIPI_write_cmos_sensor(0x3636, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x3702, 0x6e);//
	OV5646MIPI_write_cmos_sensor(0x3703, 0x52);//
	OV5646MIPI_write_cmos_sensor(0x3704, 0xa0);//
	OV5646MIPI_write_cmos_sensor(0x3705, 0x33);//
	OV5646MIPI_write_cmos_sensor(0x3708, 0x66);//
	OV5646MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x370b, 0x61);//
	OV5646MIPI_write_cmos_sensor(0x370c, 0xc3);//
	OV5646MIPI_write_cmos_sensor(0x370f, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x3715, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x3717, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x371b, 0x20);//
	OV5646MIPI_write_cmos_sensor(0x3731, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x3739, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x3901, 0x0a);//
	OV5646MIPI_write_cmos_sensor(0x3905, 0x02);//
	OV5646MIPI_write_cmos_sensor(0x3906, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x3719, 0x86);//
	OV5646MIPI_write_cmos_sensor(0x3800, 0x00);//	; HS = 0
	OV5646MIPI_write_cmos_sensor(0x3801, 0x00);//	; HS
	OV5646MIPI_write_cmos_sensor(0x3802, 0x00);//	; VS = 250
	OV5646MIPI_write_cmos_sensor(0x3803, 0x06);//	; VS
	OV5646MIPI_write_cmos_sensor(0x3804, 0x0a);//	; HW = 2623
	OV5646MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5646MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 1705
	OV5646MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
	OV5646MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
	OV5646MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
	OV5646MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPHO
	OV5646MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
	OV5646MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
	OV5646MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
	OV5646MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
	OV5646MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
	OV5646MIPI_write_cmos_sensor(0x3810, 0x00);//	; H OFF = 16
	OV5646MIPI_write_cmos_sensor(0x3811, 0x10);//	; H OFF
	OV5646MIPI_write_cmos_sensor(0x3812, 0x00);//	; V OFF = 4
	OV5646MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5646MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
	OV5646MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
	OV5646MIPI_write_cmos_sensor(0x3820, 0x47);//	; flip off, V bin on
	OV5646MIPI_write_cmos_sensor(0x3821, 0x01);//	; mirror on, H bin on
	OV5646MIPI_write_cmos_sensor(0x3826, 0x03); // 
	OV5646MIPI_write_cmos_sensor(0x3828, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
	OV5646MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
	OV5646MIPI_write_cmos_sensor(0x3a08, 0x01);//	; B50 = 222
	OV5646MIPI_write_cmos_sensor(0x3a09, 0x27); // ; B50
	OV5646MIPI_write_cmos_sensor(0x3a0a, 0x00); // ; B60 = 185
	OV5646MIPI_write_cmos_sensor(0x3a0b, 0xf6); // ; B60
	OV5646MIPI_write_cmos_sensor(0x3a0e, 0x03); // ; max 50
	OV5646MIPI_write_cmos_sensor(0x3a0d, 0x04); // ; max 60
	OV5646MIPI_write_cmos_sensor(0x3a14, 0x03); // ; max exp 50 = 740
	OV5646MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
	OV5646MIPI_write_cmos_sensor(0x3a18, 0x00);//	; gain ceiling = 15.5x
	OV5646MIPI_write_cmos_sensor(0x3a19, 0x60);//	; gain ceiling
	OV5646MIPI_write_cmos_sensor(0x3a05, 0x30);//	; enable band insert, ken,  
	OV5646MIPI_write_cmos_sensor(0x3c01, 0xb4); // ;manual banding mode
	OV5646MIPI_write_cmos_sensor(0x3c00, 0x04); // ;50 Banding mode 
	OV5646MIPI_write_cmos_sensor(0x3c04, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x3c05, 0x98);//
	OV5646MIPI_write_cmos_sensor(0x3c07, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x3c08, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x3c09, 0xc2);//
	OV5646MIPI_write_cmos_sensor(0x3c0a, 0x9c);//
	OV5646MIPI_write_cmos_sensor(0x3c0b, 0x40);//
	OV5646MIPI_write_cmos_sensor(0x4001, 0x02);//	; BLC start line
	OV5646MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
	OV5646MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC update triggered by gain change
	OV5646MIPI_write_cmos_sensor(0x4050, 0x6e);//	; BLC line number
	OV5646MIPI_write_cmos_sensor(0x4051, 0x8f);//	; BLC update triggered by gain change
	OV5646MIPI_write_cmos_sensor(0x4300, 0x30);//	; YUV 422, YUYV
	OV5646MIPI_write_cmos_sensor(0x4514, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x4520, 0xb0);//
	OV5646MIPI_write_cmos_sensor(0x460b, 0x37);//
	OV5646MIPI_write_cmos_sensor(0x4818, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x481d, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x481f, 0x50);//
	OV5646MIPI_write_cmos_sensor(0x4823, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x4831, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x4837, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x5000, 0xa7);//	; Lenc on, raw gamma on, BPC on, WPC on, color interpolation on
	OV5646MIPI_write_cmos_sensor(0x5001, 0xa3);//	; SDE on, scale off, UV adjust off, color matrix on, AWB on
	OV5646MIPI_write_cmos_sensor(0x5002, 0x80);//   
    OV5646MIPI_write_cmos_sensor(0x501d, 0x00);//   
	OV5646MIPI_write_cmos_sensor(0x501f, 0x00);//	; select ISP YUV 422
	OV5646MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x505c, 0x30);//
	OV5646MIPI_write_cmos_sensor(0x5181, 0x59);//
	OV5646MIPI_write_cmos_sensor(0x5183, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5191, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5192, 0x03);//    
	OV5646MIPI_write_cmos_sensor(0x5684, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5685, 0xa0);//
	OV5646MIPI_write_cmos_sensor(0x5686, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x5687, 0x78);//
	OV5646MIPI_write_cmos_sensor(0x5a00, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x5a21, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5a24, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x3008, 0x02);//	; wake up from software standby
    OV5646MIPI_write_cmos_sensor(0x3503, 0x00);//	; AGC on, AEC on

#if 0
	//AWB For darling modules
	OV5646MIPI_write_cmos_sensor(0x5180, 0xff);//awb
	OV5646MIPI_write_cmos_sensor(0x5181, 0xf2);//
	OV5646MIPI_write_cmos_sensor(0x5182, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5183, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5184, 0x25);//
	OV5646MIPI_write_cmos_sensor(0x5185, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5186, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x5187, 0x15);//
	OV5646MIPI_write_cmos_sensor(0x5188, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x5189, 0x7b);//0x7b
	OV5646MIPI_write_cmos_sensor(0x518a, 0x5c);//0x5c
	OV5646MIPI_write_cmos_sensor(0x518b, 0xc3);//
	OV5646MIPI_write_cmos_sensor(0x518c, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x518d, 0x3f);//
	OV5646MIPI_write_cmos_sensor(0x518e, 0x30);//
	OV5646MIPI_write_cmos_sensor(0x518f, 0x57);//
	OV5646MIPI_write_cmos_sensor(0x5190, 0x3d);//
	OV5646MIPI_write_cmos_sensor(0x5191, 0xf8);//
	OV5646MIPI_write_cmos_sensor(0x5192, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5193, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x5194, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5195, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5196, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5197, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5198, 0x05);//
	OV5646MIPI_write_cmos_sensor(0x5199, 0x1a);//
	OV5646MIPI_write_cmos_sensor(0x519a, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519b, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x519c, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519d, 0xb2);//
	OV5646MIPI_write_cmos_sensor(0x519e, 0x38);//
#else
	//AWB For darling modules
/*
	OV5646MIPI_write_cmos_sensor(0x5180, 0xff);//awb
	OV5646MIPI_write_cmos_sensor(0x5181, 0xf6);//
	OV5646MIPI_write_cmos_sensor(0x5182, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5183, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5184, 0x25);//
	OV5646MIPI_write_cmos_sensor(0x5185, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5186, 0x0d);//
	OV5646MIPI_write_cmos_sensor(0x5187, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5188, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x5189, 0x77);//0x7b
	OV5646MIPI_write_cmos_sensor(0x518a, 0x5a);//0x5c
	OV5646MIPI_write_cmos_sensor(0x518b, 0x9f);//
	OV5646MIPI_write_cmos_sensor(0x518c, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x518d, 0x3d);//
	OV5646MIPI_write_cmos_sensor(0x518e, 0x30);//
	OV5646MIPI_write_cmos_sensor(0x518f, 0x50);//
	OV5646MIPI_write_cmos_sensor(0x5190, 0x3d);//
	OV5646MIPI_write_cmos_sensor(0x5191, 0xf8);//
	OV5646MIPI_write_cmos_sensor(0x5192, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5193, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x5194, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5195, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5196, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5197, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5198, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5199, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x519a, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519b, 0x79);//
	OV5646MIPI_write_cmos_sensor(0x519c, 0x09);//
	OV5646MIPI_write_cmos_sensor(0x519d, 0x2a);//
	OV5646MIPI_write_cmos_sensor(0x519e, 0x38);//

	OV5646MIPI_write_cmos_sensor(0x5180, 0xff);//awb
	OV5646MIPI_write_cmos_sensor(0x5181, 0xf2);//
	OV5646MIPI_write_cmos_sensor(0x5182, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5183, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5184, 0x25);//
	OV5646MIPI_write_cmos_sensor(0x5185, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5186, 0x0d);//
	OV5646MIPI_write_cmos_sensor(0x5187, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x5188, 0x0d);//
	OV5646MIPI_write_cmos_sensor(0x5189, 0x75);//0x75
	OV5646MIPI_write_cmos_sensor(0x518a, 0x57);//
	OV5646MIPI_write_cmos_sensor(0x518b, 0xcc);//0xcc
	OV5646MIPI_write_cmos_sensor(0x518c, 0x81);//
	OV5646MIPI_write_cmos_sensor(0x518d, 0x3b);//
	OV5646MIPI_write_cmos_sensor(0x518e, 0x31);//
	OV5646MIPI_write_cmos_sensor(0x518f, 0x52);//
	OV5646MIPI_write_cmos_sensor(0x5190, 0x3d);//
	OV5646MIPI_write_cmos_sensor(0x5191, 0xf8);//
	OV5646MIPI_write_cmos_sensor(0x5192, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5193, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x5194, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5195, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5196, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5197, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5198, 0x05);//
	OV5646MIPI_write_cmos_sensor(0x5199, 0x52);//
	OV5646MIPI_write_cmos_sensor(0x519a, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519b, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x519c, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519d, 0xd9);//
	OV5646MIPI_write_cmos_sensor(0x519e, 0x38);//
*/
	OV5646MIPI_write_cmos_sensor(0x5180, 0xff);//awb
	OV5646MIPI_write_cmos_sensor(0x5181, 0xf6);//f2
	OV5646MIPI_write_cmos_sensor(0x5182, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5183, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5184, 0x25);//
	OV5646MIPI_write_cmos_sensor(0x5185, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5186, 0x0f);//
	OV5646MIPI_write_cmos_sensor(0x5187, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x5188, 0x0f);//
	OV5646MIPI_write_cmos_sensor(0x5189, 0x7e);//0x75
	OV5646MIPI_write_cmos_sensor(0x518a, 0x5b);//
	OV5646MIPI_write_cmos_sensor(0x518b, 0xc0);//0xcc
	OV5646MIPI_write_cmos_sensor(0x518c, 0x8a);//
	OV5646MIPI_write_cmos_sensor(0x518d, 0x3c);//
	OV5646MIPI_write_cmos_sensor(0x518e, 0x31);//
	OV5646MIPI_write_cmos_sensor(0x518f, 0x57);//
	OV5646MIPI_write_cmos_sensor(0x5190, 0x47);//
	OV5646MIPI_write_cmos_sensor(0x5191, 0xf8);//
	OV5646MIPI_write_cmos_sensor(0x5192, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5193, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x5194, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5195, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5196, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5197, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5198, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5199, 0x35);//
	OV5646MIPI_write_cmos_sensor(0x519a, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519b, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x519c, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519d, 0xef);//
	OV5646MIPI_write_cmos_sensor(0x519e, 0x38);//
/*
	OV5646MIPI_write_cmos_sensor(0x5180, 0xff);//awb
	OV5646MIPI_write_cmos_sensor(0x5181, 0x50);//
	OV5646MIPI_write_cmos_sensor(0x5182, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x5183, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5184, 0x25);//
	OV5646MIPI_write_cmos_sensor(0x5185, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5186, 0x0e);// 
	OV5646MIPI_write_cmos_sensor(0x5187, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x5188, 0x0d);// 
	OV5646MIPI_write_cmos_sensor(0x5189, 0x75);//0x75
	OV5646MIPI_write_cmos_sensor(0x518a, 0x5e);//
	OV5646MIPI_write_cmos_sensor(0x518b, 0xb0);//0xcc
	OV5646MIPI_write_cmos_sensor(0x518c, 0x1b);//
	OV5646MIPI_write_cmos_sensor(0x518d, 0x3d);//
	OV5646MIPI_write_cmos_sensor(0x518e, 0x33);//
	OV5646MIPI_write_cmos_sensor(0x518f, 0x51);//
	OV5646MIPI_write_cmos_sensor(0x5190, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x5191, 0xf8);//
	OV5646MIPI_write_cmos_sensor(0x5192, 0x04);// 
	OV5646MIPI_write_cmos_sensor(0x5193, 0x70);//
	OV5646MIPI_write_cmos_sensor(0x5194, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5195, 0xf0);//
	OV5646MIPI_write_cmos_sensor(0x5196, 0x03);// 
	OV5646MIPI_write_cmos_sensor(0x5197, 0x01);// 
	OV5646MIPI_write_cmos_sensor(0x5198, 0x05);// 
	OV5646MIPI_write_cmos_sensor(0x5199, 0x3e);//
	OV5646MIPI_write_cmos_sensor(0x519a, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x519b, 0x00);// 
	OV5646MIPI_write_cmos_sensor(0x519c, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x519d, 0x4b);//
	OV5646MIPI_write_cmos_sensor(0x519e, 0x38);//
*/
#endif
        //CCM
	OV5646MIPI_write_cmos_sensor(0x5381, 0x21);//ccm
	OV5646MIPI_write_cmos_sensor(0x5382, 0x54);
	OV5646MIPI_write_cmos_sensor(0x5383, 0x15);
	OV5646MIPI_write_cmos_sensor(0x5384, 0x08);
	OV5646MIPI_write_cmos_sensor(0x5385, 0x75);
	OV5646MIPI_write_cmos_sensor(0x5386, 0x7D);
	OV5646MIPI_write_cmos_sensor(0x5387, 0x81);
	OV5646MIPI_write_cmos_sensor(0x5388, 0x74);
	OV5646MIPI_write_cmos_sensor(0x5389, 0x0D);
	OV5646MIPI_write_cmos_sensor(0x538a, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x538b, 0x98);//

	//From OV Kobe sharpness
	OV5646MIPI_write_cmos_sensor(0x5308, 0x35);
	OV5646MIPI_write_cmos_sensor(0x5300, 0x08);//	; sharpen MT th1
	OV5646MIPI_write_cmos_sensor(0x5301, 0x48);//	; sharpen MT th2
	OV5646MIPI_write_cmos_sensor(0x5302, 0x28);//	; sharpen MT off1
	OV5646MIPI_write_cmos_sensor(0x5303, 0x20);//	; sharpen MT off2
	OV5646MIPI_write_cmos_sensor(0x5304, 0x08);//	; DNS th1
	OV5646MIPI_write_cmos_sensor(0x5305, 0x30);//	; DNS th2
	OV5646MIPI_write_cmos_sensor(0x5306, 0x08);//	; DNS off1 //0x08 0x10
	OV5646MIPI_write_cmos_sensor(0x5307, 0x10);//	; DNS off2 //0x16 0x20
	OV5646MIPI_write_cmos_sensor(0x5309, 0x08);//	; sharpen TH th1
	OV5646MIPI_write_cmos_sensor(0x530a, 0x30);//	; sharpen TH th2
	OV5646MIPI_write_cmos_sensor(0x530b, 0x04);//	; sharpen TH th2
	OV5646MIPI_write_cmos_sensor(0x530c, 0x06);//	; sharpen TH off2

#if 1
        //gamma setting
	OV5646MIPI_write_cmos_sensor(0x5480, 0x01);//	; bias on
	OV5646MIPI_write_cmos_sensor(0x5481, 0x0a);//	; Y yst 00
	OV5646MIPI_write_cmos_sensor(0x5482, 0x18);//
	OV5646MIPI_write_cmos_sensor(0x5483, 0x30);//
	OV5646MIPI_write_cmos_sensor(0x5484, 0x5a);//
	OV5646MIPI_write_cmos_sensor(0x5485, 0x68);//
	OV5646MIPI_write_cmos_sensor(0x5486, 0x79);//
	OV5646MIPI_write_cmos_sensor(0x5487, 0x83);//
	OV5646MIPI_write_cmos_sensor(0x5488, 0x90);//
	OV5646MIPI_write_cmos_sensor(0x5489, 0x9a);//
	OV5646MIPI_write_cmos_sensor(0x548a, 0xa4);//
	OV5646MIPI_write_cmos_sensor(0x548b, 0xb5);//
	OV5646MIPI_write_cmos_sensor(0x548c, 0xc0);//
	OV5646MIPI_write_cmos_sensor(0x548d, 0xce);//
	OV5646MIPI_write_cmos_sensor(0x548e, 0xda);//
	OV5646MIPI_write_cmos_sensor(0x548f, 0xe8);//
	OV5646MIPI_write_cmos_sensor(0x5490, 0x1d);//
#else
	OV5646MIPI_write_cmos_sensor(0x5480, 0x01);//	; bias on
	OV5646MIPI_write_cmos_sensor(0x5481, 0x08);//	; Y yst 00
	OV5646MIPI_write_cmos_sensor(0x5482, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5483, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x5484, 0x51);//
	OV5646MIPI_write_cmos_sensor(0x5485, 0x65);//
	OV5646MIPI_write_cmos_sensor(0x5486, 0x71);//
	OV5646MIPI_write_cmos_sensor(0x5487, 0x7d);//
	OV5646MIPI_write_cmos_sensor(0x5488, 0x87);//
	OV5646MIPI_write_cmos_sensor(0x5489, 0x91);//
	OV5646MIPI_write_cmos_sensor(0x548a, 0x9a);//
	OV5646MIPI_write_cmos_sensor(0x548b, 0xaa);//
	OV5646MIPI_write_cmos_sensor(0x548c, 0xb8);//
	OV5646MIPI_write_cmos_sensor(0x548d, 0xcd);//
	OV5646MIPI_write_cmos_sensor(0x548e, 0xdd);//
	OV5646MIPI_write_cmos_sensor(0x548f, 0xea);//
	OV5646MIPI_write_cmos_sensor(0x5490, 0x1d);//
/*
	OV5646MIPI_write_cmos_sensor(0x5480, 0x01);//	; bias on
	OV5646MIPI_write_cmos_sensor(0x5481, 0x0a);//	; Y yst 00
	OV5646MIPI_write_cmos_sensor(0x5482, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x5483, 0x2e);//
	OV5646MIPI_write_cmos_sensor(0x5484, 0x57);//
	OV5646MIPI_write_cmos_sensor(0x5485, 0x69);//
	OV5646MIPI_write_cmos_sensor(0x5486, 0x7a);//
	OV5646MIPI_write_cmos_sensor(0x5487, 0x88);//
	OV5646MIPI_write_cmos_sensor(0x5488, 0x93);//
	OV5646MIPI_write_cmos_sensor(0x5489, 0x9f);//
	OV5646MIPI_write_cmos_sensor(0x548a, 0xa9);//
	OV5646MIPI_write_cmos_sensor(0x548b, 0xb7);//
	OV5646MIPI_write_cmos_sensor(0x548c, 0xc0);//
	OV5646MIPI_write_cmos_sensor(0x548d, 0xd1);//
	OV5646MIPI_write_cmos_sensor(0x548e, 0xdf);//
	OV5646MIPI_write_cmos_sensor(0x548f, 0xea);//
	OV5646MIPI_write_cmos_sensor(0x5490, 0x1d);//
*/
#endif
        //DPC laimingji 20130815
	OV5646MIPI_write_cmos_sensor(0x5780, 0xfc);//
	OV5646MIPI_write_cmos_sensor(0x5781, 0x13);//
	OV5646MIPI_write_cmos_sensor(0x5782, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5786, 0x20);//
	OV5646MIPI_write_cmos_sensor(0x5787, 0x40);//
	OV5646MIPI_write_cmos_sensor(0x5788, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x5789, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x578a, 0x02);//
	OV5646MIPI_write_cmos_sensor(0x578b, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x578c, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x578d, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x578e, 0x02);//
	OV5646MIPI_write_cmos_sensor(0x578f, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5790, 0x01);//
#if 0
        //UV
	OV5646MIPI_write_cmos_sensor(0x5580, 0x06);//uv
	OV5646MIPI_write_cmos_sensor(0x5588, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5583, 0x40);//
	OV5646MIPI_write_cmos_sensor(0x5584, 0x30);//
	OV5646MIPI_write_cmos_sensor(0x5589, 0x18);//
	OV5646MIPI_write_cmos_sensor(0x558a, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x558b, 0x88);//
#else
	OV5646MIPI_write_cmos_sensor(0x5580, 0x06);//uv
	OV5646MIPI_write_cmos_sensor(0x5588, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5583, 0x40);//
	OV5646MIPI_write_cmos_sensor(0x5584, 0x20);//
	OV5646MIPI_write_cmos_sensor(0x5589, 0x18);//
	OV5646MIPI_write_cmos_sensor(0x558a, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x558b, 0x3c);//
#endif

#if 1
	//From OV Kobe office.
	OV5646MIPI_write_cmos_sensor(0x5800, 0x1f);//lsc
	OV5646MIPI_write_cmos_sensor(0x5801, 0x19);//
	OV5646MIPI_write_cmos_sensor(0x5802, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5803, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5804, 0x17);//
	OV5646MIPI_write_cmos_sensor(0x5805, 0x20);//
	OV5646MIPI_write_cmos_sensor(0x5806, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5807, 0xb );//
	OV5646MIPI_write_cmos_sensor(0x5808, 0x7 );//
	OV5646MIPI_write_cmos_sensor(0x5809, 0x7 );//
	OV5646MIPI_write_cmos_sensor(0x580a, 0xa );//
	OV5646MIPI_write_cmos_sensor(0x580b, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x580c, 0x9 );//
	OV5646MIPI_write_cmos_sensor(0x580d, 0x4 );//
	OV5646MIPI_write_cmos_sensor(0x580e, 0x1 );//
	OV5646MIPI_write_cmos_sensor(0x580f, 0x1 );//
	OV5646MIPI_write_cmos_sensor(0x5810, 0x4 );//
	OV5646MIPI_write_cmos_sensor(0x5811, 0xc );//
	OV5646MIPI_write_cmos_sensor(0x5812, 0xa );//
	OV5646MIPI_write_cmos_sensor(0x5813, 0x4 );//
	OV5646MIPI_write_cmos_sensor(0x5814, 0x1 );//
	OV5646MIPI_write_cmos_sensor(0x5815, 0x1 );//
	OV5646MIPI_write_cmos_sensor(0x5816, 0x4 );//
	OV5646MIPI_write_cmos_sensor(0x5817, 0xc );//
	OV5646MIPI_write_cmos_sensor(0x5818, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x5819, 0xb );//
	OV5646MIPI_write_cmos_sensor(0x581a, 0x7 );//
	OV5646MIPI_write_cmos_sensor(0x581b, 0x8 );//
	OV5646MIPI_write_cmos_sensor(0x581c, 0xc );//
	OV5646MIPI_write_cmos_sensor(0x581d, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x581e, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x581f, 0x1d);//
	OV5646MIPI_write_cmos_sensor(0x5820, 0x17);//
	OV5646MIPI_write_cmos_sensor(0x5821, 0x18);//
	OV5646MIPI_write_cmos_sensor(0x5822, 0x1c);//
	OV5646MIPI_write_cmos_sensor(0x5823, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x5824, 0x29);//
	OV5646MIPI_write_cmos_sensor(0x5825, 0x57);//
	OV5646MIPI_write_cmos_sensor(0x5826, 0x48);//
	OV5646MIPI_write_cmos_sensor(0x5827, 0x59);//
	OV5646MIPI_write_cmos_sensor(0x5828, 0x4d);//
	OV5646MIPI_write_cmos_sensor(0x5829, 0x25);//
	OV5646MIPI_write_cmos_sensor(0x582a, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x582b, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x582c, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x582d, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x582e, 0x9  );//
	OV5646MIPI_write_cmos_sensor(0x582f, 0x42);//
	OV5646MIPI_write_cmos_sensor(0x5830, 0x81);//
	OV5646MIPI_write_cmos_sensor(0x5831, 0x42);//
	OV5646MIPI_write_cmos_sensor(0x5832, 0x8  );//
	OV5646MIPI_write_cmos_sensor(0x5833, 0x26);//
	OV5646MIPI_write_cmos_sensor(0x5834, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5835, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x5836, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x5837, 0x38);//
	OV5646MIPI_write_cmos_sensor(0x5838, 0x38);//
	OV5646MIPI_write_cmos_sensor(0x5839, 0x39);//
	OV5646MIPI_write_cmos_sensor(0x583a, 0x37);//
	OV5646MIPI_write_cmos_sensor(0x583b, 0x37);//
	OV5646MIPI_write_cmos_sensor(0x583c, 0x6f);//
	OV5646MIPI_write_cmos_sensor(0x583d, 0xaf);//
/*
	//From OV Kobe office.
	OV5646MIPI_write_cmos_sensor(0x5800, 0x1C);//lsc
	OV5646MIPI_write_cmos_sensor(0x5801, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5802, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x5803, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5804, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5805, 0x1A);//
	OV5646MIPI_write_cmos_sensor(0x5806, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5807, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x5808, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5809, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x580a, 0x09);//
	OV5646MIPI_write_cmos_sensor(0x580b, 0x0F);//
	OV5646MIPI_write_cmos_sensor(0x580c, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x580d, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x580e, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x580f, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5810, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5811, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x5812, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x5813, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5814, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5815, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5816, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5817, 0x0B);//
	OV5646MIPI_write_cmos_sensor(0x5818, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5819, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x581a, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x581b, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x581c, 0x0B);//
	OV5646MIPI_write_cmos_sensor(0x581d, 0x0E);//
	OV5646MIPI_write_cmos_sensor(0x581e, 0x1C);//
	OV5646MIPI_write_cmos_sensor(0x581f, 0x15);//
	OV5646MIPI_write_cmos_sensor(0x5820, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5821, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x5822, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5823, 0x20);//
	OV5646MIPI_write_cmos_sensor(0x5824, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x5825, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x5826, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5827, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x5828, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x5829, 0x43);//
	OV5646MIPI_write_cmos_sensor(0x582a, 0x21);//
	OV5646MIPI_write_cmos_sensor(0x582b, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x582c, 0x21);//
	OV5646MIPI_write_cmos_sensor(0x582d, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x582e, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x582f, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x5830, 0x81);//
	OV5646MIPI_write_cmos_sensor(0x5831, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x5832, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5833, 0x43);//
	OV5646MIPI_write_cmos_sensor(0x5834, 0x21);//
	OV5646MIPI_write_cmos_sensor(0x5835, 0x21);//
	OV5646MIPI_write_cmos_sensor(0x5836, 0x21);//
	OV5646MIPI_write_cmos_sensor(0x5837, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x5838, 0x41);//
	OV5646MIPI_write_cmos_sensor(0x5839, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x583a, 0x45);//
	OV5646MIPI_write_cmos_sensor(0x583b, 0x23);//
	OV5646MIPI_write_cmos_sensor(0x583c, 0x45);//
	OV5646MIPI_write_cmos_sensor(0x583d, 0xAF);//
*/
#else
/*
	//From OV Kobe D65.
	OV5646MIPI_write_cmos_sensor(0x5800, 0x1e);//lsc
	OV5646MIPI_write_cmos_sensor(0x5801, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5802, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5803, 0x15);//
	OV5646MIPI_write_cmos_sensor(0x5804, 0x1e);//
	OV5646MIPI_write_cmos_sensor(0x5805, 0x27);//
	OV5646MIPI_write_cmos_sensor(0x5806, 0x0f);//
	OV5646MIPI_write_cmos_sensor(0x5807, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x5808, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5809, 0x09);//
	OV5646MIPI_write_cmos_sensor(0x580a, 0x0f);//
	OV5646MIPI_write_cmos_sensor(0x580b, 0x13);//
	OV5646MIPI_write_cmos_sensor(0x580c, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x580d, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x580e, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x580f, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5810, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5811, 0x0e);//
	OV5646MIPI_write_cmos_sensor(0x5812, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x5813, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5814, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5815, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5816, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5817, 0x0e);//
	OV5646MIPI_write_cmos_sensor(0x5818, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x5819, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x581a, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x581b, 0x09);//
	OV5646MIPI_write_cmos_sensor(0x581c, 0x0d);//
	OV5646MIPI_write_cmos_sensor(0x581d, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x581e, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x581f, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5820, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5821, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x5822, 0x17);//
	OV5646MIPI_write_cmos_sensor(0x5823, 0x21);//
	OV5646MIPI_write_cmos_sensor(0x5824, 0x46);//
	OV5646MIPI_write_cmos_sensor(0x5825, 0x0c);//
	OV5646MIPI_write_cmos_sensor(0x5826, 0x0e);//
	OV5646MIPI_write_cmos_sensor(0x5827, 0x2c);//
	OV5646MIPI_write_cmos_sensor(0x5828, 0x26);//
	OV5646MIPI_write_cmos_sensor(0x5829, 0x2c);//
	OV5646MIPI_write_cmos_sensor(0x582a, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x582b, 0x26);//
	OV5646MIPI_write_cmos_sensor(0x582c, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x582d, 0x0a);//
	OV5646MIPI_write_cmos_sensor(0x582e, 0x2c);//
	OV5646MIPI_write_cmos_sensor(0x582f, 0x44);//
	OV5646MIPI_write_cmos_sensor(0x5830, 0x40);//
	OV5646MIPI_write_cmos_sensor(0x5831, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5832, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x5833, 0x2A);//
	OV5646MIPI_write_cmos_sensor(0x5834, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x5835, 0x26);//
	OV5646MIPI_write_cmos_sensor(0x5836, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x5837, 0x2a);//
	OV5646MIPI_write_cmos_sensor(0x5838, 0x46);//
	OV5646MIPI_write_cmos_sensor(0x5839, 0x2c);//
	OV5646MIPI_write_cmos_sensor(0x583a, 0x2c);//
	OV5646MIPI_write_cmos_sensor(0x583b, 0x2a);//
	OV5646MIPI_write_cmos_sensor(0x583c, 0x28);//
	OV5646MIPI_write_cmos_sensor(0x583d, 0xcc);//
*/
	//From OV Kobe CWF.
	OV5646MIPI_write_cmos_sensor(0x5800, 0x19);//lsc
	OV5646MIPI_write_cmos_sensor(0x5801, 0x0E);//
	OV5646MIPI_write_cmos_sensor(0x5802, 0x0F);//
	OV5646MIPI_write_cmos_sensor(0x5803, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x5804, 0x1A);//
	OV5646MIPI_write_cmos_sensor(0x5805, 0x2C);//
	OV5646MIPI_write_cmos_sensor(0x5806, 0x0E);//
	OV5646MIPI_write_cmos_sensor(0x5807, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x5808, 0x05);//
	OV5646MIPI_write_cmos_sensor(0x5809, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x580a, 0x0B);//
	OV5646MIPI_write_cmos_sensor(0x580b, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x580c, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x580d, 0x02);//
	OV5646MIPI_write_cmos_sensor(0x580e, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x580f, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5810, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5811, 0x0F);//
	OV5646MIPI_write_cmos_sensor(0x5812, 0x0A);//
	OV5646MIPI_write_cmos_sensor(0x5813, 0x03);//
	OV5646MIPI_write_cmos_sensor(0x5814, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5815, 0x01);//
	OV5646MIPI_write_cmos_sensor(0x5816, 0x06);//
	OV5646MIPI_write_cmos_sensor(0x5817, 0x0E);//
	OV5646MIPI_write_cmos_sensor(0x5818, 0x0E);//
	OV5646MIPI_write_cmos_sensor(0x5819, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x581a, 0x07);//
	OV5646MIPI_write_cmos_sensor(0x581b, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x581c, 0x0E);//
	OV5646MIPI_write_cmos_sensor(0x581d, 0x14);//
	OV5646MIPI_write_cmos_sensor(0x581e, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x581f, 0x11);//
	OV5646MIPI_write_cmos_sensor(0x5820, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x5821, 0x13);//
	OV5646MIPI_write_cmos_sensor(0x5822, 0x16);//
	OV5646MIPI_write_cmos_sensor(0x5823, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x5824, 0x64);//
	OV5646MIPI_write_cmos_sensor(0x5825, 0x46);//
	OV5646MIPI_write_cmos_sensor(0x5826, 0x26);//
	OV5646MIPI_write_cmos_sensor(0x5827, 0x46);//
	OV5646MIPI_write_cmos_sensor(0x5828, 0x66);//
	OV5646MIPI_write_cmos_sensor(0x5829, 0x44);//
	OV5646MIPI_write_cmos_sensor(0x582a, 0x64);//
	OV5646MIPI_write_cmos_sensor(0x582b, 0x62);//
	OV5646MIPI_write_cmos_sensor(0x582c, 0x42);//
	OV5646MIPI_write_cmos_sensor(0x582d, 0x24);//
	OV5646MIPI_write_cmos_sensor(0x582e, 0x44);//
	OV5646MIPI_write_cmos_sensor(0x582f, 0x82);//
	OV5646MIPI_write_cmos_sensor(0x5830, 0xA0);//
	OV5646MIPI_write_cmos_sensor(0x5831, 0x40);//
	OV5646MIPI_write_cmos_sensor(0x5832, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x5833, 0x64);//
	OV5646MIPI_write_cmos_sensor(0x5834, 0x42);//
	OV5646MIPI_write_cmos_sensor(0x5835, 0x22);//
	OV5646MIPI_write_cmos_sensor(0x5836, 0x42);//
	OV5646MIPI_write_cmos_sensor(0x5837, 0x46);//
	OV5646MIPI_write_cmos_sensor(0x5838, 0x86);//
	OV5646MIPI_write_cmos_sensor(0x5839, 0x66);//
	OV5646MIPI_write_cmos_sensor(0x583a, 0x68);//
	OV5646MIPI_write_cmos_sensor(0x583b, 0x66);//
	OV5646MIPI_write_cmos_sensor(0x583c, 0x88);//
	OV5646MIPI_write_cmos_sensor(0x583d, 0x8E);//
#endif
#if 0
	/*AE Weight - CenterAverage*/ 
	OV5646MIPI_write_cmos_sensor(0x5688, 0x11);
	OV5646MIPI_write_cmos_sensor(0x5689, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568a, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568b, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568c, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568d, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568e, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568f, 0x11);
//#else
	OV5646MIPI_write_cmos_sensor(0x5688, 0x11);
	OV5646MIPI_write_cmos_sensor(0x5689, 0x11);
	OV5646MIPI_write_cmos_sensor(0x568a, 0x41);
	OV5646MIPI_write_cmos_sensor(0x568b, 0x14);
	OV5646MIPI_write_cmos_sensor(0x568c, 0x42);
	OV5646MIPI_write_cmos_sensor(0x568d, 0x24);
	OV5646MIPI_write_cmos_sensor(0x568e, 0x21);
	OV5646MIPI_write_cmos_sensor(0x568f, 0x12);
#endif

	OV5646MIPI_write_cmos_sensor(0x583e, 0x10);//
	OV5646MIPI_write_cmos_sensor(0x583f, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x5840, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5025, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
	OV5646MIPI_write_cmos_sensor(0x3a0f, 0x29);//	; AEC in H
	OV5646MIPI_write_cmos_sensor(0x3a10, 0x23);//	; AEC in L
	OV5646MIPI_write_cmos_sensor(0x3a1b, 0x29);//	; AEC out H
	OV5646MIPI_write_cmos_sensor(0x3a1e, 0x23);//	; AEC out L
	OV5646MIPI_write_cmos_sensor(0x3a11, 0x53);//	; control zone H
	OV5646MIPI_write_cmos_sensor(0x3a1f, 0x12);//	; control zone L	
	
	
	
	#if 0
    OV5646MIPI_write_cmos_sensor(0x3000 ,0x20);
    //OV5646MIPI_write_cmos_sensor(0x3001 ,0x00); 
    OV5646MIPI_write_cmos_sensor(0x3004 ,0xff);     
    //OV5646MIPI_write_cmos_sensor(0x3005 ,0x40); 
    OV5646MIPI_write_cmos_sensor(0x8000 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8001 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x8002 ,0xd6);
    OV5646MIPI_write_cmos_sensor(0x8003 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8004 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8005 ,0x8d);
    OV5646MIPI_write_cmos_sensor(0x8006 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8007 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8008 ,0xdf);
    OV5646MIPI_write_cmos_sensor(0x8009 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x800a ,0x93);
    OV5646MIPI_write_cmos_sensor(0x800b ,0xf9);
    OV5646MIPI_write_cmos_sensor(0x800c ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x800d ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x800e ,0x93);
    OV5646MIPI_write_cmos_sensor(0x800f ,0xff);
    OV5646MIPI_write_cmos_sensor(0x8010 ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x8011 ,0x99);
    OV5646MIPI_write_cmos_sensor(0x8012 ,0x50);
    OV5646MIPI_write_cmos_sensor(0x8013 ,0x03);
    OV5646MIPI_write_cmos_sensor(0x8014 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8015 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x8016 ,0xd5);
    OV5646MIPI_write_cmos_sensor(0x8017 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8018 ,0x3a);
    OV5646MIPI_write_cmos_sensor(0x8019 ,0x1b);
    OV5646MIPI_write_cmos_sensor(0x801a ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x801b ,0xfe);
    OV5646MIPI_write_cmos_sensor(0x801c ,0x90);
    OV5646MIPI_write_cmos_sensor(0x801d ,0x01);
    OV5646MIPI_write_cmos_sensor(0x801e ,0xe2);
    OV5646MIPI_write_cmos_sensor(0x801f ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8020 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8021 ,0xfd);
    OV5646MIPI_write_cmos_sensor(0x8022 ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x8023 ,0x94);
    OV5646MIPI_write_cmos_sensor(0x8024 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x8025 ,0x40);
    OV5646MIPI_write_cmos_sensor(0x8026 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8027 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x8028 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8029 ,0xad);
    OV5646MIPI_write_cmos_sensor(0x802a ,0x06);
    OV5646MIPI_write_cmos_sensor(0x802b ,0xee);
    OV5646MIPI_write_cmos_sensor(0x802c ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x802d ,0x94);
    OV5646MIPI_write_cmos_sensor(0x802e ,0x08);
    OV5646MIPI_write_cmos_sensor(0x802f ,0x40);
    OV5646MIPI_write_cmos_sensor(0x8030 ,0x06);
    OV5646MIPI_write_cmos_sensor(0x8031 ,0xee);
    OV5646MIPI_write_cmos_sensor(0x8032 ,0x24);
    OV5646MIPI_write_cmos_sensor(0x8033 ,0xf8);
    OV5646MIPI_write_cmos_sensor(0x8034 ,0xfc);
    OV5646MIPI_write_cmos_sensor(0x8035 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x8036 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8037 ,0x7c);
    OV5646MIPI_write_cmos_sensor(0x8038 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x8039 ,0xae);
    OV5646MIPI_write_cmos_sensor(0x803a ,0x04);
    OV5646MIPI_write_cmos_sensor(0x803b ,0x90);
    OV5646MIPI_write_cmos_sensor(0x803c ,0x01);
    OV5646MIPI_write_cmos_sensor(0x803d ,0xe1);
    OV5646MIPI_write_cmos_sensor(0x803e ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x803f ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8040 ,0xfc);
    OV5646MIPI_write_cmos_sensor(0x8041 ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x8042 ,0x94);
    OV5646MIPI_write_cmos_sensor(0x8043 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x8044 ,0x40);
    OV5646MIPI_write_cmos_sensor(0x8045 ,0x04);
    OV5646MIPI_write_cmos_sensor(0x8046 ,0xae);
    OV5646MIPI_write_cmos_sensor(0x8047 ,0x04);
    OV5646MIPI_write_cmos_sensor(0x8048 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x8049 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x804a ,0x90);
    OV5646MIPI_write_cmos_sensor(0x804b ,0x35);
    OV5646MIPI_write_cmos_sensor(0x804c ,0x0b);
    OV5646MIPI_write_cmos_sensor(0x804d ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x804e ,0xfc);
    OV5646MIPI_write_cmos_sensor(0x804f ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8050 ,0x56);
    OV5646MIPI_write_cmos_sensor(0x8051 ,0xa1);
    OV5646MIPI_write_cmos_sensor(0x8052 ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x8053 ,0xfb);
    OV5646MIPI_write_cmos_sensor(0x8054 ,0xec);
    OV5646MIPI_write_cmos_sensor(0x8055 ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x8056 ,0x9f);
    OV5646MIPI_write_cmos_sensor(0x8057 ,0x40);
    OV5646MIPI_write_cmos_sensor(0x8058 ,0x43);
    OV5646MIPI_write_cmos_sensor(0x8059 ,0xeb);
    OV5646MIPI_write_cmos_sensor(0x805a ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x805b ,0x9e);
    OV5646MIPI_write_cmos_sensor(0x805c ,0x50);
    OV5646MIPI_write_cmos_sensor(0x805d ,0x3e);
    OV5646MIPI_write_cmos_sensor(0x805e ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x805f ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x8060 ,0x64);
    OV5646MIPI_write_cmos_sensor(0x8061 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8062 ,0x60);
    OV5646MIPI_write_cmos_sensor(0x8063 ,0x05);
    OV5646MIPI_write_cmos_sensor(0x8064 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8065 ,0xf5);
    OV5646MIPI_write_cmos_sensor(0x8066 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x8067 ,0xf5);
    OV5646MIPI_write_cmos_sensor(0x8068 ,0x09);
    OV5646MIPI_write_cmos_sensor(0x8069 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x806a ,0x09);
    OV5646MIPI_write_cmos_sensor(0x806b ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x806c ,0x94);
    OV5646MIPI_write_cmos_sensor(0x806d ,0xff);
    OV5646MIPI_write_cmos_sensor(0x806e ,0x50);
    OV5646MIPI_write_cmos_sensor(0x806f ,0x08);
    OV5646MIPI_write_cmos_sensor(0x8070 ,0xaf);
    OV5646MIPI_write_cmos_sensor(0x8071 ,0x09);
    OV5646MIPI_write_cmos_sensor(0x8072 ,0xef);
    OV5646MIPI_write_cmos_sensor(0x8073 ,0x24);
    OV5646MIPI_write_cmos_sensor(0x8074 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8075 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x8076 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x8077 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8078 ,0x7f);
    OV5646MIPI_write_cmos_sensor(0x8079 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x807a ,0x8f);
    OV5646MIPI_write_cmos_sensor(0x807b ,0x09);
    OV5646MIPI_write_cmos_sensor(0x807c ,0x90);
    OV5646MIPI_write_cmos_sensor(0x807d ,0x01);
    OV5646MIPI_write_cmos_sensor(0x807e ,0xe3);
    OV5646MIPI_write_cmos_sensor(0x807f ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8080 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8081 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x8082 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x8083 ,0x09);
    OV5646MIPI_write_cmos_sensor(0x8084 ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x8085 ,0x9f);
    OV5646MIPI_write_cmos_sensor(0x8086 ,0x40);
    OV5646MIPI_write_cmos_sensor(0x8087 ,0x10);
    OV5646MIPI_write_cmos_sensor(0x8088 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8089 ,0x51);
    OV5646MIPI_write_cmos_sensor(0x808a ,0x83);
    OV5646MIPI_write_cmos_sensor(0x808b ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x808c ,0x44);
    OV5646MIPI_write_cmos_sensor(0x808d ,0x80);
    OV5646MIPI_write_cmos_sensor(0x808e ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x808f ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8090 ,0x51);
    OV5646MIPI_write_cmos_sensor(0x8091 ,0x91);
    OV5646MIPI_write_cmos_sensor(0x8092 ,0x74);
    OV5646MIPI_write_cmos_sensor(0x8093 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x8094 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x8095 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8096 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8097 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x8098 ,0x75);
    OV5646MIPI_write_cmos_sensor(0x8099 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x809a ,0x02);
    OV5646MIPI_write_cmos_sensor(0x809b ,0x22);
    OV5646MIPI_write_cmos_sensor(0x809c ,0xec);
    OV5646MIPI_write_cmos_sensor(0x809d ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x809e ,0x99);
    OV5646MIPI_write_cmos_sensor(0x809f ,0x40);
    OV5646MIPI_write_cmos_sensor(0x80a0 ,0x04);
    OV5646MIPI_write_cmos_sensor(0x80a1 ,0xeb);
    OV5646MIPI_write_cmos_sensor(0x80a2 ,0x9d);
    OV5646MIPI_write_cmos_sensor(0x80a3 ,0x40);
    OV5646MIPI_write_cmos_sensor(0x80a4 ,0x30);
    OV5646MIPI_write_cmos_sensor(0x80a5 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x80a6 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x80a7 ,0x64);
    OV5646MIPI_write_cmos_sensor(0x80a8 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80a9 ,0x60);
    OV5646MIPI_write_cmos_sensor(0x80aa ,0x05);
    OV5646MIPI_write_cmos_sensor(0x80ab ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80ac ,0xf5);
    OV5646MIPI_write_cmos_sensor(0x80ad ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80ae ,0xf5);
    OV5646MIPI_write_cmos_sensor(0x80af ,0x09);
    OV5646MIPI_write_cmos_sensor(0x80b0 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x80b1 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80b2 ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x80b3 ,0x94);
    OV5646MIPI_write_cmos_sensor(0x80b4 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x80b5 ,0x50);
    OV5646MIPI_write_cmos_sensor(0x80b6 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80b7 ,0xaf);
    OV5646MIPI_write_cmos_sensor(0x80b8 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80b9 ,0xef);
    OV5646MIPI_write_cmos_sensor(0x80ba ,0x24);
    OV5646MIPI_write_cmos_sensor(0x80bb ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80bc ,0xff);
    OV5646MIPI_write_cmos_sensor(0x80bd ,0x80);
    OV5646MIPI_write_cmos_sensor(0x80be ,0x02);
    OV5646MIPI_write_cmos_sensor(0x80bf ,0x7f);
    OV5646MIPI_write_cmos_sensor(0x80c0 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x80c1 ,0x8f);
    OV5646MIPI_write_cmos_sensor(0x80c2 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80c3 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x80c4 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80c5 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80c6 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80c7 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x80c8 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x80c9 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x80ca ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80cb ,0xd3);
    OV5646MIPI_write_cmos_sensor(0x80cc ,0x9f);
    OV5646MIPI_write_cmos_sensor(0x80cd ,0x40);
    OV5646MIPI_write_cmos_sensor(0x80ce ,0x03);
    OV5646MIPI_write_cmos_sensor(0x80cf ,0x12);
    OV5646MIPI_write_cmos_sensor(0x80d0 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80d1 ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x80d2 ,0x75);
    OV5646MIPI_write_cmos_sensor(0x80d3 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x80d4 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80d5 ,0x22);
    OV5646MIPI_write_cmos_sensor(0x80d6 ,0x78);
    OV5646MIPI_write_cmos_sensor(0x80d7 ,0x7f);
    OV5646MIPI_write_cmos_sensor(0x80d8 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80d9 ,0xf6);
    OV5646MIPI_write_cmos_sensor(0x80da ,0xd8);
    OV5646MIPI_write_cmos_sensor(0x80db ,0xfd);
    OV5646MIPI_write_cmos_sensor(0x80dc ,0x75);
    OV5646MIPI_write_cmos_sensor(0x80dd ,0x81);
    OV5646MIPI_write_cmos_sensor(0x80de ,0x20);
    OV5646MIPI_write_cmos_sensor(0x80df ,0x02);
    OV5646MIPI_write_cmos_sensor(0x80e0 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80e1 ,0x1d);
    OV5646MIPI_write_cmos_sensor(0x80e2 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x80e3 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80e4 ,0x62);
    OV5646MIPI_write_cmos_sensor(0x80e5 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80e6 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x80e7 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x80e8 ,0xf8);
    OV5646MIPI_write_cmos_sensor(0x80e9 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80ea ,0x93);
    OV5646MIPI_write_cmos_sensor(0x80eb ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x80ec ,0x40);
    OV5646MIPI_write_cmos_sensor(0x80ed ,0x03);
    OV5646MIPI_write_cmos_sensor(0x80ee ,0xf6);
    OV5646MIPI_write_cmos_sensor(0x80ef ,0x80);
    OV5646MIPI_write_cmos_sensor(0x80f0 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x80f1 ,0xf2);
    OV5646MIPI_write_cmos_sensor(0x80f2 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x80f3 ,0xdf);
    OV5646MIPI_write_cmos_sensor(0x80f4 ,0xf4);
    OV5646MIPI_write_cmos_sensor(0x80f5 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x80f6 ,0x29);
    OV5646MIPI_write_cmos_sensor(0x80f7 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x80f8 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x80f9 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x80fa ,0xf8);
    OV5646MIPI_write_cmos_sensor(0x80fb ,0x54);
    OV5646MIPI_write_cmos_sensor(0x80fc ,0x07);
    OV5646MIPI_write_cmos_sensor(0x80fd ,0x24);
    OV5646MIPI_write_cmos_sensor(0x80fe ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x80ff ,0xc8);
    OV5646MIPI_write_cmos_sensor(0x8100 ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x8101 ,0x33);
    OV5646MIPI_write_cmos_sensor(0x8102 ,0xc4);
    OV5646MIPI_write_cmos_sensor(0x8103 ,0x54);
    OV5646MIPI_write_cmos_sensor(0x8104 ,0x0f);
    OV5646MIPI_write_cmos_sensor(0x8105 ,0x44);
    OV5646MIPI_write_cmos_sensor(0x8106 ,0x20);
    OV5646MIPI_write_cmos_sensor(0x8107 ,0xc8);
    OV5646MIPI_write_cmos_sensor(0x8108 ,0x83);
    OV5646MIPI_write_cmos_sensor(0x8109 ,0x40);
    OV5646MIPI_write_cmos_sensor(0x810a ,0x04);
    OV5646MIPI_write_cmos_sensor(0x810b ,0xf4);
    OV5646MIPI_write_cmos_sensor(0x810c ,0x56);
    OV5646MIPI_write_cmos_sensor(0x810d ,0x80);
    OV5646MIPI_write_cmos_sensor(0x810e ,0x01);
    OV5646MIPI_write_cmos_sensor(0x810f ,0x46);
    OV5646MIPI_write_cmos_sensor(0x8110 ,0xf6);
    OV5646MIPI_write_cmos_sensor(0x8111 ,0xdf);
    OV5646MIPI_write_cmos_sensor(0x8112 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8113 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x8114 ,0x0b);
    OV5646MIPI_write_cmos_sensor(0x8115 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8116 ,0x02);
    OV5646MIPI_write_cmos_sensor(0x8117 ,0x04);
    OV5646MIPI_write_cmos_sensor(0x8118 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x8119 ,0x10);
    OV5646MIPI_write_cmos_sensor(0x811a ,0x20);
    OV5646MIPI_write_cmos_sensor(0x811b ,0x40);
    OV5646MIPI_write_cmos_sensor(0x811c ,0x80);
    OV5646MIPI_write_cmos_sensor(0x811d ,0x90);
    OV5646MIPI_write_cmos_sensor(0x811e ,0x01);
    OV5646MIPI_write_cmos_sensor(0x811f ,0xd5);
    OV5646MIPI_write_cmos_sensor(0x8120 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8121 ,0x7e);
    OV5646MIPI_write_cmos_sensor(0x8122 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8123 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8124 ,0x60);
    OV5646MIPI_write_cmos_sensor(0x8125 ,0xbc);
    OV5646MIPI_write_cmos_sensor(0x8126 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8127 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x8128 ,0x54);
    OV5646MIPI_write_cmos_sensor(0x8129 ,0x3f);
    OV5646MIPI_write_cmos_sensor(0x812a ,0x30);
    OV5646MIPI_write_cmos_sensor(0x812b ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x812c ,0x09);
    OV5646MIPI_write_cmos_sensor(0x812d ,0x54);
    OV5646MIPI_write_cmos_sensor(0x812e ,0x1f);
    OV5646MIPI_write_cmos_sensor(0x812f ,0xfe);
    OV5646MIPI_write_cmos_sensor(0x8130 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8131 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8132 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8133 ,0x60);
    OV5646MIPI_write_cmos_sensor(0x8134 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8135 ,0x0e);
    OV5646MIPI_write_cmos_sensor(0x8136 ,0xcf);
    OV5646MIPI_write_cmos_sensor(0x8137 ,0x54);
    OV5646MIPI_write_cmos_sensor(0x8138 ,0xc0);
    OV5646MIPI_write_cmos_sensor(0x8139 ,0x25);
    OV5646MIPI_write_cmos_sensor(0x813a ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x813b ,0x60);
    OV5646MIPI_write_cmos_sensor(0x813c ,0xa8);
    OV5646MIPI_write_cmos_sensor(0x813d ,0x40);
    OV5646MIPI_write_cmos_sensor(0x813e ,0xb8);
    OV5646MIPI_write_cmos_sensor(0x813f ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8140 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8141 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8142 ,0xfa);
    OV5646MIPI_write_cmos_sensor(0x8143 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8144 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8145 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8146 ,0xf8);
    OV5646MIPI_write_cmos_sensor(0x8147 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8148 ,0x93);
    OV5646MIPI_write_cmos_sensor(0x8149 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x814a ,0xc8);
    OV5646MIPI_write_cmos_sensor(0x814b ,0xc5);
    OV5646MIPI_write_cmos_sensor(0x814c ,0x82);
    OV5646MIPI_write_cmos_sensor(0x814d ,0xc8);
    OV5646MIPI_write_cmos_sensor(0x814e ,0xca);
    OV5646MIPI_write_cmos_sensor(0x814f ,0xc5);
    OV5646MIPI_write_cmos_sensor(0x8150 ,0x83);
    OV5646MIPI_write_cmos_sensor(0x8151 ,0xca);
    OV5646MIPI_write_cmos_sensor(0x8152 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x8153 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8154 ,0xc8);
    OV5646MIPI_write_cmos_sensor(0x8155 ,0xc5);
    OV5646MIPI_write_cmos_sensor(0x8156 ,0x82);
    OV5646MIPI_write_cmos_sensor(0x8157 ,0xc8);
    OV5646MIPI_write_cmos_sensor(0x8158 ,0xca);
    OV5646MIPI_write_cmos_sensor(0x8159 ,0xc5);
    OV5646MIPI_write_cmos_sensor(0x815a ,0x83);
    OV5646MIPI_write_cmos_sensor(0x815b ,0xca);
    OV5646MIPI_write_cmos_sensor(0x815c ,0xdf);
    OV5646MIPI_write_cmos_sensor(0x815d ,0xe9);
    OV5646MIPI_write_cmos_sensor(0x815e ,0xde);
    OV5646MIPI_write_cmos_sensor(0x815f ,0xe7);
    OV5646MIPI_write_cmos_sensor(0x8160 ,0x80);
    OV5646MIPI_write_cmos_sensor(0x8161 ,0xbe);
    OV5646MIPI_write_cmos_sensor(0x8162 ,0x75);
    OV5646MIPI_write_cmos_sensor(0x8163 ,0x89);
    OV5646MIPI_write_cmos_sensor(0x8164 ,0x03);
    OV5646MIPI_write_cmos_sensor(0x8165 ,0x75);
    OV5646MIPI_write_cmos_sensor(0x8166 ,0xa8);
    OV5646MIPI_write_cmos_sensor(0x8167 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8168 ,0x75);
    OV5646MIPI_write_cmos_sensor(0x8169 ,0xb8);
    OV5646MIPI_write_cmos_sensor(0x816a ,0x04);
    OV5646MIPI_write_cmos_sensor(0x816b ,0x90);
    OV5646MIPI_write_cmos_sensor(0x816c ,0x3f);
    OV5646MIPI_write_cmos_sensor(0x816d ,0x01);
    OV5646MIPI_write_cmos_sensor(0x816e ,0x74);
    OV5646MIPI_write_cmos_sensor(0x816f ,0x20);
    OV5646MIPI_write_cmos_sensor(0x8170 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x8171 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x8172 ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x8173 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x8174 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8175 ,0x3f);
    OV5646MIPI_write_cmos_sensor(0x8176 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x8177 ,0x74);
    OV5646MIPI_write_cmos_sensor(0x8178 ,0xff);
    OV5646MIPI_write_cmos_sensor(0x8179 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x817a ,0xd2);
    OV5646MIPI_write_cmos_sensor(0x817b ,0xaf);
    OV5646MIPI_write_cmos_sensor(0x817c ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x817d ,0x0b);
    OV5646MIPI_write_cmos_sensor(0x817e ,0x45);
    OV5646MIPI_write_cmos_sensor(0x817f ,0x0a);
    OV5646MIPI_write_cmos_sensor(0x8180 ,0x60);
    OV5646MIPI_write_cmos_sensor(0x8181 ,0xfa);
    OV5646MIPI_write_cmos_sensor(0x8182 ,0x12);
    OV5646MIPI_write_cmos_sensor(0x8183 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x8184 ,0xb1);
    OV5646MIPI_write_cmos_sensor(0x8185 ,0x30);
    OV5646MIPI_write_cmos_sensor(0x8186 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x8187 ,0xf4);
    OV5646MIPI_write_cmos_sensor(0x8188 ,0x12);
    OV5646MIPI_write_cmos_sensor(0x8189 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x818a ,0x06);
    OV5646MIPI_write_cmos_sensor(0x818b ,0x80);
    OV5646MIPI_write_cmos_sensor(0x818c ,0xef);
    OV5646MIPI_write_cmos_sensor(0x818d ,0xc0);
    OV5646MIPI_write_cmos_sensor(0x818e ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x818f ,0xc0);
    OV5646MIPI_write_cmos_sensor(0x8190 ,0x83);
    OV5646MIPI_write_cmos_sensor(0x8191 ,0xc0);
    OV5646MIPI_write_cmos_sensor(0x8192 ,0x82);
    OV5646MIPI_write_cmos_sensor(0x8193 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x8194 ,0x3f);
    OV5646MIPI_write_cmos_sensor(0x8195 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x8196 ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x8197 ,0xf5);
    OV5646MIPI_write_cmos_sensor(0x8198 ,0x0d);
    OV5646MIPI_write_cmos_sensor(0x8199 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x819a ,0x0d);
    OV5646MIPI_write_cmos_sensor(0x819b ,0x30);
    OV5646MIPI_write_cmos_sensor(0x819c ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x819d ,0x06);
    OV5646MIPI_write_cmos_sensor(0x819e ,0x75);
    OV5646MIPI_write_cmos_sensor(0x819f ,0x0a);
    OV5646MIPI_write_cmos_sensor(0x81a0 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81a1 ,0x75);
    OV5646MIPI_write_cmos_sensor(0x81a2 ,0x0b);
    OV5646MIPI_write_cmos_sensor(0x81a3 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x81a4 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x81a5 ,0x3f);
    OV5646MIPI_write_cmos_sensor(0x81a6 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x81a7 ,0xe5);
    OV5646MIPI_write_cmos_sensor(0x81a8 ,0x0d);
    OV5646MIPI_write_cmos_sensor(0x81a9 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x81aa ,0xd0);
    OV5646MIPI_write_cmos_sensor(0x81ab ,0x82);
    OV5646MIPI_write_cmos_sensor(0x81ac ,0xd0);
    OV5646MIPI_write_cmos_sensor(0x81ad ,0x83);
    OV5646MIPI_write_cmos_sensor(0x81ae ,0xd0);
    OV5646MIPI_write_cmos_sensor(0x81af ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x81b0 ,0x32);
    OV5646MIPI_write_cmos_sensor(0x81b1 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x81b2 ,0x30);
    OV5646MIPI_write_cmos_sensor(0x81b3 ,0x28);
    OV5646MIPI_write_cmos_sensor(0x81b4 ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x81b5 ,0x70);
    OV5646MIPI_write_cmos_sensor(0x81b6 ,0x09);
    OV5646MIPI_write_cmos_sensor(0x81b7 ,0x12);
    OV5646MIPI_write_cmos_sensor(0x81b8 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x81b9 ,0xc3);
    OV5646MIPI_write_cmos_sensor(0x81ba ,0xc2);
    OV5646MIPI_write_cmos_sensor(0x81bb ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81bc ,0xe4);
    OV5646MIPI_write_cmos_sensor(0x81bd ,0xf5);
    OV5646MIPI_write_cmos_sensor(0x81be ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x81bf ,0x22);
    OV5646MIPI_write_cmos_sensor(0x81c0 ,0xd2);
    OV5646MIPI_write_cmos_sensor(0x81c1 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81c2 ,0x22);
    OV5646MIPI_write_cmos_sensor(0x81c3 ,0x90);
    OV5646MIPI_write_cmos_sensor(0x81c4 ,0x51);
    OV5646MIPI_write_cmos_sensor(0x81c5 ,0x83);
    OV5646MIPI_write_cmos_sensor(0x81c6 ,0xe0);
    OV5646MIPI_write_cmos_sensor(0x81c7 ,0x54);
    OV5646MIPI_write_cmos_sensor(0x81c8 ,0x7f);
    OV5646MIPI_write_cmos_sensor(0x81c9 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x81ca ,0x90);
    OV5646MIPI_write_cmos_sensor(0x81cb ,0x51);
    OV5646MIPI_write_cmos_sensor(0x81cc ,0x91);
    OV5646MIPI_write_cmos_sensor(0x81cd ,0x74);
    OV5646MIPI_write_cmos_sensor(0x81ce ,0xf8);
    OV5646MIPI_write_cmos_sensor(0x81cf ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x81d0 ,0xa3);
    OV5646MIPI_write_cmos_sensor(0x81d1 ,0x74);
    OV5646MIPI_write_cmos_sensor(0x81d2 ,0x04);
    OV5646MIPI_write_cmos_sensor(0x81d3 ,0xf0);
    OV5646MIPI_write_cmos_sensor(0x81d4 ,0x22);
    OV5646MIPI_write_cmos_sensor(0x81d5 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x81d6 ,0x0c);
    OV5646MIPI_write_cmos_sensor(0x81d7 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81d8 ,0x01);
    OV5646MIPI_write_cmos_sensor(0x81d9 ,0x08);
    OV5646MIPI_write_cmos_sensor(0x81da ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81db ,0x01);
    OV5646MIPI_write_cmos_sensor(0x81dc ,0x09);
    OV5646MIPI_write_cmos_sensor(0x81dd ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81de ,0x00);
    OV5646MIPI_write_cmos_sensor(0x81df ,0x30);// ;Gth 1. exit gain
    OV5646MIPI_write_cmos_sensor(0x81e0 ,0x40);// ;Gth 2. enter gain
    OV5646MIPI_write_cmos_sensor(0x81e1 ,0x00);// ;Ith1
    OV5646MIPI_write_cmos_sensor(0x81e2 ,0x00); //;Ith2
    OV5646MIPI_write_cmos_sensor(0x81e3 ,0x08);// ;frames of awb1 stable
    OV5646MIPI_write_cmos_sensor(0x81e4 ,0x08); //;frames of awb2 stable
    OV5646MIPI_write_cmos_sensor(0x3000 ,0x00);
    OV5646MIPI_write_cmos_sensor(0x3028 ,0x01);  
#endif	
	
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIInitialSetting function:\n ");
		
} 
/*****************************************************************
* FUNCTION
*    OV5646MIPIPreviewSetting
*
* DESCRIPTION
*    This function config Preview setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5646MIPIPreviewSetting_SVGA(void)
{
	//;OV5646MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2Lane.
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIPreviewSetting_SVGA function:\n ");
	//OV5646MIPI_write_cmos_sensor(0x4202, 0x0f);//	; stop mipi stream
	OV5646MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5646MIPI_write_cmos_sensor(0x3034, 0x18);// PLL, MIPI 8-bit mode
	OV5646MIPI_write_cmos_sensor(0x3035, 0x21);// PLL
	OV5646MIPI_write_cmos_sensor(0x3036, 0x70);// PLL
	OV5646MIPI_write_cmos_sensor(0x3037, 0x13);// PLL
	OV5646MIPI_write_cmos_sensor(0x3108, 0x01);// PLL
	OV5646MIPI_write_cmos_sensor(0x3824, 0x01);// PLL
	OV5646MIPI_write_cmos_sensor(0x460c, 0x20);// PLL
	OV5646MIPI_write_cmos_sensor(0x3618, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x3600, 0x09);//
	OV5646MIPI_write_cmos_sensor(0x3601, 0x43);//
	OV5646MIPI_write_cmos_sensor(0x3708, 0x66);//
	OV5646MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x370c, 0xc3);//
	OV5646MIPI_write_cmos_sensor(0x3800, 0x00); // HS = 0
	OV5646MIPI_write_cmos_sensor(0x3801, 0x00); // HS
	OV5646MIPI_write_cmos_sensor(0x3802, 0x00); // VS = 250
	OV5646MIPI_write_cmos_sensor(0x3803, 0x06); // VS
	OV5646MIPI_write_cmos_sensor(0x3804, 0x0a); // HW = 2623
	OV5646MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5646MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 
	OV5646MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
	OV5646MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
	OV5646MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
	OV5646MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPVO = 960
	OV5646MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
	OV5646MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
	OV5646MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
	OV5646MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
	OV5646MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
	OV5646MIPI_write_cmos_sensor(0x3810, 0x00); // H OFF = 16
	OV5646MIPI_write_cmos_sensor(0x3811, 0x10); // H OFF
	OV5646MIPI_write_cmos_sensor(0x3812, 0x00); // V OFF = 4
	OV5646MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5646MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
	OV5646MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
	OV5646MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
	OV5646MIPI_write_cmos_sensor(0x3821, 0x07);//	; mirror on, H bin on
	OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
	if(OV5646MIPISensor.NightMode)
	{
		OV5646MIPI_write_cmos_sensor(0x3a02, 0x17);//	; max exp 60 = 740
		OV5646MIPI_write_cmos_sensor(0x3a03, 0x10);//	; max exp 60
		OV5646MIPI_write_cmos_sensor(0x3a14, 0x17);//	; max exp 50 = 740
		OV5646MIPI_write_cmos_sensor(0x3a15, 0x10);//	; max exp 50
	}else
	{
		OV5646MIPI_write_cmos_sensor(0x3a02, 0x0b);//	; max exp 60 = 740
		OV5646MIPI_write_cmos_sensor(0x3a03, 0x88);//	; max exp 60
		OV5646MIPI_write_cmos_sensor(0x3a14, 0x0b);//	; max exp 50 = 740
		OV5646MIPI_write_cmos_sensor(0x3a15, 0x88);//	; max exp 50
	}
	OV5646MIPI_write_cmos_sensor(0x3a08, 0x01);//	; B50 = 222
	OV5646MIPI_write_cmos_sensor(0x3a09, 0x27);//	; B50
	OV5646MIPI_write_cmos_sensor(0x3a0a, 0x00);//	; B60 = 185
	OV5646MIPI_write_cmos_sensor(0x3a0b, 0xf6);//	; B60
	OV5646MIPI_write_cmos_sensor(0x3a0e, 0x03);//	; max 50
	OV5646MIPI_write_cmos_sensor(0x3a0d, 0x04);//	; max 60
	OV5646MIPI_write_cmos_sensor(0x3c07, 0x07);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x3c08, 0x01);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x3c09, 0xc2);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
	OV5646MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC triggered by gain change
	OV5646MIPI_write_cmos_sensor(0x4837, 0x11); // MIPI global timing 16           
	OV5646MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5000, 0xa7);//
	OV5646MIPI_write_cmos_sensor(0x5001, 0xa3);//
	OV5646MIPI_write_cmos_sensor(0x5002, 0x80);//
	OV5646MIPI_write_cmos_sensor(0x5003, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x3032, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x4000, 0x89);//
	OV5646MIPI_write_cmos_sensor(0x3a00, 0x3c);//	; ae mode	
	OV5646MIPI_write_cmos_sensor(0x5302, 0x28);//	; sharpen MT
	OV5646MIPI_write_cmos_sensor(0x5303, 0x20);//	; sharpen MT 
	OV5646MIPI_write_cmos_sensor(0x5306, 0x08);//	; DNS off1  
	OV5646MIPI_write_cmos_sensor(0x5307, 0x10);//	; DNS off2  
	if(OV5646MIPI_run_test_potten)
	{
		OV5646MIPI_run_test_potten=0;
		OV5646SetTestPatternMode(1);
	}
	OV5646MIPIWriteExtraShutter(OV5646MIPISensor.PreviewExtraShutter);
	OV5646MIPIWriteExpShutter(OV5646MIPISensor.PreviewShutter);
	//OV5646MIPI_write_cmos_sensor(0x4202, 0x00);//	; open mipi stream
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
	OV5646MIPISensor.IsPVmode = KAL_TRUE;
	OV5646MIPISensor.PreviewPclk= 560;	
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIPreviewSetting_SVGA function:\n ");
}
/*************************************************************************
* FUNCTION
*     OV5646MIPIFullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5646MIPIFullSizeCaptureSetting(void)
{
	//OV5646MIPI 2592x1944,10fps
	//90Mhz, 360Mbps/Lane, 2Lane.15
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIFullSizeCaptureSetting function:\n ");
	OV5646MIPI_write_cmos_sensor(0x4202, 0x0f);//	; stop mipi stream
	OV5646MIPI_write_cmos_sensor(0x3a00, 0x38);//	; max exp 60 = 740
	OV5646MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
	OV5646MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
	OV5646MIPI_write_cmos_sensor(0x3a14, 0x03);//	; max exp 50 = 740
	OV5646MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
	OV5646MIPI_write_cmos_sensor(0x3c07, 0x07);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x3c08, 0x01);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x3c09, 0xc2);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5646MIPI_write_cmos_sensor(0x3034, 0x18); //PLL, MIPI 8-bit mode
	OV5646MIPI_write_cmos_sensor(0x3035, 0x11); //PLL
	OV5646MIPI_write_cmos_sensor(0x3036, 0x5a); //PLL
	OV5646MIPI_write_cmos_sensor(0x3037, 0x13); //PLL
	OV5646MIPI_write_cmos_sensor(0x3108, 0x01); //PLL
	OV5646MIPI_write_cmos_sensor(0x3824, 0x01); //PLL
	OV5646MIPI_write_cmos_sensor(0x460c, 0x20); //PLL
	OV5646MIPI_write_cmos_sensor(0x3618, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x3600, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x3601, 0x33);//
	OV5646MIPI_write_cmos_sensor(0x3708, 0x63);//
	OV5646MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x370c, 0xc0);//
	OV5646MIPI_write_cmos_sensor(0x3800, 0x00); //HS = 0
	OV5646MIPI_write_cmos_sensor(0x3801, 0x00); //HS
	OV5646MIPI_write_cmos_sensor(0x3802, 0x00); //VS = 0
	OV5646MIPI_write_cmos_sensor(0x3803, 0x00); //VS
	OV5646MIPI_write_cmos_sensor(0x3804, 0x0a); //HW = 2623
	OV5646MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5646MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 1705
	OV5646MIPI_write_cmos_sensor(0x3807, 0x9f);//	; VH
	OV5646MIPI_write_cmos_sensor(0x3808, 0x0a);//	; DVPHO = 2560
	OV5646MIPI_write_cmos_sensor(0x3809, 0x20);//	; DVPHO
	OV5646MIPI_write_cmos_sensor(0x380a, 0x07);//	; DVPVO = 1440
	OV5646MIPI_write_cmos_sensor(0x380b, 0x98);//	; DVPVO
	OV5646MIPI_write_cmos_sensor(0x380c, 0x0b);//	; HTS = 2984
	OV5646MIPI_write_cmos_sensor(0x380d, 0xec);//	; HTS
	OV5646MIPI_write_cmos_sensor(0x380e, 0x07);//	; VTS = 1464
	OV5646MIPI_write_cmos_sensor(0x380f, 0xb0);//	; VTS
	OV5646MIPI_write_cmos_sensor(0x3810, 0x00); //H OFF = 16
	OV5646MIPI_write_cmos_sensor(0x3811, 0x10); //H OFF
	OV5646MIPI_write_cmos_sensor(0x3812, 0x00); //V OFF = 4
	OV5646MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5646MIPI_write_cmos_sensor(0x3814, 0x11);//	; X INC
	OV5646MIPI_write_cmos_sensor(0x3815, 0x11);//	; Y INC
	OV5646MIPI_write_cmos_sensor(0x3820, 0x40);//	; flip off, V bin off
	OV5646MIPI_write_cmos_sensor(0x3821, 0x06);//	; mirror on, H bin off
	OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
	OV5646MIPI_write_cmos_sensor(0x3a0e, 0x06);//	; flip off, V bin off
	OV5646MIPI_write_cmos_sensor(0x3a0d, 0x08);//	; mirror on, H bin off
	OV5646MIPI_write_cmos_sensor(0x4004, 0x06);//	; BLC line number
	OV5646MIPI_write_cmos_sensor(0x4837, 0x16);//; MIPI global timing  
	OV5646MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5000, 0xa7);//
	OV5646MIPI_write_cmos_sensor(0x5001, 0x83);//
	OV5646MIPI_write_cmos_sensor(0x5002, 0x80);
	OV5646MIPI_write_cmos_sensor(0x5003, 0x08);
	OV5646MIPI_write_cmos_sensor(0x3032, 0x00);
	OV5646MIPI_write_cmos_sensor(0x4000, 0x89);
	OV5646MIPI_write_cmos_sensor(0x350c, 0x00);
	OV5646MIPI_write_cmos_sensor(0x350d, 0x00);
	OV5646MIPI_write_cmos_sensor(0x5302, 0x28);//	; sharpen MT
	OV5646MIPI_write_cmos_sensor(0x5303, 0x20);//	; sharpen MT 
	OV5646MIPI_write_cmos_sensor(0x5306, 0x08);//	; DNS off1	
	OV5646MIPI_write_cmos_sensor(0x5307, 0x10);//	; DNS off2 
	if(OV5646MIPI_run_test_potten)
	{
		OV5646MIPI_run_test_potten=0;
		OV5646SetTestPatternMode(1);
	}
	OV5646MIPI_write_cmos_sensor(0x4202, 0x00);//	; open mipi stream
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.IsPVmode = KAL_FALSE;
	OV5646MIPISensor.CapturePclk= 900;	
	//OV5646MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIFullSizeCaptureSetting function:\n ");
}



/*************************************************************************
* FUNCTION
*     OV5646MIPIFullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5646MIPIFullSizeZSDSetting(void)
{
	//OV5646MIPI 2592x1944,10fps
	//90Mhz, 360Mbps/Lane, 2Lane.15
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIFullSizeZSDSetting function:\n ");
	OV5646MIPI_write_cmos_sensor(0x4202, 0x0f);//	; stop mipi stream
	OV5646MIPI_write_cmos_sensor(0x3a00, 0x3c);//	; max exp 60 = 740
	OV5646MIPI_write_cmos_sensor(0x3a02, 0x0b);//	; max exp 60 = 740
	OV5646MIPI_write_cmos_sensor(0x3a03, 0x88);//	; max exp 60
	OV5646MIPI_write_cmos_sensor(0x3a14, 0x0b);//	; max exp 50 = 740
	OV5646MIPI_write_cmos_sensor(0x3a15, 0x88);//	; max exp 50
	OV5646MIPI_write_cmos_sensor(0x3c07, 0x07);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x3c08, 0x01);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x3c09, 0xc2);//	; 50/60 auto detect
	OV5646MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
	OV5646MIPI_write_cmos_sensor(0x3034, 0x18); //PLL, MIPI 8-bit mode
	OV5646MIPI_write_cmos_sensor(0x3035, 0x11); //PLL
	OV5646MIPI_write_cmos_sensor(0x3036, 0x5a); //PLL
	OV5646MIPI_write_cmos_sensor(0x3037, 0x13); //PLL
	OV5646MIPI_write_cmos_sensor(0x3108, 0x01); //PLL
	OV5646MIPI_write_cmos_sensor(0x3824, 0x01); //PLL
	OV5646MIPI_write_cmos_sensor(0x460c, 0x20); //PLL
	OV5646MIPI_write_cmos_sensor(0x3618, 0x04);//
	OV5646MIPI_write_cmos_sensor(0x3600, 0x08);//
	OV5646MIPI_write_cmos_sensor(0x3601, 0x33);//
	OV5646MIPI_write_cmos_sensor(0x3708, 0x63);//
	OV5646MIPI_write_cmos_sensor(0x3709, 0x12);//
	OV5646MIPI_write_cmos_sensor(0x370c, 0xc0);//
	OV5646MIPI_write_cmos_sensor(0x3800, 0x00); //HS = 0
	OV5646MIPI_write_cmos_sensor(0x3801, 0x00); //HS
	OV5646MIPI_write_cmos_sensor(0x3802, 0x00); //VS = 0
	OV5646MIPI_write_cmos_sensor(0x3803, 0x00); //VS
	OV5646MIPI_write_cmos_sensor(0x3804, 0x0a); //HW = 2623
	OV5646MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
	OV5646MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 1705
	OV5646MIPI_write_cmos_sensor(0x3807, 0x9f);//	; VH
	OV5646MIPI_write_cmos_sensor(0x3808, 0x0a);//	; DVPHO = 2560
	OV5646MIPI_write_cmos_sensor(0x3809, 0x20);//	; DVPHO
	OV5646MIPI_write_cmos_sensor(0x380a, 0x07);//	; DVPVO = 1440
	OV5646MIPI_write_cmos_sensor(0x380b, 0x98);//	; DVPVO
	OV5646MIPI_write_cmos_sensor(0x380c, 0x0b);//	; HTS = 2984
	OV5646MIPI_write_cmos_sensor(0x380d, 0xec);//	; HTS
	OV5646MIPI_write_cmos_sensor(0x380e, 0x07);//	; VTS = 1464
	OV5646MIPI_write_cmos_sensor(0x380f, 0xb0);//	; VTS
	OV5646MIPI_write_cmos_sensor(0x3810, 0x00); //H OFF = 16
	OV5646MIPI_write_cmos_sensor(0x3811, 0x10); //H OFF
	OV5646MIPI_write_cmos_sensor(0x3812, 0x00); //V OFF = 4
	OV5646MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
	OV5646MIPI_write_cmos_sensor(0x3814, 0x11);//	; X INC
	OV5646MIPI_write_cmos_sensor(0x3815, 0x11);//	; Y INC
	OV5646MIPI_write_cmos_sensor(0x3820, 0x40);//	; flip off, V bin off
	OV5646MIPI_write_cmos_sensor(0x3821, 0x06);//	; mirror on, H bin off
	OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
	OV5646MIPI_write_cmos_sensor(0x3a0e, 0x06);//	; flip off, V bin off
	OV5646MIPI_write_cmos_sensor(0x3a0d, 0x08);//	; mirror on, H bin off
	OV5646MIPI_write_cmos_sensor(0x4004, 0x06);//	; BLC line number
	OV5646MIPI_write_cmos_sensor(0x4837, 0x16);//; MIPI global timing  
	OV5646MIPI_write_cmos_sensor(0x503d, 0x00);//
	OV5646MIPI_write_cmos_sensor(0x5000, 0xa7);//
	OV5646MIPI_write_cmos_sensor(0x5001, 0x83);//
	OV5646MIPI_write_cmos_sensor(0x5002, 0x80);
	OV5646MIPI_write_cmos_sensor(0x5003, 0x08);
	OV5646MIPI_write_cmos_sensor(0x3032, 0x00);
	OV5646MIPI_write_cmos_sensor(0x4000, 0x89);
	OV5646MIPI_write_cmos_sensor(0x5302, 0x28);//	; sharpen MT
	OV5646MIPI_write_cmos_sensor(0x5303, 0x20);//	; sharpen MT 
	OV5646MIPI_write_cmos_sensor(0x5306, 0x08);//	; DNS off1	
	OV5646MIPI_write_cmos_sensor(0x5307, 0x10);//	; DNS off2 
	if(OV5646MIPI_run_test_potten)
	{
		OV5646MIPI_run_test_potten=0;
		OV5646SetTestPatternMode(1);
	}
	OV5646MIPI_write_cmos_sensor(0x4202, 0x00);//	; open mipi stream
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.IsPVmode = KAL_FALSE;
	OV5646MIPISensor.CapturePclk= 900;	
	//OV5646MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIFullSizeZSDSetting function:\n ");
}




/*************************************************************************
* FUNCTION
*    OV5646MIPISetHVMirror
*
* DESCRIPTION
*    This function set sensor Mirror
*
* PARAMETERS
*    Mirror
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV5646MIPISetHVMirror(kal_uint8 Mirror, kal_uint8 Mode)
{
  	kal_uint8 mirror= 0, flip=0;
    OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPISetHVMirror function:\n ");
		flip = OV5646MIPIYUV_read_cmos_sensor(0x3820);
		mirror=OV5646MIPIYUV_read_cmos_sensor(0x3821);
	
	if (Mode==SENSOR_MODE_PREVIEW)
	{
		switch (Mirror)
		{
		case IMAGE_NORMAL:
			OV5646MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_H_MIRROR:
			OV5646MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_V_MIRROR: 
			OV5646MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
			break;		
		case IMAGE_HV_MIRROR:
			OV5646MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
			break; 		
		default:
			ASSERT(0);
		}
	}
	else if (Mode== SENSOR_MODE_CAPTURE)
	{
		switch (Mirror)
		{
		case IMAGE_NORMAL:
			OV5646MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_H_MIRROR:
			OV5646MIPI_write_cmos_sensor(0x3820, flip&0xf9);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_V_MIRROR: 
			OV5646MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror&0xf9);
			OV5646MIPI_write_cmos_sensor(0x4514, 0xaa);
			break;		
		case IMAGE_HV_MIRROR:
			OV5646MIPI_write_cmos_sensor(0x3820, flip|0x06);     
			OV5646MIPI_write_cmos_sensor(0x3821, mirror|0x06);
			OV5646MIPI_write_cmos_sensor(0x4514, 0xbb);
			break; 		
		default:
			ASSERT(0);
		}
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPISetHVMirror function:\n ");
}

void OV5646MIPI_Standby(void)
{
	OV5646MIPI_write_cmos_sensor(0x3008,0x42);
}

void OV5646MIPI_Wakeup(void)
{
	OV5646MIPI_write_cmos_sensor(0x3008,0x02);
}
/*************************************************************************
* FUNCTION
*   OV5646_FOCUS_OVT_AFC_Init
* DESCRIPTION
*   This function is to load micro code for AF function
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/
static void OV5646_FOCUS_OVT_AFC_Init(void)
{
	OV5646MIPI_write_cmos_sensor(0x3000,0x20);
	iBurstWriteReg(addr_data_pair1  , 254, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair2  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair3  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair4  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair5  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair6  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair7  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair8  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair9  , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair10 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair11 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair12 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair13 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair14 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair15 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair16 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair17 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair18 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair19 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair20 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair21 , 255, OV5646MIPI_WRITE_ID);
	iBurstWriteReg(addr_data_pair22 , 181, OV5646MIPI_WRITE_ID);
	OV5646MIPI_write_cmos_sensor(0x3022,0x00);
	OV5646MIPI_write_cmos_sensor(0x3023,0x00);
	OV5646MIPI_write_cmos_sensor(0x3024,0x00);
	OV5646MIPI_write_cmos_sensor(0x3025,0x00);
	OV5646MIPI_write_cmos_sensor(0x3026,0x00);
	OV5646MIPI_write_cmos_sensor(0x3027,0x00);
	OV5646MIPI_write_cmos_sensor(0x3028,0x00);
	OV5646MIPI_write_cmos_sensor(0x3029,0x7F);
	OV5646MIPI_write_cmos_sensor(0x3000,0x00);
	OV5646MIPI_write_cmos_sensor(0x3000,0x00);
}
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Constant_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5646_FOCUS_OVT_AFC_Constant_Focus(void)
{
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);
	OV5646MIPI_write_cmos_sensor(0x3022,0x80);
	mDELAY(10);
	OV5646MIPI_write_cmos_sensor(0x3024,0x00);
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);
	OV5646MIPI_write_cmos_sensor(0x3022,0x04);		
}   
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Single_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5646_FOCUS_OVT_AFC_Single_Focus()
{
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);
	OV5646MIPI_write_cmos_sensor(0x3022,0x06); 
	mDELAY(10);
	OV5646MIPI_write_cmos_sensor(0x3024,OV5646MIPISensor.af_xcoordinate);
	OV5646MIPI_write_cmos_sensor(0x3025,OV5646MIPISensor.af_ycoordinate);   
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);   
	OV5646MIPI_write_cmos_sensor(0x3022,0x81);  
	mDELAY(10);
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);
	OV5646MIPI_write_cmos_sensor(0x3022,0x03);
}

/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Pause_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5646_FOCUS_OVT_AFC_Pause_Focus()
{
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);
    OV5646MIPI_write_cmos_sensor(0x3022,0x06);
}
static void OV5646_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  
    *pFeatureReturnPara32 = 0; //1;    
    OV5646MIPISENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

static void OV5646_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
    *pFeatureReturnPara32 = 0; //1;    
    OV5646MIPISENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}
/*************************************************************************
* FUNCTION
*   OV5646_FOCUS_OVT_AFC_Touch_AF
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5646_FOCUS_OVT_AFC_Touch_AF(UINT32 x,UINT32 y)
{
	 OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646_FOCUS_OVT_AFC_Touch_AF function:\n ");
	 int x_view,y_view;
     int x_tmp,y_tmp;
     if(x<1)
     {
               x_view=1;
     }
     else if(x>79)
     {
               x_view=79;
     }
     else
     {
               x_view= x;
     }
     
     if(y<1)
     {
               y_view=1;
     }
     else if(y>59)
     {
               y_view=59;
     }
     else
     {
               y_view= y;
     }
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.af_xcoordinate = x_view;
	OV5646MIPISensor.af_ycoordinate = y_view;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]AF x_view=%d,y_view=%d\n",x_view, y_view);
	OV5646MIPISENSORDB("[OV5646MIPI]AF x_tmp1=%d,y_tmp1=%d\n",x_tmp, y_tmp);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646_FOCUS_OVT_AFC_Touch_AF function:\n ");
}
static void OV5646_FOCUS_Set_AF_Window(UINT32 zone_addr)
{//update global zone
	  OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646_FOCUS_Set_AF_Window function:\n ");
	  UINT32 FD_XS;
	  UINT32 FD_YS;   
	  UINT32 x0, y0, x1, y1;
	  UINT32 pvx0, pvy0, pvx1, pvy1;
	  UINT32 linenum, rownum;
	  UINT32 AF_pvx, AF_pvy;
	  UINT32* zone = (UINT32*)zone_addr;
	  x0 = *zone;
	  y0 = *(zone + 1);
	  x1 = *(zone + 2);
	  y1 = *(zone + 3);   
	  FD_XS = *(zone + 4);
	  FD_YS = *(zone + 5);
	  
	  OV5646MIPISENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
	  x0, y0, x1, y1, FD_XS, FD_YS);  
	  OV5646MIPI_mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, OV5646MIPI_PRV_W, OV5646MIPI_PRV_H);
	  OV5646MIPI_mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, OV5646MIPI_PRV_W, OV5646MIPI_PRV_H);  
	  OV5646MIPISENSORDB("[OV5646MIPI]AF pvx0=%d,pvy0=%d\n",pvx0, pvy0);
	  OV5646MIPISENSORDB("[OV5646MIPI]AF pvx0=%d,pvy0=%d\n",pvx1, pvy1);
	  AF_pvx =(pvx0+pvx1)/32;
	  AF_pvy =(pvy0+pvy1)/32;
	  OV5646MIPISENSORDB("[OV5646MIPI]AF AF_pvx=%d,AF_pvy=%d\n",AF_pvx, AF_pvy);
	  OV5646_FOCUS_OVT_AFC_Touch_AF(AF_pvx ,AF_pvy);
	  OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646_FOCUS_Set_AF_Window function:\n ");
}
static void OV5646_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}
static void OV5646_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}
/*************************************************************************
//此函数变量未定义,请根据编译定义数据变量.
//prview 1280*960 
//16的整数倍 ; n*16*80/1280
//16的整数倍 ; n*16*60/960
//touch_x  为对应preview的横坐标[0-1280]
//touch_y  为对应preview的纵坐标[0-960]

*************************************************************************/ 
static UINT32 OV5646_FOCUS_Move_to(UINT32 a_u2MovePosition)//??how many bits for ov3640??
{
}
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Get_AF_Status
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/                        
static void OV5646_FOCUS_OVT_AFC_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
	UINT32 state_3028=0;
	UINT32 state_3029=0;
	*pFeatureReturnPara32 = SENSOR_AF_IDLE;
	state_3028 = OV5646MIPIYUV_read_cmos_sensor(0x3028);
	state_3029 = OV5646MIPIYUV_read_cmos_sensor(0x3029);
	mDELAY(1);
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646_FOCUS_OVT_AFC_Get_AF_Status function:state_3028=%d,state_3029=%d\n",state_3028,state_3029);
	if (state_3028==0)
	{
		*pFeatureReturnPara32 = SENSOR_AF_ERROR;    
	}
	else 
	{
		switch (state_3029)
		{
			case 0x70:
				*pFeatureReturnPara32 = SENSOR_AF_IDLE;
				break;
			case 0x00:
				*pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
				break;
			case 0x10:
				*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
				break;
			case 0x20:
				*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
				break;
			default:
				*pFeatureReturnPara32 = SENSOR_AF_SCENE_DETECTING; 
				break;
		}                                  
	}    
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646_FOCUS_OVT_AFC_Get_AF_Status function:state_3028=%d,state_3029=%d\n",state_3028,state_3029);
}


/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Cancel_Focus
* DESCRIPTION
*   cancel af 
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/     
static void OV5646_FOCUS_OVT_AFC_Cancel_Focus()
{
  //  OV5646MIPI_write_cmos_sensor(0x3023,0x01);
  //  OV5646MIPI_write_cmos_sensor(0x3022,0x08);    
}


/*************************************************************************
* FUNCTION
*   OV5646WBcalibattion
* DESCRIPTION
*   color calibration
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV5646WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
	kal_uint32 color_r_gain_w = 0;
	kal_uint32 color_b_gain_w = 0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646WBcalibattion function:\n ");
	kal_uint8 temp = OV5646MIPIYUV_read_cmos_sensor(0x350b); 
	if(color_r_gain>0x600)//d65   66b 5a7 4a7 449   //办公室
	{
	     if (temp>=0x70)
	     {
	               color_r_gain_w=color_r_gain *95/100;                                                                                                                                                                                                                                                                                                                                                                                                                                      
	               color_b_gain_w=color_b_gain*97/100;
	     }
	     else if (temp>=0x58)
	     {
	               color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	               color_b_gain_w=color_b_gain*98/100;
	     }
	     else if (temp>=0x48)
	     {
	               color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	               color_b_gain_w=color_b_gain*98/100;
	     }
	     else if (temp>=0x30)
	     {
	               color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	              color_b_gain_w=color_b_gain*98/100;
	     }
	     else
	     {
	               color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	               color_b_gain_w=color_b_gain*98/100; 
	     }        
	}else if(color_r_gain>0x540)//cwf  //窗户外
	{
	if (temp>=0x70)
	{
	     color_r_gain_w=color_r_gain *97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                      
	     color_b_gain_w=color_b_gain*98/100;
	}
	else if (temp>=0x58)
	{
	     color_r_gain_w=color_r_gain*98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*99/100;
	}
	else if (temp>=0x48)
	{
	     color_r_gain_w=color_r_gain*98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*99/100;
	}
	else if (temp>=0x30)
	{
	     color_r_gain_w=color_r_gain*99/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*99/100;
	}
	else
	{
	     color_r_gain_w=color_r_gain*99/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*99/100; 
	}                                    
	}else if(color_r_gain>0x480)//tl84
	{
	if (temp>=0x70)
	{
	     color_r_gain_w=color_r_gain *97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                      
	     color_b_gain_w=color_b_gain*97/100;
	}
	else if (temp>=0x58)
	{
	     color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100;
	}
	else if (temp>=0x48)
	{
	     color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100;
	}
	else if (temp>=0x30)
	{
	     color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100;
	}
	else
	{
	     color_r_gain_w=color_r_gain*97/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100; 
	}        
	}else//h/a
	{
	if (temp>=0x70)
	{
	     color_r_gain_w=color_r_gain *98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                     
	     color_b_gain_w=color_b_gain*97/100;
	}
	else if (temp>=0x58)
	{
	     color_r_gain_w=color_r_gain*98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100;
	}
	else if (temp>=0x48)
	{
	     color_r_gain_w=color_r_gain*98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100;
	}
	else if (temp>=0x30)
	{
	     color_r_gain_w=color_r_gain*98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100;
	}
	else
	{
	     color_r_gain_w=color_r_gain*98/100;                                                                                                                                                                                                                                                                                                                                                                                                                                       
	     color_b_gain_w=color_b_gain*97/100; 
	}        
	}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
	OV5646MIPI_write_cmos_sensor(0x3400,(color_r_gain_w & 0xff00)>>8);                                                                                                                                                                                                                                                                                                                                                                                                                                     
	OV5646MIPI_write_cmos_sensor(0x3401,color_r_gain_w & 0xff);                        
	OV5646MIPI_write_cmos_sensor(0x3404,(color_b_gain_w & 0xff00)>>8);                                                                                                                                                                                                                                                                                                                                                                                                                                     
	OV5646MIPI_write_cmos_sensor(0x3405,color_b_gain_w & 0xff); 
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646WBcalibattion function:\n ");
}	
/*************************************************************************
* FUNCTION
*	OV5646MIPIOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV5646MIPIOpen(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIOpen function:\n ");
	OV5646MIPI_write_cmos_sensor(0x3103,0x11);
	OV5646MIPI_write_cmos_sensor(0x3008,0x82);
    mDELAY(10);
	for(i=0;i<3;i++)
	{
		sensor_id = (OV5646MIPIYUV_read_cmos_sensor(0x300A) << 8) | OV5646MIPIYUV_read_cmos_sensor(0x300B);
		OV5646MIPISENSORDB("OV5646MIPI READ ID :%x",sensor_id);
		if(sensor_id != OV5645MIPI_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
		{
			sensor_id = OV5646MIPI_SENSOR_ID;
		}
	}
	OV5646MIPIinitalvariable();
	OV5646MIPIInitialSetting();
//	OV5646_FOCUS_OVT_AFC_Init();
	OV5646MIPI_set_AWB_mode(KAL_TRUE);
//  afPowerOn();
	mDELAY(20);
	//OV5646_FOCUS_OVT_AFC_Constant_Focus();
	//AF POWER ON
	//OV5646MIPIYUV_read_otp_test();
	//OV5646MIPIYUV_read_otp_test();
	//OV5646MIPIYUV_read_otp_test();
	
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIOpen function:\n ");
	return ERROR_NONE;
}	/* OV5646MIPIOpen() */

/*************************************************************************
* FUNCTION
*	OV5646MIPIClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV5646MIPIClose(void)
{
 	//CISModulePowerOn(FALSE);
  	kal_uint16 current_position;
  	kal_uint16 tmp_H, tmp_L;
  	kal_uint8 step_size,i;  
  	tmp_H = OV5646MIPIYUV_read_cmos_sensor(0x3603);
  	tmp_L = OV5646MIPIYUV_read_cmos_sensor(0x3602);
  	current_position = ((tmp_H & 0x3f) << 4) | ((tmp_L & 0xf0) >> 4);  
  	if (current_position > 150) 
	 {
		step_size = current_position / 3;
		for (i = 0; i < 6; i++) 
		{
			current_position -= step_size;
			tmp_H = current_position >> 4;
			tmp_L = (current_position & 0x0f) << 4;
			OV5646MIPI_write_cmos_sensor(0x3603, tmp_H);
			OV5646MIPI_write_cmos_sensor(0x3602, tmp_L);
			mDELAY(40);  		
			if (current_position < 180) 
			return ERROR_NONE;
		}
	 } 
	return ERROR_NONE;
}	/* OV5646MIPIClose() */
/*************************************************************************
* FUNCTION
*	OV5646MIPIPreview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV5646MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIPreview function:\n ");
	kal_uint32 zsdshutter = 0;
	switch(CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   OV5646MIPISensor.zsd_flag=1;
			   OV5646MIPIPreviewSetting_SVGA();
			   OV5646MIPIFullSizeZSDSetting();	
			   zsdshutter=OV5646MIPISensor.PreviewShutter*2;
			   OV5646MIPIWriteExpShutter(zsdshutter);
			   OV5646MIPISENSORDB("[OV5646MIPI]enter MSDK_SCENARIO_ID_CAMERA_ZSD OV5646MIPIPreview function:zsdshutter=%d\n",zsdshutter);
			   break;
		default:
			   OV5646MIPIPreviewSetting_SVGA();
			   OV5646MIPISensor.zsd_flag=0;
			   zsdshutter=0;
			   //OV5646MIPIWriteExpShutter(OV5646MIPISensor.PreviewShutter);
			   OV5646MIPISENSORDB("[OV5646MIPI]enter  OV5646MIPIPreview function:OV5646MIPISensor.PreviewShutter=%d\n",OV5646MIPISensor.PreviewShutter);
			   break;
	}
	mDELAY(100);	
	OV5646MIPI_set_AE_mode(KAL_TRUE);
	OV5646MIPI_set_AWB_mode(KAL_TRUE);
	mDELAY(20);
	//OV5646MIPI_set_AWB_switch(); 
	
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.VideoMode =KAL_FALSE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIPreview function:\n ");	
	return ERROR_NONE ;	
}	/* OV5646MIPIPreview() */
BOOL OV5646MIPI_set_param_exposure_for_HDR(UINT16 para)
{
    kal_uint32 totalGain = 0, exposureTime = 0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_param_exposure_for_HDR function:\n ");
	OV5646MIPISENSORDB("[OV5646MIPI]enter para=%d,manualAEStart%d\n",para,OV5646MIPISensor.manualAEStart);
    if (0 == OV5646MIPISensor.manualAEStart)
    {       
        OV5646MIPI_set_AE_mode(KAL_FALSE);//Manual AE enable
        spin_lock(&ov5646mipi_drv_lock);	
        OV5646MIPISensor.manualAEStart = 1;
		spin_unlock(&ov5646mipi_drv_lock);
    }
	totalGain = OV5646MIPISensor.currentAxDGain;
    exposureTime = OV5646MIPISensor.currentExposureTime;
	switch (para)
	{
	   case AE_EV_COMP_20:	//+2 EV
       case AE_EV_COMP_10:	// +1 EV
		   totalGain = totalGain<<1;
           exposureTime = exposureTime<<1;
           OV5646MIPISENSORDB("[4EC] HDR AE_EV_COMP_20\n");
		 break;
	   case AE_EV_COMP_00:	// +0 EV
           OV5646MIPISENSORDB("[4EC] HDR AE_EV_COMP_00\n");
		 break;
	   case AE_EV_COMP_n10:  // -1 EV
	   case AE_EV_COMP_n20:  // -2 EV
		   totalGain = totalGain >> 1;
           exposureTime = exposureTime >> 1;
           OV5646MIPISENSORDB("[4EC] HDR AE_EV_COMP_n20\n");
		 break;
	   default:
		 break;//return FALSE;
	}
	totalGain = (totalGain > OV5646MIPI_MAX_AXD_GAIN) ? OV5646MIPI_MAX_AXD_GAIN : totalGain;
    //exposureTime = (exposureTime > OV5646MIPI_MAX_EXPOSURE_TIME) ? OV5646MIPI_MAX_EXPOSURE_TIME : exposureTime;
    OV5646MIPIWriteSensorGain(totalGain);	
	OV5646MIPIWriteShutter(exposureTime);	
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_param_exposure_for_HDR function:\n ");
	return TRUE;
}
/*************************************************************************
* FUNCTION
*   OV5646_FOCUS_OVT_AFC_Focus_BeforeCapture
* DESCRIPTION
*   cancel af 
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/     
static void OV5646_FOCUS_OVT_AFC_Focus_BeforeCapture()
{
	UINT32 i=0;
	UINT32 ack=0;
	OV5646MIPI_write_cmos_sensor(0x3023,0x01);
	OV5646MIPI_write_cmos_sensor(0x3022,0x06);
	for(i=0;i<100;i++) 
	{
	 	ack = OV5646MIPIYUV_read_cmos_sensor (0x3023);
	 	if(ack == 0)
	  		break;
	  	mDELAY(2);
	}    
}
UINT32 OV5646MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter = 0;	
	kal_uint32 extshutter = 0;
	kal_uint32 color_r_gain = 0;
	kal_uint32 color_b_gain = 0;
	kal_uint32 readgain=0;
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPICapture function:\n ");
	OV5646_FOCUS_OVT_AFC_Focus_BeforeCapture();
	if(SENSOR_MODE_PREVIEW == OV5646MIPISensor.SensorMode )
	{		
	shutter=OV5646MIPIReadShutter();
	extshutter=OV5646MIPIReadExtraShutter();
	readgain=OV5646MIPIReadSensorGain();
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.PreviewShutter=shutter;
	OV5646MIPISensor.PreviewExtraShutter=extshutter;	
	OV5646MIPISensor.SensorGain=readgain;
	spin_unlock(&ov5646mipi_drv_lock);	
	OV5646MIPI_set_AE_mode(KAL_FALSE);
	OV5646MIPI_set_AWB_mode(KAL_FALSE);
	color_r_gain=((OV5646MIPIYUV_read_cmos_sensor(0x3401)&0xFF)+((OV5646MIPIYUV_read_cmos_sensor(0x3400)&0xFF)*256));  
	color_b_gain=((OV5646MIPIYUV_read_cmos_sensor(0x3405)&0xFF)+((OV5646MIPIYUV_read_cmos_sensor(0x3404)&0xFF)*256)); 
	OV5646MIPIFullSizeCaptureSetting();
	spin_lock(&ov5646mipi_drv_lock);	
	OV5646MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&ov5646mipi_drv_lock);
    OV5646WBcalibattion(color_r_gain,color_b_gain);  
	OV5646MIPISENSORDB("[OV5646MIPI]Before shutter=%d:\n",shutter);
	if(OV5646MIPISensor.zsd_flag==0)
	{
		shutter = shutter*2;
	}
	if (SCENE_MODE_HDR == OV5646MIPISensor.sceneMode)
    {
        spin_lock(&ov5646mipi_drv_lock);
        OV5646MIPISensor.currentExposureTime=shutter;
		OV5646MIPISensor.currentextshutter=extshutter;
		OV5646MIPISensor.currentAxDGain=readgain;
		spin_unlock(&ov5646mipi_drv_lock);
    }
	else
	{
		//OV5646MIPIWriteSensorGain(OV5646MIPISensor.SensorGain);	
		OV5646MIPIWriteShutter(shutter);
	}
	OV5646MIPISENSORDB("[OV5646MIPI]after shutter=%d:\n",shutter);
	OV5646MIPISENSORDB("[OV5646MIPI]after shutter=%d:\n",OV5646MIPISensor.SensorGain);
	//mDELAY(200);
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPICapture function:\n ");
	return ERROR_NONE; 
}/* OV5646MIPICapture() */

UINT32 OV5646MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIGetResolution function:\n ");
	pSensorResolution->SensorPreviewWidth=  OV5646MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV5646MIPI_PV_GRAB_START_X;
	pSensorResolution->SensorPreviewHeight= OV5646MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV5646MIPI_PV_GRAB_START_Y;
	pSensorResolution->SensorFullWidth= OV5646MIPI_IMAGE_SENSOR_QSXGA_WITDH-2*OV5646MIPI_FULL_GRAB_START_X; 
	pSensorResolution->SensorFullHeight= OV5646MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-2*OV5646MIPI_FULL_GRAB_START_Y;
	pSensorResolution->SensorVideoWidth= OV5646MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV5646MIPI_PV_GRAB_START_X; 
	pSensorResolution->SensorVideoHeight= OV5646MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV5646MIPI_PV_GRAB_START_Y;;
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIGetResolution function:\n ");
	return ERROR_NONE;
}	/* OV5646MIPIGetResolution() */

UINT32 OV5646MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIGetInfo function:\n ");
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=OV5646MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV5646MIPI_FULL_GRAB_START_X;//OV5646MIPI_IMAGE_SENSOR_QSXGA_WITDH ;
			pSensorInfo->SensorPreviewResolutionY=OV5646MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV5646MIPI_FULL_GRAB_START_Y;//OV5646MIPI_IMAGE_SENSOR_QSXGA_HEIGHT ;
			pSensorInfo->SensorCameraPreviewFrameRate=15;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX=OV5646MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV5646MIPI_PV_GRAB_START_X; ;
			pSensorInfo->SensorPreviewResolutionY=OV5646MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV5646MIPI_PV_GRAB_START_Y;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}		 		
	pSensorInfo->SensorFullResolutionX= OV5646MIPI_IMAGE_SENSOR_QSXGA_WITDH-2*OV5646MIPI_FULL_GRAB_START_X;
	pSensorInfo->SensorFullResolutionY= OV5646MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-2*OV5646MIPI_FULL_GRAB_START_Y;
	//pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=5;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;  
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 2;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->CaptureDelayFrame = 3;
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 3; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->YUVAwbDelayFrame = 5;
	pSensorInfo->YUVEffectDelayFrame= 3; 
	pSensorInfo->AEShutDelayFrame= 0;
 	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = OV5646MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV5646MIPI_PV_GRAB_START_Y;   
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 0x1A; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;  	
			pSensorInfo->SensorPacketECCOrder = 1;		
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = OV5646MIPI_FULL_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV5646MIPI_FULL_GRAB_START_Y;             
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =0x1A; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = OV5646MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV5646MIPI_PV_GRAB_START_Y; 			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 0x1A; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;	
			pSensorInfo->SensorPacketECCOrder = 1;
		  break;
	}
	memcpy(pSensorConfigData, &OV5646MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIGetInfo function:\n ");	
	return ERROR_NONE;
}	/* OV5646MIPIGetInfo() */

UINT32 OV5646MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	  OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIControl function:\n ");
	  spin_lock(&ov5646mipi_drv_lock);
	  CurrentScenarioId = ScenarioId;
	  spin_unlock(&ov5646mipi_drv_lock);
	  switch (ScenarioId)
	  {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 OV5646MIPIPreview(pImageWindow, pSensorConfigData);
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			 OV5646MIPICapture(pImageWindow, pSensorConfigData);
	  	     break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIControl function:\n ");
	return ERROR_NONE;
}	/* OV5646MIPIControl() */

/* [TC] YUV sensor */	

BOOL OV5646MIPI_set_param_wb(UINT16 para)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_param_wb function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
    OV5646MIPISensor.awbMode = para;
    spin_unlock(&ov5646mipi_drv_lock);
	switch (para)
    {
        case AWB_MODE_OFF:
							spin_lock(&ov5646mipi_drv_lock);
							OV5646MIPI_AWB_ENABLE = KAL_FALSE; 
							spin_unlock(&ov5646mipi_drv_lock);
							OV5646MIPI_set_AWB_mode(OV5646MIPI_AWB_ENABLE);
							break;                    
        case AWB_MODE_AUTO:
							spin_lock(&ov5646mipi_drv_lock);
							OV5646MIPI_AWB_ENABLE = KAL_TRUE; 
							spin_unlock(&ov5646mipi_drv_lock);
							OV5646MIPI_set_AWB_mode(OV5646MIPI_AWB_ENABLE);
							break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
        					OV5646MIPI_write_cmos_sensor(0x3212,0x03); 
							OV5646MIPI_set_AWB_mode(KAL_FALSE);         	                
							OV5646MIPI_write_cmos_sensor(0x3400,0x06); 
							OV5646MIPI_write_cmos_sensor(0x3401,0x30); 
							OV5646MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5646MIPI_write_cmos_sensor(0x3404,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3405,0x30);
							OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
							OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
             				break;
        case AWB_MODE_DAYLIGHT: //sunny
							OV5646MIPI_write_cmos_sensor(0x3212,0x03);
							OV5646MIPI_set_AWB_mode(KAL_FALSE);                           
							OV5646MIPI_write_cmos_sensor(0x3400,0x06); 
							OV5646MIPI_write_cmos_sensor(0x3401,0x10); 
							OV5646MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5646MIPI_write_cmos_sensor(0x3404,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3405,0x48);
							OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
							OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
							break;
        case AWB_MODE_INCANDESCENT: //office
        					OV5646MIPI_write_cmos_sensor(0x3212,0x03);
							OV5646MIPI_set_AWB_mode(KAL_FALSE);                           
							OV5646MIPI_write_cmos_sensor(0x3400,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3401,0xe0); 
							OV5646MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5646MIPI_write_cmos_sensor(0x3404,0x05); 
							OV5646MIPI_write_cmos_sensor(0x3405,0xa0);
							OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
							OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
							break; 
		case AWB_MODE_TUNGSTEN:
							OV5646MIPI_write_cmos_sensor(0x3212,0x03);
							OV5646MIPI_set_AWB_mode(KAL_FALSE);                         
							OV5646MIPI_write_cmos_sensor(0x3400,0x05); 
							OV5646MIPI_write_cmos_sensor(0x3401,0x48); 
							OV5646MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5646MIPI_write_cmos_sensor(0x3404,0x05); 
							OV5646MIPI_write_cmos_sensor(0x3405,0xe0); 
							OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
							OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
							break;
        case AWB_MODE_FLUORESCENT:
							OV5646MIPI_write_cmos_sensor(0x3212,0x03);
							OV5646MIPI_set_AWB_mode(KAL_FALSE);                           
							OV5646MIPI_write_cmos_sensor(0x3400,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3401,0x00); 
							OV5646MIPI_write_cmos_sensor(0x3402,0x04); 
							OV5646MIPI_write_cmos_sensor(0x3403,0x00); 
							OV5646MIPI_write_cmos_sensor(0x3404,0x06); 
							OV5646MIPI_write_cmos_sensor(0x3405,0x50); 
							OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
							OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
							break;
        default:
			return FALSE;
    }
	spin_lock(&ov5646mipi_drv_lock);
    OV5646MIPISensor.iWB = para;
    spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_param_wb function:\n ");
       return TRUE;
} /* OV5646MIPI_set_param_wb */
void OV5646MIPI_set_contrast(UINT16 para)
{   
    OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_contrast function:\n ");
    switch (para)
    {
        case ISP_CONTRAST_LOW:
             OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5586,0x14);
			 OV5646MIPI_write_cmos_sensor(0x5585,0x14);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13);
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_CONTRAST_HIGH:
             OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5586,0x2c);
			 OV5646MIPI_write_cmos_sensor(0x5585,0x1c);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13);
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_CONTRAST_MIDDLE:
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5586,0x24);
			 OV5646MIPI_write_cmos_sensor(0x5585,0x10);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13);
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        default:
             break;
    }
    OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_contrast function:\n ");
    return;
}

void OV5646MIPI_set_brightness(UINT16 para)
{
    OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_brightness function:\n ");
    switch (para)
    {
        case ISP_BRIGHT_LOW:
             OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5587,0x40);
			 OV5646MIPI_write_cmos_sensor(0x5588,0x09);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13);
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_BRIGHT_HIGH:
             OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5587,0x40);
			 OV5646MIPI_write_cmos_sensor(0x5588,0x01);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13);
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_BRIGHT_MIDDLE:
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5587,0x00);
			 OV5646MIPI_write_cmos_sensor(0x5588,0x01);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13);
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        default:
             return KAL_FALSE;
             break;
    }
    OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_brightness function:\n ");
    return;
}
void OV5646MIPI_set_saturation(UINT16 para)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_saturation function:\n ");
	kal_uint32 temp;
    switch (para)
    {

        case ISP_SAT_HIGH:
			
			OV5646MIPI_write_cmos_sensor(0x3212,0x01);
			OV5646MIPISENSORDB("[ylf]OV5646MIPI_set_saturation ISP_SAT_HIGH OV5646MIPI_write_cmos_sensor:\n");
			OV5646MIPI_write_cmos_sensor(0x5384,0x0a);
			OV5646MIPI_write_cmos_sensor(0x5385,0x8C);
			OV5646MIPI_write_cmos_sensor(0x5386,0x96);
			OV5646MIPI_write_cmos_sensor(0x5387,0x9B);
			OV5646MIPI_write_cmos_sensor(0x5388,0x8B);
			OV5646MIPI_write_cmos_sensor(0x5389,0x10);
			OV5646MIPI_write_cmos_sensor(0x3212,0x11);
			OV5646MIPI_write_cmos_sensor(0x3212,0xa1);
			break;
        case ISP_SAT_LOW:
			OV5646MIPI_write_cmos_sensor(0x3212,0x01);
			OV5646MIPISENSORDB("[ylf]OV5646MIPI_set_saturation ISP_SAT_LOW OV5646MIPI_write_cmos_sensor:\n");
			OV5646MIPI_write_cmos_sensor(0x5384,0x06);
			OV5646MIPI_write_cmos_sensor(0x5385,0x5E);
			OV5646MIPI_write_cmos_sensor(0x5386,0x64);
			OV5646MIPI_write_cmos_sensor(0x5387,0x67);
			OV5646MIPI_write_cmos_sensor(0x5388,0x5D);
			OV5646MIPI_write_cmos_sensor(0x5389,0x0A);
			OV5646MIPI_write_cmos_sensor(0x3212,0x11);
			OV5646MIPI_write_cmos_sensor(0x3212,0xa1);
             break;
        case ISP_SAT_MIDDLE:
			OV5646MIPI_write_cmos_sensor(0x3212, 0x01);
			OV5646MIPISENSORDB("[ylf]OV5646MIPI_set_saturation ISP_SAT_MIDDLE OV5646MIPI_write_cmos_sensor:\n");
			OV5646MIPI_write_cmos_sensor(0x5384, 0x08);
			OV5646MIPI_write_cmos_sensor(0x5385, 0x75);
			OV5646MIPI_write_cmos_sensor(0x5386, 0x7D);
			OV5646MIPI_write_cmos_sensor(0x5387, 0x81);
			OV5646MIPI_write_cmos_sensor(0x5388, 0x74);
			OV5646MIPI_write_cmos_sensor(0x5389, 0x0D);
			OV5646MIPI_write_cmos_sensor(0x3212, 0x11);
			OV5646MIPI_write_cmos_sensor(0x3212, 0xa1);
			 break;
        default:
			 break;
    }
	mDELAY(50);
	temp=OV5646MIPIYUV_read_cmos_sensor(0x5384);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_saturation function:temp=%d\n",temp);
     return;
}
void OV5646MIPI_scene_mode_PORTRAIT()
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_scene_mode_SPORTS function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPI_write_cmos_sensor(0x3212,0x03);
	OV5646MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5646MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a03,0x88);                         
	OV5646MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a15,0x88);		 		 
	/*CMX*/			 		 		 		 
	OV5646MIPI_write_cmos_sensor(0x5381,0x1e);
	OV5646MIPISENSORDB("[ylf]OV5646MIPI_scene_mode_PORTRAIT OV5646MIPI_write_cmos_sensor:\n");
	OV5646MIPI_write_cmos_sensor(0x5382,0x5b);
	OV5646MIPI_write_cmos_sensor(0x5383,0x08);
	OV5646MIPI_write_cmos_sensor(0x5384,0x0a);
	OV5646MIPI_write_cmos_sensor(0x5385,0x7e);
	OV5646MIPI_write_cmos_sensor(0x5386,0x88);
	OV5646MIPI_write_cmos_sensor(0x5387,0x7c);
	OV5646MIPI_write_cmos_sensor(0x5388,0x6c);
	OV5646MIPI_write_cmos_sensor(0x5389,0x10);
	OV5646MIPI_write_cmos_sensor(0x538a,0x01);
	OV5646MIPI_write_cmos_sensor(0x538b,0x98);	

	/*AE Weight - Average*/
	OV5646MIPI_write_cmos_sensor(0x501d,0x00); 			 
	OV5646MIPI_write_cmos_sensor(0x5688,0x11);
	OV5646MIPI_write_cmos_sensor(0x5689,0x11);
	OV5646MIPI_write_cmos_sensor(0x568a,0x14);
	OV5646MIPI_write_cmos_sensor(0x568b,0x41);
	OV5646MIPI_write_cmos_sensor(0x568c,0x14);
	OV5646MIPI_write_cmos_sensor(0x568d,0x41);
	OV5646MIPI_write_cmos_sensor(0x568e,0x11);
	OV5646MIPI_write_cmos_sensor(0x568f,0x11);	
		/*FRAME rate*/
    OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5646MIPI_write_cmos_sensor(0x3212,0xa3);

	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_scene_mode_SPORTS function:\n ");
}

void OV5646MIPI_scene_mode_LANDSCAPE()
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_scene_mode_LANDSCAPE function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPI_write_cmos_sensor(0x3212,0x03);
	OV5646MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5646MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a03,0x88);                         
	OV5646MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a15,0x88);		 		 
	/*CMX*/			 		 		 		 
	OV5646MIPI_write_cmos_sensor(0x5381,0x1c);
	OV5646MIPISENSORDB("[ylf]OV5646MIPI_scene_mode_LANDSCAPE OV5646MIPI_write_cmos_sensor:\n");
	OV5646MIPI_write_cmos_sensor(0x5382,0x5a);
	OV5646MIPI_write_cmos_sensor(0x5383,0x06);
	OV5646MIPI_write_cmos_sensor(0x5384,0x0d);
	OV5646MIPI_write_cmos_sensor(0x5385,0xa4);
	OV5646MIPI_write_cmos_sensor(0x5386,0xb1);
	OV5646MIPI_write_cmos_sensor(0x5387,0xa1);
	OV5646MIPI_write_cmos_sensor(0x5388,0x8c);
	OV5646MIPI_write_cmos_sensor(0x5389,0x15);
	OV5646MIPI_write_cmos_sensor(0x538a,0x01);
	OV5646MIPI_write_cmos_sensor(0x538b,0x98);	

	/*AE Weight - Average*/
	OV5646MIPI_write_cmos_sensor(0x501d,0x00); 			 
	OV5646MIPI_write_cmos_sensor(0x5688,0x11);
	OV5646MIPI_write_cmos_sensor(0x5689,0x11);
	OV5646MIPI_write_cmos_sensor(0x568a,0x11);
	OV5646MIPI_write_cmos_sensor(0x568b,0x11);
	OV5646MIPI_write_cmos_sensor(0x568c,0x11);
	OV5646MIPI_write_cmos_sensor(0x568d,0x11);
	OV5646MIPI_write_cmos_sensor(0x568e,0x11);
	OV5646MIPI_write_cmos_sensor(0x568f,0x11);	
		/*FRAME rate*/
    OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5646MIPI_write_cmos_sensor(0x3212,0xa3);

	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_scene_mode_LANDSCAPE function:\n ");
}
void OV5646MIPI_scene_mode_SUNSET()
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_scene_mode_SUNSET function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPI_write_cmos_sensor(0x3212,0x03);
	OV5646MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5646MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a03,0x88);                         
	OV5646MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a15,0x88);		 		 
	/*CMX*/			 		 		 		 
/*		 		 		 		 
	OV5646MIPI_write_cmos_sensor(0x5381,0x42); 
	OV5646MIPISENSORDB("[ylf]OV5646MIPI_scene_mode_SUNSET OV5646MIPI_write_cmos_sensor:\n");
	OV5646MIPI_write_cmos_sensor(0x5382,0x5a); 
	OV5646MIPI_write_cmos_sensor(0x5383,0x08); 
	OV5646MIPI_write_cmos_sensor(0x5384,0x20); 
	OV5646MIPI_write_cmos_sensor(0x5385,0x7e); 
	OV5646MIPI_write_cmos_sensor(0x5386,0x48); 
	OV5646MIPI_write_cmos_sensor(0x5387,0xbc); 
	OV5646MIPI_write_cmos_sensor(0x5388,0x6c); 
	OV5646MIPI_write_cmos_sensor(0x5389,0x06); 
	OV5646MIPI_write_cmos_sensor(0x538a,0x01); 
	OV5646MIPI_write_cmos_sensor(0x538b,0x9c); 	
*/
	/*AE Weight - Average*/
	OV5646MIPI_write_cmos_sensor(0x501d,0x00); 			 
	OV5646MIPI_write_cmos_sensor(0x5688,0x11);
	OV5646MIPI_write_cmos_sensor(0x5689,0x11);
	OV5646MIPI_write_cmos_sensor(0x568a,0x14);
	OV5646MIPI_write_cmos_sensor(0x568b,0x41);
	OV5646MIPI_write_cmos_sensor(0x568c,0x14);
	OV5646MIPI_write_cmos_sensor(0x568d,0x41);
	OV5646MIPI_write_cmos_sensor(0x568e,0x11);
	OV5646MIPI_write_cmos_sensor(0x568f,0x11);	
		/*FRAME rate*/
    OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_scene_mode_SUNSET function:\n ");
}
void OV5646MIPI_scene_mode_SPORTS()
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_scene_mode_SPORTS function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPI_write_cmos_sensor(0x3212,0x03);
	OV5646MIPI_write_cmos_sensor(0x3A00,0x38); //10-30
	OV5646MIPI_write_cmos_sensor(0x3a02,0x03); 
	OV5646MIPI_write_cmos_sensor(0x3a03,0xd8);                         
	OV5646MIPI_write_cmos_sensor(0x3a14,0x03); 
	OV5646MIPI_write_cmos_sensor(0x3a15,0xd8);		 		 
	/*CMX*/			 		 		 		 
	OV5646MIPI_write_cmos_sensor(0x5381,0x21);//CCM
	OV5646MIPISENSORDB("[ylf]OV5646MIPI_scene_mode_SPORTS OV5646MIPI_write_cmos_sensor:\n");
	OV5646MIPI_write_cmos_sensor(0x5382,0x54);
	OV5646MIPI_write_cmos_sensor(0x5383,0x15);
	OV5646MIPI_write_cmos_sensor(0x5384,0x08);
	OV5646MIPI_write_cmos_sensor(0x5385,0x75);
	OV5646MIPI_write_cmos_sensor(0x5386,0x7D);
	OV5646MIPI_write_cmos_sensor(0x5387,0x81);
	OV5646MIPI_write_cmos_sensor(0x5388,0x74);
	OV5646MIPI_write_cmos_sensor(0x5389,0x0D);
	OV5646MIPI_write_cmos_sensor(0x538a,0x01);
	OV5646MIPI_write_cmos_sensor(0x538b,0x98);
	/*AE Weight - Average*/
	OV5646MIPI_write_cmos_sensor(0x501d,0x00); 			 
	OV5646MIPI_write_cmos_sensor(0x5688,0x11);
	OV5646MIPI_write_cmos_sensor(0x5689,0x11);
	OV5646MIPI_write_cmos_sensor(0x568a,0x11);
	OV5646MIPI_write_cmos_sensor(0x568b,0x11);
	OV5646MIPI_write_cmos_sensor(0x568c,0x11);
	OV5646MIPI_write_cmos_sensor(0x568d,0x11);
	OV5646MIPI_write_cmos_sensor(0x568e,0x11);
	OV5646MIPI_write_cmos_sensor(0x568f,0x11);	
		/*FRAME rate*/
    OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_scene_mode_SPORTS function:\n ");
}
void OV5646MIPI_scene_mode_night()
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_scene_mode_night function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.NightMode=KAL_TRUE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPI_write_cmos_sensor(0x3212,0x03);
	if(OV5646MIPISensor.VideoMode)
		OV5646MIPI_write_cmos_sensor(0x3A00,0x38); //video mode下 fps=30 固定帧率
	else
		OV5646MIPI_write_cmos_sensor(0x3A00,0x3c);//
	OV5646MIPI_write_cmos_sensor(0x3a02,0x17); 
	OV5646MIPI_write_cmos_sensor(0x3a03,0x10);						   
	OV5646MIPI_write_cmos_sensor(0x3a14,0x17); 
	OV5646MIPI_write_cmos_sensor(0x3a15,0x10);				 
	/*CMX*/ 								 
	OV5646MIPI_write_cmos_sensor(0x5381,0x21);//CCM
	OV5646MIPISENSORDB("[ylf]OV5646MIPI_scene_mode_night OV5646MIPI_write_cmos_sensor:\n");
	OV5646MIPI_write_cmos_sensor(0x5382,0x54);
	OV5646MIPI_write_cmos_sensor(0x5383,0x15);
	OV5646MIPI_write_cmos_sensor(0x5384,0x08);
	OV5646MIPI_write_cmos_sensor(0x5385,0x75);
	OV5646MIPI_write_cmos_sensor(0x5386,0x7D);
	OV5646MIPI_write_cmos_sensor(0x5387,0x81);
	OV5646MIPI_write_cmos_sensor(0x5388,0x74);
	OV5646MIPI_write_cmos_sensor(0x5389,0x0D);
	OV5646MIPI_write_cmos_sensor(0x538a,0x01);
	OV5646MIPI_write_cmos_sensor(0x538b,0x98);
	/*AE Weight - Average*/
	OV5646MIPI_write_cmos_sensor(0x501d,0x00);			 
	OV5646MIPI_write_cmos_sensor(0x5688,0x11);
	OV5646MIPI_write_cmos_sensor(0x5689,0x11);
	OV5646MIPI_write_cmos_sensor(0x568a,0x11);
	OV5646MIPI_write_cmos_sensor(0x568b,0x11);
	OV5646MIPI_write_cmos_sensor(0x568c,0x11);
	OV5646MIPI_write_cmos_sensor(0x568d,0x11);
	OV5646MIPI_write_cmos_sensor(0x568e,0x11);
	OV5646MIPI_write_cmos_sensor(0x568f,0x11);	
		/*FRAME rate*/
	OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
   OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_scene_mode_OFF function:\n ");
   
}
void OV5646MIPI_scene_mode_OFF()
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_scene_mode_OFF function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&ov5646mipi_drv_lock);
	OV5646MIPI_write_cmos_sensor(0x3212,0x03);
	OV5646MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5646MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a03,0x88);                         
	OV5646MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5646MIPI_write_cmos_sensor(0x3a15,0x88);		 		 
	/*CMX*/			 		 		 		 
	OV5646MIPI_write_cmos_sensor(0x5381, 0x21);//CCM
	OV5646MIPISENSORDB("[ylf]OV5646MIPI_scene_mode_OFF OV5646MIPI_write_cmos_sensor:\n");
	OV5646MIPI_write_cmos_sensor(0x5382, 0x54);
	OV5646MIPI_write_cmos_sensor(0x5383, 0x15);
	OV5646MIPI_write_cmos_sensor(0x5384, 0x08);
	OV5646MIPI_write_cmos_sensor(0x5385, 0x75);
	OV5646MIPI_write_cmos_sensor(0x5386, 0x7D);
	OV5646MIPI_write_cmos_sensor(0x5387, 0x81);
	OV5646MIPI_write_cmos_sensor(0x5388, 0x74);
	OV5646MIPI_write_cmos_sensor(0x5389, 0x0D);
	OV5646MIPI_write_cmos_sensor(0x538a,0x01);
	OV5646MIPI_write_cmos_sensor(0x538b,0x98);
	/*AE Weight - Average*/
	OV5646MIPI_write_cmos_sensor(0x501d,0x00); 			 
	OV5646MIPI_write_cmos_sensor(0x5688,0x11);
	OV5646MIPI_write_cmos_sensor(0x5689,0x11);
	OV5646MIPI_write_cmos_sensor(0x568a,0x11);
	OV5646MIPI_write_cmos_sensor(0x568b,0x11);
	OV5646MIPI_write_cmos_sensor(0x568c,0x11);
	OV5646MIPI_write_cmos_sensor(0x568d,0x11);
	OV5646MIPI_write_cmos_sensor(0x568e,0x11);
	OV5646MIPI_write_cmos_sensor(0x568f,0x11);	
		/*FRAME rate*/
    OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
	mDELAY(100);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_scene_mode_OFF function:\n ");
}

void OV5646MIPI_set_scene_mode(UINT16 para)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_scene_mode function:\n ");	
	OV5646MIPISENSORDB("[OV5646MIPI] OV5646MIPI_set_scene_mode=%d",para);	
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.sceneMode=para;
	spin_unlock(&ov5646mipi_drv_lock);
    switch (para)
    { 
		case SCENE_MODE_NIGHTSCENE:
			 OV5646MIPI_scene_mode_night(); 
			break;
        case SCENE_MODE_PORTRAIT:
			OV5646MIPI_scene_mode_PORTRAIT();		 
             break;
        case SCENE_MODE_LANDSCAPE:
			OV5646MIPI_scene_mode_LANDSCAPE();		 
             break;
        case SCENE_MODE_SUNSET:
			OV5646MIPISENSORDB("[OV5646MIPI]enter SCENE_MODE_SUNSET:\n ");
			OV5646MIPI_scene_mode_SUNSET();		 
            break;
        case SCENE_MODE_SPORTS:
            OV5646MIPI_scene_mode_SPORTS();		 
            break;
        case SCENE_MODE_HDR:
            if (1 == OV5646MIPISensor.manualAEStart)
            {
                OV5646MIPI_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&ov5646mipi_drv_lock);
            	OV5646MIPISensor.manualAEStart = 0;
                OV5646MIPISensor.currentExposureTime = 0;
                OV5646MIPISensor.currentAxDGain = 0;
				spin_unlock(&ov5646mipi_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
			OV5646MIPISENSORDB("[OV5646MIPI]set SCENE_MODE_OFF :\n ");
        default:
			OV5646MIPI_scene_mode_OFF();
			OV5646MIPISENSORDB("[OV5646MIPI]set default mode :\n ");
            break;
    }
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_scene_mode function:\n ");
	return;
}
void OV5646MIPI_set_iso(UINT16 para)
{
    spin_lock(&ov5646mipi_drv_lock);
    OV5646MIPISensor.isoSpeed = para;
    spin_unlock(&ov5646mipi_drv_lock);   
    switch (para)
    {
        case AE_ISO_100:
		case AE_ISO_AUTO:
             OV5646MIPI_write_cmos_sensor(0x3a18, 0x00);
             OV5646MIPI_write_cmos_sensor(0x3a19, 0x60);
             break;
        case AE_ISO_200:
             //ISO 200
             OV5646MIPI_write_cmos_sensor(0x3a18, 0x00);
             OV5646MIPI_write_cmos_sensor(0x3a19, 0x90);
             break;
        case AE_ISO_400:
             //ISO 400
             OV5646MIPI_write_cmos_sensor(0x3a18, 0x00);
             OV5646MIPI_write_cmos_sensor(0x3a19, 0xc0);
             break;
        default:
             break;
    }
    return;
}

BOOL OV5646MIPI_set_param_effect(UINT16 para)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_param_effect function:para=%d\n ",para);
	switch (para)
    {
        case MEFFECT_OFF:  
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
		   	 OV5646MIPI_write_cmos_sensor(0x5580,0x06); 
             OV5646MIPI_write_cmos_sensor(0x5583,0x40); 
             OV5646MIPI_write_cmos_sensor(0x5584,0x28);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
	         break;
        case MEFFECT_SEPIA: 
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5580,0x1e);
             OV5646MIPI_write_cmos_sensor(0x5583,0x40); 
             OV5646MIPI_write_cmos_sensor(0x5584,0xa0);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        case MEFFECT_NEGATIVE:
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5580,0x46);
			 OV5646MIPI_write_cmos_sensor(0x5583,0x40); 
             OV5646MIPI_write_cmos_sensor(0x5584,0x28);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        case MEFFECT_SEPIAGREEN:
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5580,0x1e);			 
			 OV5646MIPI_write_cmos_sensor(0x5583,0x60); 
			 OV5646MIPI_write_cmos_sensor(0x5584,0x60);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        case MEFFECT_SEPIABLUE:
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5580,0x1e);             
             OV5646MIPI_write_cmos_sensor(0x5583,0xa0); 
             OV5646MIPI_write_cmos_sensor(0x5584,0x40);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
		case MEFFECT_MONO: //B&W
			 OV5646MIPI_write_cmos_sensor(0x3212,0x03);
			 OV5646MIPI_write_cmos_sensor(0x5580,0x1e);      		
        	 OV5646MIPI_write_cmos_sensor(0x5583,0x80); 
        	 OV5646MIPI_write_cmos_sensor(0x5584,0x80);
			 OV5646MIPI_write_cmos_sensor(0x3212,0x13); 
			 OV5646MIPI_write_cmos_sensor(0x3212,0xa3);
        	 break;
        default:
             return KAL_FALSE;
    }
	mDELAY(50);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_param_effect function:\n ");
    return KAL_FALSE;
} /* OV5646MIPI_set_param_effect */

BOOL OV5646MIPI_set_param_banding(UINT16 para)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_param_banding function:\n ");
	switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
						spin_lock(&ov5646mipi_drv_lock);
						OV5646MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;
						spin_unlock(&ov5646mipi_drv_lock);
						OV5646MIPI_write_cmos_sensor(0x3c00,0x04);
						OV5646MIPI_write_cmos_sensor(0x3c01,0x80);
            			break;
        case AE_FLICKER_MODE_60HZ:			
						spin_lock(&ov5646mipi_drv_lock);
						OV5646MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;
						spin_unlock(&ov5646mipi_drv_lock);
						OV5646MIPI_write_cmos_sensor(0x3c00,0x00);
						OV5646MIPI_write_cmos_sensor(0x3c01,0x80);
            			break;
        default:
             return FALSE;
    }
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_param_banding function:\n ");
    return TRUE;
} /* OV5646MIPI_set_param_banding */

BOOL OV5646MIPI_set_param_exposure(UINT16 para)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_set_param_exposure function:\n ");
	OV5646MIPISENSORDB("[OV5646MIPI]para=%d:\n",para);
   if (SCENE_MODE_HDR == OV5646MIPISensor.sceneMode && 
    SENSOR_MODE_CAPTURE == OV5646MIPISensor.SensorMode)
   {
       OV5646MIPI_set_param_exposure_for_HDR(para);
       return TRUE;
   }
	switch (para)
    {	
       case AE_EV_COMP_20:	                   
				OV5646MIPI_write_cmos_sensor(0x3a0f, 0x50);//	; AEC in H
				OV5646MIPI_write_cmos_sensor(0x3a10, 0x48);//	; AEC in L
				OV5646MIPI_write_cmos_sensor(0x3a11, 0x90);//	; AEC out H
				OV5646MIPI_write_cmos_sensor(0x3a1b, 0x50);//	; AEC out L
				OV5646MIPI_write_cmos_sensor(0x3a1e, 0x48);//	; control zone H
				OV5646MIPI_write_cmos_sensor(0x3a1f, 0x24);//	; control zone L   
				break;
		case AE_EV_COMP_10:	                   
				OV5646MIPI_write_cmos_sensor(0x3a0f, 0x40);//	; AEC in H
				OV5646MIPI_write_cmos_sensor(0x3a10, 0x38);//	; AEC in L
				OV5646MIPI_write_cmos_sensor(0x3a11, 0x80);//	; AEC out H
				OV5646MIPI_write_cmos_sensor(0x3a1b, 0x40);//	; AEC out L
				OV5646MIPI_write_cmos_sensor(0x3a1e, 0x38);//	; control zone H
				OV5646MIPI_write_cmos_sensor(0x3a1f, 0x1c);//	; control zone L   
			  break;
		case AE_EV_COMP_00:
    			OV5646MIPI_write_cmos_sensor(0x3a0f, 0x29);//	; AEC in H
    			OV5646MIPI_write_cmos_sensor(0x3a10, 0x23);//	; AEC in L
    			OV5646MIPI_write_cmos_sensor(0x3a11, 0x53);//	; AEC out H
    			OV5646MIPI_write_cmos_sensor(0x3a1b, 0x29);//	; AEC out L
    			OV5646MIPI_write_cmos_sensor(0x3a1e, 0x23);//	; control zone H
    			OV5646MIPI_write_cmos_sensor(0x3a1f, 0x12);//	; control zone L  
			  break;
   		 case AE_EV_COMP_n10:
				OV5646MIPI_write_cmos_sensor(0x3a0f, 0x20);//	; AEC in H
				OV5646MIPI_write_cmos_sensor(0x3a10, 0x18);//	; AEC in L
				OV5646MIPI_write_cmos_sensor(0x3a11, 0x41);//	; AEC out H
				OV5646MIPI_write_cmos_sensor(0x3a1b, 0x20);//	; AEC out L
				OV5646MIPI_write_cmos_sensor(0x3a1e, 0x18);//	; control zone H
				OV5646MIPI_write_cmos_sensor(0x3a1f, 0x10);//	; control zone L   
			  break;
      	case AE_EV_COMP_n20:  // -2 EV
        	    OV5646MIPI_write_cmos_sensor(0x3a0f, 0x18);//	; AEC in H
				OV5646MIPI_write_cmos_sensor(0x3a10, 0x14);//	; AEC in L
				OV5646MIPI_write_cmos_sensor(0x3a11, 0x32);//	; AEC out H
				OV5646MIPI_write_cmos_sensor(0x3a1b, 0x18);//	; AEC out L
				OV5646MIPI_write_cmos_sensor(0x3a1e, 0x14);//	; control zone H
				OV5646MIPI_write_cmos_sensor(0x3a1f, 0x0c);//	; control zone L
        	  break;
        default:
              return FALSE;
    }
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_set_param_exposure function:\n ");
    return TRUE;
} /* OV5646MIPI_set_param_exposure */

UINT32 OV5646MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	OV5646MIPISENSORDB("OV5646MIPIYUVSensorSetting:iCmd=%d,iPara=%d, %d \n",iCmd, iPara);
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIYUVSensorSetting function:\n ");
	switch (iCmd) 
	{
		case FID_SCENE_MODE:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_SCENE_MODE\n");
			OV5646MIPI_set_scene_mode(iPara);
	    	break; 	    
		case FID_AWB_MODE:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_AWB_MODE para=%d\n", iPara);
			OV5646MIPI_set_param_wb(iPara);
			  break;
		case FID_COLOR_EFFECT:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_COLOR_EFFECT para=%d\n", iPara);
			OV5646MIPI_set_param_effect(iPara);
		 	  break;
		case FID_AE_EV:   
			OV5646MIPI_set_param_exposure(iPara);
			OV5646MIPISENSORDB("[OV5646MIPI]FID_AE_EV para=%d\n", iPara);
		    break;
		case FID_AE_FLICKER:
			 OV5646MIPISENSORDB("[OV5646MIPI]FID_AE_FLICKER para=%d\n", iPara);
			 OV5646MIPI_set_param_banding(iPara);
		 	 break;
		case FID_AE_SCENE_MODE: 
			 OV5646MIPISENSORDB("[OV5646MIPI]FID_AE_SCENE_MODE para=%d\n", iPara);
        break; 
		case FID_ISP_CONTRAST:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_ISP_CONTRAST para=%d\n", iPara);
            OV5646MIPI_set_contrast(iPara);
            break;
        case FID_ISP_BRIGHT:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_ISP_BRIGHT para=%d\n", iPara);
            OV5646MIPI_set_brightness(iPara);
            break;
        case FID_ISP_SAT:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_ISP_SAT para=%d\n", iPara);
            OV5646MIPI_set_saturation(iPara);
            break;
    	case FID_ZOOM_FACTOR:
   		    OV5646MIPISENSORDB("[OV5646MIPI]FID_ZOOM_FACTOR:%d\n", iPara); 	    
			spin_lock(&ov5646mipi_drv_lock);
	        zoom_factor = iPara; 
			spin_unlock(&ov5646mipi_drv_lock);
            break;
		case FID_AE_ISO:
			OV5646MIPISENSORDB("[OV5646MIPI]FID_AE_ISO:%d\n", iPara);
            OV5646MIPI_set_iso(iPara);
            break;           
	  	default:
		    break;
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIYUVSensorSetting function:\n ");
	return TRUE;
}   /* OV5646MIPIYUVSensorSetting */

UINT32 OV5646MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIYUVSetVideoMode function:\n ");
	spin_lock(&ov5646mipi_drv_lock);
	OV5646MIPISensor.VideoMode =KAL_TRUE;
	spin_unlock(&ov5646mipi_drv_lock);
	if (u2FrameRate == 30)
	{
		//;OV5646MIPI 1280x960,30fps
		//56Mhz, 224Mbps/Lane, 2Lane.
		OV5646MIPISENSORDB("[OV5646MIPI]OV5646MIPIYUVSetVideoMode enter u2FrameRate == 30 setting  :\n ");	
		OV5646MIPI_write_cmos_sensor(0x3A00, 0x38); //fix30
		OV5646MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
		OV5646MIPI_write_cmos_sensor(0x3034, 0x18); // PLL, MIPI 8-bit mode
		OV5646MIPI_write_cmos_sensor(0x3035, 0x21); // PLL
		OV5646MIPI_write_cmos_sensor(0x3036, 0x70); // PLL
		OV5646MIPI_write_cmos_sensor(0x3037, 0x13); // PLL
		OV5646MIPI_write_cmos_sensor(0x3108, 0x01); // PLL
		OV5646MIPI_write_cmos_sensor(0x3824, 0x01); // PLL
		OV5646MIPI_write_cmos_sensor(0x460c, 0x20); // PLL
		OV5646MIPI_write_cmos_sensor(0x3618, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x3600, 0x09);//
		OV5646MIPI_write_cmos_sensor(0x3601, 0x43);//
		OV5646MIPI_write_cmos_sensor(0x3708, 0x66);//
		OV5646MIPI_write_cmos_sensor(0x3709, 0x12);//
		OV5646MIPI_write_cmos_sensor(0x370c, 0xc3);//
		OV5646MIPI_write_cmos_sensor(0x3800, 0x00); // HS = 0
		OV5646MIPI_write_cmos_sensor(0x3801, 0x00); // HS
		OV5646MIPI_write_cmos_sensor(0x3802, 0x00); // VS = 250
		OV5646MIPI_write_cmos_sensor(0x3803, 0x06); // VS
		OV5646MIPI_write_cmos_sensor(0x3804, 0x0a); // HW = 2623
		OV5646MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
		OV5646MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 
		OV5646MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
		OV5646MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
		OV5646MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
		OV5646MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPVO = 960
		OV5646MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
		OV5646MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
		OV5646MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
		OV5646MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
		OV5646MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
		OV5646MIPI_write_cmos_sensor(0x3810, 0x00); // H OFF = 16
		OV5646MIPI_write_cmos_sensor(0x3811, 0x10); // H OFF
		OV5646MIPI_write_cmos_sensor(0x3812, 0x00); // V OFF = 4
		OV5646MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
		OV5646MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
		OV5646MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
		OV5646MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
		OV5646MIPI_write_cmos_sensor(0x3821, 0x07);//	; mirror on, H bin on
		OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
		OV5646MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
		OV5646MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
		OV5646MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
		OV5646MIPI_write_cmos_sensor(0x3a08, 0x01);//	; B50 = 222
		OV5646MIPI_write_cmos_sensor(0x3a09, 0x27);//	; B50
		OV5646MIPI_write_cmos_sensor(0x3a0a, 0x00);//	; B60 = 185
		OV5646MIPI_write_cmos_sensor(0x3a0b, 0xf6);//	; B60
		OV5646MIPI_write_cmos_sensor(0x3a0e, 0x03);//	; max 50
		OV5646MIPI_write_cmos_sensor(0x3a0d, 0x04);//	; max 60
		OV5646MIPI_write_cmos_sensor(0x3a14, 0x03);//	; max exp 50 = 740
		OV5646MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
		OV5646MIPI_write_cmos_sensor(0x3c07, 0x07);//	; 50/60 auto detect
		OV5646MIPI_write_cmos_sensor(0x3c08, 0x01);//	; 50/60 auto detect
		OV5646MIPI_write_cmos_sensor(0x3c09, 0xc2);//	; 50/60 auto detect
		OV5646MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
		OV5646MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC triggered by gain change
		OV5646MIPI_write_cmos_sensor(0x4837, 0x11); // MIPI global timing 16           
		OV5646MIPI_write_cmos_sensor(0x503d, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x5000, 0xa7);//
		OV5646MIPI_write_cmos_sensor(0x5001, 0xa3);//
		OV5646MIPI_write_cmos_sensor(0x5002, 0x80);//
		OV5646MIPI_write_cmos_sensor(0x5003, 0x08);//
		OV5646MIPI_write_cmos_sensor(0x3032, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x4000, 0x89);//
		OV5646MIPI_write_cmos_sensor(0x350c, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x350d, 0x00);//
		OV5646MIPISENSORDB("[OV5646MIPI]OV5646MIPIYUVSetVideoMode exit u2FrameRate == 30 setting  :\n ");
		}
    else if (u2FrameRate == 15)   
	{
		//;OV5646MIPI 1280x960,15fps
		//28Mhz, 112Mbps/Lane, 2Lane.
		OV5646MIPISENSORDB("[OV5646MIPI]OV5646MIPIYUVSetVideoMode enter u2FrameRate == 15 setting  :\n ");
		OV5646MIPI_write_cmos_sensor(0x3A00, 0x38); //fix15
		OV5646MIPI_write_cmos_sensor(0x300e, 0x45);//	; MIPI 2 lane
		OV5646MIPI_write_cmos_sensor(0x3034, 0x18); // PLL, MIPI 8-bit mode
		OV5646MIPI_write_cmos_sensor(0x3035, 0x21); // PLL
		OV5646MIPI_write_cmos_sensor(0x3036, 0x38); // PLL
		OV5646MIPI_write_cmos_sensor(0x3037, 0x13); // PLL
		OV5646MIPI_write_cmos_sensor(0x3108, 0x01); // PLL
		OV5646MIPI_write_cmos_sensor(0x3824, 0x01); // PLL
		OV5646MIPI_write_cmos_sensor(0x460c, 0x20); // PLL
		OV5646MIPI_write_cmos_sensor(0x3618, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x3600, 0x09);//
		OV5646MIPI_write_cmos_sensor(0x3601, 0x43);//
		OV5646MIPI_write_cmos_sensor(0x3708, 0x66);//
		OV5646MIPI_write_cmos_sensor(0x3709, 0x12);//
		OV5646MIPI_write_cmos_sensor(0x370c, 0xc3);//
		OV5646MIPI_write_cmos_sensor(0x3800, 0x00); // HS = 0
		OV5646MIPI_write_cmos_sensor(0x3801, 0x00); // HS
		OV5646MIPI_write_cmos_sensor(0x3802, 0x00); // VS = 250
		OV5646MIPI_write_cmos_sensor(0x3803, 0x06); // VS
		OV5646MIPI_write_cmos_sensor(0x3804, 0x0a); // HW = 2623
		OV5646MIPI_write_cmos_sensor(0x3805, 0x3f);//	; HW
		OV5646MIPI_write_cmos_sensor(0x3806, 0x07);//	; VH = 
		OV5646MIPI_write_cmos_sensor(0x3807, 0x9d);//	; VH
		OV5646MIPI_write_cmos_sensor(0x3808, 0x05);//	; DVPHO = 1280
		OV5646MIPI_write_cmos_sensor(0x3809, 0x00);//	; DVPHO
		OV5646MIPI_write_cmos_sensor(0x380a, 0x03);//	; DVPVO = 960
		OV5646MIPI_write_cmos_sensor(0x380b, 0xc0);//	; DVPVO
		OV5646MIPI_write_cmos_sensor(0x380c, 0x07);//	; HTS = 2160
		OV5646MIPI_write_cmos_sensor(0x380d, 0x68);//	; HTS
		OV5646MIPI_write_cmos_sensor(0x380e, 0x03);//	; VTS = 740
		OV5646MIPI_write_cmos_sensor(0x380f, 0xd8);//	; VTS
		OV5646MIPI_write_cmos_sensor(0x3810, 0x00); // H OFF = 16
		OV5646MIPI_write_cmos_sensor(0x3811, 0x10); // H OFF
		OV5646MIPI_write_cmos_sensor(0x3812, 0x00); // V OFF = 4
		OV5646MIPI_write_cmos_sensor(0x3813, 0x06);//	; V OFF
		OV5646MIPI_write_cmos_sensor(0x3814, 0x31);//	; X INC
		OV5646MIPI_write_cmos_sensor(0x3815, 0x31);//	; Y INC
		OV5646MIPI_write_cmos_sensor(0x3820, 0x41);//	; flip off, V bin on
		OV5646MIPI_write_cmos_sensor(0x3821, 0x07);//	; mirror on, H bin on
		OV5646MIPI_write_cmos_sensor(0x4514, 0x00);
		OV5646MIPI_write_cmos_sensor(0x3a00, 0x38);//	; ae mode	
		OV5646MIPI_write_cmos_sensor(0x3a02, 0x03);//	; max exp 60 = 740
		OV5646MIPI_write_cmos_sensor(0x3a03, 0xd8);//	; max exp 60
		OV5646MIPI_write_cmos_sensor(0x3a08, 0x00);//	; B50 = 222
		OV5646MIPI_write_cmos_sensor(0x3a09, 0x94);//	; B50
		OV5646MIPI_write_cmos_sensor(0x3a0a, 0x00);//	; B60 = 185
		OV5646MIPI_write_cmos_sensor(0x3a0b, 0x7b);//	; B60
		OV5646MIPI_write_cmos_sensor(0x3a0e, 0x06);//	; max 50
		OV5646MIPI_write_cmos_sensor(0x3a0d, 0x07);//	; max 60
		OV5646MIPI_write_cmos_sensor(0x3a14, 0x03);//	; max exp 50 = 740
		OV5646MIPI_write_cmos_sensor(0x3a15, 0xd8);//	; max exp 50
		OV5646MIPI_write_cmos_sensor(0x3c07, 0x08);//	; 50/60 auto detect
		OV5646MIPI_write_cmos_sensor(0x3c08, 0x00);//	; 50/60 auto detect
		OV5646MIPI_write_cmos_sensor(0x3c09, 0x1c);//	; 50/60 auto detect
		OV5646MIPI_write_cmos_sensor(0x4004, 0x02);//	; BLC line number
		OV5646MIPI_write_cmos_sensor(0x4005, 0x18);//	; BLC triggered by gain change
		OV5646MIPI_write_cmos_sensor(0x4837, 0x11); // MIPI global timing 16           
		OV5646MIPI_write_cmos_sensor(0x503d, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x5000, 0xa7);//
		OV5646MIPI_write_cmos_sensor(0x5001, 0xa3);//
		OV5646MIPI_write_cmos_sensor(0x5002, 0x80);//
		OV5646MIPI_write_cmos_sensor(0x5003, 0x08);//
		OV5646MIPI_write_cmos_sensor(0x3032, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x4000, 0x89);//
		OV5646MIPI_write_cmos_sensor(0x350c, 0x00);//
		OV5646MIPI_write_cmos_sensor(0x350d, 0x00);//
		OV5646MIPISENSORDB("[OV5646MIPI]OV5646MIPIYUVSetVideoMode exit u2FrameRate == 15 setting  :\n ");
	}   
    else 
    {
        printk("Wrong frame rate setting \n");
    } 
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIYUVSetVideoMode function:\n ");
    return TRUE; 
}

/**************************/
static void OV5646MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIGetEvAwbRef function:\n ");
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIGetEvAwbRef function:\n ");
}

static void OV5646MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIGetCurAeAwbInfo function:\n ");
	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=OV5646MIPIReadShutter();
	Info->SensorAECur.AeCurGain=OV5646MIPIReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=((OV5646MIPIYUV_read_cmos_sensor(0x3401)&&0xff)+((OV5646MIPIYUV_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=((OV5646MIPIYUV_read_cmos_sensor(0x3405)&&0xff)+((OV5646MIPIYUV_read_cmos_sensor(0x3404)&&0xff)*256));
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIGetCurAeAwbInfo function:\n ");
}
UINT32 OV5646MIPIMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
	{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		OV5646MIPISENSORDB("OV5646MIPIMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIMaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = OV5646MIPI_IMAGE_SENSOR_SVGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV5646MIPI_IMAGE_SENSOR_SVGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&ov5646mipi_drv_lock);
				OV5646MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
				OV5646MIPISensor.PreviewDummyLines = dummyLine;
				spin_unlock(&ov5646mipi_drv_lock);
				//OV5646MIPISetDummy(OV5646MIPISensor.PreviewDummyPixels, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				lineLength = OV5646MIPI_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV5646MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&ov5646mipi_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&ov5646mipi_drv_lock);
				//OV5646MIPISetDummy(0, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:			
				pclk = 90000000;
				lineLength = OV5646MIPI_IMAGE_SENSOR_QSXGA_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV5646MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&ov5646mipi_drv_lock);
				OV5646MIPISensor.CaptureDummyLines = dummyLine;
				OV5646MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&ov5646mipi_drv_lock);
				//OV5646MIPISetDummy(OV5646MIPISensor.CaptureDummyPixels, dummyLine);			
				break;		
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
				break;		
			default:
				break;
		}	
		OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIMaxFramerateByScenario function:\n ");
		return ERROR_NONE;
	}
UINT32 OV5646MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPIGetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 150;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIGetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void OV5646MIPI_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	OV5646MIPISENSORDB("[OV5646MIPI]OV5646MIPI_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_get_AEAWB_lock function:\n ");
}
void OV5646MIPI_GetDelayInfo(UINT32 delayAddr)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter OV5646MIPI_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=0;
	pDelayInfo->EffectDelay=0;
	pDelayInfo->AwbDelay=0;
	pDelayInfo->AFSwitchDelayFrame=50;
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPI_GetDelayInfo function:\n ");
}
void OV5646MIPI_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	OV5646MIPISENSORDB("[OV5646MIPI]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&ov5646mipi_drv_lock);
          OV5646MIPISensor.userAskAeLock = TRUE;
          spin_unlock(&ov5646mipi_drv_lock);
          OV5646MIPI_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&ov5646mipi_drv_lock);
          OV5646MIPISensor.userAskAeLock = FALSE;
          spin_unlock(&ov5646mipi_drv_lock);
          OV5646MIPI_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&ov5646mipi_drv_lock);
          OV5646MIPISensor.userAskAwbLock = TRUE;
          spin_unlock(&ov5646mipi_drv_lock);
          OV5646MIPI_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&ov5646mipi_drv_lock);
          OV5646MIPISensor.userAskAwbLock = FALSE;
          spin_unlock(&ov5646mipi_drv_lock);
          OV5646MIPI_set_AWB_mode_UNLOCK();
      break;
      default:
      	break;
   }
   OV5646MIPISENSORDB("[OV5646MIPI]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}
#define FLASH_BV_THRESHOLD 0x25 
static void OV5646MIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	unsigned int NormBr;	   
	NormBr = OV5646MIPIYUV_read_cmos_sensor(0x56A1); 
	if (NormBr > FLASH_BV_THRESHOLD)
	{
	   *pFeatureReturnPara32 = FALSE;
		return;
	}
	*pFeatureReturnPara32 = TRUE;
	return;
}

UINT32 OV5646MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	OV5646MIPISENSORDB("[OV5646MIPI]enter[OV5646MIPIFeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV5646MIPI_IMAGE_SENSOR_QSXGA_WITDH;
			*pFeatureReturnPara16=OV5646MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=OV5646MIPI_FULL_PERIOD_PIXEL_NUMS + OV5646MIPISensor.CaptureDummyPixels;
					*pFeatureReturnPara16=OV5646MIPI_FULL_PERIOD_LINE_NUMS + OV5646MIPISensor.CaptureDummyLines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=OV5646MIPI_PV_PERIOD_PIXEL_NUMS + OV5646MIPISensor.PreviewDummyPixels;
					*pFeatureReturnPara16=OV5646MIPI_PV_PERIOD_LINE_NUMS + OV5646MIPISensor.PreviewDummyLines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV5646MIPISensor.ZsdturePclk * 1000 *100;	 //unit: Hz				
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV5646MIPISensor.PreviewPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		/**********************Strobe Ctrl Start *******************************/
		case SENSOR_FEATURE_SET_ESHUTTER:
			OV5646MIPISENSORDB("[OV5646MIPI] F_SET_ESHUTTER: Not Support\n");
			break;
		case SENSOR_FEATURE_SET_GAIN:
			OV5646MIPISENSORDB("[OV5646MIPI] F_SET_GAIN: Not Support\n");
			break;
		case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
			OV5646MIPISENSORDB("[OV5646MIPI] F_GET_AE_FLASHLIGHT_INFO: Not Support\n");
			break;
	    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
            OV5646MIPI_FlashTriggerCheck(pFeatureData32);
            OV5646MIPISENSORDB("[OV5646MIPI] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", *pFeatureData32);
            break;		
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			OV5646MIPISENSORDB("OV5646MIPI SENSOR_FEATURE_SET_FLASHLIGHT\n");
			break;
		/**********************Strobe Ctrl End *******************************/
		
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			OV5646MIPISENSORDB("[OV5646MIPI] F_SET_ISP_MASTER_CLOCK_FREQ\n");
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV5646MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV5646MIPIYUV_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV5646MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=0;
            *pFeatureParaLen=4;	   
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			OV5646MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;	
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            OV5646MIPI_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_SET_VIDEO_MODE\n");
		    OV5646MIPIYUVSetVideoMode(*pFeatureData16);
		    break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_CHECK_SENSOR_ID\n");
			OV5646MIPI_GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_EV_AWB_REF\n");
			OV5646MIPIGetEvAwbRef(*pFeatureData32);
			break;		
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN\n");
			OV5646MIPIGetCurAeAwbInfo(*pFeatureData32);			
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_EXIF_INFO\n");
            OV5646MIPIGetExifInfo(*pFeatureData32);
            break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_DELAY_INFO\n");
			OV5646MIPI_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_SLAVE_I2C_ID:
             OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_SET_SLAVE_I2C_ID\n");
             OV5646MIPI_sensor_socket = *pFeatureData32;
             break;
		case SENSOR_FEATURE_SET_TEST_PATTERN: 
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_SET_TEST_PATTERN\n");
			OV5646SetTestPatternMode((BOOL)*pFeatureData16);            
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			OV5646MIPISENSORDB("[OV5646MIPI]OV5646_TEST_PATTERN_CHECKSUM\n");
			*pFeatureReturnPara32=OV5646_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;				
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO\n");
			OV5646MIPIMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:\
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
			OV5646MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
	    /**********************below is AF control**********************/	
		case SENSOR_FEATURE_INITIALIZE_AF:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_INITIALIZE_AF\n");
		//	OV5646_FOCUS_OVT_AFC_Init();
            break;
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
            OV5646_FOCUS_Move_to(*pFeatureData16);
            break;
		case SENSOR_FEATURE_GET_AF_STATUS:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
            OV5646_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
			OV5646_FOCUS_OVT_AFC_Single_Focus();
            break;
		case SENSOR_FEATURE_CONSTANT_AF:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_CONSTANT_AF\n");
			OV5646_FOCUS_OVT_AFC_Constant_Focus();
			break;
		case SENSOR_FEATURE_CANCEL_AF:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_CANCEL_AF\n");
            OV5646_FOCUS_OVT_AFC_Cancel_Focus();
            break;
		case SENSOR_FEATURE_GET_AF_INF:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_AF_INF\n");
            OV5646_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_GET_AF_MACRO:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_AF_MACRO\n");
            OV5646_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_SET_AF_WINDOW: 
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_SET_AF_WINDOW\n");
			OV5646_FOCUS_Set_AF_Window(*pFeatureData32);
            break;       					
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
            OV5646_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break; 			
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			OV5646MIPISENSORDB("[OV5646MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
			OV5646MIPI_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;					                              	               
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			OV5646MIPISENSORDB("[OV5646MIPI]AE zone addr = 0x%x\n",*pFeatureData32);
            OV5646_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_SET_AE_WINDOW:
            OV5646MIPISENSORDB("[OV5646MIPI]AE zone addr = 0x%x\n",*pFeatureData32);			
            OV5646_FOCUS_Set_AE_Window(*pFeatureData32);
            break; 
		default:
			break;			
	}
	OV5646MIPISENSORDB("[OV5646MIPI]exit OV5646MIPIFeatureControl function:\n ");
	return ERROR_NONE;
}	/* OV5646MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncOV5646MIPI=
{
	OV5646MIPIOpen,
	OV5646MIPIGetInfo,
	OV5646MIPIGetResolution,
	OV5646MIPIFeatureControl,
	OV5646MIPIControl,
	OV5646MIPIClose
};

UINT32 OV5646_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV5646MIPI;
	return ERROR_NONE;
}	/* SensorInit() */



