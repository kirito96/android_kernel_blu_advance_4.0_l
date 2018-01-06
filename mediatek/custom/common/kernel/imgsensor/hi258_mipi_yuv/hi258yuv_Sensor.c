#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "hi258yuv_Sensor.h"
#include "hi258yuv_Camera_Sensor_para.h"
#include "hi258yuv_CameraCustomized.h"
#define FLASH_BV_THRESHOLD 0xb0

#define HI258_DEBUG
#ifdef HI258_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

MSDK_SENSOR_CONFIG_STRUCT HI258SensorConfigData;
#define SENSOR_CORE_PCLK	83200000	//48M PCLK Output 78000000 

#define WINMO_USE 0
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT
#define MIPI_INTERFACE

kal_bool HI258_VEDIO_MPEG4 = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4);

kal_uint8 HI258_Sleep_Mode;
kal_uint32 HI258_PV_dummy_pixels=616,HI258_PV_dummy_lines=20,HI258_isp_master_clock=260/*0*/;

static HI258_SENSOR_INFO_ST HI258_sensor;
static HI258_OPERATION_STATE_ST HI258_op_state;

static kal_uint32 HI258_zoom_factor = 0; 
static kal_bool HI258_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool HI258_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool HI258_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint8 HI258_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add
static kal_uint16  HI258_PV_Shutter = 0;
static kal_uint32  HI258_sensor_pclk=260;//520 //20110518

static kal_uint32 HI258_pv_HI258_exposure_lines=0x05f370, HI258_cp_HI258_exposure_lines=0;
static kal_uint16 HI258_Capture_Max_Gain16= 6*16;
static kal_uint16 HI258_Capture_Gain16=0 ;    
static kal_uint16 HI258_Capture_Shutter=0;
static kal_uint16 HI258_Capture_Extra_Lines=0;

static int HI258_CAPATURE_FLAG = 0;//Add By Paul
static int HI258_CAPATUREB_FLAG = 0;//Add By Paul


static kal_uint16  HI258_PV_Gain16 = 0;
static kal_uint16  HI258_PV_Extra_Lines = 0;
kal_uint32 HI258_capture_pclk_in_M=520,HI258_preview_pclk_in_M=390;
struct
{
  kal_bool    NightMode;
  kal_uint8   ZoomFactor; /* Zoom Index */
  kal_uint16  Banding;
  kal_uint32  PvShutter;
  kal_uint32  PvDummyPixels;
  kal_uint32  PvDummyLines;
  kal_uint32  CapDummyPixels;
  kal_uint32  CapDummyLines;
  kal_uint32  PvOpClk;
  kal_uint32  CapOpClk;
  
  /* Video frame rate 300 means 30.0fps. Unit Multiple 10. */
  kal_uint32  MaxFrameRate; 
  kal_uint32  MiniFrameRate; 
  /* Sensor Register backup. */
  kal_uint8   VDOCTL2; /* P0.0x11. */
  kal_uint8   ISPCTL3; /* P10.0x12. */
  kal_uint8   ISPCTL4; /* P10.0x13. */
  kal_uint8   AECTL1;  /* P20.0x10. */
  kal_uint8   AWBCTL1; /* P22.0x10. */
  kal_uint8   SATCTL;  /* P10.0x60. */

  kal_uint8   wb; /* P22.0x10. */
  kal_uint8   Effect;
  kal_uint8   Brightness;
} HI258Status;
//extern static CAMERA_DUAL_CAMERA_SENSOR_ENUM g_currDualSensorIdx;
//extern static char g_currSensorName[32];
//extern int kdModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name);

//extern int iReadReg_Byte(u8 addr, u8 *buf, u8 i2cId);
//extern int iWriteReg_Byte(u8 addr, u8 buf, u32 size, u16 i2cId);
//SENSOR_REG_STRUCT HI258SensorCCT[FACTORY_END_ADDR]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
//SENSOR_REG_STRUCT HI258SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
//	camera_para.SENSOR.cct	SensorCCT	=> SensorCCT
//	camera_para.SENSOR.reg	SensorReg

BOOL HI258_set_param_banding(UINT16 para);

//extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff, u16 i2cId);
//extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
/*ergate-017*/
//extern int iWriteRegI2C_ext(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId, u16 speed);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static void HI258_set_mirror_flip(kal_uint8 image_mirror);
kal_uint16 HI258_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    //char puSendCmd[2] = {(char)(addr & 0xFF) ,(char)(para & 0xFF)};

    //iWriteRegI2C_ext(puSendCmd , 2,HI258_WRITE_ID_0, 50);
    //iWriteReg_Byte(addr, para, 1, HI258_WRITE_ID_0);
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
    iWriteRegI2C(puSendCmd , 2,HI258_WRITE_ID_0);
    return 0;
}

kal_uint8 HI258_read_cmos_sensor(kal_uint8 addr)
{
    //kal_uint8 get_byte=0;
    //iReadReg_Byte(addr, &get_byte, HI258_WRITE_ID_0);
    //return get_byte;
    
    kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
    iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte,1,HI258_WRITE_ID_0);
    return get_byte;
}
#define GPIO_SUB_CAM_ID   GPIO118
#define GPIO_SUB_CAM_ID_M_GPIO   GPIO_MODE_00 
static kal_uint32 HI258_GetSensorID(kal_uint32 *sensorID)
{
    kal_uint16 sensor_id=0,hw_id=0;
    volatile signed char i;
    for(i=0;i<3;i++){
        sensor_id = HI258_read_cmos_sensor(0x04);
   /*     
        mt_set_gpio_mode(GPIO_SUB_CAM_ID,GPIO_SUB_CAM_ID_M_GPIO);
        mt_set_gpio_dir(GPIO_SUB_CAM_ID,GPIO_DIR_IN);
	 mt_set_gpio_pull_enable(GPIO_SUB_CAM_ID, GPIO_PULL_ENABLE);
        hw_id=mt_get_gpio_in(GPIO_SUB_CAM_ID);
        mdelay(1);
        hw_id=mt_get_gpio_in(GPIO_SUB_CAM_ID);*/

	//mdelay(3000);	
        printk("[HI258] sensor id = 0x%x,====hw_id=%d\n", sensor_id,hw_id);
        
        if (HI258_SENSOR_ID == sensor_id)
            break;
    }
printk("[HI255]  sensor_id = %d  hw_id = %d",sensor_id , hw_id);
    *sensorID=sensor_id;
    if (HI258_SENSOR_ID == sensor_id && 0x00 == hw_id)
    {
        printk("[HI255] this is cmk HI258 camera module.");
    }
    else
    {
        *sensorID=0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;    
}


//#define HI258_LOAD_FROM_T_FLASH
#ifdef HI258_LOAD_FROM_T_FLASH

static kal_uint8 fromsd = 0;
//kal_uint16 HI258_write_cmos_sensor(kal_uint8 addr, kal_uint8 para);

#define HI256_OP_CODE_INI		0x00		/* Initial value. */
#define HI256_OP_CODE_REG		0x01		/* Register */
#define HI256_OP_CODE_DLY		0x02		/* Delay */
#define HI256_OP_CODE_END		0x03		/* End of initial setting. */

	typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} HI256_initial_set_struct;

	HI256_initial_set_struct HI256_Init_Reg[2000];
	
	u32 strtol(const char *nptr, u8 base)
	{
		u8 ret;
		if(!nptr || (base!=16 && base!=10 && base!=8))
		{
			printk("%s(): NULL pointer input\n", __FUNCTION__);
			return -1;
		}
		for(ret=0; *nptr; nptr++)
		{
			if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
			{
				ret *= base;
				if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
				else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
				else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
				else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
			}
			else
				return ret;
		}
		return ret;
	}

	u8 Hi258_Initialize_from_T_Flash()
	{
		//FS_HANDLE fp = -1;				/* Default, no file opened. */
		//u8 *data_buff = NULL;
		u8 *curr_ptr = NULL;
		u32 file_size = 0;
		//u32 bytes_read = 0;
		u32 i = 0, j = 0;
		u8 func_ind[4] = {0};	/* REG or DLY */


		struct file *fp; 
		mm_segment_t fs; 
		loff_t pos = 0; 
		static u8 data_buff[50*1024] ;

		fp = filp_open("/storage/sdcard0/hi258_sd.dat", O_RDONLY , 0); 
		if (IS_ERR(fp)) { 
			printk("create file error\n"); 
			return 2;//-1; 
		} 
		else
		printk("Hi256_Initialize_from_T_Flash Open File Success\n");

		fs = get_fs(); 
		set_fs(KERNEL_DS); 

		file_size = vfs_llseek(fp, 0, SEEK_END);
		vfs_read(fp, data_buff, file_size, &pos); 
		filp_close(fp, NULL); 
		set_fs(fs);

		printk("1\n");

		/* Start parse the setting witch read from t-flash. */
		curr_ptr = data_buff;
		while (curr_ptr < (data_buff + file_size))
		{
			while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

			if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
			{
				while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
				{
					curr_ptr++;		/* Skip block comment code. */
				}

				while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
				{
					curr_ptr++;
				}

				curr_ptr += 2;						/* Skip the enter line */

				continue ;
			}

			if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
			{
				while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
				{
					curr_ptr++;
				}

				curr_ptr += 2;						/* Skip the enter line */

				continue ;
			}
			/* This just content one enter line. */
			if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
			{
				curr_ptr += 2;
				continue ;
			}
			//printk(" curr_ptr1 = %s\n",curr_ptr);
			memcpy(func_ind, curr_ptr, 3);

			if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
			{
				curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
				HI256_Init_Reg[i].op_code = HI256_OP_CODE_REG;

				HI256_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
				curr_ptr += 6;	/* Skip "00, 0x" */

				HI256_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
				curr_ptr += 4;	/* Skip "00);" */
			}
			else									/* DLY */
			{
				/* Need add delay for this setting. */
				curr_ptr += 4;	
				HI256_Init_Reg[i].op_code = HI256_OP_CODE_DLY;

				HI256_Init_Reg[i].init_reg = 0xFF;
				HI256_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
			}
			i++;

			/* Skip to next line directly. */
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}
			curr_ptr += 2;
		}
		printk("2\n");
		/* (0xFFFF, 0xFFFF) means the end of initial setting. */
		HI256_Init_Reg[i].op_code = HI256_OP_CODE_END;
		HI256_Init_Reg[i].init_reg = 0xFF;
		HI256_Init_Reg[i].init_val = 0xFF;
		i++;
		//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP2528_Init_Reg[j].init_reg, SP2528_Init_Reg[j].init_val);

		/* Start apply the initial setting to sensor. */
#if 1
		for (j=0; j<i; j++)
		{
			if (HI256_Init_Reg[j].op_code == HI256_OP_CODE_END)	/* End of the setting. */
			{
				break ;
			}
			else if (HI256_Init_Reg[j].op_code == HI256_OP_CODE_DLY)
			{
				msleep(HI256_Init_Reg[j].init_val);		/* Delay */
			}
			else if (HI256_Init_Reg[j].op_code == HI256_OP_CODE_REG)
			{
				HI258_write_cmos_sensor((kal_uint8)HI256_Init_Reg[j].init_reg, (kal_uint8)HI256_Init_Reg[j].init_val);
			}
			else
			{
				printk("REG ERROR!\n");
			}
		}
#endif
		printk("3\n");
		return 1;	
	}

#endif

void HI258_Init_Cmds_CMK(void) 
{
///// Start Sleep /////
printk("HI258_Init_Cmds_CMK\n");
HI258_write_cmos_sensor(0x01, 0x01); //sleep on
HI258_write_cmos_sensor(0x01, 0x03); //sleep off
HI258_write_cmos_sensor(0x01, 0x01); //sleep on
// PAGE 20
HI258_write_cmos_sensor(0x03, 0x20); // page 20
HI258_write_cmos_sensor(0x10, 0x1c); // AE off 60hz

// PAGE 22
HI258_write_cmos_sensor(0x03, 0x22); // page 22
HI258_write_cmos_sensor(0x10, 0x69); // AWB off

HI258_write_cmos_sensor(0x03, 0x00); //Dummy 750us
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);

HI258_write_cmos_sensor(0x08, 0x00);
HI258_write_cmos_sensor(0x09, 0x77);// pad strength = max
HI258_write_cmos_sensor(0x0a, 0x07);// pad strength = max

//PLL Setting
HI258_write_cmos_sensor(0x03, 0x00); 
HI258_write_cmos_sensor(0xd0, 0x05); //PLL pre_div 1/6 = 4 Mhz
HI258_write_cmos_sensor(0xd1, 0x30); //PLL maim_div 
HI258_write_cmos_sensor(0xd2, 0x05); //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    
HI258_write_cmos_sensor(0xd3, 0x20); //isp_clk_inv[0]  mipi_4x_inv[1]  mipi_1x_inv[2]
HI258_write_cmos_sensor(0xd0, 0x85);
HI258_write_cmos_sensor(0xd0, 0x85);
HI258_write_cmos_sensor(0xd0, 0x85);
HI258_write_cmos_sensor(0xd0, 0x95);

HI258_write_cmos_sensor(0x03, 0x00); //Dummy 750us
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);


///// PAGE 20 /////
HI258_write_cmos_sensor(0x03, 0x20); //page 20
HI258_write_cmos_sensor(0x10, 0x1c); //AE off 50hz

///// PAGE 22 /////
HI258_write_cmos_sensor(0x03, 0x22); //page 22
HI258_write_cmos_sensor(0x10, 0x69); //AWB off


///// Initial Start /////
///// PAGE 0 Start /////
HI258_write_cmos_sensor(0x03, 0x00); //page 0
HI258_write_cmos_sensor(0x10, 0x00);
HI258_write_cmos_sensor(0x11, 0x90); //Windowing On + 1Frame Skip
HI258_write_cmos_sensor(0x12, 0x04); //Rinsing edge 0x04 // Falling edge 0x00
HI258_write_cmos_sensor(0x14, 0x05);

HI258_write_cmos_sensor(0x20, 0x00); //Row H
HI258_write_cmos_sensor(0x21, 0x04); //Row L
HI258_write_cmos_sensor(0x22, 0x00); //Col H
HI258_write_cmos_sensor(0x23, 0x04); //Col L
HI258_write_cmos_sensor(0x24, 0x04);
HI258_write_cmos_sensor(0x25, 0xb0);
HI258_write_cmos_sensor(0x26, 0x06);
HI258_write_cmos_sensor(0x27, 0x40);
HI258_write_cmos_sensor(0x40, 0x01);
HI258_write_cmos_sensor(0x41, 0x78);
HI258_write_cmos_sensor(0x42, 0x00);
HI258_write_cmos_sensor(0x43, 0x14);
HI258_write_cmos_sensor(0x50, 0x00);
HI258_write_cmos_sensor(0x80, 0x2e);
HI258_write_cmos_sensor(0x81, 0x7e);
HI258_write_cmos_sensor(0x82, 0x90);
HI258_write_cmos_sensor(0x83, 0x00);
HI258_write_cmos_sensor(0x84, 0x0c);
HI258_write_cmos_sensor(0x85, 0x00);
HI258_write_cmos_sensor(0x86, 0x00);
HI258_write_cmos_sensor(0x87, 0x0f);
HI258_write_cmos_sensor(0x88, 0x34);
HI258_write_cmos_sensor(0x8a, 0x0b);
HI258_write_cmos_sensor(0x8e, 0x80);
HI258_write_cmos_sensor(0x90, 0x0d);
HI258_write_cmos_sensor(0x91, 0x0d);
HI258_write_cmos_sensor(0x92, 0xe8);
HI258_write_cmos_sensor(0x93, 0xe0);
HI258_write_cmos_sensor(0x96, 0xdc);
HI258_write_cmos_sensor(0x97, 0xfe);
HI258_write_cmos_sensor(0x98, 0x38);
HI258_write_cmos_sensor(0x99, 0x43);
HI258_write_cmos_sensor(0xa0, 0x01);
HI258_write_cmos_sensor(0xa8, 0x43);



//page 2
HI258_write_cmos_sensor(0x03, 0x02);
HI258_write_cmos_sensor(0x10, 0x00);
HI258_write_cmos_sensor(0x13, 0x00);
HI258_write_cmos_sensor(0x14, 0x00);
HI258_write_cmos_sensor(0x18, 0xcc);
HI258_write_cmos_sensor(0x19, 0x01);
HI258_write_cmos_sensor(0x1A, 0x09);
HI258_write_cmos_sensor(0x1B, 0x00);
HI258_write_cmos_sensor(0x1C, 0x1a);
HI258_write_cmos_sensor(0x1D, 0x14);
HI258_write_cmos_sensor(0x1E, 0x30);
HI258_write_cmos_sensor(0x1F, 0x10);
HI258_write_cmos_sensor(0x20, 0x77);
HI258_write_cmos_sensor(0x21, 0xde);
HI258_write_cmos_sensor(0x22, 0xa7);
HI258_write_cmos_sensor(0x23, 0x30);
HI258_write_cmos_sensor(0x24, 0x77);
HI258_write_cmos_sensor(0x25, 0x10);
HI258_write_cmos_sensor(0x26, 0x10);
HI258_write_cmos_sensor(0x27, 0x3c);
HI258_write_cmos_sensor(0x2b, 0x80);
HI258_write_cmos_sensor(0x2c, 0x02);
HI258_write_cmos_sensor(0x2d, 0x58);
HI258_write_cmos_sensor(0x2e, 0xde);
HI258_write_cmos_sensor(0x2f, 0xa7);
HI258_write_cmos_sensor(0x30, 0x00);
HI258_write_cmos_sensor(0x31, 0x99);
HI258_write_cmos_sensor(0x32, 0x00);
HI258_write_cmos_sensor(0x33, 0x00);
HI258_write_cmos_sensor(0x34, 0x22);
HI258_write_cmos_sensor(0x36, 0x75);
HI258_write_cmos_sensor(0x38, 0x88);
HI258_write_cmos_sensor(0x39, 0x88);
HI258_write_cmos_sensor(0x3d, 0x03);
HI258_write_cmos_sensor(0x3f, 0x02);
HI258_write_cmos_sensor(0x49, 0x87);
HI258_write_cmos_sensor(0x4a, 0x10);
HI258_write_cmos_sensor(0x50, 0x21);
HI258_write_cmos_sensor(0x53, 0xb1);
HI258_write_cmos_sensor(0x54, 0x10);
HI258_write_cmos_sensor(0x55, 0x1c); 
HI258_write_cmos_sensor(0x56, 0x11);
HI258_write_cmos_sensor(0x5d, 0xa2);
HI258_write_cmos_sensor(0x5e, 0x5a);
HI258_write_cmos_sensor(0x5d, 0xa2);
HI258_write_cmos_sensor(0x5e, 0x5a);
HI258_write_cmos_sensor(0x60, 0x87);
HI258_write_cmos_sensor(0x61, 0x98);
HI258_write_cmos_sensor(0x62, 0x88);
HI258_write_cmos_sensor(0x63, 0x96);
HI258_write_cmos_sensor(0x64, 0x88);
HI258_write_cmos_sensor(0x65, 0x96);
HI258_write_cmos_sensor(0x67, 0x3f);
HI258_write_cmos_sensor(0x68, 0x3f);
HI258_write_cmos_sensor(0x69, 0x3f);
HI258_write_cmos_sensor(0x72, 0x89);
HI258_write_cmos_sensor(0x73, 0x95);
HI258_write_cmos_sensor(0x74, 0x89);
HI258_write_cmos_sensor(0x75, 0x95);
HI258_write_cmos_sensor(0x7C, 0x84);
HI258_write_cmos_sensor(0x7D, 0xaf);
HI258_write_cmos_sensor(0x80, 0x01);
HI258_write_cmos_sensor(0x81, 0x7a);
HI258_write_cmos_sensor(0x82, 0x13);
HI258_write_cmos_sensor(0x83, 0x24);
HI258_write_cmos_sensor(0x84, 0x78);
HI258_write_cmos_sensor(0x85, 0x7c);
HI258_write_cmos_sensor(0x92, 0x44);
HI258_write_cmos_sensor(0x93, 0x59);
HI258_write_cmos_sensor(0x94, 0x78);
HI258_write_cmos_sensor(0x95, 0x7c);
HI258_write_cmos_sensor(0xA0, 0x02);
HI258_write_cmos_sensor(0xA1, 0x74);
HI258_write_cmos_sensor(0xA4, 0x74);
HI258_write_cmos_sensor(0xA5, 0x02);
HI258_write_cmos_sensor(0xA8, 0x85);
HI258_write_cmos_sensor(0xA9, 0x8c);
HI258_write_cmos_sensor(0xAC, 0x10);
HI258_write_cmos_sensor(0xAD, 0x16);
HI258_write_cmos_sensor(0xB0, 0x99);
HI258_write_cmos_sensor(0xB1, 0xa3);
HI258_write_cmos_sensor(0xB4, 0x9b);
HI258_write_cmos_sensor(0xB5, 0xa2);
HI258_write_cmos_sensor(0xB8, 0x9b);
HI258_write_cmos_sensor(0xB9, 0x9f);
HI258_write_cmos_sensor(0xBC, 0x9b);
HI258_write_cmos_sensor(0xBD, 0x9f);
HI258_write_cmos_sensor(0xc4, 0x29);
HI258_write_cmos_sensor(0xc5, 0x40);
HI258_write_cmos_sensor(0xc6, 0x5c);
HI258_write_cmos_sensor(0xc7, 0x72);
HI258_write_cmos_sensor(0xc8, 0x2a);
HI258_write_cmos_sensor(0xc9, 0x3f);
HI258_write_cmos_sensor(0xcc, 0x5d);
HI258_write_cmos_sensor(0xcd, 0x71);
HI258_write_cmos_sensor(0xd0, 0x10);
HI258_write_cmos_sensor(0xd1, 0x14);
HI258_write_cmos_sensor(0xd2, 0x20);
HI258_write_cmos_sensor(0xd3, 0x00);
HI258_write_cmos_sensor(0xd4, 0x0d);
HI258_write_cmos_sensor(0xd5, 0x0d);
HI258_write_cmos_sensor(0xd6, 0xe8);
HI258_write_cmos_sensor(0xd7, 0xe0);
HI258_write_cmos_sensor(0xdc, 0x00);
HI258_write_cmos_sensor(0xdd, 0xa3);
HI258_write_cmos_sensor(0xde, 0x00);
HI258_write_cmos_sensor(0xdf, 0x84);
HI258_write_cmos_sensor(0xe0, 0xa4);
HI258_write_cmos_sensor(0xe1, 0xa4);
HI258_write_cmos_sensor(0xe2, 0xa4);
HI258_write_cmos_sensor(0xe3, 0xa4);
HI258_write_cmos_sensor(0xe4, 0xa4);
HI258_write_cmos_sensor(0xe5, 0x01);
HI258_write_cmos_sensor(0xe8, 0x00);
HI258_write_cmos_sensor(0xe9, 0x00);
HI258_write_cmos_sensor(0xea, 0x77);
HI258_write_cmos_sensor(0xF0, 0x00);
HI258_write_cmos_sensor(0xF1, 0x00);
HI258_write_cmos_sensor(0xF2, 0x00);
HI258_write_cmos_sensor(0x03, 0x10);//page10
HI258_write_cmos_sensor(0x10, 0x03);
HI258_write_cmos_sensor(0x11, 0x03);
HI258_write_cmos_sensor(0x12, 0xf0);
HI258_write_cmos_sensor(0x13, 0x03);
HI258_write_cmos_sensor(0x20, 0x00);
HI258_write_cmos_sensor(0x21, 0x40);
HI258_write_cmos_sensor(0x22, 0x0f);
HI258_write_cmos_sensor(0x24, 0x20);
HI258_write_cmos_sensor(0x25, 0x10);
HI258_write_cmos_sensor(0x26, 0x01);
HI258_write_cmos_sensor(0x27, 0x02);
HI258_write_cmos_sensor(0x28, 0x11);
HI258_write_cmos_sensor(0x40, 0x80);
HI258_write_cmos_sensor(0x41, 0x12); 
HI258_write_cmos_sensor(0x42, 0x05); 
HI258_write_cmos_sensor(0x43, 0x05); 
HI258_write_cmos_sensor(0x44, 0x80);
HI258_write_cmos_sensor(0x45, 0x80);
HI258_write_cmos_sensor(0x46, 0xf0);
HI258_write_cmos_sensor(0x48, 0x8a);
HI258_write_cmos_sensor(0x4a, 0x80);
HI258_write_cmos_sensor(0x50, 0xa0); 
HI258_write_cmos_sensor(0x60, 0x0f);
HI258_write_cmos_sensor(0x61, 0x90); //b a7
HI258_write_cmos_sensor(0x62, 0x90); //r 9e
HI258_write_cmos_sensor(0x63, 0x70); 
HI258_write_cmos_sensor(0x66, 0x42);
HI258_write_cmos_sensor(0x67, 0x22);
HI258_write_cmos_sensor(0x6a, 0x88); 
HI258_write_cmos_sensor(0x74, 0x0a); 
HI258_write_cmos_sensor(0x76, 0x01); 

HI258_write_cmos_sensor(0x03, 0x11); //page11
HI258_write_cmos_sensor(0x20, 0x00);
HI258_write_cmos_sensor(0x21, 0x00);
HI258_write_cmos_sensor(0x26, 0x58); 
HI258_write_cmos_sensor(0x27, 0x52); 
HI258_write_cmos_sensor(0x28, 0x0f);
HI258_write_cmos_sensor(0x29, 0x10);
HI258_write_cmos_sensor(0x2b, 0x30);
HI258_write_cmos_sensor(0x2c, 0x32);
HI258_write_cmos_sensor(0x70, 0x2b);
HI258_write_cmos_sensor(0x74, 0x30);
HI258_write_cmos_sensor(0x75, 0x18);
HI258_write_cmos_sensor(0x76, 0x30);
HI258_write_cmos_sensor(0x77, 0xff);
HI258_write_cmos_sensor(0x78, 0xa0);
HI258_write_cmos_sensor(0x79, 0xff); 
HI258_write_cmos_sensor(0x7a, 0x30);
HI258_write_cmos_sensor(0x7b, 0x20);
HI258_write_cmos_sensor(0x7c, 0xf4); 
HI258_write_cmos_sensor(0x03, 0x12);//page12 
HI258_write_cmos_sensor(0x10, 0x03); 
HI258_write_cmos_sensor(0x11, 0x08); 
HI258_write_cmos_sensor(0x12, 0x10); 
HI258_write_cmos_sensor(0x20, 0x53); 
HI258_write_cmos_sensor(0x21, 0x03); 
HI258_write_cmos_sensor(0x22, 0xe6); 
HI258_write_cmos_sensor(0x23, 0x0a); //outdoor
HI258_write_cmos_sensor(0x24, 0x0b); //indoor
HI258_write_cmos_sensor(0x25, 0x1a);//dark 
HI258_write_cmos_sensor(0x30, 0xff); 
HI258_write_cmos_sensor(0x31, 0x00); 
HI258_write_cmos_sensor(0x32, 0xf0); 
HI258_write_cmos_sensor(0x33, 0x00); 
HI258_write_cmos_sensor(0x34, 0x00); 
HI258_write_cmos_sensor(0x35, 0xff); 
HI258_write_cmos_sensor(0x36, 0x00); 
HI258_write_cmos_sensor(0x37, 0xff); 
HI258_write_cmos_sensor(0x38, 0x00); 
HI258_write_cmos_sensor(0x39, 0x00); 
HI258_write_cmos_sensor(0x3a, 0xff); 
HI258_write_cmos_sensor(0x3b, 0x00); 
HI258_write_cmos_sensor(0x3c, 0x93); 
HI258_write_cmos_sensor(0x3d, 0x00); 
HI258_write_cmos_sensor(0x3e, 0x00); 
HI258_write_cmos_sensor(0x46, 0xa0);//Out Lum Hi 
HI258_write_cmos_sensor(0x47, 0x40); //Out Lum Lo
HI258_write_cmos_sensor(0x4c, 0xa0);//Indoor Lum Hi bigger will be darker
HI258_write_cmos_sensor(0x4d, 0x40);//Indoor Lum Lo
HI258_write_cmos_sensor(0x52, 0xa0);//Dark Lum Hi 
HI258_write_cmos_sensor(0x53, 0x40); //Dark Lum Lo
HI258_write_cmos_sensor(0x70, 0x10); 
HI258_write_cmos_sensor(0x71, 0x0a); 
HI258_write_cmos_sensor(0x72, 0x10); 
HI258_write_cmos_sensor(0x73, 0x0a); 
HI258_write_cmos_sensor(0x74, 0x14); 
HI258_write_cmos_sensor(0x75, 0x0c); 
HI258_write_cmos_sensor(0x90, 0x3d);
HI258_write_cmos_sensor(0x91, 0x34);
HI258_write_cmos_sensor(0x99, 0x28);
HI258_write_cmos_sensor(0x9c, 0x14);
HI258_write_cmos_sensor(0x9d, 0x15);
HI258_write_cmos_sensor(0x9e, 0x28);
HI258_write_cmos_sensor(0x9f, 0x28);
HI258_write_cmos_sensor(0xb0, 0x0e); 
HI258_write_cmos_sensor(0xb8, 0x44);
HI258_write_cmos_sensor(0xb9, 0x15);

HI258_write_cmos_sensor(0x03, 0x13);//page13 
HI258_write_cmos_sensor(0x80, 0xfd); 
HI258_write_cmos_sensor(0x81, 0x07); 
HI258_write_cmos_sensor(0x82, 0x73); 
HI258_write_cmos_sensor(0x83, 0x00); 
HI258_write_cmos_sensor(0x85, 0x00);
HI258_write_cmos_sensor(0x92, 0x33); 
HI258_write_cmos_sensor(0x93, 0x30); 
HI258_write_cmos_sensor(0x94, 0x02); 
HI258_write_cmos_sensor(0x95, 0xf0); 
HI258_write_cmos_sensor(0x96, 0x1e); 
HI258_write_cmos_sensor(0x97, 0x40); 
HI258_write_cmos_sensor(0x98, 0x80);
HI258_write_cmos_sensor(0x99, 0x40);
HI258_write_cmos_sensor(0xa2, 0x04);//outdoor clip
HI258_write_cmos_sensor(0xa3, 0x05);
HI258_write_cmos_sensor(0xa4, 0x05);//indoor clip
HI258_write_cmos_sensor(0xa5, 0x06);
HI258_write_cmos_sensor(0xa6, 0x07);//dark1 clip
HI258_write_cmos_sensor(0xa7, 0x08);//dark1 clip
HI258_write_cmos_sensor(0xb6, 0x36);
HI258_write_cmos_sensor(0xb7, 0x36);
HI258_write_cmos_sensor(0xb8, 0x36);
HI258_write_cmos_sensor(0xb9, 0x36);
HI258_write_cmos_sensor(0xba, 0x36);
HI258_write_cmos_sensor(0xbb, 0x36);
HI258_write_cmos_sensor(0xbc, 0x32);//indoor
HI258_write_cmos_sensor(0xbd, 0x2e);
HI258_write_cmos_sensor(0xbe, 0x2e);
HI258_write_cmos_sensor(0xbf, 0x32);
HI258_write_cmos_sensor(0xc0, 0x2e);
HI258_write_cmos_sensor(0xc1, 0x2e);
HI258_write_cmos_sensor(0xc2, 0x1d);//dark
HI258_write_cmos_sensor(0xc3, 0x1d);//Lum negative middle
HI258_write_cmos_sensor(0xc4, 0x1d);
HI258_write_cmos_sensor(0xc5, 0x1d);
HI258_write_cmos_sensor(0xc6, 0x1d);//Lum postive middle
HI258_write_cmos_sensor(0xc7, 0x1d);
///// PAGE 14 START /////
HI258_write_cmos_sensor(0x03, 0x14); //page 14
HI258_write_cmos_sensor(0x10, 0x09);

HI258_write_cmos_sensor(0x20, 0x80); //X-Center
HI258_write_cmos_sensor(0x21, 0x80); //Y-Center

HI258_write_cmos_sensor(0x22, 0x58); //LSC R 60 4a 4a 
HI258_write_cmos_sensor(0x23, 0x4b); //LSC G
HI258_write_cmos_sensor(0x24, 0x4a); //LSC B

HI258_write_cmos_sensor(0x25, 0xf0); //LSC Off
HI258_write_cmos_sensor(0x26, 0xf0); //LSC On
///// PAGE 14 END /////

/////// PAGE 15 START ///////
HI258_write_cmos_sensor(0x03, 0x15);//page 15
	HI258_write_cmos_sensor(0x10, 0x0f); 
	HI258_write_cmos_sensor(0x14, 0x4b); 
	HI258_write_cmos_sensor(0x15, 0x3d); 
	HI258_write_cmos_sensor(0x16, 0x2e); 
	HI258_write_cmos_sensor(0x17, 0x2f); 
//CMC
	//CMC
	HI258_write_cmos_sensor(0x30, 0x8e); 
	HI258_write_cmos_sensor(0x31, 0x75); 
	HI258_write_cmos_sensor(0x32, 0x25); 
	HI258_write_cmos_sensor(0x33, 0x15); 
	HI258_write_cmos_sensor(0x34, 0x5a); 
	HI258_write_cmos_sensor(0x35, 0x05); 
	HI258_write_cmos_sensor(0x36, 0x07); 
	HI258_write_cmos_sensor(0x37, 0x40); 
	HI258_write_cmos_sensor(0x38, 0x85); 

	//CMC OFS
	HI258_write_cmos_sensor(0x40, 0x95); 
	HI258_write_cmos_sensor(0x41, 0x1f); 
	HI258_write_cmos_sensor(0x42, 0x8a); 
	HI258_write_cmos_sensor(0x43, 0x86); 
	HI258_write_cmos_sensor(0x44, 0x0a); 
	HI258_write_cmos_sensor(0x45, 0x84); 
	HI258_write_cmos_sensor(0x46, 0x87); 
	HI258_write_cmos_sensor(0x47, 0x9b); 
	HI258_write_cmos_sensor(0x48, 0x23); 

	//CMC POFS
	HI258_write_cmos_sensor(0x50, 0x8c); 
	HI258_write_cmos_sensor(0x51, 0x0c); 
	HI258_write_cmos_sensor(0x52, 0x00); 
	HI258_write_cmos_sensor(0x53, 0x07); 
	HI258_write_cmos_sensor(0x54, 0x17); 
	HI258_write_cmos_sensor(0x55, 0x9d); 
	HI258_write_cmos_sensor(0x56, 0x00); 
	HI258_write_cmos_sensor(0x57, 0x0b); 
	HI258_write_cmos_sensor(0x58, 0x89); 
///// PAGE 15 END /////
///// PAGE 16 START /////
HI258_write_cmos_sensor(0x03, 0x16);
HI258_write_cmos_sensor(0x10, 0x31);//GMA_CTL
HI258_write_cmos_sensor(0x18, 0x5e);//AG_ON
HI258_write_cmos_sensor(0x19, 0x5d);//AG_OFF
HI258_write_cmos_sensor(0x1A, 0x0e);//TIME_ON
HI258_write_cmos_sensor(0x1B, 0x01);//TIME_OFF
HI258_write_cmos_sensor(0x1C, 0xdc);//OUT_ON
HI258_write_cmos_sensor(0x1D, 0xfe);//OUT_OFF
//GMA
HI258_write_cmos_sensor(0x30, 0x00);
HI258_write_cmos_sensor(0x31, 0x04);
HI258_write_cmos_sensor(0x32, 0x0f);
HI258_write_cmos_sensor(0x33, 0x19);
HI258_write_cmos_sensor(0x34, 0x3c);
HI258_write_cmos_sensor(0x35, 0x56);
HI258_write_cmos_sensor(0x36, 0x6b);
HI258_write_cmos_sensor(0x37, 0x7f);
HI258_write_cmos_sensor(0x38, 0x90);
HI258_write_cmos_sensor(0x39, 0xa0);
HI258_write_cmos_sensor(0x3a, 0xae);
HI258_write_cmos_sensor(0x3b, 0xbc);
HI258_write_cmos_sensor(0x3c, 0xc8);
HI258_write_cmos_sensor(0x3d, 0xd4);
HI258_write_cmos_sensor(0x3e, 0xde);
HI258_write_cmos_sensor(0x3f, 0xe8);
HI258_write_cmos_sensor(0x40, 0xf1);
HI258_write_cmos_sensor(0x41, 0xf8);
HI258_write_cmos_sensor(0x42, 0xff);


//Outdoor
HI258_write_cmos_sensor(0x50, 0x00);
HI258_write_cmos_sensor(0x51, 0x03);
HI258_write_cmos_sensor(0x52, 0x10);
HI258_write_cmos_sensor(0x53, 0x26);
HI258_write_cmos_sensor(0x54, 0x43);
HI258_write_cmos_sensor(0x55, 0x5d);
HI258_write_cmos_sensor(0x56, 0x79);
HI258_write_cmos_sensor(0x57, 0x8c);
HI258_write_cmos_sensor(0x58, 0x9f);
HI258_write_cmos_sensor(0x59, 0xaa);
HI258_write_cmos_sensor(0x5a, 0xb6);
HI258_write_cmos_sensor(0x5b, 0xc3);
HI258_write_cmos_sensor(0x5c, 0xce);
HI258_write_cmos_sensor(0x5d, 0xd9);
HI258_write_cmos_sensor(0x5e, 0xe1);
HI258_write_cmos_sensor(0x5f, 0xe9);
HI258_write_cmos_sensor(0x60, 0xf0);
HI258_write_cmos_sensor(0x61, 0xf4);
HI258_write_cmos_sensor(0x62, 0xf5);

//Dark
HI258_write_cmos_sensor(0x70, 0x00);
HI258_write_cmos_sensor(0x71, 0x07);
HI258_write_cmos_sensor(0x72, 0x0d);
HI258_write_cmos_sensor(0x73, 0x1c);
HI258_write_cmos_sensor(0x74, 0x3d);
HI258_write_cmos_sensor(0x75, 0x56);
HI258_write_cmos_sensor(0x76, 0x6a);
HI258_write_cmos_sensor(0x77, 0x7f);
HI258_write_cmos_sensor(0x78, 0x90);
HI258_write_cmos_sensor(0x79, 0xa0);
HI258_write_cmos_sensor(0x7a, 0xae);
HI258_write_cmos_sensor(0x7b, 0xbc);
HI258_write_cmos_sensor(0x7c, 0xc8);
HI258_write_cmos_sensor(0x7d, 0xd4);
HI258_write_cmos_sensor(0x7e, 0xde);
HI258_write_cmos_sensor(0x7f, 0xe8);
HI258_write_cmos_sensor(0x80, 0xf1);
HI258_write_cmos_sensor(0x81, 0xf8);
HI258_write_cmos_sensor(0x82, 0xff);///// PAGE 16 END /////

///// PAGE 17 START /////
HI258_write_cmos_sensor(0x03, 0x17); //page 17
HI258_write_cmos_sensor(0xc1, 0x00);
HI258_write_cmos_sensor(0xc4, 0x4b);
HI258_write_cmos_sensor(0xc5, 0x3f);
HI258_write_cmos_sensor(0xc6, 0x02);
HI258_write_cmos_sensor(0xc7, 0x20);
HI258_write_cmos_sensor(0x03, 0x18);//page18
HI258_write_cmos_sensor(0x10, 0x00);
HI258_write_cmos_sensor(0x03, 0x19);//page19
HI258_write_cmos_sensor(0x10, 0x7f);
HI258_write_cmos_sensor(0x11, 0x7f);
HI258_write_cmos_sensor(0x12, 0x1e);
HI258_write_cmos_sensor(0x13, 0x32);
HI258_write_cmos_sensor(0x14, 0x1e);
HI258_write_cmos_sensor(0x15, 0x5e);
HI258_write_cmos_sensor(0x16, 0x0a);
HI258_write_cmos_sensor(0x17, 0xb8);
HI258_write_cmos_sensor(0x18, 0x1e);
HI258_write_cmos_sensor(0x19, 0xe6);
HI258_write_cmos_sensor(0x1a, 0x9e);
HI258_write_cmos_sensor(0x1b, 0x22);
HI258_write_cmos_sensor(0x1c, 0x9e);
HI258_write_cmos_sensor(0x1d, 0x5e);
HI258_write_cmos_sensor(0x1e, 0x3b);
HI258_write_cmos_sensor(0x1f, 0x30);//26
HI258_write_cmos_sensor(0x20, 0x40);//50
HI258_write_cmos_sensor(0x21, 0x40);//60
HI258_write_cmos_sensor(0x22, 0x2f);
HI258_write_cmos_sensor(0x23, 0x27);
HI258_write_cmos_sensor(0x24, 0x00);
HI258_write_cmos_sensor(0x25, 0x01);
HI258_write_cmos_sensor(0x26, 0x0e);
HI258_write_cmos_sensor(0x27, 0x04);
HI258_write_cmos_sensor(0x28, 0x00);
HI258_write_cmos_sensor(0x29, 0x8c);
HI258_write_cmos_sensor(0x2a, 0x40);
HI258_write_cmos_sensor(0x2b, 0x3f);
HI258_write_cmos_sensor(0x2c, 0x00);
HI258_write_cmos_sensor(0x2d, 0x00);
HI258_write_cmos_sensor(0x2e, 0x00);
HI258_write_cmos_sensor(0x2f, 0x00);
HI258_write_cmos_sensor(0x30, 0x00);
HI258_write_cmos_sensor(0x31, 0x00);
HI258_write_cmos_sensor(0x32, 0x00);
HI258_write_cmos_sensor(0x33, 0x00);
HI258_write_cmos_sensor(0x34, 0x00);
HI258_write_cmos_sensor(0x35, 0x00);
HI258_write_cmos_sensor(0x36, 0x00);
HI258_write_cmos_sensor(0x37, 0x00);
HI258_write_cmos_sensor(0x38, 0x00);
HI258_write_cmos_sensor(0x39, 0x00);
HI258_write_cmos_sensor(0x3a, 0x00);
HI258_write_cmos_sensor(0x3b, 0x00);
HI258_write_cmos_sensor(0x3c, 0x00);
HI258_write_cmos_sensor(0x3d, 0x00);
HI258_write_cmos_sensor(0x3e, 0x00);
HI258_write_cmos_sensor(0x3f, 0x00);
HI258_write_cmos_sensor(0x40, 0x00);
HI258_write_cmos_sensor(0x41, 0x00);
HI258_write_cmos_sensor(0x42, 0x00);
HI258_write_cmos_sensor(0x43, 0x00);
HI258_write_cmos_sensor(0x44, 0x00);
HI258_write_cmos_sensor(0x45, 0x00);
HI258_write_cmos_sensor(0x46, 0x00);
HI258_write_cmos_sensor(0x47, 0x00);
HI258_write_cmos_sensor(0x48, 0x00);
HI258_write_cmos_sensor(0x49, 0x00);
HI258_write_cmos_sensor(0x4a, 0x00);
HI258_write_cmos_sensor(0x4b, 0x00);
HI258_write_cmos_sensor(0x4c, 0x00);
HI258_write_cmos_sensor(0x4d, 0x00);
HI258_write_cmos_sensor(0x4e, 0x00);
HI258_write_cmos_sensor(0x4f, 0x00);
HI258_write_cmos_sensor(0x50, 0x00);
HI258_write_cmos_sensor(0x51, 0x00);
HI258_write_cmos_sensor(0x52, 0x00);
HI258_write_cmos_sensor(0x53, 0x10);
HI258_write_cmos_sensor(0x54, 0x00);
HI258_write_cmos_sensor(0x55, 0x01);
HI258_write_cmos_sensor(0x56, 0x1b);
HI258_write_cmos_sensor(0x57, 0x39);
HI258_write_cmos_sensor(0x58, 0x5a);
HI258_write_cmos_sensor(0x59, 0x80);
HI258_write_cmos_sensor(0x5a, 0xa6);
HI258_write_cmos_sensor(0x5b, 0xc1);
HI258_write_cmos_sensor(0x5c, 0xe8);
HI258_write_cmos_sensor(0x5d, 0x38);
HI258_write_cmos_sensor(0x5e, 0x3a);
HI258_write_cmos_sensor(0x5f, 0x3c);
HI258_write_cmos_sensor(0x60, 0x3f);
HI258_write_cmos_sensor(0x61, 0x3f);
HI258_write_cmos_sensor(0x62, 0x3f);
HI258_write_cmos_sensor(0x63, 0x3f);
HI258_write_cmos_sensor(0x64, 0x3f);
HI258_write_cmos_sensor(0x65, 0x00);
HI258_write_cmos_sensor(0x66, 0x00);
HI258_write_cmos_sensor(0x67, 0x00);
HI258_write_cmos_sensor(0x68, 0x00);
HI258_write_cmos_sensor(0x69, 0x00);
HI258_write_cmos_sensor(0x6a, 0xff);
HI258_write_cmos_sensor(0x6b, 0x00);
HI258_write_cmos_sensor(0x6c, 0xff);
HI258_write_cmos_sensor(0x6d, 0x3f);
HI258_write_cmos_sensor(0x6e, 0x00);
HI258_write_cmos_sensor(0x6f, 0x00);
HI258_write_cmos_sensor(0x70, 0x00);
HI258_write_cmos_sensor(0x71, 0x3f);
HI258_write_cmos_sensor(0x72, 0x3f);
HI258_write_cmos_sensor(0x73, 0x3f);
HI258_write_cmos_sensor(0x74, 0x3f);
HI258_write_cmos_sensor(0x75, 0x30);
HI258_write_cmos_sensor(0x76, 0x50);
HI258_write_cmos_sensor(0x77, 0x80);
HI258_write_cmos_sensor(0x78, 0xb0);
HI258_write_cmos_sensor(0x79, 0x3f);
HI258_write_cmos_sensor(0x7a, 0x3f);
HI258_write_cmos_sensor(0x7b, 0x3f);
HI258_write_cmos_sensor(0x7c, 0x3f);
HI258_write_cmos_sensor(0x7d, 0x28);
HI258_write_cmos_sensor(0x7e, 0x50);
HI258_write_cmos_sensor(0x7f, 0x80);
HI258_write_cmos_sensor(0x80, 0xb0);
HI258_write_cmos_sensor(0x81, 0x28);
HI258_write_cmos_sensor(0x82, 0x3f);
HI258_write_cmos_sensor(0x83, 0x3f);
HI258_write_cmos_sensor(0x84, 0x3f);
HI258_write_cmos_sensor(0x85, 0x28);
HI258_write_cmos_sensor(0x86, 0x50);
HI258_write_cmos_sensor(0x87, 0x80);
HI258_write_cmos_sensor(0x88, 0xb0);
HI258_write_cmos_sensor(0x89, 0x1a);
HI258_write_cmos_sensor(0x8a, 0x28);
HI258_write_cmos_sensor(0x8b, 0x3f);
HI258_write_cmos_sensor(0x8c, 0x3f);
HI258_write_cmos_sensor(0x8d, 0x10);
HI258_write_cmos_sensor(0x8e, 0x30);
HI258_write_cmos_sensor(0x8f, 0x60);
HI258_write_cmos_sensor(0x90, 0x90);
HI258_write_cmos_sensor(0x91, 0x1a);
HI258_write_cmos_sensor(0x92, 0x28);
HI258_write_cmos_sensor(0x93, 0x3f);
HI258_write_cmos_sensor(0x94, 0x3f);
HI258_write_cmos_sensor(0x95, 0x28);
HI258_write_cmos_sensor(0x96, 0x50);
HI258_write_cmos_sensor(0x97, 0x80);
HI258_write_cmos_sensor(0x98, 0xb0);
HI258_write_cmos_sensor(0x99, 0x1a);
HI258_write_cmos_sensor(0x9a, 0x28);
HI258_write_cmos_sensor(0x9b, 0x3f);
HI258_write_cmos_sensor(0x9c, 0x3f);
HI258_write_cmos_sensor(0x9d, 0x28);
HI258_write_cmos_sensor(0x9e, 0x50);
HI258_write_cmos_sensor(0x9f, 0x80);
HI258_write_cmos_sensor(0xa0, 0xb0);
HI258_write_cmos_sensor(0xa2, 0x00);
HI258_write_cmos_sensor(0xe5, 0x80);
HI258_write_cmos_sensor(0x03, 0x20);//page20
HI258_write_cmos_sensor(0x11, 0x1c);
HI258_write_cmos_sensor(0x18, 0x30);
HI258_write_cmos_sensor(0x20, 0x25);
HI258_write_cmos_sensor(0x21, 0x30);
HI258_write_cmos_sensor(0x22, 0x10);
HI258_write_cmos_sensor(0x23, 0x00);
HI258_write_cmos_sensor(0x28, 0xe7);
HI258_write_cmos_sensor(0x29, 0x0d);
HI258_write_cmos_sensor(0x2a, 0xff);
HI258_write_cmos_sensor(0x2b, 0x04);
HI258_write_cmos_sensor(0x2c, 0x83);
HI258_write_cmos_sensor(0x2d, 0x03);
HI258_write_cmos_sensor(0x2e, 0x13);
HI258_write_cmos_sensor(0x2f, 0x0b);
HI258_write_cmos_sensor(0x30, 0x78);
HI258_write_cmos_sensor(0x31, 0xd7);
HI258_write_cmos_sensor(0x32, 0x10);
HI258_write_cmos_sensor(0x33, 0x2e);
HI258_write_cmos_sensor(0x34, 0x20);
HI258_write_cmos_sensor(0x35, 0xd4);
HI258_write_cmos_sensor(0x36, 0xfe);
HI258_write_cmos_sensor(0x37, 0x32);
HI258_write_cmos_sensor(0x38, 0x04);
HI258_write_cmos_sensor(0x39, 0x22);
HI258_write_cmos_sensor(0x3a, 0xde);
HI258_write_cmos_sensor(0x3b, 0x22);
HI258_write_cmos_sensor(0x3c, 0xde);
HI258_write_cmos_sensor(0x3d, 0xe1);
HI258_write_cmos_sensor(0x50, 0x45);
HI258_write_cmos_sensor(0x51, 0x88);
HI258_write_cmos_sensor(0x56, 0x1f);
HI258_write_cmos_sensor(0x57, 0xa6);
HI258_write_cmos_sensor(0x58, 0x1a);
HI258_write_cmos_sensor(0x59, 0x7a);
HI258_write_cmos_sensor(0x5a, 0x04);
HI258_write_cmos_sensor(0x5b, 0x04);
HI258_write_cmos_sensor(0x5e, 0xc7);
HI258_write_cmos_sensor(0x5f, 0x95);
HI258_write_cmos_sensor(0x62, 0x10);
HI258_write_cmos_sensor(0x63, 0xc0);
HI258_write_cmos_sensor(0x64, 0x10);
HI258_write_cmos_sensor(0x65, 0x8a);
HI258_write_cmos_sensor(0x66, 0x58);
HI258_write_cmos_sensor(0x67, 0x58);
HI258_write_cmos_sensor(0x70, 0x56);
HI258_write_cmos_sensor(0x71, 0x83);
HI258_write_cmos_sensor(0x76, 0x32);
HI258_write_cmos_sensor(0x77, 0xa1);
HI258_write_cmos_sensor(0x78, 0x22);
HI258_write_cmos_sensor(0x79, 0x30);
HI258_write_cmos_sensor(0x7a, 0x23);
HI258_write_cmos_sensor(0x7b, 0x22);
HI258_write_cmos_sensor(0x7d, 0x23);
HI258_write_cmos_sensor(0x83, 0x02);
HI258_write_cmos_sensor(0x84, 0xbf);
HI258_write_cmos_sensor(0x85, 0x20);
HI258_write_cmos_sensor(0x86, 0x01);
HI258_write_cmos_sensor(0x87, 0xf4);
HI258_write_cmos_sensor(0x88, 0x12);
HI258_write_cmos_sensor(0x89, 0x4f);
HI258_write_cmos_sensor(0x8a, 0x80);
HI258_write_cmos_sensor(0xa5, 0x0b);
HI258_write_cmos_sensor(0xa6, 0xe6);
HI258_write_cmos_sensor(0xa7, 0xe0);
HI258_write_cmos_sensor(0x8B, 0xea);
HI258_write_cmos_sensor(0x8C, 0x60);
HI258_write_cmos_sensor(0x8D, 0xc3);
HI258_write_cmos_sensor(0x8E, 0x50);
HI258_write_cmos_sensor(0x98, 0x9d);
HI258_write_cmos_sensor(0x99, 0x45);
HI258_write_cmos_sensor(0x9a, 0x0d);
HI258_write_cmos_sensor(0x9b, 0xde);
HI258_write_cmos_sensor(0x9c, 0x17);
HI258_write_cmos_sensor(0x9d, 0x70);
HI258_write_cmos_sensor(0x9e, 0x01);
HI258_write_cmos_sensor(0x9f, 0xf4);
HI258_write_cmos_sensor(0xb0, 0x15);
HI258_write_cmos_sensor(0xb1, 0x14);
HI258_write_cmos_sensor(0xb2, 0xb0);
HI258_write_cmos_sensor(0xb3, 0x15);
HI258_write_cmos_sensor(0xb4, 0x16);
HI258_write_cmos_sensor(0xb5, 0x3c);
HI258_write_cmos_sensor(0xb6, 0x29);
HI258_write_cmos_sensor(0xb7, 0x23);
HI258_write_cmos_sensor(0xb8, 0x20);
HI258_write_cmos_sensor(0xb9, 0x1e);
HI258_write_cmos_sensor(0xba, 0x1c);
HI258_write_cmos_sensor(0xbb, 0x1b);
HI258_write_cmos_sensor(0xbc, 0x1b);
HI258_write_cmos_sensor(0xbd, 0x1a);
HI258_write_cmos_sensor(0xc0, 0x10);
HI258_write_cmos_sensor(0xc1, 0x40);
HI258_write_cmos_sensor(0xc2, 0x40);
HI258_write_cmos_sensor(0xc3, 0x40);
HI258_write_cmos_sensor(0xc4, 0x06);
HI258_write_cmos_sensor(0xc8, 0x80);
HI258_write_cmos_sensor(0xc9, 0x80);
HI258_write_cmos_sensor(0x03, 0x21);//page21
HI258_write_cmos_sensor(0x20, 0x11);
HI258_write_cmos_sensor(0x21, 0x11);
HI258_write_cmos_sensor(0x22, 0x11);
HI258_write_cmos_sensor(0x23, 0x11);
HI258_write_cmos_sensor(0x24, 0x14);
HI258_write_cmos_sensor(0x25, 0x44);
HI258_write_cmos_sensor(0x26, 0x44);
HI258_write_cmos_sensor(0x27, 0x41);
HI258_write_cmos_sensor(0x28, 0x14);
HI258_write_cmos_sensor(0x29, 0x44);
HI258_write_cmos_sensor(0x2a, 0x44);
HI258_write_cmos_sensor(0x2b, 0x41);
HI258_write_cmos_sensor(0x2c, 0x14);
HI258_write_cmos_sensor(0x2d, 0x47);
HI258_write_cmos_sensor(0x2e, 0x74);
HI258_write_cmos_sensor(0x2f, 0x41);
HI258_write_cmos_sensor(0x30, 0x14);
HI258_write_cmos_sensor(0x31, 0x47);
HI258_write_cmos_sensor(0x32, 0x74);
HI258_write_cmos_sensor(0x33, 0x41);
HI258_write_cmos_sensor(0x34, 0x14);
HI258_write_cmos_sensor(0x35, 0x44);
HI258_write_cmos_sensor(0x36, 0x44);
HI258_write_cmos_sensor(0x37, 0x41);
HI258_write_cmos_sensor(0x38, 0x14);
HI258_write_cmos_sensor(0x39, 0x44);
HI258_write_cmos_sensor(0x3a, 0x44);
HI258_write_cmos_sensor(0x3b, 0x41);
HI258_write_cmos_sensor(0x3c, 0x11);
HI258_write_cmos_sensor(0x3d, 0x11);
HI258_write_cmos_sensor(0x3e, 0x11);
HI258_write_cmos_sensor(0x3f, 0x11);
HI258_write_cmos_sensor(0x40, 0x11);
HI258_write_cmos_sensor(0x41, 0x11);
HI258_write_cmos_sensor(0x42, 0x11);
HI258_write_cmos_sensor(0x43, 0x11);
HI258_write_cmos_sensor(0x44, 0x14);
HI258_write_cmos_sensor(0x45, 0x44);
HI258_write_cmos_sensor(0x46, 0x44);
HI258_write_cmos_sensor(0x47, 0x41);
HI258_write_cmos_sensor(0x48, 0x14);
HI258_write_cmos_sensor(0x49, 0x44);
HI258_write_cmos_sensor(0x4a, 0x44);
HI258_write_cmos_sensor(0x4b, 0x41);
HI258_write_cmos_sensor(0x4c, 0x14);
HI258_write_cmos_sensor(0x4d, 0x47);
HI258_write_cmos_sensor(0x4e, 0x74);
HI258_write_cmos_sensor(0x4f, 0x41);
HI258_write_cmos_sensor(0x50, 0x14);
HI258_write_cmos_sensor(0x51, 0x47);
HI258_write_cmos_sensor(0x52, 0x74);
HI258_write_cmos_sensor(0x53, 0x41);
HI258_write_cmos_sensor(0x54, 0x14);
HI258_write_cmos_sensor(0x55, 0x44);
HI258_write_cmos_sensor(0x56, 0x44);
HI258_write_cmos_sensor(0x57, 0x41);
HI258_write_cmos_sensor(0x58, 0x14);
HI258_write_cmos_sensor(0x59, 0x44);
HI258_write_cmos_sensor(0x5a, 0x44);
HI258_write_cmos_sensor(0x5b, 0x41);
HI258_write_cmos_sensor(0x5c, 0x11);
HI258_write_cmos_sensor(0x5d, 0x11);
HI258_write_cmos_sensor(0x5e, 0x11);
HI258_write_cmos_sensor(0x5f, 0x11);
///// PAGE 22 START /////
HI258_write_cmos_sensor(0x03, 0x22); //page 22
HI258_write_cmos_sensor(0x10, 0xfd);
HI258_write_cmos_sensor(0x11, 0x2e);
HI258_write_cmos_sensor(0x19, 0x00);
HI258_write_cmos_sensor(0x20, 0x30); //For AWB Speed
HI258_write_cmos_sensor(0x21, 0x80);
HI258_write_cmos_sensor(0x22, 0x00);
HI258_write_cmos_sensor(0x23, 0x00);
HI258_write_cmos_sensor(0x24, 0x01);
HI258_write_cmos_sensor(0x25, 0x4f); //2013-09-13 AWB Hunting

HI258_write_cmos_sensor(0x30, 0x80);
HI258_write_cmos_sensor(0x31, 0x80);
HI258_write_cmos_sensor(0x38, 0x11);
HI258_write_cmos_sensor(0x39, 0x34);
HI258_write_cmos_sensor(0x40, 0xe4); //Stb Yth
HI258_write_cmos_sensor(0x41, 0x33); //Stb cdiff
HI258_write_cmos_sensor(0x42, 0x22); //Stb csum
HI258_write_cmos_sensor(0x43, 0xf3); //Unstb Yth
HI258_write_cmos_sensor(0x44, 0x55); //Unstb cdiff
HI258_write_cmos_sensor(0x45, 0x33); //Unstb csum
HI258_write_cmos_sensor(0x46, 0x00);
HI258_write_cmos_sensor(0x47, 0x09); //2013-09-13 AWB Hunting
HI258_write_cmos_sensor(0x48, 0x00); //2013-09-13 AWB Hunting
HI258_write_cmos_sensor(0x49, 0x0a);

HI258_write_cmos_sensor(0x60, 0x04);
HI258_write_cmos_sensor(0x61, 0xc4);
HI258_write_cmos_sensor(0x62, 0x04);
HI258_write_cmos_sensor(0x63, 0x92);
HI258_write_cmos_sensor(0x66, 0x04);
HI258_write_cmos_sensor(0x67, 0xc4);
HI258_write_cmos_sensor(0x68, 0x04);
HI258_write_cmos_sensor(0x69, 0x92);

HI258_write_cmos_sensor(0x80, 0x36);
HI258_write_cmos_sensor(0x81, 0x20);
HI258_write_cmos_sensor(0x82, 0x2a);

HI258_write_cmos_sensor(0x83, 0x52);
HI258_write_cmos_sensor(0x84, 0x10);
HI258_write_cmos_sensor(0x85, 0x5f);
HI258_write_cmos_sensor(0x86, 0x18);

HI258_write_cmos_sensor(0x87, 0x44);
HI258_write_cmos_sensor(0x88, 0x30);
HI258_write_cmos_sensor(0x89, 0x2c);
HI258_write_cmos_sensor(0x8a, 0x18);

HI258_write_cmos_sensor(0x8b, 0x3b);
HI258_write_cmos_sensor(0x8c, 0x32);
HI258_write_cmos_sensor(0x8d, 0x2a);
HI258_write_cmos_sensor(0x8e, 0x1b);

HI258_write_cmos_sensor(0x8f, 0x4d);
HI258_write_cmos_sensor(0x90, 0x4a);
HI258_write_cmos_sensor(0x91, 0x45);
HI258_write_cmos_sensor(0x92, 0x3f);
HI258_write_cmos_sensor(0x93, 0x38);
HI258_write_cmos_sensor(0x94, 0x31);
HI258_write_cmos_sensor(0x95, 0x27);
HI258_write_cmos_sensor(0x96, 0x1d);
HI258_write_cmos_sensor(0x97, 0x13);
HI258_write_cmos_sensor(0x98, 0x12);
HI258_write_cmos_sensor(0x99, 0x11);
HI258_write_cmos_sensor(0x9a, 0x10);

HI258_write_cmos_sensor(0x9b, 0xaa);
HI258_write_cmos_sensor(0x9c, 0xaa);
HI258_write_cmos_sensor(0x9d, 0x48);
HI258_write_cmos_sensor(0x9e, 0x38);
HI258_write_cmos_sensor(0x9f, 0x30);

HI258_write_cmos_sensor(0xa0, 0x70);
HI258_write_cmos_sensor(0xa1, 0x54);
HI258_write_cmos_sensor(0xa2, 0x6f);
HI258_write_cmos_sensor(0xa3, 0xff);

HI258_write_cmos_sensor(0xa4, 0x14); //1536fps
HI258_write_cmos_sensor(0xa5, 0x2c); //698fps
HI258_write_cmos_sensor(0xa6, 0xcf); //148fps

HI258_write_cmos_sensor(0xad, 0x2e);
HI258_write_cmos_sensor(0xae, 0x28);

HI258_write_cmos_sensor(0xaf, 0x18); //Low temp Rgain
HI258_write_cmos_sensor(0xb0, 0x16); //Low temp Rgain

HI258_write_cmos_sensor(0xb1, 0x08);
HI258_write_cmos_sensor(0xb4, 0xbf); //For Tracking AWB Weight
HI258_write_cmos_sensor(0xb8, 0x5d); //(0+,1-)High Cb , (0+,1-)Low Cr
HI258_write_cmos_sensor(0xb9, 0xb0);
/////// PAGE 22 END ///////

//// MIPI Setting /////
HI258_write_cmos_sensor(0x03, 0x48);
HI258_write_cmos_sensor(0x39, 0x4f); //lvds_bias_ctl    [2:0]mipi_tx_bias   [4:3]mipi_vlp_sel   [6:5]mipi_vcm_sel
HI258_write_cmos_sensor(0x10, 0x1c); //lvds_ctl_1       [5]mipi_pad_disable [4]lvds_en [0]serial_data_len 
HI258_write_cmos_sensor(0x11, 0x10); //lvds_ctl_2       [4]mipi continous mode setting
//HI258_write_cmos_sensor(0x14, 0x00} //ser_out_ctl_1  [2:0]serial_sout_a_phase   [6:4]serial_cout_a_phase

HI258_write_cmos_sensor(0x16, 0x00); //lvds_inout_ctl1  [0]vs_packet_pos_sel [1]data_neg_sel [4]first_vsync_end_opt
HI258_write_cmos_sensor(0x18, 0x80); //lvds_inout_ctl3
HI258_write_cmos_sensor(0x19, 0x00); //lvds_inout_ctl4
HI258_write_cmos_sensor(0x1a, 0xf0); //lvds_time_ctl
HI258_write_cmos_sensor(0x24, 0x1e); //long_packet_id

//====== MIPI Timing Setting =========
HI258_write_cmos_sensor(0x36, 0x01); //clk_tlpx_time_dp
HI258_write_cmos_sensor(0x37, 0x05); //clk_tlpx_time_dn
HI258_write_cmos_sensor(0x34, 0x04); //clk_prepare_time
HI258_write_cmos_sensor(0x32, 0x15); //clk_zero_time
HI258_write_cmos_sensor(0x35, 0x04); //clk_trail_time
HI258_write_cmos_sensor(0x33, 0x0d); //clk_post_time

HI258_write_cmos_sensor(0x1c, 0x01); //tlps_time_l_dp
HI258_write_cmos_sensor(0x1d, 0x0b); //tlps_time_l_dn
HI258_write_cmos_sensor(0x1e, 0x06); //hs_zero_time
HI258_write_cmos_sensor(0x1f, 0x09); //hs_trail_time

//long_packet word count 
HI258_write_cmos_sensor(0x30, 0x06);
HI258_write_cmos_sensor(0x31, 0x40); //long_packet word count

HI258_write_cmos_sensor(0x03, 0x20);
HI258_write_cmos_sensor(0x10, 0x9c);
HI258_write_cmos_sensor(0x03, 0x22);
HI258_write_cmos_sensor(0x10, 0xe9);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x0e, 0x03);
HI258_write_cmos_sensor(0x0e, 0x73);

HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x01, 0x00);
}

void HI258_Init_Cmds(void) 
{
printk("HI258_Init_Cmds\n");
///// Start Sleep /////
HI258_write_cmos_sensor(0x01, 0x01); //sleep on
HI258_write_cmos_sensor(0x01, 0x03); //sleep off
HI258_write_cmos_sensor(0x01, 0x01); //sleep on
// PAGE 20
HI258_write_cmos_sensor(0x03, 0x20); // page 20
HI258_write_cmos_sensor(0x10, 0x1c); // AE off 60hz

// PAGE 22
HI258_write_cmos_sensor(0x03, 0x22); // page 22
HI258_write_cmos_sensor(0x10, 0x69); // AWB off

HI258_write_cmos_sensor(0x03, 0x00); //Dummy 750us
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);

HI258_write_cmos_sensor(0x08, 0x00);
HI258_write_cmos_sensor(0x09, 0x77);// pad strength = max
HI258_write_cmos_sensor(0x0a, 0x07);// pad strength = max

//PLL Setting
HI258_write_cmos_sensor(0x03, 0x00); 
HI258_write_cmos_sensor(0xd0, 0x05); //PLL pre_div 1/6 = 4 Mhz
HI258_write_cmos_sensor(0xd1, 0x30); //PLL maim_div 
HI258_write_cmos_sensor(0xd2, 0x05); //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    
HI258_write_cmos_sensor(0xd3, 0x20); //isp_clk_inv[0]  mipi_4x_inv[1]  mipi_1x_inv[2]
HI258_write_cmos_sensor(0xd0, 0x85);
HI258_write_cmos_sensor(0xd0, 0x85);
HI258_write_cmos_sensor(0xd0, 0x85);
HI258_write_cmos_sensor(0xd0, 0x95);

HI258_write_cmos_sensor(0x03, 0x00); //Dummy 750us
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);


///// PAGE 20 /////
HI258_write_cmos_sensor(0x03, 0x20); //page 20
HI258_write_cmos_sensor(0x10, 0x1c); //AE off 50hz

///// PAGE 22 /////
HI258_write_cmos_sensor(0x03, 0x22); //page 22
HI258_write_cmos_sensor(0x10, 0x69); //AWB off


///// Initial Start /////
///// PAGE 0 Start /////
HI258_write_cmos_sensor(0x03, 0x00); //page 0
HI258_write_cmos_sensor(0x10, 0x00);
HI258_write_cmos_sensor(0x11, 0x90); //Windowing On + 1Frame Skip
HI258_write_cmos_sensor(0x12, 0x04); //Rinsing edge 0x04 // Falling edge 0x00
HI258_write_cmos_sensor(0x14, 0x05);

HI258_write_cmos_sensor(0x20, 0x00); //Row H
HI258_write_cmos_sensor(0x21, 0x04); //Row L
HI258_write_cmos_sensor(0x22, 0x00); //Col H
HI258_write_cmos_sensor(0x23, 0x04); //Col L
HI258_write_cmos_sensor(0x24, 0x04);
HI258_write_cmos_sensor(0x25, 0xb0);
HI258_write_cmos_sensor(0x26, 0x06);
HI258_write_cmos_sensor(0x27, 0x40);
HI258_write_cmos_sensor(0x40, 0x01);
HI258_write_cmos_sensor(0x41, 0x78);
HI258_write_cmos_sensor(0x42, 0x00);
HI258_write_cmos_sensor(0x43, 0x14);
HI258_write_cmos_sensor(0x50, 0x00);
HI258_write_cmos_sensor(0x80, 0x2e);
HI258_write_cmos_sensor(0x81, 0x7e);
HI258_write_cmos_sensor(0x82, 0x90);
HI258_write_cmos_sensor(0x83, 0x00);
HI258_write_cmos_sensor(0x84, 0x0c);
HI258_write_cmos_sensor(0x85, 0x00);
HI258_write_cmos_sensor(0x86, 0x00);
HI258_write_cmos_sensor(0x87, 0x0f);
HI258_write_cmos_sensor(0x88, 0x34);
HI258_write_cmos_sensor(0x8a, 0x0b);
HI258_write_cmos_sensor(0x8e, 0x80);
HI258_write_cmos_sensor(0x90, 0x0d);
HI258_write_cmos_sensor(0x91, 0x0d);
HI258_write_cmos_sensor(0x92, 0xe8);
HI258_write_cmos_sensor(0x93, 0xe0);
HI258_write_cmos_sensor(0x96, 0xdc);
HI258_write_cmos_sensor(0x97, 0xfe);
HI258_write_cmos_sensor(0x98, 0x38);
HI258_write_cmos_sensor(0x99, 0x43);
HI258_write_cmos_sensor(0xa0, 0x01);
HI258_write_cmos_sensor(0xa8, 0x43);



//page 2
HI258_write_cmos_sensor(0x03, 0x02);
HI258_write_cmos_sensor(0x10, 0x00);
HI258_write_cmos_sensor(0x13, 0x00);
HI258_write_cmos_sensor(0x14, 0x00);
HI258_write_cmos_sensor(0x18, 0xcc);
HI258_write_cmos_sensor(0x19, 0x01);
HI258_write_cmos_sensor(0x1A, 0x09);
HI258_write_cmos_sensor(0x1B, 0x00);
HI258_write_cmos_sensor(0x1C, 0x1a);
HI258_write_cmos_sensor(0x1D, 0x14);
HI258_write_cmos_sensor(0x1E, 0x30);
HI258_write_cmos_sensor(0x1F, 0x10);
HI258_write_cmos_sensor(0x20, 0x77);
HI258_write_cmos_sensor(0x21, 0xde);
HI258_write_cmos_sensor(0x22, 0xa7);
HI258_write_cmos_sensor(0x23, 0x30);
HI258_write_cmos_sensor(0x24, 0x77);
HI258_write_cmos_sensor(0x25, 0x10);
HI258_write_cmos_sensor(0x26, 0x10);
HI258_write_cmos_sensor(0x27, 0x3c);
HI258_write_cmos_sensor(0x2b, 0x80);
HI258_write_cmos_sensor(0x2c, 0x02);
HI258_write_cmos_sensor(0x2d, 0x58);
HI258_write_cmos_sensor(0x2e, 0xde);
HI258_write_cmos_sensor(0x2f, 0xa7);
HI258_write_cmos_sensor(0x30, 0x00);
HI258_write_cmos_sensor(0x31, 0x99);
HI258_write_cmos_sensor(0x32, 0x00);
HI258_write_cmos_sensor(0x33, 0x00);
HI258_write_cmos_sensor(0x34, 0x22);
HI258_write_cmos_sensor(0x36, 0x75);
HI258_write_cmos_sensor(0x38, 0x88);
HI258_write_cmos_sensor(0x39, 0x88);
HI258_write_cmos_sensor(0x3d, 0x03);
HI258_write_cmos_sensor(0x3f, 0x02);
HI258_write_cmos_sensor(0x49, 0x87);
HI258_write_cmos_sensor(0x4a, 0x10);
HI258_write_cmos_sensor(0x50, 0x21);
HI258_write_cmos_sensor(0x53, 0xb1);
HI258_write_cmos_sensor(0x54, 0x10);
HI258_write_cmos_sensor(0x55, 0x1c); 
HI258_write_cmos_sensor(0x56, 0x11);
HI258_write_cmos_sensor(0x5d, 0xa2);
HI258_write_cmos_sensor(0x5e, 0x5a);
HI258_write_cmos_sensor(0x5d, 0xa2);
HI258_write_cmos_sensor(0x5e, 0x5a);
HI258_write_cmos_sensor(0x60, 0x87);
HI258_write_cmos_sensor(0x61, 0x98);
HI258_write_cmos_sensor(0x62, 0x88);
HI258_write_cmos_sensor(0x63, 0x96);
HI258_write_cmos_sensor(0x64, 0x88);
HI258_write_cmos_sensor(0x65, 0x96);
HI258_write_cmos_sensor(0x67, 0x3f);
HI258_write_cmos_sensor(0x68, 0x3f);
HI258_write_cmos_sensor(0x69, 0x3f);
HI258_write_cmos_sensor(0x72, 0x89);
HI258_write_cmos_sensor(0x73, 0x95);
HI258_write_cmos_sensor(0x74, 0x89);
HI258_write_cmos_sensor(0x75, 0x95);
HI258_write_cmos_sensor(0x7C, 0x84);
HI258_write_cmos_sensor(0x7D, 0xaf);
HI258_write_cmos_sensor(0x80, 0x01);
HI258_write_cmos_sensor(0x81, 0x7a);
HI258_write_cmos_sensor(0x82, 0x13);
HI258_write_cmos_sensor(0x83, 0x24);
HI258_write_cmos_sensor(0x84, 0x78);
HI258_write_cmos_sensor(0x85, 0x7c);
HI258_write_cmos_sensor(0x92, 0x44);
HI258_write_cmos_sensor(0x93, 0x59);
HI258_write_cmos_sensor(0x94, 0x78);
HI258_write_cmos_sensor(0x95, 0x7c);
HI258_write_cmos_sensor(0xA0, 0x02);
HI258_write_cmos_sensor(0xA1, 0x74);
HI258_write_cmos_sensor(0xA4, 0x74);
HI258_write_cmos_sensor(0xA5, 0x02);
HI258_write_cmos_sensor(0xA8, 0x85);
HI258_write_cmos_sensor(0xA9, 0x8c);
HI258_write_cmos_sensor(0xAC, 0x10);
HI258_write_cmos_sensor(0xAD, 0x16);
HI258_write_cmos_sensor(0xB0, 0x99);
HI258_write_cmos_sensor(0xB1, 0xa3);
HI258_write_cmos_sensor(0xB4, 0x9b);
HI258_write_cmos_sensor(0xB5, 0xa2);
HI258_write_cmos_sensor(0xB8, 0x9b);
HI258_write_cmos_sensor(0xB9, 0x9f);
HI258_write_cmos_sensor(0xBC, 0x9b);
HI258_write_cmos_sensor(0xBD, 0x9f);
HI258_write_cmos_sensor(0xc4, 0x29);
HI258_write_cmos_sensor(0xc5, 0x40);
HI258_write_cmos_sensor(0xc6, 0x5c);
HI258_write_cmos_sensor(0xc7, 0x72);
HI258_write_cmos_sensor(0xc8, 0x2a);
HI258_write_cmos_sensor(0xc9, 0x3f);
HI258_write_cmos_sensor(0xcc, 0x5d);
HI258_write_cmos_sensor(0xcd, 0x71);
HI258_write_cmos_sensor(0xd0, 0x10);
HI258_write_cmos_sensor(0xd1, 0x14);
HI258_write_cmos_sensor(0xd2, 0x20);
HI258_write_cmos_sensor(0xd3, 0x00);
HI258_write_cmos_sensor(0xd4, 0x0d);
HI258_write_cmos_sensor(0xd5, 0x0d);
HI258_write_cmos_sensor(0xd6, 0xe8);
HI258_write_cmos_sensor(0xd7, 0xe0);
HI258_write_cmos_sensor(0xdc, 0x00);
HI258_write_cmos_sensor(0xdd, 0xa3);
HI258_write_cmos_sensor(0xde, 0x00);
HI258_write_cmos_sensor(0xdf, 0x84);
HI258_write_cmos_sensor(0xe0, 0xa4);
HI258_write_cmos_sensor(0xe1, 0xa4);
HI258_write_cmos_sensor(0xe2, 0xa4);
HI258_write_cmos_sensor(0xe3, 0xa4);
HI258_write_cmos_sensor(0xe4, 0xa4);
HI258_write_cmos_sensor(0xe5, 0x01);
HI258_write_cmos_sensor(0xe8, 0x00);
HI258_write_cmos_sensor(0xe9, 0x00);
HI258_write_cmos_sensor(0xea, 0x77);
HI258_write_cmos_sensor(0xF0, 0x00);
HI258_write_cmos_sensor(0xF1, 0x00);
HI258_write_cmos_sensor(0xF2, 0x00);
HI258_write_cmos_sensor(0x03, 0x10);//page10
HI258_write_cmos_sensor(0x10, 0x03);
HI258_write_cmos_sensor(0x11, 0x03);
HI258_write_cmos_sensor(0x12, 0xf0);
HI258_write_cmos_sensor(0x13, 0x03);
HI258_write_cmos_sensor(0x20, 0x00);
HI258_write_cmos_sensor(0x21, 0x40);
HI258_write_cmos_sensor(0x22, 0x0f);
HI258_write_cmos_sensor(0x24, 0x20);
HI258_write_cmos_sensor(0x25, 0x10);
HI258_write_cmos_sensor(0x26, 0x01);
HI258_write_cmos_sensor(0x27, 0x02);
HI258_write_cmos_sensor(0x28, 0x11);
HI258_write_cmos_sensor(0x40, 0x80);
HI258_write_cmos_sensor(0x41, 0x12); 
HI258_write_cmos_sensor(0x42, 0x05); 
HI258_write_cmos_sensor(0x43, 0x05); 
HI258_write_cmos_sensor(0x44, 0x80);
HI258_write_cmos_sensor(0x45, 0x80);
HI258_write_cmos_sensor(0x46, 0xf0);
HI258_write_cmos_sensor(0x48, 0x80);
HI258_write_cmos_sensor(0x4a, 0x80);
HI258_write_cmos_sensor(0x50, 0xa0); 
HI258_write_cmos_sensor(0x60, 0x0f);
HI258_write_cmos_sensor(0x61, 0x90); //b a7
HI258_write_cmos_sensor(0x62, 0x90); //r 9e
HI258_write_cmos_sensor(0x63, 0x70); 
HI258_write_cmos_sensor(0x66, 0x42);
HI258_write_cmos_sensor(0x67, 0x22);
HI258_write_cmos_sensor(0x6a, 0x88); 
HI258_write_cmos_sensor(0x74, 0x0a); 
HI258_write_cmos_sensor(0x76, 0x01); 

HI258_write_cmos_sensor(0x03, 0x11); //page11
HI258_write_cmos_sensor(0x20, 0x00);
HI258_write_cmos_sensor(0x21, 0x00);
HI258_write_cmos_sensor(0x26, 0x58); 
HI258_write_cmos_sensor(0x27, 0x52); 
HI258_write_cmos_sensor(0x28, 0x0f);
HI258_write_cmos_sensor(0x29, 0x10);
HI258_write_cmos_sensor(0x2b, 0x30);
HI258_write_cmos_sensor(0x2c, 0x32);
HI258_write_cmos_sensor(0x70, 0x2b);
HI258_write_cmos_sensor(0x74, 0x30);
HI258_write_cmos_sensor(0x75, 0x18);
HI258_write_cmos_sensor(0x76, 0x30);
HI258_write_cmos_sensor(0x77, 0xff);
HI258_write_cmos_sensor(0x78, 0xa0);
HI258_write_cmos_sensor(0x79, 0xff); 
HI258_write_cmos_sensor(0x7a, 0x30);
HI258_write_cmos_sensor(0x7b, 0x20);
HI258_write_cmos_sensor(0x7c, 0xf4); 
HI258_write_cmos_sensor(0x03, 0x12);//page12 
HI258_write_cmos_sensor(0x10, 0x03); 
HI258_write_cmos_sensor(0x11, 0x08); 
HI258_write_cmos_sensor(0x12, 0x10); 
HI258_write_cmos_sensor(0x20, 0x53); 
HI258_write_cmos_sensor(0x21, 0x03); 
HI258_write_cmos_sensor(0x22, 0xe6); 
HI258_write_cmos_sensor(0x23, 0x0a); //outdoor
HI258_write_cmos_sensor(0x24, 0x0b); //indoor
HI258_write_cmos_sensor(0x25, 0x1b);//dark 
HI258_write_cmos_sensor(0x30, 0xff); 
HI258_write_cmos_sensor(0x31, 0x00); 
HI258_write_cmos_sensor(0x32, 0xf0); 
HI258_write_cmos_sensor(0x33, 0x00); 
HI258_write_cmos_sensor(0x34, 0x00); 
HI258_write_cmos_sensor(0x35, 0xff); 
HI258_write_cmos_sensor(0x36, 0x00); 
HI258_write_cmos_sensor(0x37, 0xff); 
HI258_write_cmos_sensor(0x38, 0x00); 
HI258_write_cmos_sensor(0x39, 0x00); 
HI258_write_cmos_sensor(0x3a, 0xff); 
HI258_write_cmos_sensor(0x3b, 0x00); 
HI258_write_cmos_sensor(0x3c, 0x93); 
HI258_write_cmos_sensor(0x3d, 0x00); 
HI258_write_cmos_sensor(0x3e, 0x00); 
HI258_write_cmos_sensor(0x46, 0xa0);//Out Lum Hi 
HI258_write_cmos_sensor(0x47, 0x40); //Out Lum Lo
HI258_write_cmos_sensor(0x4c, 0xa0);//Indoor Lum Hi bigger will be darker
HI258_write_cmos_sensor(0x4d, 0x40);//Indoor Lum Lo
HI258_write_cmos_sensor(0x52, 0xa0);//Dark Lum Hi 
HI258_write_cmos_sensor(0x53, 0x40); //Dark Lum Lo
HI258_write_cmos_sensor(0x70, 0x10); 
HI258_write_cmos_sensor(0x71, 0x0a); 
HI258_write_cmos_sensor(0x72, 0x10); 
HI258_write_cmos_sensor(0x73, 0x0a); 
HI258_write_cmos_sensor(0x74, 0x14); 
HI258_write_cmos_sensor(0x75, 0x0c); 
HI258_write_cmos_sensor(0x90, 0x7d);
HI258_write_cmos_sensor(0x91, 0x34);
HI258_write_cmos_sensor(0x99, 0x28);
HI258_write_cmos_sensor(0x9c, 0x14);
HI258_write_cmos_sensor(0x9d, 0x15);
HI258_write_cmos_sensor(0x9e, 0x28);
HI258_write_cmos_sensor(0x9f, 0x28);
HI258_write_cmos_sensor(0xb0, 0x0e); 
HI258_write_cmos_sensor(0xb8, 0x44);
HI258_write_cmos_sensor(0xb9, 0x15);

HI258_write_cmos_sensor(0x03, 0x13);//page13 
HI258_write_cmos_sensor(0x80, 0xfd); 
HI258_write_cmos_sensor(0x81, 0x07); 
HI258_write_cmos_sensor(0x82, 0x73); 
HI258_write_cmos_sensor(0x83, 0x00); 
HI258_write_cmos_sensor(0x85, 0x00);
HI258_write_cmos_sensor(0x92, 0x33); 
HI258_write_cmos_sensor(0x93, 0x30); 
HI258_write_cmos_sensor(0x94, 0x02); 
HI258_write_cmos_sensor(0x95, 0xf0); 
HI258_write_cmos_sensor(0x96, 0x1e); 
HI258_write_cmos_sensor(0x97, 0x40); 
HI258_write_cmos_sensor(0x98, 0x80);
HI258_write_cmos_sensor(0x99, 0x40);
HI258_write_cmos_sensor(0xa2, 0x04);//outdoor clip
HI258_write_cmos_sensor(0xa3, 0x05);
HI258_write_cmos_sensor(0xa4, 0x05);//indoor clip
HI258_write_cmos_sensor(0xa5, 0x06);
HI258_write_cmos_sensor(0xa6, 0x08);//dark1 clip
HI258_write_cmos_sensor(0xa7, 0x09);//dark1 clip
HI258_write_cmos_sensor(0xb6, 0x36);
HI258_write_cmos_sensor(0xb7, 0x36);
HI258_write_cmos_sensor(0xb8, 0x36);
HI258_write_cmos_sensor(0xb9, 0x36);
HI258_write_cmos_sensor(0xba, 0x36);
HI258_write_cmos_sensor(0xbb, 0x36);
HI258_write_cmos_sensor(0xbc, 0x30);//indoor
HI258_write_cmos_sensor(0xbd, 0x28);
HI258_write_cmos_sensor(0xbe, 0x28);
HI258_write_cmos_sensor(0xbf, 0x30);
HI258_write_cmos_sensor(0xc0, 0x28);
HI258_write_cmos_sensor(0xc1, 0x28);
HI258_write_cmos_sensor(0xc2, 0x18);//dark
HI258_write_cmos_sensor(0xc3, 0x18);//Lum negative middle
HI258_write_cmos_sensor(0xc4, 0x18);
HI258_write_cmos_sensor(0xc5, 0x18);
HI258_write_cmos_sensor(0xc6, 0x18);//Lum postive middle
HI258_write_cmos_sensor(0xc7, 0x18);
///// PAGE 14 START /////
HI258_write_cmos_sensor(0x03, 0x14); //page 14
HI258_write_cmos_sensor(0x10, 0x09);

HI258_write_cmos_sensor(0x20, 0xa0); //X-Center
HI258_write_cmos_sensor(0x21, 0x80); //Y-Center

HI258_write_cmos_sensor(0x22, 0x56); //LSC R 60 4a 4a 
HI258_write_cmos_sensor(0x23, 0x4c); //LSC G
HI258_write_cmos_sensor(0x24, 0x48); //LSC B

HI258_write_cmos_sensor(0x25, 0xf0); //LSC Off
HI258_write_cmos_sensor(0x26, 0xf0); //LSC On
///// PAGE 14 END /////

/////// PAGE 15 START ///////
HI258_write_cmos_sensor(0x03, 0x15);//page 15
	HI258_write_cmos_sensor(0x10, 0x0f); 
	HI258_write_cmos_sensor(0x14, 0x4b); 
	HI258_write_cmos_sensor(0x15, 0x3d); 
	HI258_write_cmos_sensor(0x16, 0x2e); 
	HI258_write_cmos_sensor(0x17, 0x2f); 
//CMC
	//CMC
	HI258_write_cmos_sensor(0x30, 0x8e); 
	HI258_write_cmos_sensor(0x31, 0x75); 
	HI258_write_cmos_sensor(0x32, 0x25); 
	HI258_write_cmos_sensor(0x33, 0x15); 
	HI258_write_cmos_sensor(0x34, 0x5a); 
	HI258_write_cmos_sensor(0x35, 0x05); 
	HI258_write_cmos_sensor(0x36, 0x07); 
	HI258_write_cmos_sensor(0x37, 0x40); 
	HI258_write_cmos_sensor(0x38, 0x85); 

	//CMC OFS
	HI258_write_cmos_sensor(0x40, 0x95); 
	HI258_write_cmos_sensor(0x41, 0x1f); 
	HI258_write_cmos_sensor(0x42, 0x8a); 
	HI258_write_cmos_sensor(0x43, 0x86); 
	HI258_write_cmos_sensor(0x44, 0x0a); 
	HI258_write_cmos_sensor(0x45, 0x84); 
	HI258_write_cmos_sensor(0x46, 0x87); 
	HI258_write_cmos_sensor(0x47, 0x9b); 
	HI258_write_cmos_sensor(0x48, 0x23); 

	//CMC POFS
	HI258_write_cmos_sensor(0x50, 0x8c); 
	HI258_write_cmos_sensor(0x51, 0x0c); 
	HI258_write_cmos_sensor(0x52, 0x00); 
	HI258_write_cmos_sensor(0x53, 0x07); 
	HI258_write_cmos_sensor(0x54, 0x17); 
	HI258_write_cmos_sensor(0x55, 0x9d); 
	HI258_write_cmos_sensor(0x56, 0x00); 
	HI258_write_cmos_sensor(0x57, 0x0b); 
	HI258_write_cmos_sensor(0x58, 0x89); 
///// PAGE 15 END /////
///// PAGE 16 START /////
HI258_write_cmos_sensor(0x03, 0x16);
HI258_write_cmos_sensor(0x10, 0x31);//GMA_CTL
HI258_write_cmos_sensor(0x18, 0x5e);//AG_ON
HI258_write_cmos_sensor(0x19, 0x5d);//AG_OFF
HI258_write_cmos_sensor(0x1A, 0x0e);//TIME_ON
HI258_write_cmos_sensor(0x1B, 0x01);//TIME_OFF
HI258_write_cmos_sensor(0x1C, 0xdc);//OUT_ON
HI258_write_cmos_sensor(0x1D, 0xfe);//OUT_OFF
//GMA
HI258_write_cmos_sensor(0x30, 0x00);
HI258_write_cmos_sensor(0x31, 0x03);
HI258_write_cmos_sensor(0x32, 0x0e);
HI258_write_cmos_sensor(0x33, 0x19);
HI258_write_cmos_sensor(0x34, 0x3d);
HI258_write_cmos_sensor(0x35, 0x57);
HI258_write_cmos_sensor(0x36, 0x6b);
HI258_write_cmos_sensor(0x37, 0x7f);
HI258_write_cmos_sensor(0x38, 0x90);
HI258_write_cmos_sensor(0x39, 0xa0);
HI258_write_cmos_sensor(0x3a, 0xae);
HI258_write_cmos_sensor(0x3b, 0xbc);
HI258_write_cmos_sensor(0x3c, 0xc8);
HI258_write_cmos_sensor(0x3d, 0xd4);
HI258_write_cmos_sensor(0x3e, 0xde);
HI258_write_cmos_sensor(0x3f, 0xe8);
HI258_write_cmos_sensor(0x40, 0xf1);
HI258_write_cmos_sensor(0x41, 0xf8);
HI258_write_cmos_sensor(0x42, 0xff);


//Outdoor
HI258_write_cmos_sensor(0x50, 0x00);
HI258_write_cmos_sensor(0x51, 0x03);
HI258_write_cmos_sensor(0x52, 0x10);
HI258_write_cmos_sensor(0x53, 0x26);
HI258_write_cmos_sensor(0x54, 0x43);
HI258_write_cmos_sensor(0x55, 0x5d);
HI258_write_cmos_sensor(0x56, 0x79);
HI258_write_cmos_sensor(0x57, 0x8c);
HI258_write_cmos_sensor(0x58, 0x9f);
HI258_write_cmos_sensor(0x59, 0xaa);
HI258_write_cmos_sensor(0x5a, 0xb6);
HI258_write_cmos_sensor(0x5b, 0xc3);
HI258_write_cmos_sensor(0x5c, 0xce);
HI258_write_cmos_sensor(0x5d, 0xd9);
HI258_write_cmos_sensor(0x5e, 0xe1);
HI258_write_cmos_sensor(0x5f, 0xe9);
HI258_write_cmos_sensor(0x60, 0xf0);
HI258_write_cmos_sensor(0x61, 0xf4);
HI258_write_cmos_sensor(0x62, 0xf5);

//Dark
HI258_write_cmos_sensor(0x70, 0x00);
HI258_write_cmos_sensor(0x71, 0x05);
HI258_write_cmos_sensor(0x72, 0x12);
HI258_write_cmos_sensor(0x73, 0x1c);
HI258_write_cmos_sensor(0x74, 0x3e);
HI258_write_cmos_sensor(0x75, 0x56);
HI258_write_cmos_sensor(0x76, 0x6b);
HI258_write_cmos_sensor(0x77, 0x7f);
HI258_write_cmos_sensor(0x78, 0x90);
HI258_write_cmos_sensor(0x79, 0xa0);
HI258_write_cmos_sensor(0x7a, 0xae);
HI258_write_cmos_sensor(0x7b, 0xbc);
HI258_write_cmos_sensor(0x7c, 0xc8);
HI258_write_cmos_sensor(0x7d, 0xd4);
HI258_write_cmos_sensor(0x7e, 0xde);
HI258_write_cmos_sensor(0x7f, 0xe8);
HI258_write_cmos_sensor(0x80, 0xf1);
HI258_write_cmos_sensor(0x81, 0xf8);
HI258_write_cmos_sensor(0x82, 0xff);///// PAGE 16 END /////

///// PAGE 17 START /////
HI258_write_cmos_sensor(0x03, 0x17); //page 17
HI258_write_cmos_sensor(0xc1, 0x00);
HI258_write_cmos_sensor(0xc4, 0x4b);
HI258_write_cmos_sensor(0xc5, 0x3f);
HI258_write_cmos_sensor(0xc6, 0x02);
HI258_write_cmos_sensor(0xc7, 0x20);
HI258_write_cmos_sensor(0x03, 0x18);//page18
HI258_write_cmos_sensor(0x10, 0x00);
HI258_write_cmos_sensor(0x03, 0x19);//page19
HI258_write_cmos_sensor(0x10, 0x7f);
HI258_write_cmos_sensor(0x11, 0x7f);
HI258_write_cmos_sensor(0x12, 0x1e);
HI258_write_cmos_sensor(0x13, 0x32);
HI258_write_cmos_sensor(0x14, 0x1e);
HI258_write_cmos_sensor(0x15, 0x5e);
HI258_write_cmos_sensor(0x16, 0x0a);
HI258_write_cmos_sensor(0x17, 0xb8);
HI258_write_cmos_sensor(0x18, 0x1e);
HI258_write_cmos_sensor(0x19, 0xe6);
HI258_write_cmos_sensor(0x1a, 0x9e);
HI258_write_cmos_sensor(0x1b, 0x22);
HI258_write_cmos_sensor(0x1c, 0x9e);
HI258_write_cmos_sensor(0x1d, 0x5e);
HI258_write_cmos_sensor(0x1e, 0x3b);
HI258_write_cmos_sensor(0x1f, 0x30);//26
HI258_write_cmos_sensor(0x20, 0x40);//50
HI258_write_cmos_sensor(0x21, 0x40);//60
HI258_write_cmos_sensor(0x22, 0x2f);
HI258_write_cmos_sensor(0x23, 0x27);
HI258_write_cmos_sensor(0x24, 0x00);
HI258_write_cmos_sensor(0x25, 0x01);
HI258_write_cmos_sensor(0x26, 0x0e);
HI258_write_cmos_sensor(0x27, 0x04);
HI258_write_cmos_sensor(0x28, 0x00);
HI258_write_cmos_sensor(0x29, 0x8c);
HI258_write_cmos_sensor(0x2a, 0x40);
HI258_write_cmos_sensor(0x2b, 0x3f);
HI258_write_cmos_sensor(0x2c, 0x00);
HI258_write_cmos_sensor(0x2d, 0x00);
HI258_write_cmos_sensor(0x2e, 0x00);
HI258_write_cmos_sensor(0x2f, 0x00);
HI258_write_cmos_sensor(0x30, 0x00);
HI258_write_cmos_sensor(0x31, 0x00);
HI258_write_cmos_sensor(0x32, 0x00);
HI258_write_cmos_sensor(0x33, 0x00);
HI258_write_cmos_sensor(0x34, 0x00);
HI258_write_cmos_sensor(0x35, 0x00);
HI258_write_cmos_sensor(0x36, 0x00);
HI258_write_cmos_sensor(0x37, 0x00);
HI258_write_cmos_sensor(0x38, 0x00);
HI258_write_cmos_sensor(0x39, 0x00);
HI258_write_cmos_sensor(0x3a, 0x00);
HI258_write_cmos_sensor(0x3b, 0x00);
HI258_write_cmos_sensor(0x3c, 0x00);
HI258_write_cmos_sensor(0x3d, 0x00);
HI258_write_cmos_sensor(0x3e, 0x00);
HI258_write_cmos_sensor(0x3f, 0x00);
HI258_write_cmos_sensor(0x40, 0x00);
HI258_write_cmos_sensor(0x41, 0x00);
HI258_write_cmos_sensor(0x42, 0x00);
HI258_write_cmos_sensor(0x43, 0x00);
HI258_write_cmos_sensor(0x44, 0x00);
HI258_write_cmos_sensor(0x45, 0x00);
HI258_write_cmos_sensor(0x46, 0x00);
HI258_write_cmos_sensor(0x47, 0x00);
HI258_write_cmos_sensor(0x48, 0x00);
HI258_write_cmos_sensor(0x49, 0x00);
HI258_write_cmos_sensor(0x4a, 0x00);
HI258_write_cmos_sensor(0x4b, 0x00);
HI258_write_cmos_sensor(0x4c, 0x00);
HI258_write_cmos_sensor(0x4d, 0x00);
HI258_write_cmos_sensor(0x4e, 0x00);
HI258_write_cmos_sensor(0x4f, 0x00);
HI258_write_cmos_sensor(0x50, 0x00);
HI258_write_cmos_sensor(0x51, 0x00);
HI258_write_cmos_sensor(0x52, 0x00);
HI258_write_cmos_sensor(0x53, 0x10);
HI258_write_cmos_sensor(0x54, 0x00);
HI258_write_cmos_sensor(0x55, 0x01);
HI258_write_cmos_sensor(0x56, 0x1b);
HI258_write_cmos_sensor(0x57, 0x39);
HI258_write_cmos_sensor(0x58, 0x5a);
HI258_write_cmos_sensor(0x59, 0x80);
HI258_write_cmos_sensor(0x5a, 0xa6);
HI258_write_cmos_sensor(0x5b, 0xc1);
HI258_write_cmos_sensor(0x5c, 0xe8);
HI258_write_cmos_sensor(0x5d, 0x38);
HI258_write_cmos_sensor(0x5e, 0x3a);
HI258_write_cmos_sensor(0x5f, 0x3c);
HI258_write_cmos_sensor(0x60, 0x3f);
HI258_write_cmos_sensor(0x61, 0x3f);
HI258_write_cmos_sensor(0x62, 0x3f);
HI258_write_cmos_sensor(0x63, 0x3f);
HI258_write_cmos_sensor(0x64, 0x3f);
HI258_write_cmos_sensor(0x65, 0x00);
HI258_write_cmos_sensor(0x66, 0x00);
HI258_write_cmos_sensor(0x67, 0x00);
HI258_write_cmos_sensor(0x68, 0x00);
HI258_write_cmos_sensor(0x69, 0x00);
HI258_write_cmos_sensor(0x6a, 0xff);
HI258_write_cmos_sensor(0x6b, 0x00);
HI258_write_cmos_sensor(0x6c, 0xff);
HI258_write_cmos_sensor(0x6d, 0x3f);
HI258_write_cmos_sensor(0x6e, 0x00);
HI258_write_cmos_sensor(0x6f, 0x00);
HI258_write_cmos_sensor(0x70, 0x00);
HI258_write_cmos_sensor(0x71, 0x3f);
HI258_write_cmos_sensor(0x72, 0x3f);
HI258_write_cmos_sensor(0x73, 0x3f);
HI258_write_cmos_sensor(0x74, 0x3f);
HI258_write_cmos_sensor(0x75, 0x30);
HI258_write_cmos_sensor(0x76, 0x50);
HI258_write_cmos_sensor(0x77, 0x80);
HI258_write_cmos_sensor(0x78, 0xb0);
HI258_write_cmos_sensor(0x79, 0x3f);
HI258_write_cmos_sensor(0x7a, 0x3f);
HI258_write_cmos_sensor(0x7b, 0x3f);
HI258_write_cmos_sensor(0x7c, 0x3f);
HI258_write_cmos_sensor(0x7d, 0x28);
HI258_write_cmos_sensor(0x7e, 0x50);
HI258_write_cmos_sensor(0x7f, 0x80);
HI258_write_cmos_sensor(0x80, 0xb0);
HI258_write_cmos_sensor(0x81, 0x28);
HI258_write_cmos_sensor(0x82, 0x3f);
HI258_write_cmos_sensor(0x83, 0x3f);
HI258_write_cmos_sensor(0x84, 0x3f);
HI258_write_cmos_sensor(0x85, 0x28);
HI258_write_cmos_sensor(0x86, 0x50);
HI258_write_cmos_sensor(0x87, 0x80);
HI258_write_cmos_sensor(0x88, 0xb0);
HI258_write_cmos_sensor(0x89, 0x1a);
HI258_write_cmos_sensor(0x8a, 0x28);
HI258_write_cmos_sensor(0x8b, 0x3f);
HI258_write_cmos_sensor(0x8c, 0x3f);
HI258_write_cmos_sensor(0x8d, 0x10);
HI258_write_cmos_sensor(0x8e, 0x30);
HI258_write_cmos_sensor(0x8f, 0x60);
HI258_write_cmos_sensor(0x90, 0x90);
HI258_write_cmos_sensor(0x91, 0x1a);
HI258_write_cmos_sensor(0x92, 0x28);
HI258_write_cmos_sensor(0x93, 0x3f);
HI258_write_cmos_sensor(0x94, 0x3f);
HI258_write_cmos_sensor(0x95, 0x28);
HI258_write_cmos_sensor(0x96, 0x50);
HI258_write_cmos_sensor(0x97, 0x80);
HI258_write_cmos_sensor(0x98, 0xb0);
HI258_write_cmos_sensor(0x99, 0x1a);
HI258_write_cmos_sensor(0x9a, 0x28);
HI258_write_cmos_sensor(0x9b, 0x3f);
HI258_write_cmos_sensor(0x9c, 0x3f);
HI258_write_cmos_sensor(0x9d, 0x28);
HI258_write_cmos_sensor(0x9e, 0x50);
HI258_write_cmos_sensor(0x9f, 0x80);
HI258_write_cmos_sensor(0xa0, 0xb0);
HI258_write_cmos_sensor(0xa2, 0x00);
HI258_write_cmos_sensor(0xe5, 0x80);
HI258_write_cmos_sensor(0x03, 0x20);//page20
HI258_write_cmos_sensor(0x11, 0x1c);
HI258_write_cmos_sensor(0x18, 0x30);
HI258_write_cmos_sensor(0x20, 0x25);
HI258_write_cmos_sensor(0x21, 0x30);
HI258_write_cmos_sensor(0x22, 0x10);
HI258_write_cmos_sensor(0x23, 0x00);
HI258_write_cmos_sensor(0x28, 0xe7);
HI258_write_cmos_sensor(0x29, 0x0d);
HI258_write_cmos_sensor(0x2a, 0xff);
HI258_write_cmos_sensor(0x2b, 0x04);
HI258_write_cmos_sensor(0x2c, 0x83);
HI258_write_cmos_sensor(0x2d, 0x03);
HI258_write_cmos_sensor(0x2e, 0x13);
HI258_write_cmos_sensor(0x2f, 0x0b);
HI258_write_cmos_sensor(0x30, 0x78);
HI258_write_cmos_sensor(0x31, 0xd7);
HI258_write_cmos_sensor(0x32, 0x10);
HI258_write_cmos_sensor(0x33, 0x2e);
HI258_write_cmos_sensor(0x34, 0x20);
HI258_write_cmos_sensor(0x35, 0xd4);
HI258_write_cmos_sensor(0x36, 0xfe);
HI258_write_cmos_sensor(0x37, 0x32);
HI258_write_cmos_sensor(0x38, 0x04);
HI258_write_cmos_sensor(0x39, 0x22);
HI258_write_cmos_sensor(0x3a, 0xde);
HI258_write_cmos_sensor(0x3b, 0x22);
HI258_write_cmos_sensor(0x3c, 0xde);
HI258_write_cmos_sensor(0x3d, 0xe1);
HI258_write_cmos_sensor(0x50, 0x45);
HI258_write_cmos_sensor(0x51, 0x88);
HI258_write_cmos_sensor(0x56, 0x1f);
HI258_write_cmos_sensor(0x57, 0xa6);
HI258_write_cmos_sensor(0x58, 0x1a);
HI258_write_cmos_sensor(0x59, 0x7a);
HI258_write_cmos_sensor(0x5a, 0x04);
HI258_write_cmos_sensor(0x5b, 0x04);
HI258_write_cmos_sensor(0x5e, 0xc7);
HI258_write_cmos_sensor(0x5f, 0x95);
HI258_write_cmos_sensor(0x62, 0x10);
HI258_write_cmos_sensor(0x63, 0xc0);
HI258_write_cmos_sensor(0x64, 0x10);
HI258_write_cmos_sensor(0x65, 0x8a);
HI258_write_cmos_sensor(0x66, 0x58);
HI258_write_cmos_sensor(0x67, 0x58);
HI258_write_cmos_sensor(0x70, 0x56);
HI258_write_cmos_sensor(0x71, 0x83);
HI258_write_cmos_sensor(0x76, 0x32);
HI258_write_cmos_sensor(0x77, 0xa1);
HI258_write_cmos_sensor(0x78, 0x22);
HI258_write_cmos_sensor(0x79, 0x30);
HI258_write_cmos_sensor(0x7a, 0x23);
HI258_write_cmos_sensor(0x7b, 0x22);
HI258_write_cmos_sensor(0x7d, 0x23);
HI258_write_cmos_sensor(0x83, 0x02);
HI258_write_cmos_sensor(0x84, 0xbf);
HI258_write_cmos_sensor(0x85, 0x20);
HI258_write_cmos_sensor(0x86, 0x01);
HI258_write_cmos_sensor(0x87, 0xf4);
HI258_write_cmos_sensor(0x88, 0x12);
HI258_write_cmos_sensor(0x89, 0x4f);
HI258_write_cmos_sensor(0x8a, 0x80);
HI258_write_cmos_sensor(0xa5, 0x0b);
HI258_write_cmos_sensor(0xa6, 0xe6);
HI258_write_cmos_sensor(0xa7, 0xe0);
HI258_write_cmos_sensor(0x8B, 0xea);
HI258_write_cmos_sensor(0x8C, 0x60);
HI258_write_cmos_sensor(0x8D, 0xc3);
HI258_write_cmos_sensor(0x8E, 0x50);
HI258_write_cmos_sensor(0x98, 0x9d);
HI258_write_cmos_sensor(0x99, 0x45);
HI258_write_cmos_sensor(0x9a, 0x0d);
HI258_write_cmos_sensor(0x9b, 0xde);
HI258_write_cmos_sensor(0x9c, 0x17);
HI258_write_cmos_sensor(0x9d, 0x70);
HI258_write_cmos_sensor(0x9e, 0x01);
HI258_write_cmos_sensor(0x9f, 0xf4);
HI258_write_cmos_sensor(0xb0, 0x15);
HI258_write_cmos_sensor(0xb1, 0x14);
HI258_write_cmos_sensor(0xb2, 0xb0);//AGmax
HI258_write_cmos_sensor(0xb3, 0x15);
HI258_write_cmos_sensor(0xb4, 0x16);
HI258_write_cmos_sensor(0xb5, 0x3c);
HI258_write_cmos_sensor(0xb6, 0x29);
HI258_write_cmos_sensor(0xb7, 0x23);
HI258_write_cmos_sensor(0xb8, 0x20);
HI258_write_cmos_sensor(0xb9, 0x1e);
HI258_write_cmos_sensor(0xba, 0x1c);
HI258_write_cmos_sensor(0xbb, 0x1b);
HI258_write_cmos_sensor(0xbc, 0x1b);
HI258_write_cmos_sensor(0xbd, 0x1a);
HI258_write_cmos_sensor(0xc0, 0x10);
HI258_write_cmos_sensor(0xc1, 0x40);
HI258_write_cmos_sensor(0xc2, 0x40);
HI258_write_cmos_sensor(0xc3, 0x40);
HI258_write_cmos_sensor(0xc4, 0x06);
HI258_write_cmos_sensor(0xc8, 0x80);
HI258_write_cmos_sensor(0xc9, 0x80);
HI258_write_cmos_sensor(0x03, 0x21);//page21
HI258_write_cmos_sensor(0x20, 0x11);
HI258_write_cmos_sensor(0x21, 0x11);
HI258_write_cmos_sensor(0x22, 0x11);
HI258_write_cmos_sensor(0x23, 0x11);
HI258_write_cmos_sensor(0x24, 0x14);
HI258_write_cmos_sensor(0x25, 0x44);
HI258_write_cmos_sensor(0x26, 0x44);
HI258_write_cmos_sensor(0x27, 0x41);
HI258_write_cmos_sensor(0x28, 0x14);
HI258_write_cmos_sensor(0x29, 0x44);
HI258_write_cmos_sensor(0x2a, 0x44);
HI258_write_cmos_sensor(0x2b, 0x41);
HI258_write_cmos_sensor(0x2c, 0x14);
HI258_write_cmos_sensor(0x2d, 0x47);
HI258_write_cmos_sensor(0x2e, 0x74);
HI258_write_cmos_sensor(0x2f, 0x41);
HI258_write_cmos_sensor(0x30, 0x14);
HI258_write_cmos_sensor(0x31, 0x47);
HI258_write_cmos_sensor(0x32, 0x74);
HI258_write_cmos_sensor(0x33, 0x41);
HI258_write_cmos_sensor(0x34, 0x14);
HI258_write_cmos_sensor(0x35, 0x44);
HI258_write_cmos_sensor(0x36, 0x44);
HI258_write_cmos_sensor(0x37, 0x41);
HI258_write_cmos_sensor(0x38, 0x14);
HI258_write_cmos_sensor(0x39, 0x44);
HI258_write_cmos_sensor(0x3a, 0x44);
HI258_write_cmos_sensor(0x3b, 0x41);
HI258_write_cmos_sensor(0x3c, 0x11);
HI258_write_cmos_sensor(0x3d, 0x11);
HI258_write_cmos_sensor(0x3e, 0x11);
HI258_write_cmos_sensor(0x3f, 0x11);
HI258_write_cmos_sensor(0x40, 0x11);
HI258_write_cmos_sensor(0x41, 0x11);
HI258_write_cmos_sensor(0x42, 0x11);
HI258_write_cmos_sensor(0x43, 0x11);
HI258_write_cmos_sensor(0x44, 0x14);
HI258_write_cmos_sensor(0x45, 0x44);
HI258_write_cmos_sensor(0x46, 0x44);
HI258_write_cmos_sensor(0x47, 0x41);
HI258_write_cmos_sensor(0x48, 0x14);
HI258_write_cmos_sensor(0x49, 0x44);
HI258_write_cmos_sensor(0x4a, 0x44);
HI258_write_cmos_sensor(0x4b, 0x41);
HI258_write_cmos_sensor(0x4c, 0x14);
HI258_write_cmos_sensor(0x4d, 0x47);
HI258_write_cmos_sensor(0x4e, 0x74);
HI258_write_cmos_sensor(0x4f, 0x41);
HI258_write_cmos_sensor(0x50, 0x14);
HI258_write_cmos_sensor(0x51, 0x47);
HI258_write_cmos_sensor(0x52, 0x74);
HI258_write_cmos_sensor(0x53, 0x41);
HI258_write_cmos_sensor(0x54, 0x14);
HI258_write_cmos_sensor(0x55, 0x44);
HI258_write_cmos_sensor(0x56, 0x44);
HI258_write_cmos_sensor(0x57, 0x41);
HI258_write_cmos_sensor(0x58, 0x14);
HI258_write_cmos_sensor(0x59, 0x44);
HI258_write_cmos_sensor(0x5a, 0x44);
HI258_write_cmos_sensor(0x5b, 0x41);
HI258_write_cmos_sensor(0x5c, 0x11);
HI258_write_cmos_sensor(0x5d, 0x11);
HI258_write_cmos_sensor(0x5e, 0x11);
HI258_write_cmos_sensor(0x5f, 0x11);
///// PAGE 22 START /////
HI258_write_cmos_sensor(0x03, 0x22); //page 22
HI258_write_cmos_sensor(0x10, 0xfd);
HI258_write_cmos_sensor(0x11, 0x2e);
HI258_write_cmos_sensor(0x19, 0x00);
HI258_write_cmos_sensor(0x20, 0x30); //For AWB Speed
HI258_write_cmos_sensor(0x21, 0x80);
HI258_write_cmos_sensor(0x22, 0x00);
HI258_write_cmos_sensor(0x23, 0x00);
HI258_write_cmos_sensor(0x24, 0x01);
HI258_write_cmos_sensor(0x25, 0x4f); //2013-09-13 AWB Hunting

HI258_write_cmos_sensor(0x30, 0x80);
HI258_write_cmos_sensor(0x31, 0x80);
HI258_write_cmos_sensor(0x38, 0x11);
HI258_write_cmos_sensor(0x39, 0x34);
HI258_write_cmos_sensor(0x40, 0xe4); //Stb Yth
HI258_write_cmos_sensor(0x41, 0x33); //Stb cdiff
HI258_write_cmos_sensor(0x42, 0x22); //Stb csum
HI258_write_cmos_sensor(0x43, 0xf3); //Unstb Yth
HI258_write_cmos_sensor(0x44, 0x55); //Unstb cdiff
HI258_write_cmos_sensor(0x45, 0x33); //Unstb csum
HI258_write_cmos_sensor(0x46, 0x00);
HI258_write_cmos_sensor(0x47, 0x09); //2013-09-13 AWB Hunting
HI258_write_cmos_sensor(0x48, 0x00); //2013-09-13 AWB Hunting
HI258_write_cmos_sensor(0x49, 0x0a);

HI258_write_cmos_sensor(0x60, 0x04);
HI258_write_cmos_sensor(0x61, 0xc4);
HI258_write_cmos_sensor(0x62, 0x04);
HI258_write_cmos_sensor(0x63, 0x92);
HI258_write_cmos_sensor(0x66, 0x04);
HI258_write_cmos_sensor(0x67, 0xc4);
HI258_write_cmos_sensor(0x68, 0x04);
HI258_write_cmos_sensor(0x69, 0x92);

HI258_write_cmos_sensor(0x80, 0x36);
HI258_write_cmos_sensor(0x81, 0x20);
HI258_write_cmos_sensor(0x82, 0x2a);

HI258_write_cmos_sensor(0x83, 0x52);
HI258_write_cmos_sensor(0x84, 0x10);
HI258_write_cmos_sensor(0x85, 0x5f);
HI258_write_cmos_sensor(0x86, 0x18);

HI258_write_cmos_sensor(0x87, 0x44);
HI258_write_cmos_sensor(0x88, 0x30);
HI258_write_cmos_sensor(0x89, 0x2c);
HI258_write_cmos_sensor(0x8a, 0x18);

HI258_write_cmos_sensor(0x8b, 0x3b);
HI258_write_cmos_sensor(0x8c, 0x32);
HI258_write_cmos_sensor(0x8d, 0x2a);
HI258_write_cmos_sensor(0x8e, 0x1b);

HI258_write_cmos_sensor(0x8f, 0x4d);
HI258_write_cmos_sensor(0x90, 0x4a);
HI258_write_cmos_sensor(0x91, 0x45);
HI258_write_cmos_sensor(0x92, 0x3f);
HI258_write_cmos_sensor(0x93, 0x38);
HI258_write_cmos_sensor(0x94, 0x31);
HI258_write_cmos_sensor(0x95, 0x27);
HI258_write_cmos_sensor(0x96, 0x1d);
HI258_write_cmos_sensor(0x97, 0x13);
HI258_write_cmos_sensor(0x98, 0x12);
HI258_write_cmos_sensor(0x99, 0x11);
HI258_write_cmos_sensor(0x9a, 0x10);

HI258_write_cmos_sensor(0x9b, 0xaa);
HI258_write_cmos_sensor(0x9c, 0xaa);
HI258_write_cmos_sensor(0x9d, 0x48);
HI258_write_cmos_sensor(0x9e, 0x38);
HI258_write_cmos_sensor(0x9f, 0x30);

HI258_write_cmos_sensor(0xa0, 0x70);
HI258_write_cmos_sensor(0xa1, 0x54);
HI258_write_cmos_sensor(0xa2, 0x6f);
HI258_write_cmos_sensor(0xa3, 0xff);

HI258_write_cmos_sensor(0xa4, 0x14); //1536fps
HI258_write_cmos_sensor(0xa5, 0x2c); //698fps
HI258_write_cmos_sensor(0xa6, 0xcf); //148fps

HI258_write_cmos_sensor(0xad, 0x2e);
HI258_write_cmos_sensor(0xae, 0x28);

HI258_write_cmos_sensor(0xaf, 0x18); //Low temp Rgain
HI258_write_cmos_sensor(0xb0, 0x16); //Low temp Rgain

HI258_write_cmos_sensor(0xb1, 0x08);
HI258_write_cmos_sensor(0xb4, 0xbf); //For Tracking AWB Weight
HI258_write_cmos_sensor(0xb8, 0x5d); //(0+,1-)High Cb , (0+,1-)Low Cr
HI258_write_cmos_sensor(0xb9, 0xb0);
/////// PAGE 22 END ///////

//// MIPI Setting /////
HI258_write_cmos_sensor(0x03, 0x48);
HI258_write_cmos_sensor(0x39, 0x4f); //lvds_bias_ctl    [2:0]mipi_tx_bias   [4:3]mipi_vlp_sel   [6:5]mipi_vcm_sel
HI258_write_cmos_sensor(0x10, 0x1c); //lvds_ctl_1       [5]mipi_pad_disable [4]lvds_en [0]serial_data_len 
HI258_write_cmos_sensor(0x11, 0x10); //lvds_ctl_2       [4]mipi continous mode setting
//HI258_write_cmos_sensor(0x14, 0x00} //ser_out_ctl_1  [2:0]serial_sout_a_phase   [6:4]serial_cout_a_phase

HI258_write_cmos_sensor(0x16, 0x00); //lvds_inout_ctl1  [0]vs_packet_pos_sel [1]data_neg_sel [4]first_vsync_end_opt
HI258_write_cmos_sensor(0x18, 0x80); //lvds_inout_ctl3
HI258_write_cmos_sensor(0x19, 0x00); //lvds_inout_ctl4
HI258_write_cmos_sensor(0x1a, 0xf0); //lvds_time_ctl
HI258_write_cmos_sensor(0x24, 0x1e); //long_packet_id

//====== MIPI Timing Setting =========
HI258_write_cmos_sensor(0x36, 0x01); //clk_tlpx_time_dp
HI258_write_cmos_sensor(0x37, 0x05); //clk_tlpx_time_dn
HI258_write_cmos_sensor(0x34, 0x04); //clk_prepare_time
HI258_write_cmos_sensor(0x32, 0x15); //clk_zero_time
HI258_write_cmos_sensor(0x35, 0x04); //clk_trail_time
HI258_write_cmos_sensor(0x33, 0x0d); //clk_post_time

HI258_write_cmos_sensor(0x1c, 0x01); //tlps_time_l_dp
HI258_write_cmos_sensor(0x1d, 0x0b); //tlps_time_l_dn
HI258_write_cmos_sensor(0x1e, 0x06); //hs_zero_time
HI258_write_cmos_sensor(0x1f, 0x09); //hs_trail_time

//long_packet word count 
HI258_write_cmos_sensor(0x30, 0x06);
HI258_write_cmos_sensor(0x31, 0x40); //long_packet word count

HI258_write_cmos_sensor(0x03, 0x20);
HI258_write_cmos_sensor(0x10, 0x9c);
HI258_write_cmos_sensor(0x03, 0x22);
HI258_write_cmos_sensor(0x10, 0xe9);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x0e, 0x03);
HI258_write_cmos_sensor(0x0e, 0x73);

HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x03, 0x00);
HI258_write_cmos_sensor(0x01, 0x00);
}
kal_uint32 HI258_read_shutter(void)
{
    kal_uint8 temp_reg0, temp_reg1, temp_reg2;
    kal_uint32 shutter;

    HI258_write_cmos_sensor(0x03, 0x20); 
    temp_reg0 = HI258_read_cmos_sensor(0x80); 
    temp_reg1 = HI258_read_cmos_sensor(0x81);
    temp_reg2 = HI258_read_cmos_sensor(0x82); 
    shutter = (temp_reg0 << 16) | (temp_reg1 << 8) | (temp_reg2 & 0xFF);

    return shutter;
}   

static void HI258_write_shutter(kal_uint16 shutter)
{
    SENSORDB("[HI258] %s \n",__func__);
    HI258_write_cmos_sensor(0x03, 0x20);
    HI258_write_cmos_sensor(0x83, shutter >> 16);			
    HI258_write_cmos_sensor(0x84, (shutter >> 8) & 0x00FF);	
    HI258_write_cmos_sensor(0x85, shutter & 0x0000FF);		
}    

void HI258_night_mode(kal_bool enable)	
{
    SENSORDB("[HI258] %s ==enable =%d \n",__func__,enable);
    if (HI258_sensor_cap_state == KAL_TRUE) 
    {
        return ;	
    }
    if (enable) 
    {

        if (HI258_Banding_setting == AE_FLICKER_MODE_50HZ) 
        {
            HI258_write_cmos_sensor(0x03,0x00); 	
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x01;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x1c);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x18, 0x38);

            HI258_write_cmos_sensor(0x83, 0x13); //EXP Normal 33.33 fps 
            HI258_write_cmos_sensor(0x84, 0xd6); 
            HI258_write_cmos_sensor(0x85, 0x20); 
            HI258_write_cmos_sensor(0x86, 0x01); //EXPMin 6500.00 fps
            HI258_write_cmos_sensor(0x87, 0xf4); 
		          //EXP Max 60hz
            HI258_write_cmos_sensor(0x88, 0x13); //EXP Max 5 fps
            HI258_write_cmos_sensor(0x89, 0xc6);
            HI258_write_cmos_sensor(0x8a, 0x80);

			        //EXP Max 50hz
            HI258_write_cmos_sensor(0xa5, 0x13); //EXP Max 5 fps
            HI258_write_cmos_sensor(0xa6, 0xd6);
            HI258_write_cmos_sensor(0xa7, 0x20);

            HI258_write_cmos_sensor(0x03, 0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x00;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x9c);

            HI258_write_cmos_sensor(0x18, 0x30);
            msleep(10);
        } 
        else
        {
            HI258_write_cmos_sensor(0x03,0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x01;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x1c);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x18, 0x38);

            HI258_write_cmos_sensor(0x83, 0x13); //EXP Normal 33.33 fps 
            HI258_write_cmos_sensor(0x84, 0xc6); 
            HI258_write_cmos_sensor(0x85, 0x80); 
            HI258_write_cmos_sensor(0x86, 0x01); //EXPMin 6500.00 fps
            HI258_write_cmos_sensor(0x87, 0xf4);
			
		          //EXP Max 60hz
            HI258_write_cmos_sensor(0x88, 0x13); //EXP Max 5 fps
            HI258_write_cmos_sensor(0x89, 0xc6);
            HI258_write_cmos_sensor(0x8a, 0x80);

			        //EXP Max 50hz
            HI258_write_cmos_sensor(0xa5, 0x13); //EXP Max 5 fps
            HI258_write_cmos_sensor(0xa6, 0xd6);
            HI258_write_cmos_sensor(0xa7, 0x20);

            HI258_write_cmos_sensor(0x03, 0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x00;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x8c);

            HI258_write_cmos_sensor(0x18, 0x30);
            msleep(10);
        }
    } 
    else 
    {

        if (HI258_Banding_setting == AE_FLICKER_MODE_50HZ)
        {
            HI258_write_cmos_sensor(0x03,0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x01;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);
            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x1c);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x18, 0x38);

            HI258_write_cmos_sensor(0x83, 0x0b); //EXP Normal 33.33 fps
            HI258_write_cmos_sensor(0x84, 0xe6);
            HI258_write_cmos_sensor(0x85, 0xe0);
            HI258_write_cmos_sensor(0x86, 0x01); //EXPMin 6500.00 fps
            HI258_write_cmos_sensor(0x87, 0xf4);
		          //EXP Max 60hz
            HI258_write_cmos_sensor(0x88, 0x0c); //EXP Max 8.33 fps
            HI258_write_cmos_sensor(0x89, 0x5c);
            HI258_write_cmos_sensor(0x8a, 0x10);

			        //EXP Max 50hz
            HI258_write_cmos_sensor(0xa5, 0x0b); //EXP Max 8.33 fps
            HI258_write_cmos_sensor(0xa6, 0xe6);
            HI258_write_cmos_sensor(0xa7, 0xe0);
			

            HI258_write_cmos_sensor(0x03, 0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x00;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x9c);

            HI258_write_cmos_sensor(0x18, 0x30);
            msleep(10);
        }
        else
        {
            HI258_write_cmos_sensor(0x03,0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x01;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);
            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x1c);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x18, 0x38);

            HI258_write_cmos_sensor(0x83, 0x0c); //EXP Normal 33.33 fps
            HI258_write_cmos_sensor(0x84, 0x5c);
            HI258_write_cmos_sensor(0x85, 0x10);
            HI258_write_cmos_sensor(0x86, 0x01); //EXPMin 6500.00 fps
            HI258_write_cmos_sensor(0x87, 0xf4);
			           //EXP Max 60hz
            HI258_write_cmos_sensor(0x88, 0x0c); //EXP Max 8.33 fps
            HI258_write_cmos_sensor(0x89, 0x5c);
            HI258_write_cmos_sensor(0x8a, 0x10);
			           //EXP Max 50hz
            HI258_write_cmos_sensor(0xa5, 0x0b); //EXP Max 8.33 fps
            HI258_write_cmos_sensor(0xa6, 0xe6);
            HI258_write_cmos_sensor(0xa7, 0xe0);

            HI258_write_cmos_sensor(0x03, 0x00);
            HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
            HI258_Sleep_Mode |= 0x00;
            HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

            HI258_write_cmos_sensor(0x03, 0x20);
            HI258_write_cmos_sensor(0x10, 0x8c);

            HI258_write_cmos_sensor(0x18, 0x30);
            msleep(10);
        }
    }
}
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
int g_vendor_flag = 0;
void HI258_Initial_Cmds(void)
{
    kal_uint16 i,cnt;
	
	  int Channel=0;
      int data[4];
      int adcVol=0;
        
      int res=IMM_GetOneChannelValue(Channel,data,0);
      adcVol=data[0]*1000+data[1]*10;
	  printk("[HI258YUV]:Read ADC succ:%d\n", adcVol); 
	  if(adcVol<150)
	  {
	  	HI258_Init_Cmds_CMK();
		g_vendor_flag = 1;
	  }
	  else
	  {
	  	HI258_Init_Cmds();
		g_vendor_flag = 2;
	  }
	  
}

UINT32 HI258Open(void)
{
    SENSORDB("[HI258] %s \n",__func__);
    volatile signed char i;
    kal_uint32 sensor_id=0;
    kal_uint8 temp_sccb_addr = 0;

    HI258_write_cmos_sensor(0x03, 0x00);
    HI258_write_cmos_sensor(0x01, 0xf1);
    HI258_write_cmos_sensor(0x01, 0xf3);
    HI258_write_cmos_sensor(0x01, 0xf1);

    HI258_write_cmos_sensor(0x01, 0xf1);
    HI258_write_cmos_sensor(0x01, 0xf3);
    HI258_write_cmos_sensor(0x01, 0xf1);
    
    do{
        for (i=0; i < 3; i++)
        {
            sensor_id = HI258_read_cmos_sensor(0x04);
printk("[HI258YUV]:Read Sensor ID succ:0x%x\n", sensor_id);  
            if (sensor_id == HI258_SENSOR_ID)
            {
#ifdef HI258_DEBUG
                printk("[HI258YUV]:Read Sensor ID succ:0x%x\n", sensor_id);  
#endif
                break;
            }
        }

        mdelay(20);
    }while(0);

    if (sensor_id != HI258_SENSOR_ID)
    {
	
#ifdef HI258_DEBUG
        printk("[HI258YUV]:Read Sensor ID fail:0x%x\n", sensor_id);  
#endif
        return ERROR_SENSOR_CONNECT_FAIL;
    }
#ifdef HI258_DEBUG
    printk("[HI258YUV]:Read Sensor ID pass:0x%x\n", sensor_id);
#endif

#ifdef HI258_LOAD_FROM_T_FLASH 

   struct file *fp; 
   mm_segment_t fs; 
   loff_t pos = 0; 
   //static char buf[10*1024] ;
   
   printk("HI258 Open File Start\n");
   fp = filp_open("/storage/sdcard0/hi258_sd.dat", O_RDONLY , 0); 
   if (IS_ERR(fp)) 
   { 
	   fromsd = 0;	 
	   printk("open file error\n");
	   ////////return 0;
   } 
   else 
   {
	   printk("open file success\n");
	   fromsd = 1;
	   //SP2528_Initialize_from_T_Flash();
	   filp_close(fp, NULL); 
   }
   //set_fs(fs);
   if(fromsd == 1)//¡§o??¡è???¡§?SD?¡§¡é¡§¡§?//gepeiwei   120903
	   Hi258_Initialize_from_T_Flash();//??¡§?SD??¡ì?¡§¡é¡§¡§?|¨¬????¡§¡ãao?£¤¡§oy
   else
   {
	  HI258_Init_Cmds(); 
   }
   //HI256_Init_Parameter(); 
#else

    HI258_Initial_Cmds();
#endif
    HI258_set_mirror_flip(3);

    return ERROR_NONE;
}	

UINT32 HI258Close(void)
{
    //CISModulePowerOn(FALSE);
    //kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM) g_currDualSensorIdx, g_currSensorName,false, CAMERA_HW_DRVNAME);
    return ERROR_NONE;
}	

static void HI258_set_mirror_flip(kal_uint8 image_mirror)
{
    kal_uint8 HI258_HV_Mirror;
    SENSORDB("[HI258] %s \n",__func__);
    HI258_write_cmos_sensor(0x03,0x00); 	
    HI258_HV_Mirror = (HI258_read_cmos_sensor(0x11) & 0xfc);

    switch (image_mirror) {
        case IMAGE_NORMAL:		
            HI258_HV_Mirror |= 0x03;
            break;
        case IMAGE_H_MIRROR:
            HI258_HV_Mirror |= 0x01;
            break;
        case IMAGE_V_MIRROR:
            HI258_HV_Mirror |= 0x02; 
            break;
        case IMAGE_HV_MIRROR:
            HI258_HV_Mirror |= 0x00; 
            break;
        default:
            break;
    }
    HI258_write_cmos_sensor(0x11, HI258_HV_Mirror);
}

UINT32 HI258Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{	
    SENSORDB("[HI258] %s \n",__func__);
    kal_uint16  iStartX = 0, iStartY = 0;

    HI258_sensor_cap_state = KAL_FALSE;
    HI258_sensor_pclk=390;

    HI258_gPVmode = KAL_TRUE;

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        HI258_VEDIO_encode_mode = KAL_TRUE;
    }
    else
    {
        HI258_VEDIO_encode_mode = KAL_FALSE;
    }

    HI258_write_cmos_sensor(0x03,0x00); 	
    HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
    HI258_Sleep_Mode |= 0x01;
    HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

    HI258_write_cmos_sensor(0x03, 0x20); 
	HI258_write_cmos_sensor(0x10, (HI258_read_cmos_sensor(0x10) & 0x7f));

    HI258_write_cmos_sensor(0x03, 0x22);
    HI258_write_cmos_sensor(0x10, 0x69);

    HI258_write_cmos_sensor(0x03, 0x00);
    HI258_write_cmos_sensor(0x10, 0x10); // preview mode off
    HI258_write_cmos_sensor(0x12, 0x04);

    HI258_write_cmos_sensor(0x20, 0x00); 
    HI258_write_cmos_sensor(0x21, 0x04);
    HI258_write_cmos_sensor(0x22, 0x00);
    HI258_write_cmos_sensor(0x23, 0x07);

    HI258_write_cmos_sensor(0x40, 0x01); //Hblank 378
    HI258_write_cmos_sensor(0x41, 0x78); 
    HI258_write_cmos_sensor(0x42, 0x00); //Vblank 20
    HI258_write_cmos_sensor(0x43, 0x14); 

    HI258_write_cmos_sensor(0x03, 0x20); 
    //HI258_write_cmos_sensor(0x18, 0x38);

    HI258_write_cmos_sensor(0x86, 0x01); //EXPMin 6500.00 fps
    HI258_write_cmos_sensor(0x87, 0xf4); 

    HI258_write_cmos_sensor(0x8B, 0xfd); //EXP100 
    HI258_write_cmos_sensor(0x8C, 0xe8); 
    HI258_write_cmos_sensor(0x8D, 0xd2); //EXP120 
    HI258_write_cmos_sensor(0x8E, 0xf0);

    HI258_write_cmos_sensor(0x9c, 0x21); //EXP Limit 1083.33fps 
    HI258_write_cmos_sensor(0x9d, 0x34); 
    HI258_write_cmos_sensor(0x9e, 0x01); //EXP Unit 
    HI258_write_cmos_sensor(0x9f, 0xf4); 

	//PLL Setting
	HI258_write_cmos_sensor(0x03, 0x00); 
	HI258_write_cmos_sensor(0xd0, 0x05); //PLL pre_div 1/6 = 4 Mhz
	HI258_write_cmos_sensor(0xd1, 0x30); //PLL maim_div 
	HI258_write_cmos_sensor(0xd2, 0x05); //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]	  
	HI258_write_cmos_sensor(0xd3, 0x20); //isp_clk_inv[0]  mipi_4x_inv[1]  mipi_1x_inv[2]
	HI258_write_cmos_sensor(0xd0, 0x85);
	HI258_write_cmos_sensor(0xd0, 0x85);
	HI258_write_cmos_sensor(0xd0, 0x85);
	HI258_write_cmos_sensor(0xd0, 0x95);
	
	HI258_write_cmos_sensor(0x03, 0x00); //Dummy 750us
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);
	HI258_write_cmos_sensor(0x03, 0x00);


    // MIPI TX Setting //
    HI258_write_cmos_sensor(0x03, 0x48);
    HI258_write_cmos_sensor(0x30, 0x06);
    HI258_write_cmos_sensor(0x31, 0x40);



	
    HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
    HI258_Sleep_Mode |= 0x00;
    HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

    HI258_write_cmos_sensor(0x03, 0x20);//page 20
    HI258_write_cmos_sensor(0x10, (HI258_read_cmos_sensor(0x10) | 0x80));

    HI258_write_cmos_sensor(0x03, 0x22);
    HI258_write_cmos_sensor(0x10, 0xe9);//AWB ON

    //HI258_write_cmos_sensor(0x03, 0x20);
    //HI258_write_cmos_sensor(0x18, 0x30);

    image_window->GrabStartX = iStartX;
    image_window->GrabStartY = iStartY;
    image_window->ExposureWindowWidth = HI258_IMAGE_SENSOR_PV_WIDTH - 16;
    image_window->ExposureWindowHeight = HI258_IMAGE_SENSOR_PV_HEIGHT - 12;
    msleep(10);
    // copy sensor_config_data
    memcpy(&HI258SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}	
UINT32 HI258ZSD(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint8 temp_AE_reg;
    kal_uint8 CLK_DIV_REG = 0;
    SENSORDB("[HI258] %s \n",__func__);
    
        /* ZSD Mode */

    HI258_write_cmos_sensor(0x03, 0x00); 	
    HI258_write_cmos_sensor(0x01, 0x01);


	
    HI258_write_cmos_sensor(0x03, 0x00);
    HI258_write_cmos_sensor(0x10, 0x00); 

    HI258_write_cmos_sensor(0x20, 0x00); 
    HI258_write_cmos_sensor(0x21, 0x0a); 
    HI258_write_cmos_sensor(0x22, 0x00); 
    HI258_write_cmos_sensor(0x23, 0x0a); 

    HI258_write_cmos_sensor(0x40, 0x01); //Hblank 360
    HI258_write_cmos_sensor(0x41, 0x78); 
    HI258_write_cmos_sensor(0x42, 0x00); //Vblank 18
    HI258_write_cmos_sensor(0x43, 0x14); 

	HI258_write_cmos_sensor(0x03, 0x12);
	HI258_write_cmos_sensor(0x9c, 0x0a);

    // 1600*1200	
    HI258_write_cmos_sensor(0x03, 0x00);
    HI258_write_cmos_sensor(0x10, 0x00);
    HI258_write_cmos_sensor(0xd2, 0x01); //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    

    HI258_write_cmos_sensor(0x03, 0x48);
    HI258_write_cmos_sensor(0x30, 0x0c);
    HI258_write_cmos_sensor(0x31, 0x80);





    HI258_write_cmos_sensor(0x03, 0x00); 	
    HI258_write_cmos_sensor(0x01, 0x00);      



    msleep(10);
    memcpy(&HI258SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}

UINT32 HI258Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint8 temp_AE_reg;
    kal_uint8 CLK_DIV_REG = 0;
    SENSORDB("[HI258] %s \n",__func__);
    HI258_sensor_cap_state = KAL_TRUE;
#if 0
    if ((image_window->ImageTargetWidth<=HI258_IMAGE_SENSOR_PV_WIDTH)&&
        (image_window->ImageTargetHeight<=HI258_IMAGE_SENSOR_PV_HEIGHT))
    {
        /* Less than PV Mode */
        HI258_gPVmode=KAL_TRUE;
        HI258_capture_pclk_in_M = HI258_preview_pclk_in_M;   //Don't change the clk

        HI258_write_cmos_sensor(0x03, 0x00);

        HI258_write_cmos_sensor(0x20, 0x00); 
        HI258_write_cmos_sensor(0x21, 0x04); 
        HI258_write_cmos_sensor(0x22, 0x00); 
        HI258_write_cmos_sensor(0x23, 0x07); 

        HI258_write_cmos_sensor(0x40, 0x01); //Hblank 360
        HI258_write_cmos_sensor(0x41, 0x68); 
        HI258_write_cmos_sensor(0x42, 0x00); //Vblank 18
        HI258_write_cmos_sensor(0x43, 0x12); 

        HI258_write_cmos_sensor(0x03, 0x10);
        HI258_write_cmos_sensor(0x3f, 0x00);	

        //Page12
        HI258_write_cmos_sensor(0x03, 0x12); //Function
        HI258_write_cmos_sensor(0x20, 0x0f);
        HI258_write_cmos_sensor(0x21, 0x0f);
        HI258_write_cmos_sensor(0x90, 0x5d);  

        //Page13
        HI258_write_cmos_sensor(0x03, 0x13); //Function
        HI258_write_cmos_sensor(0x80, 0xfd); //Function

        // 800*600	
        HI258_write_cmos_sensor(0x03, 0x00);
        HI258_write_cmos_sensor(0x10, 0x11);

        HI258_write_cmos_sensor(0x03, 0x48);
        HI258_write_cmos_sensor(0x72, 0x81); 
        HI258_write_cmos_sensor(0x30, 0x06);
        HI258_write_cmos_sensor(0x31, 0x40);

        HI258_write_cmos_sensor(0x03, 0x20);
        HI258_pv_HI258_exposure_lines = (HI258_read_cmos_sensor(0x80) << 16)|(HI258_read_cmos_sensor(0x81) << 8)|HI258_read_cmos_sensor(0x82);

        HI258_cp_HI258_exposure_lines=HI258_pv_HI258_exposure_lines;	

        if(HI258_cp_HI258_exposure_lines<1)
            HI258_cp_HI258_exposure_lines=1;

        HI258_write_cmos_sensor(0x03, 0x20); 
        HI258_write_cmos_sensor(0x83, HI258_cp_HI258_exposure_lines >> 16);
        HI258_write_cmos_sensor(0x84, (HI258_cp_HI258_exposure_lines >> 8) & 0x000000FF);
        HI258_write_cmos_sensor(0x85, HI258_cp_HI258_exposure_lines & 0x000000FF);

        image_window->GrabStartX = 1;
        image_window->GrabStartY = 1;
        image_window->ExposureWindowWidth= HI258_IMAGE_SENSOR_PV_WIDTH - 16;
        image_window->ExposureWindowHeight = HI258_IMAGE_SENSOR_PV_HEIGHT - 12;
    }
    else 
#endif
    {    
        /* 2M FULL Mode */
        HI258_gPVmode = KAL_FALSE;

        HI258_write_cmos_sensor(0x03,0x00); 	
        HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
        HI258_Sleep_Mode |= 0x01;
        HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);

        CLK_DIV_REG=(HI258_read_cmos_sensor(0x12)&0xFc);    // don't divide,PCLK=48M
        CLK_DIV_REG |= 0x00;
        //read the shutter (manual exptime)
        HI258_write_cmos_sensor(0x03, 0x20);
        HI258_pv_HI258_exposure_lines = (HI258_read_cmos_sensor(0x80) << 16)|(HI258_read_cmos_sensor(0x81) << 8)|HI258_read_cmos_sensor(0x82);

        HI258_cp_HI258_exposure_lines = HI258_pv_HI258_exposure_lines;
		
        HI258_write_cmos_sensor(0x03, 0x00);

        HI258_write_cmos_sensor(0x20, 0x00); 
        HI258_write_cmos_sensor(0x21, 0x0a); 
        HI258_write_cmos_sensor(0x22, 0x00); 
        HI258_write_cmos_sensor(0x23, 0x0a); 

        HI258_write_cmos_sensor(0x40, 0x01); //Hblank 360
        HI258_write_cmos_sensor(0x41, 0x78); 
        HI258_write_cmos_sensor(0x42, 0x00); //Vblank 18
        HI258_write_cmos_sensor(0x43, 0x14); 

	
        // 1600*1200	
        HI258_write_cmos_sensor(0x03, 0x00);
        HI258_write_cmos_sensor(0x10, 0x00);
        HI258_write_cmos_sensor(0xd2, 0x01); //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    

        HI258_write_cmos_sensor(0x03, 0x48);
        HI258_write_cmos_sensor(0x72, 0x81); 
        HI258_write_cmos_sensor(0x30, 0x0c);
        HI258_write_cmos_sensor(0x31, 0x80);
#if 0
        if ((image_window->ImageTargetWidth<=HI258_IMAGE_SENSOR_FULL_WIDTH)&&
        (image_window->ImageTargetHeight<=HI258_IMAGE_SENSOR_FULL_HEIGHT))
        {
            HI258_capture_pclk_in_M = 520;
            HI258_sensor_pclk = 520;                 
        }
        else//Interpolate to 3M
        {
            HI258_capture_pclk_in_M = 520;
            HI258_sensor_pclk = 520;                
        }
#endif

        HI258_write_cmos_sensor(0x03, 0x00); 
        HI258_write_cmos_sensor(0x12,/*CLK_DIV_REG*/ 0x04);

        if(HI258_cp_HI258_exposure_lines<1)
            HI258_cp_HI258_exposure_lines=1;

        HI258_write_cmos_sensor(0x03, 0x20); 
        HI258_write_cmos_sensor(0x83, HI258_cp_HI258_exposure_lines >> 16);
        HI258_write_cmos_sensor(0x84, (HI258_cp_HI258_exposure_lines >> 8) & 0x000000FF);
        HI258_write_cmos_sensor(0x85, HI258_cp_HI258_exposure_lines & 0x000000FF);
	
        HI258_write_cmos_sensor(0x03,0x00); 	
        HI258_Sleep_Mode = (HI258_read_cmos_sensor(0x01) & 0xfe);
        HI258_Sleep_Mode |= 0x00;
        HI258_write_cmos_sensor(0x01, HI258_Sleep_Mode);       
#if 0
        image_window->GrabStartX=1;
        image_window->GrabStartY=1;
        image_window->ExposureWindowWidth=HI258_IMAGE_SENSOR_FULL_WIDTH - 16;
        image_window->ExposureWindowHeight=HI258_IMAGE_SENSOR_FULL_HEIGHT - 12;
#endif
    }

    msleep(10);
    // copy sensor_config_data
    memcpy(&HI258SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    HI258_CAPATURE_FLAG = 1;
    HI258_CAPATUREB_FLAG = 1;
    return ERROR_NONE;
}

UINT32 HI258GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[HI258] %s \n",__func__);
    pSensorResolution->SensorFullWidth=HI258_FULL_GRAB_WIDTH;  
    pSensorResolution->SensorFullHeight=HI258_FULL_GRAB_HEIGHT;
    pSensorResolution->SensorPreviewWidth=HI258_PV_GRAB_WIDTH;
    pSensorResolution->SensorPreviewHeight=HI258_PV_GRAB_HEIGHT;
    pSensorResolution->SensorVideoWidth=HI258_PV_GRAB_WIDTH;
    pSensorResolution->SensorVideoHeight=HI258_PV_GRAB_HEIGHT;
    return ERROR_NONE;
}	

UINT32 HI258GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[HI258] %s \n",__func__);
    pSensorInfo->SensorPreviewResolutionX=HI258_PV_GRAB_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=HI258_PV_GRAB_HEIGHT;
    pSensorInfo->SensorFullResolutionX=HI258_FULL_GRAB_WIDTH;
    pSensorInfo->SensorFullResolutionY=HI258_FULL_GRAB_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;

    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;

#ifdef MIPI_INTERFACE
    pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;
#else
    pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_PARALLEL;
#endif

    pSensorInfo->CaptureDelayFrame = 1; 
    pSensorInfo->PreviewDelayFrame = 1; 
    pSensorInfo->VideoDelayFrame = 1; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   	
	
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            /*ergate-004*/
            pSensorInfo->SensorGrabStartX = HI258_PV_GRAB_START_X;//0; 
            pSensorInfo->SensorGrabStartY = HI258_PV_GRAB_START_Y;//0;    		
#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;// 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
#endif
            break;

        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            /*ergate-004*/
            pSensorInfo->SensorGrabStartX = HI258_FULL_GRAB_START_X;//0; 
            pSensorInfo->SensorGrabStartY = HI258_FULL_GRAB_START_Y;//0;     		
#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;//14 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
#endif
            break;

        default:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=3;
            pSensorInfo->SensorClockRisingCount=0;
            pSensorInfo->SensorClockFallingCount=2;
            pSensorInfo->SensorPixelClockCount=3;
            pSensorInfo->SensorDataLatchCount=2;
            /*ergate-004*/
            pSensorInfo->SensorGrabStartX = HI258_PV_GRAB_START_X;//0; 
            pSensorInfo->SensorGrabStartY = HI258_PV_GRAB_START_Y;//0;     			
            break;
    }
    //HI258_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &HI258SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}	

UINT32 HI258Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[HI258] %s ==ScenarioID=%d \n",__func__,ScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			//HI258ZSD(pImageWindow, pSensorConfigData);

		//break;
		
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            HI258Preview(pImageWindow, pSensorConfigData);
            break;
	
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HI258Capture(pImageWindow, pSensorConfigData);
            break;
	
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }

    return TRUE;
}	


void HI258set_iso(UINT16 para)
{
	
	
	
	SENSORDB("[HI257]CONTROLFLOW HI257set_iso Para:%d;\n",para);

		switch (para)
		{
			case AE_ISO_100:
	//ISO100
	HI258_write_cmos_sensor(0x03, 0x20); 
	HI258_write_cmos_sensor(0xb2, 0x40); 
		


				 break;
			case AE_ISO_200:
	//ISO200
	HI258_write_cmos_sensor(0x03, 0x20); 
	HI258_write_cmos_sensor(0xb2, 0x58); 
				 break;
			case AE_ISO_400:
	//ISO400
	HI258_write_cmos_sensor(0x03, 0x20); 
	HI258_write_cmos_sensor(0xb2, 0xb0); 


				 break;
			default:
			case AE_ISO_AUTO://
	//Auto
	HI258_write_cmos_sensor(0x03, 0x20); 
	HI258_write_cmos_sensor(0xb2, 0xb0); 


		   /*	   */
				 break;
		}
		return;

}

static void HI258_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
    unsigned int NormBr;

	HI258_write_cmos_sensor(0x03, 0x20);

    NormBr = HI258_read_cmos_sensor(0xb0);
    if (NormBr < FLASH_BV_THRESHOLD)
    {
       *pFeatureReturnPara32 = FALSE;
        return;
    }
    *pFeatureReturnPara32 = TRUE;
    return;
}



BOOL HI258_set_param_wb(UINT16 para)
{
    SENSORDB("[HI258] %s ====para=%d \n",__func__,para);
    switch (para)
    {
        case AWB_MODE_AUTO:
            HI258_write_cmos_sensor(0x03, 0x22);			
            HI258_write_cmos_sensor(0x11, 0x2e);		
            HI258_write_cmos_sensor(0x83, 0x52);
            HI258_write_cmos_sensor(0x84, 0x10);
            HI258_write_cmos_sensor(0x85, 0x5f);
            HI258_write_cmos_sensor(0x86, 0x18);
            break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
            HI258_write_cmos_sensor(0x03, 0x22);
            HI258_write_cmos_sensor(0x11, 0x28);
            HI258_write_cmos_sensor(0x80, 0x51);
            HI258_write_cmos_sensor(0x82, 0x2b);
            HI258_write_cmos_sensor(0x83, 0x52);
            HI258_write_cmos_sensor(0x84, 0x50);
            HI258_write_cmos_sensor(0x85, 0x2b);
            HI258_write_cmos_sensor(0x86, 0x28);
            break;
        case AWB_MODE_DAYLIGHT: //sunny
            HI258_write_cmos_sensor(0x03, 0x22);
            HI258_write_cmos_sensor(0x11, 0x28);		  
            HI258_write_cmos_sensor(0x80, 0x59);
            HI258_write_cmos_sensor(0x82, 0x29);
            HI258_write_cmos_sensor(0x83, 0x60);
            HI258_write_cmos_sensor(0x84, 0x50);
            HI258_write_cmos_sensor(0x85, 0x2f);
            HI258_write_cmos_sensor(0x86, 0x23);
            break;
        case AWB_MODE_INCANDESCENT: //office
            HI258_write_cmos_sensor(0x03, 0x22);
            HI258_write_cmos_sensor(0x11, 0x28);		  
            HI258_write_cmos_sensor(0x80, 0x29);
            HI258_write_cmos_sensor(0x82, 0x54);
            HI258_write_cmos_sensor(0x83, 0x2e);
            HI258_write_cmos_sensor(0x84, 0x23);
            HI258_write_cmos_sensor(0x85, 0x58);
            HI258_write_cmos_sensor(0x86, 0x4f);
            break;
        case AWB_MODE_TUNGSTEN: //home
            HI258_write_cmos_sensor(0x03, 0x22);
            HI258_write_cmos_sensor(0x80, 0x24);
            HI258_write_cmos_sensor(0x81, 0x20);
            HI258_write_cmos_sensor(0x82, 0x58);
            HI258_write_cmos_sensor(0x83, 0x27);
            HI258_write_cmos_sensor(0x84, 0x22);
            HI258_write_cmos_sensor(0x85, 0x58);
            HI258_write_cmos_sensor(0x86, 0x52);
            break;
        case AWB_MODE_FLUORESCENT:
            HI258_write_cmos_sensor(0x03, 0x22);
            HI258_write_cmos_sensor(0x11, 0x28);
            HI258_write_cmos_sensor(0x80, 0x41);
            HI258_write_cmos_sensor(0x82, 0x42);
            HI258_write_cmos_sensor(0x83, 0x44);
            HI258_write_cmos_sensor(0x84, 0x34);
            HI258_write_cmos_sensor(0x85, 0x46);
            HI258_write_cmos_sensor(0x86, 0x3a);
            break;	
        default:
            return FALSE;
    }

    return TRUE;
}

BOOL HI258_set_param_effect(UINT16 para)
{
    SENSORDB("[HI258] %s ====para=%d \n",__func__,para);
    kal_uint32 ret = KAL_TRUE;
    switch (para)
    {
        case MEFFECT_OFF:
            HI258_write_cmos_sensor(0x03, 0x10);
            HI258_write_cmos_sensor(0x11, 0x03);
            HI258_write_cmos_sensor(0x12, 0x30);
            HI258_write_cmos_sensor(0x13, 0x03);
            HI258_write_cmos_sensor(0x44, 0x80);
            HI258_write_cmos_sensor(0x45, 0x80);
            HI258_write_cmos_sensor(0x4a, 0x80);
            break;

        case MEFFECT_SEPIA:
            HI258_write_cmos_sensor(0x03, 0x10);
            HI258_write_cmos_sensor(0x11, 0x03);
            HI258_write_cmos_sensor(0x12, 0x33);
            HI258_write_cmos_sensor(0x13, 0x02);
            HI258_write_cmos_sensor(0x44, 0x70);
            HI258_write_cmos_sensor(0x45, 0x98);
            HI258_write_cmos_sensor(0x4a, 0x80);
            break;  
	
        case MEFFECT_NEGATIVE:		
            HI258_write_cmos_sensor(0x03, 0x10);
            HI258_write_cmos_sensor(0x11, 0x03);
            HI258_write_cmos_sensor(0x12, 0x38);
            HI258_write_cmos_sensor(0x13, 0x02);
            HI258_write_cmos_sensor(0x14, 0x00);
            HI258_write_cmos_sensor(0x4a, 0x80);
            break; 
	
        case MEFFECT_SEPIAGREEN:		
            HI258_write_cmos_sensor(0x03, 0x10);
            HI258_write_cmos_sensor(0x11, 0x03);
            HI258_write_cmos_sensor(0x12, 0x33);
            HI258_write_cmos_sensor(0x13, 0x02);
            HI258_write_cmos_sensor(0x44, 0x30);
            HI258_write_cmos_sensor(0x45, 0x50);
            HI258_write_cmos_sensor(0x4a, 0x80);
            break;
	
        case MEFFECT_SEPIABLUE:	
            HI258_write_cmos_sensor(0x03, 0x10);
            HI258_write_cmos_sensor(0x11, 0x03);
            HI258_write_cmos_sensor(0x12, 0x33);
            HI258_write_cmos_sensor(0x13, 0x02);
            HI258_write_cmos_sensor(0x44, 0xb0);
            HI258_write_cmos_sensor(0x45, 0x40);
            HI258_write_cmos_sensor(0x4a, 0x80);
            break;
	
        case MEFFECT_MONO:				
            HI258_write_cmos_sensor(0x03, 0x10);
            HI258_write_cmos_sensor(0x11, 0x03);
            HI258_write_cmos_sensor(0x12, 0x33);
            HI258_write_cmos_sensor(0x13, 0x02);
            HI258_write_cmos_sensor(0x44, 0x80);
            HI258_write_cmos_sensor(0x45, 0x80);
            HI258_write_cmos_sensor(0x4a, 0x80);
            break;

        default:
            ret = FALSE;
    }

    return ret;
}

BOOL HI258_set_param_banding(UINT16 para)
{
    kal_uint8 banding;
    SENSORDB("[HI258] %s \n",__func__);
    banding = HI258_read_cmos_sensor(0x3014);
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
            HI258_Banding_setting = AE_FLICKER_MODE_50HZ;
            HI258_write_cmos_sensor(0x03,0x20);
    HI258_write_cmos_sensor(0x10,0x1c);
    HI258_write_cmos_sensor(0x18,0x38);

    HI258_write_cmos_sensor(0x83, 0x0b);//EXP Normal 8.00 fps
    HI258_write_cmos_sensor(0x84, 0xe6);
    HI258_write_cmos_sensor(0x85, 0xe0);

    HI258_write_cmos_sensor(0x18, 0x30);
    HI258_write_cmos_sensor(0x10, 0x9c);
    break;
    case AE_FLICKER_MODE_60HZ:
    HI258_Banding_setting = AE_FLICKER_MODE_60HZ;
    HI258_write_cmos_sensor(0x03, 0x20);
    HI258_write_cmos_sensor(0x10, 0x0c);
    HI258_write_cmos_sensor(0x18, 0x38);

    HI258_write_cmos_sensor(0x83, 0x0c); //EXP Normal 8.00 fps
    HI258_write_cmos_sensor(0x84, 0x5c);
    HI258_write_cmos_sensor(0x85, 0x10);

            HI258_write_cmos_sensor(0x18,0x30);
            HI258_write_cmos_sensor(0x10,0x8c);
            break;
        default:
            return FALSE;
    }

    return TRUE;
} 

BOOL HI258_set_param_exposure(UINT16 para)
{
    SENSORDB("[HI258] %s %d\n",__func__, para);
    HI258_write_cmos_sensor(0x03,0x10);
    HI258_write_cmos_sensor(0x12,(HI258_read_cmos_sensor(0x12)|0x10));//make sure the Yoffset control is opened.

    switch (para)
    {
        case AE_EV_COMP_n20:
            HI258_write_cmos_sensor(0x40,0xc0);
            break;
        case AE_EV_COMP_n10:
            HI258_write_cmos_sensor(0x40,0xb0);
            break;
        case AE_EV_COMP_n07:
            HI258_write_cmos_sensor(0x40,0xa0);
            break;
        case AE_EV_COMP_n03:
            HI258_write_cmos_sensor(0x40,0x98);
            break;
        case AE_EV_COMP_00:
            HI258_write_cmos_sensor(0x40,0x85);
            break;
        case AE_EV_COMP_03:
            HI258_write_cmos_sensor(0x40,0x10);
            break;
        case AE_EV_COMP_07:
            HI258_write_cmos_sensor(0x40,0x20);
            break;
        case AE_EV_COMP_10:
            HI258_write_cmos_sensor(0x40,0x30);
            break;
        case AE_EV_COMP_20:
            HI258_write_cmos_sensor(0x40,0x40);
            break;
        default:
            return FALSE;
    }

    return TRUE;
} 
void HI258set_brightness(UINT16 para)
{
         SENSORDB("[HI258]CONTROLFLOW HI258set_brightness Para:%d;\n",para);
         HI258_write_cmos_sensor(0x03,0x20);    
         

    switch (para)
    {
        case ISP_BRIGHT_LOW:
                   HI258_write_cmos_sensor(0x70, 0x30);
      
             break;
        case ISP_BRIGHT_HIGH:
                   HI258_write_cmos_sensor(0x70, 0x80); 
    
             break;
        case ISP_BRIGHT_MIDDLE:
        default:
                   HI258_write_cmos_sensor(0x70, 0x56);
             
             break;
    }

    
    return;
}



void HI258set_contrast(UINT16 para)
{
  kal_uint8   ISPCTL4; /* P10.0x13. */


  SENSORDB("[HI258]CONTROLFLOW HI258set_contrast Para:%d;\n",para);
  HI258_write_cmos_sensor(0x03,0x10);  

    switch (para)
    {
        case ISP_CONTRAST_LOW:
         //Low
         HI258_write_cmos_sensor(0x03, 0x10);
         HI258_write_cmos_sensor(0x48, 0x50); 
       break;
            
        case ISP_CONTRAST_HIGH:
         //Hig
         HI258_write_cmos_sensor(0x03, 0x10);
         HI258_write_cmos_sensor(0x48, 0xa8);
       break;
            
        case ISP_CONTRAST_MIDDLE:
        default:
         //Med
         HI258_write_cmos_sensor(0x03, 0x10);
         HI258_write_cmos_sensor(0x48, 0x85); 
       break;
    }
    
    return;
}





void HI258set_saturation(UINT16 para)
{
         
         SENSORDB("[HI258]CONTROLFLOW HI258set_saturation Para:%d;\n",para);

         HI258_write_cmos_sensor(0x03,0x10);   
         
         
    switch (para)
    {
        case ISP_SAT_HIGH:
         //Hig
         HI258_write_cmos_sensor(0x03, 0x10);
         HI258_write_cmos_sensor(0x61, 0xe0); //Sat B
         HI258_write_cmos_sensor(0x62, 0xe0); //Sat R
             break;
        case ISP_SAT_LOW:
         //Low
         HI258_write_cmos_sensor(0x03, 0x10);
         HI258_write_cmos_sensor(0x61, 0x30); //Sat B
         HI258_write_cmos_sensor(0x62, 0x30); //Sat R
             break;
        case ISP_SAT_MIDDLE:
        default:
         //Med

         HI258_write_cmos_sensor(0x03, 0x10);
         HI258_write_cmos_sensor(0x61, 0x9a); //Sat B
         HI258_write_cmos_sensor(0x62, 0x95); // SATR
             break;
    }
     return;
}
void HI258GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = HI258Status.wb;
    pExifInfo->CapExposureTime = HI258_read_shutter();
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
	SENSORDB("HI258GetExifInfo---CapExposureTime= %d \n",pExifInfo->CapExposureTime);
}

UINT32 HI258YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    SENSORDB("[HI258] %s \n",__func__);
	SENSORDB("HI258YUVSensorSetting---iCmd= %d \n",iCmd);
    if (HI258_op_state.sensor_cap_state == KAL_TRUE)	/* Don't need it when capture mode. */
    {
    return KAL_TRUE;
    }

    switch (iCmd) 
    {
        case FID_SCENE_MODE:	    
			
			SENSORDB("HI258YUVSensorSetting---iCmd=FID_SCENE_MODE\n");
            if( HI258_CAPATURE_FLAG == 0)
            {
                if (iPara == SCENE_MODE_OFF)
                {
                    HI258_night_mode(0);
                }
                else if (iPara == SCENE_MODE_NIGHTSCENE)
                {
                    HI258_night_mode(1);
                }
            }
            else
                HI258_CAPATURE_FLAG = 0;
            break; 

        case FID_AWB_MODE:
			
			SENSORDB("HI258YUVSensorSetting---iCmd=FID_AWB_MODE\n");
            HI258_set_param_wb(iPara);
            break;

        case FID_COLOR_EFFECT:	    
			
			SENSORDB("HI258YUVSensorSetting---iCmd=FID_COLOR_EFFECT\n");
            HI258_set_param_effect(iPara);
            break;

	   case FID_ISP_CONTRAST:
     
	 SENSORDB("HI258YUVSensorSetting---iCmd=FID_ISP_CONTRAST\n");
        HI258set_contrast(iPara);
        break;
    case FID_ISP_BRIGHT:
       SENSORDB("HI258YUVSensorSetting---iCmd=FID_ISP_BRIGHT\n");
        HI258set_brightness(iPara);
        break;
    case FID_ISP_SAT:
		SENSORDB("HI258YUVSensorSetting---iCmd=FID_ISP_SAT\n");
      
        HI258set_saturation(iPara);
        break;

        case FID_AE_EV:	       	      
			
			SENSORDB("HI258YUVSensorSetting---iCmd=FID_AE_EV\n");
            HI258_set_param_exposure(iPara);
            break;

        case FID_AE_FLICKER:    	    	    
			
			SENSORDB("HI258YUVSensorSetting---iCmd=FID_AE_FLICKER\n");
            if( HI258_CAPATUREB_FLAG == 0)
                HI258_set_param_banding(iPara);
            else
                HI258_CAPATUREB_FLAG = 0;
            break;

        case FID_AE_SCENE_MODE: 
            break; 

        case FID_ZOOM_FACTOR:
            HI258_zoom_factor = iPara;
            break; 
	    case FID_AE_ISO:
		    SENSORDB("[HI258]FID_AE_ISO:%d\n",iPara);
			HI258set_iso(iPara);
		break;

        default:
            break;
    }
    return TRUE;
}   

UINT32 HI258YUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[HI258] %s \n",__func__);
    kal_uint16 temp_AE_reg = 0;
	
    HI258_write_cmos_sensor(0x03, 0x20); 
    temp_AE_reg = HI258_read_cmos_sensor(0x10);

    return TRUE;
}

UINT32 HI258FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 u2Temp = 0; 

    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;

    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
    SENSORDB("[HI258] %s =====FeatureID= %d \n",__func__,FeatureId);

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=HI258_FULL_GRAB_WIDTH;
            *pFeatureReturnPara16=HI258_FULL_GRAB_HEIGHT;
            *pFeatureParaLen=4;
            break;
	
        case SENSOR_FEATURE_GET_PERIOD:
            *pFeatureReturnPara16++=HI258_PV_PERIOD_PIXEL_NUMS+HI258_PV_dummy_pixels;
            *pFeatureReturnPara16=HI258_PV_PERIOD_LINE_NUMS+HI258_PV_dummy_lines;
            *pFeatureParaLen=4;
            break;
	
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *pFeatureReturnPara32 = HI258_sensor_pclk/10;
            *pFeatureParaLen=4;
            break;
	
        case SENSOR_FEATURE_SET_ESHUTTER:
            u2Temp = HI258_read_shutter(); 		
            break;
	
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HI258_night_mode((BOOL) *pFeatureData16);
            break;
	
        case SENSOR_FEATURE_SET_GAIN:
            break; 
	
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
	
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            HI258_isp_master_clock=*pFeatureData32;
            break;
	
        case SENSOR_FEATURE_SET_REGISTER:
            HI258_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
	
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HI258_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
	
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HI258SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;

	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
	  HI258_FlashTriggerCheck(pFeatureData32);
	  SENSORDB("[4EC] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", pFeatureData32);
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

        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            HI258_GetSensorID(pFeatureData32); 
            break;

        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;
	
        case SENSOR_FEATURE_SET_YUV_CMD:
            HI258YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
            break;	
	
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            HI258YUVSetVideoMode(*pFeatureData16);
            break; 
 case SENSOR_FEATURE_GET_EXIF_INFO:

		   HI258GetExifInfo(*pFeatureData32);
		   break;
        default:
            break;			
    }
    return ERROR_NONE;
}	

SENSOR_FUNCTION_STRUCT	SensorFuncHI258=
{
    HI258Open,
    HI258GetInfo,
    HI258GetResolution,
    HI258FeatureControl,
    HI258Control,
    HI258Close
};

UINT32 HI258_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI258;

    return ERROR_NONE;
}	





