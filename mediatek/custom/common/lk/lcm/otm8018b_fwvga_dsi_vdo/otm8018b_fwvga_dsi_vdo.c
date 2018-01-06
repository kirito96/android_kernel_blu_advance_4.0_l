#ifdef BUILD_LK
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
	#define LCM_PRINT printf
#else
	#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[LCM-OTM8018B-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM8018B	0x8009

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

static unsigned char esdSwitch =  1;
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_V2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = 
{
	/*
	Note :

	Data ID will depends on the following rule.

	count of parameters > 1	=> Data ID = 0x39
	count of parameters = 1	=> Data ID = 0x15
	count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0x00, 1 , {0x00}}, 
	{0xFF,  3 ,{0x80,0x09,0x01}}, 

	{0x00, 1 , {0x80}}, 
	{0xFF,  2 ,{0x80,0x09}}, 

	{0x00, 1 , {0x03}}, 
	{0xFF,  1 ,{0x01}}, 

	{0x00, 1 , {0x81}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x83}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x85}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x87}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x89}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x8B}}, 
	{0xF5,  1 ,{0x20}}, 

	{0x00, 1 , {0x91}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x93}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x95}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x97}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0x99}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xA1}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xA3}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xA5}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xA7}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xB1}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xB3}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xB5}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xB7}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xB9}}, 
	{0xF5,  1 ,{0x18}}, 

	{0x00, 1 , {0xA2}}, 
	{0xC0,  6 ,{0x00,0x01,0x03,0x01,0x0D,0x0D}}, 

	{0x00, 1 , {0x90}}, 
	{0xC5,  2 ,{0x96,0xB6}}, 

	{0x00, 1 , {0x94}}, 
	{0xC5,  2 ,{0x55,0x55}}, 

	{0x00, 1 , {0xB4}}, 
	{0xC0,  1 ,{0x55}}, 

	{0x00, 1 , {0x00}}, 
	{0xD8,  2 ,{0x6F,0x6F}}, 

	{0x00, 1 , {0x82}}, 
	{0xB2,  1 ,{0x20}}, 

	//{0x00, 1 , {0x00}}, 
	//{0xD9,  1 ,{0x3D}}, //VCOM  SET 

	{0x00, 1 , {0x90}}, 
	{0xB3,  1 ,{0x02}}, 

	{0x00, 1 , {0x92}}, 
	{0xB3,  1 ,{0x45}}, 

	{0x00, 1 , {0x00}}, 
	{0x26,  1 ,{0x01}}, 

	{0x00, 1 , {0x00}}, 
	{0xE1, 16 ,{0x00,0x06,0x0B,0x0D,0x0A,0x1F,0x0F,0x0F,0x00,0x04,0x02,0x07,0x0F,0x20,0x1C,0x06}}, 

	{0x00, 1 , {0x00}}, 
	{0xE2, 16 ,{0x00,0x06,0x0B,0x0D,0x0A,0x1F,0x0F,0x0F,0x00,0x04,0x02,0x07,0x0F,0x20,0x1C,0x06}}, 

	{0x00, 1 , {0x80}}, 
	{0xC0,  9 ,{0x00,0x58,0x00,0x15,0x15,0x00,0x58,0x15,0x15}}, 

	{0x00, 1 , {0x90}}, 
	{0xC0,  6 ,{0x00,0x44,0x00,0x00,0x00,0x03}}, 

	{0x00, 1 , {0xA6}}, 
	{0xC1,  3 ,{0x01,0x00,0x00}}, 

	{0x00, 1 , {0x80}}, 
	{0xCE, 12 ,{0x86,0x03,0x00,0x85,0x03,0x00,0x84,0x03,0x00,0x83,0x03,0x00}}, 

	{0x00, 1 , {0x90}}, 
	{0xCE, 14 ,{0x33,0x52,0x00,0x33,0x53,0x00,0x33,0x54,0x00,0x33,0x55,0x00,0x00,0x00}}, 

	{0x00, 1 , {0xA0}}, 
	{0xCE, 14 ,{0x38,0x05,0x03,0x56,0x00,0x73,0x00,0x38,0x04,0x03,0x57,0x00,0x73,0x00}}, 

	{0x00, 1 , {0xB0}}, 
	{0xCE, 14 ,{0x38,0x03,0x03,0x58,0x00,0x73,0x00,0x38,0x02,0x03,0x59,0x00,0x73,0x00}}, 

	{0x00, 1 , {0xC0}}, 
	{0xCE, 14 ,{0x38,0x01,0x03,0x5A,0x00,0x73,0x00,0x38,0x00,0x03,0x5C,0x00,0x73,0x00}}, 

	{0x00, 1 , {0xD0}}, 
	{0xCE, 14 ,{0x30,0x00,0x03,0x5C,0x00,0x73,0x00,0x30,0x01,0x03,0x5D,0x00,0x73,0x00}}, 

	{0x00, 1 , {0xC3}}, 
	{0xCB,  8 ,{0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54}}, 

	{0x00, 1 , {0xD8}}, 
	{0xCB,  7 ,{0x54,0x54,0x54,0x54,0x54,0x54,0x54}}, 

	{0x00, 1 , {0xE0}}, 
	{0xCB,  1 ,{0x54}}, 

	{0x00, 1 , {0x83}}, 
	{0xCC,  7 ,{0x03,0x01,0x09,0x0B,0x0D,0x0F,0x05}}, 

	{0x00, 1 , {0x90}}, 
	{0xCC,  1 ,{0x07}}, 

	{0x00, 1 , {0x9D}}, 
	{0xCC,  2 ,{0x04,0x02}}, 

	{0x00, 1 , {0xA0}}, 
	{0xCC,  6 ,{0x0A,0x0C,0x0E,0x10,0x06,0x08}}, 

	{0x00, 1 , {0xB3}}, 
	{0xCC,  7 ,{0x06,0x08,0x0A,0x10,0x0E,0x0C,0x04}}, 

	{0x00, 1 , {0xC0}}, 
	{0xCC,  1 ,{0x02}}, 

	{0x00, 1 , {0xCD}}, 
	{0xCC,  2 ,{0x05,0x07}}, 

	{0x00, 1 , {0xD0}}, 
	{0xCC,  7 ,{0x09,0x0F,0x0D,0x0B,0x03,0x01,0x01}}, 

	{0x00, 1 , {0xC7}}, 
	{0xCF,  1 ,{0x02}}, 

	{0x00, 1 , {0xB2}}, 
	{0x00,  4 ,{0x15,0x00,0x15,0x00}}, 

	{0x00, 1 , {0x93}}, 
	{0xC5,  1 ,{0x03}}, 

	{0x00, 1 , {0x80}}, 
	{0xC4,  1 ,{0x30}}, 

	{0x00, 1 , {0x8A}}, 
	{0xC4,  1 ,{0x40}}, 

	{0x00, 1 , {0x81}}, 
	{0xC1,  1 ,{0x66}}, 

	{0x00, 1 , {0xA0}}, 
	{0xC1,  1 ,{0xEA}}, 

	{0x00, 1 , {0x81}}, 
	{0xC5,  1 ,{0x66}}, 

	{0x00, 1 , {0xC0}}, 
	{0xC5,  1 ,{0x00}}, 

	{0x00, 1 , {0xB6}}, 
	{0xF5,  1 ,{0x06}}, 

	{0x00, 1 , {0x8B}}, 
	{0xB0,  1 ,{0x40}}, 

	{0x00, 1 , {0xC6}}, 
	{0xB0,  1 ,{0x03}}, 

	{0x00, 1 , {0xB1}}, 
	{0xB0,  1 ,{0x80}}, 

	{0x35, 1, {0x00}},
	{0x11, 1,{0x00}},
	{REGFLAG_DELAY, 200, {}},
	{0x29, 1,{0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}} 
};



static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=480*3;
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 21;
	params->dsi.vertical_frontporch					= 21;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 6;
	params->dsi.horizontal_backporch				= 44;
	params->dsi.horizontal_frontporch				= 46;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	params->dsi.compatibility_for_nvk = 0;	

	params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
	params->dsi.fbk_div =26;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
}


static void lcm_init(void)
{
	LCM_DBG(" Magnum lcm_init...\n");
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(100);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	esdSwitch = 0;
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
	
}


static unsigned int check_display_normal(void)
{
	unsigned int normal=0;
	unsigned char buffer1[1];
	unsigned char buffer2[1];
	unsigned char buffer3[1];
	unsigned char buffer4[1];
	unsigned int array[16]; 
	array[0] = 0x00013700;// set return byte number
	dsi_set_cmdq(array, 1, 1);

	read_reg_V2(0x0A, &buffer1, 1);
	read_reg_V2(0x0B, &buffer2, 1);
	read_reg_V2(0x0C, &buffer3, 1);
	read_reg_V2(0x0D, &buffer4, 1);
	LCM_DBG("[Magnum] --test ic normal == 0x%x , 0x%x , 0x%x , 0x%x\n",buffer1[0],buffer2[0],buffer3[0],buffer4[0]);
        if(buffer1[0] == 0x9c && buffer2[0] == 0x0 && buffer3[0] == 0x7 && buffer4[0] == 0x0)
	 	return 1;
	else
		return 0;
}

static unsigned int lcm_esd_check(void)
{
	if(esdSwitch == 0){
		LCM_DBG("[Magnum] --esdSwitch == false\n");
		return 0;
	}
	unsigned int ret=0;
	ret = check_display_normal();
	if (ret)
		return 0;
	return 1;
	
}

static void lcm_resume(void)
{
	esdSwitch = 1;
	lcm_init();
}

#if defined(BUILD_UBOOT) || defined(BUILD_LK)
#include "cust_adc.h"
#define LCM_MAX_VOLTAGE 600 
#define LCM_MIN_VOLTAGE  300 
#endif

static struct LCM_setting_table lcm_compare_id_setting[] = {

	{0x00,	1,	{0x00}},
	{0xff,	3,	{0x80,0x09,0x01}}, 
	{REGFLAG_DELAY, 10, {}},

	{0x00,	1,	{0x80}},
	{0xff,	3,	{0x80,0x09,0x01}}, 
	{REGFLAG_DELAY, 10, {}},

	{0x00,	1,	{0x02}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static unsigned int lcm_compare_id(void)
{
	int data[4];
	int adcVol=0;
	int res = IMM_GetOneChannelValue( 1 , data , 0 );
	adcVol=data[0]*1000+data[1]*10;

#ifdef BUILD_LK
	printf("LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol); 
#else
	printk("LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol);   
#endif

	if((adcVol>200) && (adcVol<=500))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


LCM_DRIVER otm8018b_dsi_vdo_lcm_drv = 
{
	.name			= "otm8018b_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    	= lcm_compare_id,	
	.esd_check 		= lcm_esd_check,
	.esd_recover 	= lcm_init,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};

