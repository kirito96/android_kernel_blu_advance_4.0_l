
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

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	


	{0xFF, 4,	{0xAA,0x55,0x69,0x10}},  // add for esd
       {0x6f,	1,	{0x10}},// add for esd
       {0xf7,	1,	{0x01}},// add for esd
	{0xFF, 4,	{0xAA,0x55,0x69,0x00}},  // add for esd

	{0xFF,	4,	{0xAA, 0x55, 0xA5, 0x80}},
	{0x6f,	1,	{0x0e}},
	{0xf4,	1,	{0x0a}},
	{0xFF,	4,	{0xAA, 0x55, 0xA5, 0x00}},
	
	{0xF0,	5,	{0x55, 0xAA, 0x52, 0x08, 0x02}},
	{0xB7,	1,	{0x01}},
	
	{0xF0,	5,	{0x55, 0xAA, 0x52, 0x08, 0x01}},
	{0xB0,	1,	{0x0d}},
	{0xB1,	1,	{0x0d}},
	{0xB6,	1,	{0x34}}, // 34  wrw
	{0xB7,	1,	{0x45}}, // 45  wrw  
	{0xB2,	1,	{0x00}},
	{0xB5,	1,	{0x08}},
	{0xB8,	1,	{0x24}},// vcl 24 wrw  
	{0xBF,	1,	{0x01}},
	{0xB3,	1,	{0x08}},
	{0xB9,	1,	{0x34}},

	{0xBa,	1,	{0x14}},
	{0xC2,	1,	{0x03}},

	{0xBC,	3,	{0x00,0x75,0x00}}, // adj
	{0xBD,	3,	{0x00,0x75,0x00}}, // adj
	{0xBE,	2,	{0x00,0x8A}}, // VCOM adj
		

	{0xD1,	52,{0x00,0x00,// 0
	          0x00,0x01,// 1
	          0x00,0x09,// 3
	          0x00,0x19,// 5
	          0x00,0x2D,// 7
	          0x00,0x5D,// 11
	          0x00,0x8B,// 15
	          0x00,0xD5,// 23
	          0x01,0x09,// 31
	          0x01,0x52,// 47
	          0x01,0x85,// 63
	          0x01,0xCF,// 95
	          0x02,0x02,// 127
	          0x02,0x04,// 128
	          0x02,0x34,// 160
	          0x02,0x61,// 192
	          0x02,0x79,// 208
	          0x02,0x97,// 224
	          0x02,0xAA,// 232
	          0x02,0xc1,// 240
	          0x02,0xD0,// 244
	          0x02,0xE4,// 248
	          0x02,0xF2,// 250
	          0x03,0x08,// 252
	          0x03,0x3E,// 254
	          0x03,0xFE  }},
	
	

	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},
	{0xB5,	1,	{0x50}},
	{0xB1,	2,	{0xfc, 0x00}},
	{0xB6,	1,	{0x05}}, 
	{0xBB,	1,	{0xee,0Xe0}},           // SOURCE AP TEST RF wrw 22 20  adj
	{0xB7,	2,	{0x70,0x70}},
	{0xB8,	4,	{0x01,0x02,0x02,0x02}}, // SOURCE EQ TEST RF wrw 3  3  3 adj
	{0xBC,	1,	{0x02}},
	
	{0xC9,	5,	{0xc0,0x02,0x50,0x50,0x50}},
	
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

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
				//MDELAY(10);//soso add or it will fail to send register
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

		params->physical_width  = 51.84;
		params->physical_height = 86.40;


		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
	
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

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch				= 50;//50 30
		params->dsi.vertical_frontporch				= 20;//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;
		params->dsi.horizontal_backporch				= 90;// 90
		params->dsi.horizontal_frontporch				= 50; // 70
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 200; // add by peter
              params->dsi.ssc_disable = 1; // add by peter
}

static void lcm_init(void)
{
	unsigned int data_array[64];

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 120ms
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(130);//Must > 120ms

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{

	lcm_init();

}
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x04, buffer, 3);
	id = buffer[1]; //we only need ID
#if defined(BUILD_UBOOT)
	/*The Default Value should be 0x00,0x80,0x00*/
	printf("\n\n\n\n[soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif
    return (id == 0x80)?1:0;
}

#ifndef BUILD_UBOOT

static unsigned int lcm_esd_check(void)
{
	unsigned char buffer_0[3];
	unsigned char buffer_1[6];
	unsigned int array[16];
	static int err_count = 0;
	
	array[0] = 0x00063700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0A, buffer_1, 6);  //read device id,command is 0x0A
	
		//LCM_DBG("lcm_esd_check buffer_1[0]:0x%x,buffer_1[3]:0x%x,buffer_1[4]:0x%x\n",buffer_1[0],buffer_1[3],buffer_1[4]);
	
	if(buffer_1[0] == 0x9c)
	{
		if((buffer_1[3] == 0x02) && (buffer_1[4] & 0x02))
			err_count ++;
		else
			err_count = 0;
	}
	else
	{
		err_count ++;
	}
	if(err_count >= 2){
		 
	 //LCM_DBG("lcm_esd_check register[0x0A] value = 0x%x, 0x%x, 0x%x,0x%x, 0x%x,0x%x\n",buffer_1[0], buffer_1[1], buffer_1[2], buffer_1[3],buffer_1[4], buffer_1[5]);
		 
		err_count = 0;
		return 1;
	} else {
		return 0;
	}
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();

	return 1;
}

#endif


LCM_DRIVER nt35512_wvga_dsi_vdo_dj_drv = 
{
    .name			= "nt35512_dsi_vdo_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
	 .esd_check 		= lcm_esd_check,
	 .esd_recover 	= lcm_esd_recover,
};

