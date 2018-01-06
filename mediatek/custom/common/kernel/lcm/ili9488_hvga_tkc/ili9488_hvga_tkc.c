/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK
  #include <platform/mt_typedefs.h>
  #include <platform/mt_gpio.h>
  #define LCM_PRINT printf
#else
  #include <linux/string.h>
  #include <linux/kernel.h>
  #include <mach/mt_gpio.h>
  #define LCM_PRINT printk
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (320)
#define FRAME_HEIGHT (480)

#define LCM_ID       (0x9488)

#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("LIMI==>[LCM-ili9488-DBI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
static __inline unsigned int HIGH_BYTE(unsigned int val)
{
    return (val >> 8) & 0xFF;
}

static __inline unsigned int LOW_BYTE(unsigned int val)
{
    return (val & 0xFF);
}

static __inline void send_ctrl_cmd(unsigned int cmd) {
    lcm_util.send_cmd(cmd);
}

static __inline void send_data_cmd(unsigned int data) {
    lcm_util.send_data(data);
}

static __inline unsigned int read_data_cmd()
{
    return lcm_util.read_data();
}


static __inline void set_lcm_register(unsigned int regIndex,
        unsigned int regData) {
    send_ctrl_cmd(regIndex);
    send_data_cmd(regData);
}


static void init_lcm_registers(void)
{
#if 0
    send_ctrl_cmd(0xE0); 
    send_data_cmd(0x00); 
    send_data_cmd(0x07); 
    send_data_cmd(0x0f); 
    send_data_cmd(0x0D); 
    send_data_cmd(0x1B); 
    send_data_cmd(0x0A); 
    send_data_cmd(0x3c); 
    send_data_cmd(0x78); 
    send_data_cmd(0x4A); 
    send_data_cmd(0x07); 
    send_data_cmd(0x0E); 
    send_data_cmd(0x09); 
    send_data_cmd(0x1B); 
    send_data_cmd(0x1e); 
    send_data_cmd(0x0f);  
 
    send_ctrl_cmd(0xE1); 
    send_data_cmd(0x00); 
    send_data_cmd(0x22); 
    send_data_cmd(0x24); 
    send_data_cmd(0x06); 
    send_data_cmd(0x12); 
    send_data_cmd(0x07); 
    send_data_cmd(0x36); 
    send_data_cmd(0x47); 
    send_data_cmd(0x47); 
    send_data_cmd(0x06); 
    send_data_cmd(0x0a); 
    send_data_cmd(0x07); 
    send_data_cmd(0x30); 
    send_data_cmd(0x37); 
    send_data_cmd(0x0f); 

/*
    send_ctrl_cmd(0xC0); 
    send_data_cmd(0x10); 
    send_data_cmd(0x10); 
 
    send_ctrl_cmd(0xC1); 
    send_data_cmd(0x41); 
*/

    send_ctrl_cmd(0xC5); 
    send_data_cmd(0x00); 
    send_data_cmd(0x2c); //22
    send_data_cmd(0x80); 

    send_ctrl_cmd(0x36); 
    send_data_cmd(0x48); 

    send_ctrl_cmd(0x3A); //Interface Mode Control
    send_data_cmd(0x55);


    send_ctrl_cmd(0XB0);  //Interface Mode Control  
    send_data_cmd(0x00); 
    
    send_ctrl_cmd(0xB1);   //Frame rate 70HZ  
    send_data_cmd(0xB0);   // 90, D0 Frame Rate
    //send_data_cmd(0xD0);

    send_ctrl_cmd(0xB4); 
    //send_data_cmd(0x02);   // 00 , 01,  02  
    //send_data_cmd(0x00);   //Column inversion  
    //send_data_cmd(0x01);   //1 dot inversion 
    send_data_cmd(0x02);     //2 dot inversion 

    send_ctrl_cmd(0xB6); //RGB/MCU Interface Control
    send_data_cmd(0x02); 
    send_data_cmd(0x02); 

    send_ctrl_cmd(0xE9); 
    send_data_cmd(0x00);
 
    send_ctrl_cmd(0XF7);    
    send_data_cmd(0xA9); 
    send_data_cmd(0x51); 
    send_data_cmd(0x2C); 
    send_data_cmd(0x82);

    send_ctrl_cmd(0x11); 
    MDELAY(150); 
    send_ctrl_cmd(0x29); 
#else
    send_ctrl_cmd(0XF7);
    send_data_cmd(0xA9);
    send_data_cmd(0x51);
    send_data_cmd(0x2C);
    send_data_cmd(0x82);

    send_ctrl_cmd(0xC0);
    send_data_cmd(0x07);
    send_data_cmd(0x07);

    send_ctrl_cmd(0xC1);
    send_data_cmd(0x41);

    send_ctrl_cmd(0XC5);
    send_data_cmd(0x00);
    send_data_cmd(0x1c);//0x2a
    send_data_cmd(0x80);

    send_ctrl_cmd(0xB1);
    send_data_cmd(0xB0);
    send_data_cmd(0x11);

    send_ctrl_cmd(0xB4);
    send_data_cmd(0x02);

    send_ctrl_cmd(0xB6);
    send_data_cmd(0x02);
    send_data_cmd(0x22);

    send_ctrl_cmd(0xB7);
    send_data_cmd(0xc6);

    send_ctrl_cmd(0xBE);
    send_data_cmd(0x00);
    send_data_cmd(0x04);

    send_ctrl_cmd(0xE9);
    send_data_cmd(0x00);

    send_ctrl_cmd(0x36);
    send_data_cmd(0x08);

    send_ctrl_cmd(0x3A);
    send_data_cmd(0x66);

    send_ctrl_cmd(0xE0);
    send_data_cmd(0x00);
    send_data_cmd(0x07);
    send_data_cmd(0x12);
    send_data_cmd(0x0B);
    send_data_cmd(0x18);
    send_data_cmd(0x0B);
    send_data_cmd(0x3F);
    send_data_cmd(0x9B);
    send_data_cmd(0x4B);
    send_data_cmd(0x0B);
    send_data_cmd(0x0F);
    send_data_cmd(0x0B);
    send_data_cmd(0x15);
    send_data_cmd(0x17);
    send_data_cmd(0x0F);

    send_ctrl_cmd(0XE1);
    send_data_cmd(0x00);
    send_data_cmd(0x16);
    send_data_cmd(0x1B);
    send_data_cmd(0x02);
    send_data_cmd(0x0F);
    send_data_cmd(0x06);
    send_data_cmd(0x34);
    send_data_cmd(0x46);
    send_data_cmd(0x48);
    send_data_cmd(0x04);
    send_data_cmd(0x0D);
    send_data_cmd(0x0D);
    send_data_cmd(0x35);
    send_data_cmd(0x36);
    send_data_cmd(0x0F);

    send_ctrl_cmd(0X35);
    send_data_cmd(0x00);

    send_ctrl_cmd(0x11);
    MDELAY(150); 
    send_ctrl_cmd(0x29);
#endif
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util) {
    memcpy(&lcm_util, util, sizeof (LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params) {
     memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DBI;
	params->ctrl   = LCM_CTRL_PARALLEL_DBI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->io_select_mode = 3;	
	
	params->dbi.port                    = 0;
	params->dbi.clock_freq              = LCM_DBI_CLOCK_FREQ_104M;
	params->dbi.data_width              = LCM_DBI_DATA_WIDTH_18BITS;
	params->dbi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dbi.data_format.trans_seq   = LCM_DBI_TRANS_SEQ_MSB_FIRST;
	params->dbi.data_format.padding     = LCM_DBI_PADDING_ON_MSB;
	params->dbi.data_format.format      = LCM_DBI_FORMAT_RGB666;
	params->dbi.data_format.width       = LCM_DBI_DATA_WIDTH_18BITS;
	params->dbi.cpu_write_bits          = LCM_DBI_CPU_WRITE_32_BITS;
	params->dbi.io_driving_current      = LCM_DRIVING_CURRENT_8MA;
	#ifdef LENOVO_DVT 
	params->dbi.parallel.write_setup    = 2;
	params->dbi.parallel.write_hold     = 3;
	params->dbi.parallel.write_wait     = 10;
	#else
	params->dbi.parallel.write_setup    = 2;
	params->dbi.parallel.write_hold     = 2;
	params->dbi.parallel.write_wait     = 6;
	#endif
	params->dbi.parallel.read_setup     = 3;
	params->dbi.parallel.read_latency   = 40;
	params->dbi.parallel.wait_period    = 0;

	// enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_FALLING;

}

static unsigned int lcm_compare_id()
{
    int para1,para2,id;

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    send_ctrl_cmd(0xd3);
    read_data_cmd();    //Dummy read
    read_data_cmd();    //Dummy read
    para1 = read_data_cmd();
    para2 = read_data_cmd();

    para1 = para1 & 0xFF;
    para2 = para2 & 0xFF;
    id = para2 | (para1 << 8);
    LCM_DBG("read id1 = 0x%x \n",para1);
    LCM_DBG("read id2 = 0x%x \n",para2);
    LCM_DBG("read id = 0x%x \n",id);
    if (id == LCM_ID)	//0x9488
	return 1;
    else
	return 0;			//default lcm driver if nothing match the lcd list
}

static void lcm_init(void) {
    LCM_DBG();	
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
    init_lcm_registers();

    //Set TE register
    send_ctrl_cmd(0x35);
    send_data_cmd(0x00);

    send_ctrl_cmd(0X0044); // Set TE signal delay scanline
    send_data_cmd(0X0000); // Set as 0-th scanline
    send_data_cmd(0X0000);
    //sw_clear_panel(0);
}

static void lcm_suspend(void)
{
	send_ctrl_cmd(0x28);
	
	send_ctrl_cmd(0xb7);
	send_data_cmd(0x0e);
	MDELAY(150);
}


static void lcm_resume(void)
{
	//send_ctrl_cmd(0x29);
	SET_RESET_PIN(1);
    MDELAY(50);
    SET_RESET_PIN(0);
    MDELAY(120);
    SET_RESET_PIN(1);
    MDELAY(50);
    init_lcm_registers();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

	send_ctrl_cmd(0x2A);
	send_data_cmd(HIGH_BYTE(x0));
	send_data_cmd(LOW_BYTE(x0));
	send_data_cmd(HIGH_BYTE(x1));
	send_data_cmd(LOW_BYTE(x1));

	send_ctrl_cmd(0x2B);
	send_data_cmd(HIGH_BYTE(y0));
	send_data_cmd(LOW_BYTE(y0));
	send_data_cmd(HIGH_BYTE(y1));
	send_data_cmd(LOW_BYTE(y1));

	// Write To GRAM
	send_ctrl_cmd(0x2C);
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9488_hvga_tkc_drv = 
{
    .name			= "ili9488_hvga_tkc_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.update         = lcm_update,
	.compare_id     = lcm_compare_id
};
