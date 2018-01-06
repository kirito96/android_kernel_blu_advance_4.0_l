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

#define LCM_ID       (0x99)

#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("LIMI==>[LCM-HX8357D-DBI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

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

//static int vcom=0x2b;
static void init_lcm_registers(void) {
#if 0
/*Start Initial IC = HX8357D, VCI = 2.8V*/

    send_ctrl_cmd(0x11);
    MDELAY(0xd0);

    send_ctrl_cmd(0xB9);
    send_data_cmd(0xFF);
    send_data_cmd(0x83);
    send_data_cmd(0x57);
    MDELAY(0x01);

    send_ctrl_cmd(0xB1);
    send_data_cmd(0x00);
    send_data_cmd(0x14);
    send_data_cmd(0x1c);
    send_data_cmd(0x1c);
    send_data_cmd(0xC3);
    send_data_cmd(0x36);
    MDELAY(0x01);

    send_ctrl_cmd(0xB6);
    send_data_cmd(0x2E);
    MDELAY(0x01);

    send_ctrl_cmd(0xB4);
    send_data_cmd(0x02);
    send_data_cmd(0x40);
    send_data_cmd(0x00);
    send_data_cmd(0x2A);
    send_data_cmd(0x2A);
    send_data_cmd(0x0d);
    send_data_cmd(0x78);
    MDELAY(0x01);
	
    send_ctrl_cmd(0xC0);
    send_data_cmd(0x78); 
    send_data_cmd(0x50); 
    send_data_cmd(0x01); 
    send_data_cmd(0x3C); 
    send_data_cmd(0xC8); 
    send_data_cmd(0x08);

    send_ctrl_cmd(0xB5); 
    send_data_cmd(0x05); 
    send_data_cmd(0x05); 
    send_data_cmd(0x67); 

    send_ctrl_cmd(0xC2); 
    send_data_cmd(0x00); 
    send_data_cmd(0x08); 
    send_data_cmd(0x04); 	
	
    send_ctrl_cmd(0xE0);
    send_data_cmd(0x0B); 
    send_data_cmd(0x09); 
    send_data_cmd(0x16); 
    send_data_cmd(0x28); 
    send_data_cmd(0x32); 
    send_data_cmd(0x44); 
    send_data_cmd(0x4F); 
    send_data_cmd(0x56); 
    send_data_cmd(0x43); 
    send_data_cmd(0x3D); 
    send_data_cmd(0x38); 
    send_data_cmd(0x30); 
    send_data_cmd(0x2E); 
    send_data_cmd(0x2A); 
    send_data_cmd(0x26); 
    send_data_cmd(0x21); 
    send_data_cmd(0x0B); 
    send_data_cmd(0x09); 
    send_data_cmd(0x16); 
    send_data_cmd(0x28); 
    send_data_cmd(0x32); 
    send_data_cmd(0x44); 
    send_data_cmd(0x4F); 
    send_data_cmd(0x56); 
    send_data_cmd(0x43); 
    send_data_cmd(0x3D); 
    send_data_cmd(0x38); 
    send_data_cmd(0x30); 
    send_data_cmd(0x2E); 
    send_data_cmd(0x2A); 
    send_data_cmd(0x26); 
    send_data_cmd(0x21); 
    send_data_cmd(0x44); 
    send_data_cmd(0x01); 
	
    send_ctrl_cmd(0xCC);
    send_data_cmd(0x09); //0D //09
	
    send_ctrl_cmd(0x36);
    send_data_cmd(0x00);//40 90
    MDELAY(0x01);

    send_ctrl_cmd(0x35);
    send_data_cmd(0x00);

    send_ctrl_cmd(0xB0);
    send_data_cmd(0x0d);
    send_data_cmd(0x01);
	
    send_ctrl_cmd(0x3A);
    send_data_cmd(0x55);
	
    send_ctrl_cmd(0x29);
    MDELAY(50);
    send_ctrl_cmd(0x2C);  //Memory Write Start
    MDELAY(120);
#else
    send_ctrl_cmd(0xB9); // SET password 
    send_data_cmd(0xFF);
    send_data_cmd(0x83);
    send_data_cmd(0x57);
    MDELAY(1);
    /*
    send_ctrl_cmd(0xB0); 
    send_data_cmd(0x88);
    send_data_cmd(0x00); */
    
    send_ctrl_cmd(0xB1); //SETPower 
    send_data_cmd(0x00); //STB 
    send_data_cmd(0x14); //VGH = 13V, VGL = -10V 
    send_data_cmd(0x18); //VSPR = 4.41V 
    send_data_cmd(0x18); //VSNR = -4.41V 
    send_data_cmd(0xC3); //AP 
    send_data_cmd(0x21); //38     //FS  
    MDELAY(1);
    /*send_ctrl_cmd(0xB3); //SETRGB 
    send_data_cmd(0x00); //send_data_cmd(0x43); //SDO_EN=1, BYPASS=1, RM=1, DM=1 
    send_data_cmd(0x00); //DPL=0, HSPL=0, VSPL=0, EPL=0 
    send_data_cmd(0x06); //HPL 
    send_data_cmd(0x06); //VPL 
    MDELAY(1);*/
    
    //zhangxiaofei start
  //  send_ctrl_cmd(0xB0); //SETCYC 
 //   send_data_cmd(0xbb); //2-dot 
    //zhangxiaofei end
    
    send_ctrl_cmd(0xB4); //SETCYC 
    send_data_cmd(0x22); //2-dot 
    //send_data_cmd(0x00); //Column inversion 
    //send_data_cmd(0x44);   //4-dot  inversion  
    
    send_data_cmd(0x40); //RTN 
    send_data_cmd(0x00); //DIV 
    send_data_cmd(0x2A); //N_DUM 
    send_data_cmd(0x2A); //I_DUM 
    send_data_cmd(0x20); //GDON 
    send_data_cmd(0x4E); //GDOFF 
    MDELAY(1);

    send_ctrl_cmd(0xB6); //VCOMDC 
    send_data_cmd(0x38);
    //send_data_cmd(vcom );   //34 0x29 0x37
   // vcom ++;
    MDELAY(1);
    send_ctrl_cmd(0xC0); //SETSTBA 
    send_data_cmd(0x24); //N_OPON 
    send_data_cmd(0x24); //I_OPON 
    send_data_cmd(0x00); //STBA 
    send_data_cmd(0x10); //STBA 
    send_data_cmd(0xC8); //STBA 
    send_data_cmd(0x08); //GENON 
    MDELAY(1);
    send_ctrl_cmd(0xC2); // Set Gate EQ 
    send_data_cmd(0x00);
    send_data_cmd(0x08);
    send_data_cmd(0x04);
    MDELAY(1);
    send_ctrl_cmd(0xCC); //Set Panel 
    send_data_cmd(0x01); //SS_Panel = 1, BG
    MDELAY(1);
/*
    //GAMMA 2.2"
    send_ctrl_cmd(0xE0); //Set Gamma 
    send_data_cmd(0x01); //VRP0[6:0]
    send_data_cmd(0x05); //VRP1[6:0]
    send_data_cmd(0x0a); //VRP2[6:0]
    send_data_cmd(0x1d); //VRP3[6:0]
    send_data_cmd(0x29); //VRP4[6:0]
    send_data_cmd(0x3C); //VRP5[6:0]
    send_data_cmd(0x49); //VRP6[6:0]
    send_data_cmd(0x52); //VRP7[6:0]
    send_data_cmd(0x47); //VRP8[6:0]
    send_data_cmd(0x40); //VRP9[6:0]
    send_data_cmd(0x3A); //VRP10[6:0]
    send_data_cmd(0x32); //VRP11[6:0]
    send_data_cmd(0x30); //VRP12[6:0]
    send_data_cmd(0x2B); //VRP13[6:0]
    send_data_cmd(0x27); //VRP14[6:0]
    send_data_cmd(0x1C); //VRP15[6:0]
    send_data_cmd(0x03); //VRP0[6:0]
    send_data_cmd(0x07); //VRP1[6:0]
    send_data_cmd(0x13); //VRP2[6:0]
    send_data_cmd(0x20); //VRP3[6:0]
    send_data_cmd(0x29); //VRP4[6:0]
    send_data_cmd(0x3C); //VRP5[6:0]
    send_data_cmd(0x49); //VRP6[6:0]
    send_data_cmd(0x52); //VRP7[6:0]
    send_data_cmd(0x47); //VRP8[6:0]
    send_data_cmd(0x40); //VRP9[6:0]
    send_data_cmd(0x3A); //VRP10[6:0]
    send_data_cmd(0x32); //VRP11[6:0]
    send_data_cmd(0x30); //VRP12[6:0]
    send_data_cmd(0x2B); //VRP13[6:0]
    send_data_cmd(0x27); //VRP14[6:0]
    send_data_cmd(0x1C); //VRP15[6:0]
    send_data_cmd(0x00);
    send_data_cmd(0x01);
*/

//20130723@Jovan.niu
    //GAMMA 2.2"
    send_ctrl_cmd(0xE0); //Set Gamma 
	send_data_cmd(0x01); //VRP0[6:0]
    send_data_cmd(0x06); //VRP1[6:0]1
    send_data_cmd(0x0c); //VRP2[6:0]2
    send_data_cmd(0x20); //VRP3[6:0]3
    send_data_cmd(0x2d); //VRP4[6:0]4
    send_data_cmd(0x42); //VRP5[6:0]5
    send_data_cmd(0x4f); //VRP6[6:0]6
    send_data_cmd(0x59); //VRP7[6:0]7 
    send_data_cmd(0x40); //VRP8[6:0]7
    send_data_cmd(0x3a); //VRP9[6:0]6
    send_data_cmd(0x35); //VRP10[6:0]5
    send_data_cmd(0x2e); //VRP11[6:0]4
    send_data_cmd(0x2d); //VRP12[6:0]3
    send_data_cmd(0x29); //VRP13[6:0]2
    send_data_cmd(0x26); //VRP14[6:0]1
    send_data_cmd(0x1C); //VRP15[6:0]

    send_data_cmd(0x03); //VRP0[6:0]
    send_data_cmd(0x08); //VRP1[6:0]1
    send_data_cmd(0x15); //VRP2[6:0]2
    send_data_cmd(0x23); //VRP3[6:0]3
    send_data_cmd(0x2d); //VRP4[6:0]4
    send_data_cmd(0x42); //VRP5[6:0]5
    send_data_cmd(0x4f); //VRP6[6:0]6
    send_data_cmd(0x59); //VRP7[6:0]7
    send_data_cmd(0x40); //VRP8[6:0]7
    send_data_cmd(0x3a); //VRP9[6:0]6
    send_data_cmd(0x35); //VRP10[6:0]5
    send_data_cmd(0x2e); //VRP11[6:0]4
    send_data_cmd(0x2d); //VRP12[6:0]3
    send_data_cmd(0x29); //VRP13[6:0]2
    send_data_cmd(0x26); //VRP14[6:0]1
    send_data_cmd(0x1C); //VRP15[6:0]
    send_data_cmd(0x00);
    send_data_cmd(0x01);

    MDELAY(1);
    send_ctrl_cmd(0xEA); // SETMESSI 
    send_data_cmd(0x03);
    MDELAY(1);
    send_ctrl_cmd(0x3A); //COLMOD 
    send_data_cmd(0x66);
    //send_data_cmd(0x55);
    MDELAY(1);
    send_ctrl_cmd(0x36); 
    send_data_cmd(0x40);
    MDELAY(1);
    send_ctrl_cmd(0x35); 
    send_data_cmd(0x00);
    MDELAY(1);
    send_ctrl_cmd(0x11); //Sleep Out 
    MDELAY(120);
    send_ctrl_cmd(0x29); //Display on 
    MDELAY(50);
    send_ctrl_cmd(0x2C);  //Memory Write Start
    MDELAY(120);
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

#if defined(BUILD_UBOOT) || defined(BUILD_LK)
static unsigned int lcm_compare_id()
{
    int para1,para2;

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    send_ctrl_cmd(0xB9);                  //EXTC
    send_data_cmd(0xFF);                //EXTC
    send_data_cmd(0x83);                //EXTC
    send_data_cmd(0x57);                //EXTC
    MDELAY(10);
    send_ctrl_cmd(0xD0);
    para1 = read_data_cmd();    //Dummy read
    para2 = read_data_cmd();

    para1 = para1 & 0xFF;
    para2 = para2 & 0xFF;

    LCM_DBG("read id1 = 0x%x \n",para1);
    LCM_DBG("read id2 = 0x%x \n",para2);	
    
  if (para2 == LCM_ID )
	return 1;
    else
	return 0;			//default lcm driver if nothing match the lcd list
}
#endif

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

static void lcm_suspend(void) {
    LCM_DBG();	
    send_ctrl_cmd(0x10);
    MDELAY(120);
}

static void lcm_resume(void) {
    LCM_DBG();	
   send_ctrl_cmd(0x11);
  MDELAY(120);
    //lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y,
        unsigned int width, unsigned int height) {
    unsigned short x0, y0, x1, y1;
    unsigned short h_X_start, l_X_start, h_X_end, l_X_end, h_Y_start, l_Y_start, h_Y_end, l_Y_end;

    x0 = (unsigned short) x;
    y0 = (unsigned short) y;
    x1 = (unsigned short) x + width - 1;
    y1 = (unsigned short) y + height - 1;

    h_X_start = ((x0 & 0xFF00) >> 8);
    l_X_start = (x0 & 0x00FF);
    h_X_end = ((x1 & 0xFF00) >> 8);
    l_X_end = (x1 & 0x00FF);

    h_Y_start = ((y0 & 0xFF00) >> 8);
    l_Y_start = (y0 & 0x00FF);
    h_Y_end = ((y1 & 0xFF00) >> 8);
    l_Y_end = (y1 & 0x00FF);

    send_ctrl_cmd(0x2A);
    send_data_cmd(h_X_start);
    send_data_cmd(l_X_start);
    send_data_cmd(h_X_end);
    send_data_cmd(l_X_end);

    send_ctrl_cmd(0x2B);
    send_data_cmd(h_Y_start);
    send_data_cmd(l_Y_start);
    send_data_cmd(h_Y_end);
    send_data_cmd(l_Y_end);

    send_ctrl_cmd(0x29);

    send_ctrl_cmd(0x2C);
}

LCM_DRIVER hx8357d_hvga_txd_drv ={
    .name = "hx8357d_hvga_txd_drv",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .update = lcm_update,
#if defined(BUILD_UBOOT) || defined(BUILD_LK)
    .compare_id     = lcm_compare_id
#endif
};
