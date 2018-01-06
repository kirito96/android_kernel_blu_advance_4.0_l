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
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

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
#else
#include <linux/string.h>

#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#define LCM_PRINT printf
#ifndef KERN_INFO
#define KERN_INFO
#endif
#else
#include <linux/kernel.h>
#include <mach/mt_gpio.h>
#define LCM_PRINT printk
#endif
#endif

#if defined(BUILD_LK)
#define LCM_PRINT printf
#ifndef KERN_INFO
#define KERN_INFO
#endif
#endif

#if 1
#define LCM_DBG(fmt, arg...) \
    LCM_PRINT("[LCM-ILI9806H-HALFRAM-DSI] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
#else
#define LCM_DBG(fmt, arg...) do {} while (0)
#endif

#include "lcm_drv.h"

#define LCM_DSI_CMD_MODE                                    1

//TN801031 added 2012.09.05
#define TINNO_INT
static unsigned int lcm_esd_test = 0;      ///only for ESD test
#define TINNO_INT_ESD        1
#define TINNO_INT__30

//TN801031 added 2012.09.05

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
//TN801031 added 2013.06.17
//#define TN_DISPLAY 1
//TN801031 added 2013.06.17

#if TN_DISPLAY
#define FRAME_WIDTH                                         (480)
#define FRAME_HEIGHT                                        (800)
#else
#define FRAME_WIDTH                                         (320)
#define FRAME_HEIGHT                                        (480)
#endif



#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0xFB   // END OF REGISTERS MARKER

//#define LCM_ID_OTM8009A       0x8009
#define LCM_ID_ILI9806H     0x9826


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

static int i = 0; //used for CE d400h, edit by Magnum 2012-7-17
unsigned  char mm[2] = {0x00,0x00};
//enable CE -- Color enhance edit by Magnum 2012-7-16
//define par1 and par2.

//tinnohd ext_te
#define TRUE    1
#define FALSE   0
//tinnohd ext_te

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_V2(cmd,buffer,buffer_size)                 lcm_util.dsi_dcs_read_lcm_reg_v2(cmd,buffer,buffer_size)


struct LCM_setting_table
{
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[360];
};

static struct LCM_setting_table lcm_set_window[] =
{
    {0x2A,  4,  {0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
    {0x2B,  4,  {0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_compare_id_setting[] =
{
    // Display off sequence
    {0xB9,  3,  {0xFF, 0x83, 0x69}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    // {0xC3, 1, {0xFF}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void dsi_send_cmdq_tinno(unsigned cmd, unsigned char count, unsigned char *para_list, unsigned char force_update)
{
    unsigned int item[16];
    unsigned char dsi_cmd = (unsigned char)cmd;
    unsigned char dc;
    int index = 0, length = 0;

    memset(item,0,sizeof(item));
    if(count+1 > 60)
    {
        LCM_DBG("Exceed 16 entry\n");
        return;
    }
    /*
        Data ID will depends on the following rule.

            count of parameters > 1 => Data ID = 0x39
            count of parameters = 1 => Data ID = 0x15
            count of parameters = 0 => Data ID = 0x05
    */
    if(count == 0)
    {
        item[0] = 0x0500 | (dsi_cmd<<16);
        length = 1;
    }
    else if(count == 1)
    {
        item[0] = 0x1500 | (dsi_cmd<<16) | (para_list[0]<<24);
        length = 1;
    }
    else
    {
        item[0] = 0x3902 | ((count+1)<<16);//Count include command.
        ++length;
        while(1)
        {
            if (index == count+1)
                break;
            if ( 0 == index )
            {
                dc = cmd;
            }
            else
            {
                dc = para_list[index-1];
            }
            // an item make up of 4data.
            item[index/4+1] |= (dc<<(8*(index%4)));
            if ( index%4 == 0 ) ++length;
            ++index;
        }
    }

    //dsi_set_cmdq(&item, length, force_update);
    dsi_set_cmdq(item, length, force_update);

}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                //dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                dsi_send_cmdq_tinno(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}
//tinnohd
static void push_table_tkc(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                MDELAY(10);
        }
    }

}
//tinnohd

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
//  params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
//  params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;//LCM_POLARITY_FALLING;//LCM_POLARITY_RISING   TINNO_INT test

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;

    // Video mode setting
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=480*3;//480*3;
    params->dsi.vertical_sync_active=3;
    params->dsi.vertical_backporch=3;
    params->dsi.vertical_frontporch=3;
    params->dsi.vertical_active_line=800;

#ifdef BUILD_LK
    params->dsi.lcm_ext_te_enable   =FALSE;//tinnohd ext_te
#else
    params->dsi.lcm_ext_te_enable   = FALSE;//TRUE;//tinnohd ext_te
#endif

    params->dsi.line_byte=2180;     // 2256 = 752*3
    params->dsi.horizontal_sync_active_byte=32;
    params->dsi.horizontal_backporch_byte=32;
    params->dsi.horizontal_frontporch_byte=32;
    params->dsi.rgb_byte=(480*3+6);

    params->dsi.horizontal_sync_active_word_count=20;
    params->dsi.horizontal_backporch_word_count=200;
    params->dsi.horizontal_frontporch_word_count=200;

    //edit by Magnum 2012-7-4 try to solve lcd not updating....
    params->dsi.HS_TRAIL=0x08;
    params->dsi.HS_ZERO=0x07;
    params->dsi.HS_PRPR=0x03;
    params->dsi.LPX=0x04;

    params->dsi.TA_SACK=0x01;
    params->dsi.TA_GET=0x14;
    params->dsi.TA_SURE=0x06;
    params->dsi.TA_GO=0x10;

    params->dsi.CLK_TRAIL=0x04;
    params->dsi.CLK_ZERO=0x0f;
    params->dsi.LPX_WAIT=0x01;
    params->dsi.CONT_DET=0x00;

    params->dsi.CLK_HS_PRPR=0x04;
    //params->dsi.pll_div1=0x1a;
    //params->dsi.pll_div2=1;

    /*
    // Bit rate calculation
    #ifdef TINNO_INT
        params->dsi.pll_div1=38;////26//34;     // fref=26MHz, fvco=fref*(div1+1)   (div1=0~63, fvco=500MHZ~1GHz)
    #else
        params->dsi.pll_div1=22;//0x1A;////26//34;      // fref=26MHz, fvco=fref*(div1+1)   (div1=0~63, fvco=500MHZ~1GHz)
    #endif
        params->dsi.pll_div2=1;         // div2=0~15: fout=fvo/(2*div2)
    */
//tinnotp
    ////1 Every lane speed
    //params->dsi.pll_div1=1;       // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    //params->dsi.pll_div2=1;       // div2=0,1,2,3;div1_real=1,2,4,4
    //params->dsi.fbk_div =22;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

    params->dsi.pll_div1=0;
    params->dsi.pll_div2=1;
//        params->dsi.fbk_div =13;
    params->dsi.fbk_div =16;
//tinnotp
}

//x:可以是4字节
#define WRITE_INT(x) do{\
                            unsigned int x_t=0;\
                            unsigned int data_array[16];\
                            x_t=(unsigned int)x;\
                            data_array[0] = 0x00023902;\
                            data_array[1] = x_t;\
                            dsi_set_cmdq(data_array, 2, 1);\
                        }while(0);
//x:可以是2字节
#define WRITE_CHAR(x) do{\
                        unsigned int x_t=0;\
                        unsigned int data_array[16];\
                        x_t=(unsigned int)x;\
                        data_array[0] = ((((unsigned int)x_t)<<16)|0x0500);\
                        dsi_set_cmdq(data_array, 1, 1);\
                    }while(0);

#define LCD_ILI9806H_CMD(x)     WRITE_CHAR(x)
#define LCD_ILI9806H_INDEX(x)     WRITE_CHAR(x)

static struct LCM_setting_table lcm_initialization_setting[] =
{
    {0xFF,3,{0xff,0x98,0x26}},

    {
        0xBC,26,{
            0x21,0x06,0x20,0x02,0x01,0x01,0x9A,0x03,0x06,0x00,
            0x00,0x00,0x03,0x00,0xff,0xe0,0x00,0x00,0x00,0x00,
            0x08,0x00,0x00,0x00,0x00,0x00
        }
    },

    {0xBD,8,{0x01,0x23,0x45,0x67,0x01,0x23,0x45,0x67}},

    {
        0xBE,17,{
            0x12,0x22,0x22,0x22,0x87,0x96,0xBA,0xAB,0xDC,0xCD,
            0x78,0x69,0x22,0x22,0x22,0x22,0x22
        }
    },

    {0x3A,1,{0x77}},

    {0xFA,5,{0x08,0x00,0x00,0x02,0x08}},

    {0xB1,3,{0x00,0x62,0x03}},

    {0xB4,3,{0x02,0x02,0x02}}, //0x00,0x00,0x00,nei

    {0xC1,3,{0x15,0x78,0x6A}},

    {0xC7,4,{0x62,0x00,0x51,0x00}},



    {0x35,1,{0x00}},//TE on

    {0xF7,1,{0x02}},

    {0xF2,6,{0x05,0x08,0x08,0x8A,0x07,0x04}},

    {
        0xE0,16,{
            0x00,0x06,0x15,0x11,0x12,0x1C,0xCA,0x08,0x02,0x08,
            0x02,0x0D,0x0B,0x36,0x31,0x00
        }
    }, /*Positive Gamma Control*/

    {
        0xE1,16,{
            0x00,0x05,0x0D,0x10,0x12,0x16,0x79,0x07,0x05,0x09,
            0x07,0x0C,0x0B,0x21,0x1B,0x00
        }
    }, /*Negative Gamma Control*/

    {0xF9,5,{0x00,0xda,0x80,0x70,0xc0}},

    {0x44,2,{0x01,0x00}},    //retain  for Tear Scan //Write Tear Scan Line  //

    {0x11,0,{}},

    {REGFLAG_DELAY, 120, {}},

    {0x29,0,{}},

    {REGFLAG_DELAY, 50, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
    // Display off sequence
    {0x28, 0, {}},
    {REGFLAG_DELAY, 10, {}},
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] =
{
    // Sleep Out
    {0x11, 0, {}},
    {REGFLAG_DELAY, 10, {}},
    {0x29, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void ILI9806H_IV0397_Initial (void)
{
    // VCI=2.8V  CPU_Mode_480*800
    //************* Reset LCD Driver ****************//
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
    //************* Start Initial Sequence **********//
    push_table_tkc(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

static void LCD_Enter_Standby_ILI9806H(void)
{
    push_table_tkc(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}

static void LCD_Exit_Standby_ILI9806H(void)
{
    push_table_tkc(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}
static void lcm_init(void)
{
    LCM_DBG("[tcp_test_esd:ILI9806H] -- lcm_init start=%d\n",sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table));
    LCM_DBG();
    ILI9806H_IV0397_Initial();
    LCM_DBG("[tcp_test:ILI9806H] -- lcm_init end\n");
}


static void lcm_suspend(void)
{
    LCM_DBG();
    LCM_DBG("[tcp_test:ILI9806H] -- lcm_suspend\n");
    LCD_Enter_Standby_ILI9806H();
}

static unsigned int check_cmp_id_test(void)
{
    unsigned int id=0;
    unsigned char buffer[4];
    unsigned int array[16];

    /*
    //reset before read id
    array[0]=0x00043902;
    array[1]=0x010980ff;
    array[2]=0x80001500;
    array[3]=0x00033902;
    array[4]=0x010980ff;
    dsi_set_cmdq(array, 5, 1);
    MDELAY(10);

    array[0] = 0x02001500;
    dsi_set_cmdq(array, 1, 1);
           */
    /*
    array[0] = 0x00043700;// set return byte number
    dsi_set_cmdq(array, 1, 1);
           */
    //MCU和CMD MIPI接口直接读D3
    read_reg_V2(0xD3, &buffer, 4);
    //id = buffer[0]<<8 |buffer[1];
    id = buffer[1]<<8 |buffer[2]; //0x00,0x00,0x98,0x26//0x00,0x98,0x26,0x00

    LCM_DBG("[tcp_test]ILI9806H LCM_ID=0x%x , 0x%x , 0x%x , 0x%x , 0x%x \n",buffer[0],buffer[1],buffer[2],buffer[3],id);
    return (id == LCM_ID_ILI9806H)?1:0;

}

static void lcm_resume(void)
{
    // Work around for Novatek driver IC. If it entered ULP mode, it must be reset before resume.
    lcm_init();
    LCM_DBG();
    LCM_DBG("[tcp_test:ILI9806H] -- lcm_resume\n");
    //LCD_Exit_Standby_ILI9806H();
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    //  LCM_DBG("UPDATE-UPDATE-UPDATE");
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

#if 0
    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    data_array[3]= 0x00053902;
    data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[5]= (y1_LSB);
    data_array[6]= 0x002c3909;
    dsi_set_cmdq(&data_array, 7, 0);
#else
    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
#endif
//LCM_DBG("tcp_test:lcm_update\n");
}

// ---------------------------------------------------------------------------
//  Get LCM ID Information glimon
// ---------------------------------------------------------------------------
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[2];
    unsigned int array[16];

    int Channel = 1;
    int data[4];
    int adcVol = 0;

    int res = IMM_GetOneChannelValue(Channel, data, 0);

    //reset before read id
    //SET_RESET_PIN(1);
    //SET_RESET_PIN(0);
    //MDELAY(25);
    //SET_RESET_PIN(1);
    //MDELAY(50);

    //if(1 == check_cmp_id_test())
    {
        adcVol = data[0]*1000+data[1]*10;
#ifdef BUILD_LK
        printf("LINE=%d %s, res=%d, lli9806h_adcVol = %d \n", __LINE__,__func__, res, adcVol);
#else
        printk("LINE=%d %s, id1 = 0x%08x\n",__LINE__,__func__, id);
#endif

        if(adcVol > 550 && adcVol < 1300)
        {
            return 1;
        }
    }

    return 0;
}

#if (TINNO_INT_ESD)
static unsigned int check_cmp_id(void)
{
    unsigned int id=0;
    unsigned char buffer[4];
    unsigned int array[16];

    //reset before read id
    array[0]=0x00043902;
    array[1]=0x010980ff;
    array[2]=0x80001500;
    array[3]=0x00033902;
    array[4]=0x010980ff;
    dsi_set_cmdq(array, 5, 1);
    MDELAY(10);

    array[0] = 0x02001500;
    dsi_set_cmdq(array, 1, 1);

    array[0] = 0x00043700;// set return byte number
    dsi_set_cmdq(array, 1, 1);

    read_reg_V2(0xD3, &buffer, 4);

    id = buffer[0]<<8 |buffer[1];

    //LCM_DBG("[LSQ] -- ILI9806H 0x%x , 0x%x , 0x%x \n",buffer[0],buffer[1],id);
    LCM_DBG("[tcp_test] -- ILI9806H =0x%x , 0x%x , 0x%x , 0x%x , 0x%x \n",buffer[0],buffer[1],buffer[2],buffer[3],id);

    return (id == LCM_ID_ILI9806H)?0:1;

}

static unsigned int check_esd(void)
{
    unsigned char buffer_1[2];
    unsigned char buffer_2[2];
    unsigned char buffer_3[2];
    unsigned char buffer_4[2];
    unsigned int array[16];

    //reset before read id
    array[0]=0x00043902;
    array[1]=0x010980ff;
    array[2]=0x80001500;
    array[3]=0x00033902;

    array[4]=0x010980ff;
    dsi_set_cmdq(array, 5, 1);
    MDELAY(10);

    array[0] = 0x02001500;
    dsi_set_cmdq(array, 1, 1);

    array[0] = 0x00023700;// set return byte number
    dsi_set_cmdq(array, 1, 1);
    //

    read_reg_V2(0x0a, &buffer_1, 1);
    return (buffer_1[0] == 0x9c)?0:1;
    read_reg_V2(0x0b, &buffer_2, 1);
    return (buffer_2[0] == 0x00)?0:1;
    read_reg_V2(0x0c, &buffer_3, 1);
    return (buffer_3[0] == 0x07)?0:1;
    read_reg_V2(0x0d, &buffer_4, 1);
    return (buffer_4[0] == 0x00)?0:1;

    //LCM_DBG("tcp_test_esd:lcm_esd_check= 0x%x , 0x%x , 0x%x , 0x%x\n",buffer_1[0],buffer_2[0],buffer_3[0],buffer_4[0]);
    //return ((buffer_1[0] == 0x9c)&&(buffer_2[0]==0x00)&&(buffer_3[0] == 0x07)&&(buffer_4[0]==0x00))?0:1;
}
static unsigned int lcm_esd_check(void)
{
    unsigned int ret=0; //normal:0;abnormal:1

#ifndef BUILD_LK//BUILD_UBOOT
    unsigned int retry=2;
    while(retry--)
    {
        //ret = check_cmp_id();      //compare id mode
        ret = check_esd();
        if(ret==0)
            break;
        else
            MDELAY(10);
    }

    if(lcm_esd_test==20)
    {
        lcm_esd_test = 0;
        LCM_DBG("tcp_test_esd:esdddddddddddddddhappen\n");
        return 1;   //esd issue happened
    }
    else
    {
        ////lcm_esd_test++;
    }

#endif
    //LCM_DBG("tcp_test_esd:lcm_esd_check=%d\n",ret);
    if(ret)
        LCM_DBG("tcp_test_esd:esdddddddddddddddhappen 01\n");

    return ret;
}
static unsigned int lcm_esd_recover(void)
{
    unsigned char para = 0;

    LCM_DBG("tcp_test_esd:lcm_esd_recover start\n");
    lcm_init();
    LCM_DBG("tcp_test_esd:lcm_esd_recover end\n");
    return 1;

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(100);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(5);
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

    return 1;
}
#endif
//tinnohd
static struct LCM_setting_table lcm_backlight_level_setting[] =
{
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_setbacklight(unsigned int level)
{
    // Refresh value of backlight level.
    lcm_backlight_level_setting[0].para_list[0] = level;

    push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_setpwm(unsigned int divider)
{
    // TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
    unsigned int pwm_clk = 23706 / (1<<divider);
    return pwm_clk;
}
//tinnohd
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806h_dsi_lcm_drv_txd =
{
    .name           = "ili9806h_txd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
//tinnohd
    /*
        .set_backlight  = lcm_setbacklight,
        .set_pwm        = lcm_setpwm,
        .get_pwm        = lcm_getpwm,
    */
//tinnohd

//TN801031 modified 2013.05.27
#if (TINNO_INT_ESD)
    .esd_check   = lcm_esd_check,
    .esd_recover   = lcm_esd_recover,//lcm_init,//lcm_esd_recover,
#else
    .esd_check   = 0,//NULL,
    .esd_recover   = 0,//NULL,//lcm_init,//lcm_esd_recover,
#endif
//TN801031 modified 2013.05.27

    .compare_id     = lcm_compare_id,
};
