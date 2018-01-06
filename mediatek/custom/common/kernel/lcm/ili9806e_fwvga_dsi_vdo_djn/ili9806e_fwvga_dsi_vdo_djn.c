#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)
#define ILI9806E_LCM_ID                                     (0x0604)

#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0xFD   // END OF REGISTERS MARKER


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                  lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)              lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
//****************************************************************************//
//****************************** Page 1 Command ******************************//
//****************************************************************************//
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1
    {0x08,1,{0x10}},                 // output SDA
    {0x21,1,{0x01}},                 // DE = 1 Active
    {0x30,1,{0x01}},                 // 480 X 854
    {0x31,1,{0x00}},                 // Inversion
    {0x40,1,{0x10}},                 // BT
    {0x41,1,{0x33}},                 // DVDDH DVDDL clamp
    {0x42,1,{0x02}},                 // VGH/VGL 01
    {0x43,1,{0x09}},                 // VGH_CLAMP 0FF ;
    {0x44,1,{0x09}},                  // VGL_CLAMP OFF ;
    {0x50,1,{0x80}},                   //VGMP
    {0x51,1,{0x80}},                   //VGMN
    {0x52,1,{0x00}},                   //Flicker
    {0x53,1,{0x37}},                  //Flicker
    {0x60,1,{0x0a}},                  // SDTI
    {0x61,1,{0x00}},                // CRTI
    {0x62,1,{0x08}},                 // EQTI
    {0x63,1,{0x00}},                 // PCTI
    {0x57,1,{0x50}},

//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
    {0xA0,1,{0x00}},  // Gamma 0
    {0xA1,1,{0x01}},  // Gamma 4
    {0xA2,1,{0x0B}},  // Gamma 8
    {0xA3,1,{0x0D}},  // Gamma 16
    {0xA4,1,{0x09}},  // Gamma 24
    {0xA5,1,{0x14}},  // Gamma 52
    {0xA6,1,{0x0A}},  // Gamma 80
    {0xA7,1,{0x09}}, // Gamma 108
    {0xA8,1,{0x03}},  // Gamma 147
    {0xA9,1,{0x0A}},  // Gamma 175
    {0xAA,1,{0x0C}},  // Gamma 203
    {0xAB,1,{0x0B}},  // Gamma 231
    {0xAC,1,{0x17}}, // Gamma 239
    {0xAD,1,{0x2C}},  // Gamma 247
    {0xAE,1,{0x27}},  // Gamma 251
    {0xAF,1,{0x00}},  // Gamma 255

///==============Nagitive
    {0xC0,1,{0x00}},  // Gamma 0
    {0xC1,1,{0x03}},  // Gamma 4
    {0xC2,1,{0x0A}},  // Gamma 8
    {0xC3,1,{0x11}},  // Gamma 16
    {0xC4,1,{0x09}},  // Gamma 24
    {0xC5,1,{0x19}},  // Gamma 52
    {0xC6,1,{0x09}},  // Gamma 80
    {0xC7,1,{0x09}},  // Gamma 108
    {0xC8,1,{0x05}},  // Gamma 147
    {0xC9,1,{0x08}},  // Gamma 175
    {0xCA,1,{0x02}},  // Gamma 203
    {0xCB,1,{0x01}},  // Gamma 231
    {0xCC,1,{0x00}},  // Gamma 239
    {0xCD,1,{0x2B}},  // Gamma 247
    {0xCE,1,{0x26}},  // Gamma 251
    {0xCF,1,{0x00}},  // Gamma 255

//****************************************************************************//
//****************************** Page 6 Command ******************************//
//****************************************************************************//
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
    {0x00,1,{0x21}},
    {0x01,1,{0x0A}},
    {0x02,1,{0x00}},
    {0x03,1,{0x00}},
    {0x04,1,{0x01}},
    {0x05,1,{0x01}},
    {0x06,1,{0x80}},
    {0x07,1,{0x06}},
    {0x08,1,{0x01}},
    {0x09,1,{0x80}},
    {0x0A,1,{0x00}},
    {0x0B,1,{0x00}},
    {0x0C,1,{0x0a}},
    {0x0D,1,{0x0a}},
    {0x0E,1,{0x00}},
    {0x0F,1,{0x00}},
    {0x10,1,{0xF0}},
    {0x11,1,{0xF4}},
    {0x12,1,{0x04}},
    {0x13,1,{0x00}},
    {0x14,1,{0x00}},
    {0x15,1,{0xC0}},
    {0x16,1,{0x08}},
    {0x17,1,{0x00}},
    {0x18,1,{0x00}},
    {0x19,1,{0x00}},
    {0x1A,1,{0x00}},
    {0x1B,1,{0x00}},
    {0x1C,1,{0x00}},
    {0x1D,1,{0x00}},
    {0x20,1,{0x01}},
    {0x21,1,{0x23}},
    {0x22,1,{0x45}},
    {0x23,1,{0x67}},
    {0x24,1,{0x01}},
    {0x25,1,{0x23}},
    {0x26,1,{0x45}},
    {0x27,1,{0x67}},
    {0x30,1,{0x01}},
    {0x31,1,{0x11}},    //GOUT1
    {0x32,1,{0x00}},    //GOUT2
    {0x33,1,{0xEE}},    //GOUT3
    {0x34,1,{0xFF}},    //GOUT4
    {0x35,1,{0xBB}},    //GOUT5
    {0x36,1,{0xCA}},    //GOUT6
    {0x37,1,{0xDD}},    //GOUT7
    {0x38,1,{0xAC}},    //GOUT8
    {0x39,1,{0x76}},    //GOUT9
    {0x3A,1,{0x67}},    //GOUT10
    {0x3B,1,{0x22}},    //GOUT11
    {0x3C,1,{0x22}},    //GOUT12
    {0x3D,1,{0x22}},    //GOUT13
    {0x3E,1,{0x22}},    //GOUT14
    {0x3F,1,{0x22}},
    {0x40,1,{0x22}},
    {0x52,1,{0x10}},
    {0x53,1,{0x10}}, // VGLO tie VGL_REG

//****************************************************************************//
//****************************** Page 7 Command ******************************//
//****************************************************************************//
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
    {0x17,1,{0x22}},  //
    {0x02,1,{0x77}},  //
    {0xE1,1,{0x79}},  //
//****************************************************************************//
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 0
    {0x35,1,{0x00}},                 // TE on
    {0x11,1,{0x00}},                 // Sleep-Out
    {REGFLAG_DELAY, 120, {}},
    {0x29,1,{0x00}},                 // Display on
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] =
{
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                MDELAY(10);
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

    params->type = LCM_TYPE_DSI;
    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dsi.mode = BURST_VDO_MODE;
    params->dsi.LANE_NUM = LCM_TWO_LANE;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active = 3;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 2;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 10;
    params->dsi.horizontal_backporch = 50;
    params->dsi.horizontal_frontporch = 50;
    params->dsi.horizontal_blanking_pixel = 60;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
    //params->dsi.ssc_range = 8;

    params->dsi.PLL_CLOCK = 208;
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
}

static unsigned int lcm_compare_id(void);

static void lcm_resume(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    //lcm_compare_id();
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#else
    printk("lcm_setbacklight = %d\n", level);
#endif

    if(level > 255)
        level = 255;

    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
    dsi_set_cmdq(data_array, 2, 1);
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    /// please notice: the max return packet size is 1
    /// if you want to change it, you can refer to the following marked code
    /// but read_reg currently only support read no more than 4 bytes....
    /// if you need to read more, please let BinHan knows.
    /*
            unsigned int data_array[16];
            unsigned int max_return_size = 1;

            data_array[0]= 0x00003700 | (max_return_size << 16);

            dsi_set_cmdq(&data_array, 1, 1);
    */

    if(read_reg(0xB6) == 0x42)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
#endif
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();

    return TRUE;
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_midd=0;
    char id_low=0;
    int id=0;
    int data[4];
    int adcVol=0;
    //int res = IMM_GetOneChannelValue( 0 , data , 0 );
    //Do reset here
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(25);
    SET_RESET_PIN(1);
    MDELAY(120);
    array[0]=0x00063902;
    array[1]=0x0698ffff;
    array[2]=0x00000104;
    dsi_set_cmdq(array, 3, 1);
    MDELAY(10);

    array[0]=0x00023700;//0x00023700;
    dsi_set_cmdq(array, 1, 1);
    //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.

    read_reg_v2(0x00, buffer,1);
    id_high = buffer[0]; ///////////////////////0x98

    array[0]=0x00043700;//0x00023700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x01, buffer,1);
    id_midd = buffer[0]; ///////////////////////0x06

    array[0]=0x00043700;//0x00023700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x02, buffer,1);
    id_low = buffer[0]; ////////////////////////0x04

    id = (id_midd << 8) | id_low;

#if defined(BUILD_UBOOT) || defined(BUILD_LK)
    printf("ILI9806: %s, id = 0x%x 0x%x 0x%x 0x%x 0x%x\n", __func__, id_high,id_midd,id_low,buffer[4]);//buffer[0],buffer[1],buffer[2],buffer[3],buffer[4] );
#else
    printk("ILI9806: %s, id = 0x%08x\n", __func__, id);
#endif

    if(ILI9806E_LCM_ID == id)
    {
        int res = IMM_GetOneChannelValue( 0 , data , 0 );
        adcVol=data[0]*1000+data[1]*10;

#ifdef BUILD_LK
        printf("LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol);
#else
        printk("LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol);
#endif

        if(adcVol >= 500 && adcVol < 1000)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
    //return (ILI9806E_LCM_ID == id)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_fwvga_dsi_vdo_djn_drv =
{
    .name           = "ili9806e_fwvga_dsi_vdo_djn",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    //.set_backlight  = lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .compare_id     = lcm_compare_id,
    //.esd_check      = lcm_esd_check,
    //.esd_recover    = lcm_esd_recover,
};

