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

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

#define REGFLAG_DELAY                                       0xFC
#define REGFLAG_END_OF_TABLE                                0xFE   // END OF REGISTERS MARKER

unsigned int vvv = 0x00008000;

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
#define ILI9806E_LCM_ID                                     (0x0604)

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
    //2014.08.18 ls

    {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1
    {0x08,1,{0x10 }},                 // output SDA
    {0x21,1,{0x01 }},                 // DE = 1 Active
    {0x40,1,{0x10 }},                // DDVDH/L BT 17 -10 5.94 -5.47 -2.3 1.6 43.4 21.88 0.17 15
    {0x41,1,{0x75 }},             // DDVDH/L CLAMP
    {0x30,1,{0x01 }},          // 480 X 854
    {0x31,1,{0x00 }},       // 2-dot Inversion
    {0x50,1,{0x84 }},   // VGMP 4.5 90 78
    {0x51,1,{0x84 }},   // VGMN 4.5 90 78
    {0x57,1,{0x50 }},               // LOW VOLTAGE DETECTION
    {0x60,1,{0x07 }},               // SDTI
    {0x61,1,{0x04 }},              // CRTI 00
    {0x62,1,{0x06 }},               // EQTI 07
    {0x63,1,{0x02 }},              // PCTI  00
    {0x42,1,{0x02 }},               // VGH/VGL
    {0x44,1,{0x09 }},               // VGH/VGL

    {0x56,1,{0x00 }},  // 11==> 00        peter wang for otp
    {0x52,1,{0x00 }},                  //Flicker
    {0x53,1,{0x5e }},                  //Flicker

    /*++++++++++++++++++ Gamma Setting ++++++++++++++++++*/
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1
    {0xA0,1,{0x00 }},  // Gamma 0
    {0xA1,1,{0x0f }}, // Gamma 4
    {0xA2,1,{0x2a }}, // Gamma 8
    {0xA3,1,{0x0f }}, // Gamma 16
    {0xA4,1,{0x0c }}, // Gamma 24
    {0xA5,1,{0x16 }}, // Gamma 52
    {0xA6,1,{0x0a }},// Gamma 80
    {0xA7,1,{0x0b }}, // Gamma 108
    {0xA8,1,{0x03 }}, // Gamma 147
    {0xA9,1,{0x0a }}, // Gamma 175
    {0xAA,1,{0x0c }}, // Gamma 203  // 05
    {0xAB,1,{0x08 }}, // Gamma 231
    {0xAC,1,{0x0d }}, // Gamma 239
    {0xAD,1,{0x22 }}, // Gamma 247
    {0xAE,1,{0x1d }}, // Gamma 251  // 29
    {0xAF,1,{0x00 }}, // Gamma 255
    ///==============Nagitive
    {0xC0,1,{0x00 }}, // Gamma 0
    {0xC1,1,{0x0f }}, // Gamma 4
    {0xC2,1,{0x1b }}, // Gamma 8
    {0xC3,1,{0x0d }}, // Gamma 16
    {0xC4,1,{0x0a }}, // Gamma 24
    {0xC5,1,{0x16 }}, // Gamma 52
    {0xC6,1,{0x08 }}, // Gamma 80
    {0xC7,1,{0x07 }}, // Gamma 108
    {0xC8,1,{0x06 }}, // Gamma 147
    {0xC9,1,{0x09 }}, // Gamma 175
    {0xCA,1,{0x03 }}, // Gamma 203   //07
    {0xCB,1,{0x01 }}, // Gamma 231
    {0xCC,1,{0x05 }}, // Gamma 239
    {0xCD,1,{0x26 }}, // Gamma 247
    {0xCE,1,{0x22 }}, // Gamma 251  // 1a
    {0xCF,1,{0x00 }}, // Gamma 255



    //****************************************************************************//
    //****************************** Page 6 Command ******************************//
    //****************************************************************************//
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},    // Change to Page 6
    {0x00,1,{0x20}},
    {0x01,1,{0x06}},
    {0x02,1,{0x20}},
    {0x03,1,{0x02}},
    {0x04,1,{0x01}},
    {0x05,1,{0x01}},
    {0x06,1,{0x98}},
    {0x07,1,{0x04}},
    {0x08,1,{0x05}},
    {0x09,1,{0x00}},
    {0x0A,1,{0x00}},
    {0x0B,1,{0x00}},
    {0x0C,1,{0x01}},
    {0x0D,1,{0x01}},
    {0x0E,1,{0x00}},
    {0x0F,1,{0x00}},
    {0x10,1,{0xFF}},
    {0x11,1,{0xF2}},
    {0x12,1,{0x01}},
    {0x13,1,{0x00}},
    {0x14,1,{0x00}},
    {0x15,1,{0x43}},
    {0x16,1,{0x0B}},
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
    {0x30,1,{0x02}},
    {0x31,1,{0x22}},
    {0x32,1,{0x22}},
    {0x33,1,{0x22}},
    {0x34,1,{0x97}},//77
    {0x35,1,{0x86}},//66
    {0x36,1,{0xdA}},//aa
    {0x37,1,{0xaB}},//bb
    {0x38,1,{0xbC}},//cc
    {0x39,1,{0xcD}},//dd
    {0x3A,1,{0x68}},//88
    {0x3B,1,{0x79}},//99
    {0x3C,1,{0x22}},
    {0x3D,1,{0x22}},
    {0x3E,1,{0x22}},
    {0x3F,1,{0x22}},
    {0x40,1,{0x22}},
    {0x53,1,{0x10}},
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},   // Change to Page 7
    {0x17,1,{0x22}},
    {0x18,1,{0x1d}},
    {0x02,1,{0x77}},
    {0xe1,1,{0x79}},
    //****************************************************************************//
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},    // Change to Page 0
    {REGFLAG_DELAY, 20, {}},
    {0x11,1,{0x00}},                // Sleep-Out
    {REGFLAG_DELAY, 120, {}},
    {0x29,1,{0x00}},               // Display On
    {REGFLAG_DELAY, 50, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] =
{
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
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
                //MDELAY(10);
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

#if defined(LCM_DSI_CMD_MODE)
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
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                = 7; // 3 2 18      4 6 6
    params->dsi.vertical_backporch                  = 5; // 7 6 24
    params->dsi.vertical_frontporch                 = 12; // 6, 6 24
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 10; // 12 12 32 32
    params->dsi.horizontal_backporch                = 80; //132 92 52 60
    params->dsi.horizontal_frontporch               = 80; //132 92 60 60
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    // Bit rate calculation
		params->dsi.PLL_CLOCK=208;
}



static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    //init_lcm_registers();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#if defined(BUILD_LK)
    //disp_bls_set_backlight(0);
    //MDELAY(20);
    //disp_bls_set_backlight(255);
#endif
}

extern int disp_bls_set_backlight(unsigned int level);
static void lcm_suspend(void)
{
    unsigned int data_array[16];
#if defined(BUILD_LK)
    printf("%s\n", __func__);
#elif defined(BUILD_UBOOT)
    printf("%s\n", __func__);
#else
    printk(" 9806e lcm_suspend");
#endif

    disp_bls_set_backlight(0);

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);


 //   SET_RESET_PIN(0);
//    data_array[0] = 0x014F1500;
//    dsi_set_cmdq(data_array, 1, 1);
//    MDELAY(40);
}

//unsigned int vcomff=0x00004000;

static void lcm_resume(void)
{

    unsigned int data_array[16];
    unsigned int data;
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(15);
    SET_RESET_PIN(1);
    MDELAY(60);
    // init_lcm_registers();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
/*
    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000104;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00023902;
    data_array[1]=vcomff +0x00000053;//40
    dsi_set_cmdq(data_array, 2, 1);
    vcomff=vcomff+0x00000200;
    MDELAY(20);
*/

}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#elif defined(BUILD_UBOOT)
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

////ESD start ///

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_UBOOT
    unsigned char buffer[2];
    int array[4];
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
    unsigned int data_array[16];
    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000004;
    dsi_set_cmdq(data_array, 3, 1);

    array[0]=0x00043700;//0x00023700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0x0A, buffer, 4);
#if defined(BUILD_LK)
    printf("%s\n", __func__);
#elif defined(BUILD_UBOOT)
    printf("%s\n", __func__);
#else
    printk(" 9806e lcm_esd_check:%x",buffer[0]);
#endif
    //if(read_reg(0x0A) == 0x9C)
    if(buffer[0] == 0x9C)
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
//  unsigned char para = 0;
#if defined(BUILD_LK)
    printf("%s\n", __func__);
#elif defined(BUILD_UBOOT)
    printf("%s\n", __func__);
#else
    printk(" 9806e lcm_esd_recover");
#endif
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(20);

    MDELAY(20);
    lcm_resume();
    MDELAY(20);
//    dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE
//    MDELAY(10);
    return TRUE;
}
/////ESD end///

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

        if(adcVol >= 0 && adcVol < 100)
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
LCM_DRIVER ili9806e_fwvga_dsi_vdo_tkc_drv =
{
    .name           = "ili9806e_fwvga_dsi_vdo_tkc",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
//    .set_backlight    = lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
//ESD start ///
    // .esd_check   = lcm_esd_check,
    //.esd_recover   = lcm_esd_recover,
//ESD end ///
    .compare_id    = lcm_compare_id,
};
