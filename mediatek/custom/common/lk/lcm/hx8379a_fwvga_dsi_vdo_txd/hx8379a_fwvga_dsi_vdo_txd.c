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
#define FALSE (0)
#define TRUE (1)
#define HX8379A_LCM_ID                                     (0x8379)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0x00   // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                  lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)              lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

//#define LCM_DSI_CMD_MODE

struct LCM_setting_table
{
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//static kal_uint16 Vcom=0x98;
static struct LCM_setting_table lcm_initialization_setting[] =
{

    /*
    Note :

    Data ID will depends on the following rule.

        count of parameters > 1 => Data ID = 0x39
        count of parameters = 1 => Data ID = 0x15
        count of parameters = 0 => Data ID = 0x05

    Structure Format :

    {DCS command, count of parameters, {parameter list}}
    {REGFLAG_DELAY, milliseconds of time, {}},

    ...

    Setting ending by predefined flag

    {REGFLAG_END_OF_TABLE, 0x00, {}}
    */

    //yixuhong 20140625 add,hx8379a+boe4.5_Gamma2.2_20140620
    {0xB9,3,{0xFF,0x83,0x79}},
    {0xBA,2,{0x51,0x93}},
    {0xB1,19,{0x00,0x50,0x44,0xE8,0x99,0x08,0x11,0x11,0x11,0x2F,0x37,0x9E,0x1E,0x42,0x0b,0x6E,0xF1,0x00,0xE6}},
    {0xB2,13,{0x00,0x00,0xFE,0x09,0x04,0x19,0x42,0x00,0xff,0x0B,0x11,0x19,0x20}},
    {0xB4,31,{0x80,0x08,0x00,0x32,0x10,0x05,0x32,0x13,0x70,0x32,0x10,0x08,0x37,0x00,0x40,0x06,0x13,0x04,0x40,0x08,0x38,0x34,0x04,0x00,0x40,0x08,0x28,0x08,0x30,0x30,0x04}},
    {0xD5,47,{0x00,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0x10,0x88,0x99,0x10,0x32,0x10,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,0x23,0x10,0x99,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xCC,1,{0x02}},
    {0xE0,35,{0x79,0x00,0x0C,0x13,0x2C,0x2D,0x3F,0x42,0x51,0x08,0x0C,0x10,0x15,0x19,0x15,0x15,0x13,0x17,0x00,0x0C,0x13,0x2C,0x2D,0x3F,0x42,0x51,0x08,0x0C,0x10,0x15,0x19,0x15,0x15,0x13,0x17}},
    {0xB6,4,{0x00,0x9b,0x00,0x9b}},
    {0xc6,2,{0x00,0x02,}},
    {0xB9,1,{0x00}},
    {0x11,0,{}}, //Sleep Out
    {REGFLAG_DELAY, 120, {}},
     
    {0x29,0,{}}, //Display On
    //{REGFLAG_DELAY, 20, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_set_window[] =
{
    {0x2A,  4,  {0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
    {0x2B,  4,  {0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] =
{
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 130, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] =
{
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_compare_id_setting[] =
{
    // Display off sequence
    {0xB9,  3,  {0xFF, 0x83, 0x79}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    // {0xC3, 1, {0xFF}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};



void push_table_txd(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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

    params->dsi.vertical_sync_active                = 5; // 3 2 18   7   4 6 6
    params->dsi.vertical_backporch                  = 7; // 7 6 24 5
    params->dsi.vertical_frontporch                 = 6; // 6, 6 24 6
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 10; // 12 12 32 32 33
    params->dsi.horizontal_backporch                = 50; //132 92 52 60 32
    params->dsi.horizontal_frontporch               = 50; //132 92 60 60 32
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    // Bit rate calculation
    //params->dsi.pll_div1=1;     // fref=26MHz, fvco=fref*(div1+1)   (div1=0~63, fvco=500MHZ~1GHz)
    //params->dsi.pll_div2=1;         // div2=0~15: fout=fvo/(2*div2)
    //params->dsi.fbk_div=31; //27 24 26;          // 21(default) 22 henxian 23 baiping
    params->dsi.PLL_CLOCK=200;
}

static void lcm_reset(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
}

static void lcm_init(void)
{
    lcm_reset();
    push_table_txd(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
    unsigned int array[16];

    array[0] = 0x00280500;//
    dsi_set_cmdq(&array, 1, 1);
    MDELAY(10);

    array[0] = 0x00100500;//             10！
    dsi_set_cmdq(&array, 1, 1);
    MDELAY(120);

    lcm_reset();

}

//unsigned int vcomf=0xa0;
static void lcm_resume(void)
{
    lcm_init();

    // push_table_txd(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
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

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    data_array[3]= 0x00053902;
    data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[5]= (y1_LSB);
    data_array[6]= 0x002c3909;

    dsi_set_cmdq(&data_array, 7, 0);

}

static void lcm_setbacklight(unsigned int level)
{
    // Refresh value of backlight level.
    lcm_backlight_level_setting[0].para_list[0] = level;

    push_table_txd(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
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

static unsigned int lcm_esd_check(void)
{

    int id,id1,id2,id3;
    unsigned char buffer[4];
    unsigned int array[16];
    unsigned char lcd_id = 0;
    id=id1=id2=id3=0;

    array[0] = 0x00290500;// return byte number 4
    dsi_set_cmdq(&array, 1, 1);

    array[0] = 0x00043700;// return byte number 4
    dsi_set_cmdq(&array, 1, 1);
    //MDELAY(10);
    read_reg_v2(0x09, buffer, 4);
    id = buffer[0];
    id1 = buffer[1];
    id2= buffer[2];
    id3 = buffer[3];
#ifdef BUILD_LK
    //  printf("%s, LINE = %d,id=%x,id1=%x,id2=%x,id3=%x\n", __func__,__LINE__,id,id1,id2,id3);
#else
    printk("%s, LINE = %d,id=%x,id1=%x,id2=%x,id3=%x\n", __func__,__LINE__,id,id1,id2,id3);
#endif

    //if(0x80==id&&0x73==id1&&0x00==id2&&0x60==id3)
    if(0x80==id&&0x73==id1)
        return FALSE;
    else
        return TRUE;
}


static unsigned int lcm_esd_recover(void)
{
    unsigned char para = 0;
#ifdef BUILD_LK
    printf("%s, LINE = %d\n", __func__,__LINE__);
#else
    printk("%s, LINE = %d\n", __func__,__LINE__);
#endif

    lcm_reset();
    push_table_txd(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

    return TRUE;

}

// ---------------------------------------------------------------------------
//  Get LCM ID Information
// ---------------------------------------------------------------------------
static unsigned int lcm_compare_id()
{
    unsigned int id=0,id_high=0,id_low=0;
    unsigned char buffer[2];
    unsigned int array[16];

    //Do reset here
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(25);

    SET_RESET_PIN(1);
    MDELAY(50);

    //  push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
    array[0]=0x00043902;
    array[1]=0x7983FFB9;
    dsi_set_cmdq(array, 2, 1);

    array[0] = 0x00063700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xC3, buffer, 5);
    id_high = buffer[0]; //id=0x83
    id_low = buffer[1];//id1=0x79
    //id2 = buffer[2];//id2=0x0c
    id = (id_high << 8) | id_low;

#if defined(BUILD_UBOOT) || defined(BUILD_LK)
    printf("HX8379A: %s, id = 0x%x 0x%x 0x%x \n", __func__, buffer[0],buffer[1],buffer[2]);
#else
    printk("HX8379A: %s, id = 0x%08x\n", __func__, id);
#endif

    return (HX8379A_LCM_ID == id)?1:0;
}



LCM_DRIVER hx8379a_fwvga_dsi_vdo_drv_txd =
{
    .name           = "hx8379a_fwvga_dsi_txd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    //.set_backlight  = lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .esd_check   = lcm_esd_check,
    .esd_recover   = lcm_esd_recover,
    .compare_id     = lcm_compare_id,
#if defined(LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

