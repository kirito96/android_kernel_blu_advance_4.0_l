#include <linux/types.h>
#include <cust_alsps.h>
#include <mach/mt_pm_ldo.h>

//#include <mach/mt6577_pm_ldo.h>
static struct alsps_hw cust_alsps_hw =
{
    .i2c_num    = 1,
    .polling_mode_ps =0,
    .polling_mode_als =1,       //Ivan Interrupt mode not support
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x0C, 0x48, 0x78, 0x00},

    // .als_level  = { 1,  2,  10, 200,  400, 1024,  65535,  65535,  65535, 65535, 65535, 65535, 65535, 65535, 65535},
    // .als_value  = { 40,  160,  280,  2500,  3500,  4500,  10243,  10243, 10243, 10243, 10243, 10243, 10243, 10243, 10243},
    .als_level  = { 0,  640,  2320, 4000, 5680, 7360,  9040,  10720,  12400, 14080, 15760, 17440, 19120, 20486, 65535},
    .als_value  = {  0,  320,  1160, 2000, 2840, 3680,  4520,  5360,  6200, 7040, 7880, 8720, 9560, 10243, 10243},

    .ps_threshold = 2,  //3,
        .ps_threshold_high = 0xf0, // 0x11c,
    .ps_threshold_low = 0x60, // 0x85, 
    //.ps_threshold_high = 0x32, // 0x11c,
  //  .ps_threshold_low = 0x23, // 0x85, 
    .als_threshold_high = 0xFFFF,
    .als_threshold_low = 0,

};
struct alsps_hw *ltr553_get_cust_alsps_hw(void)
{
    return &cust_alsps_hw;
}

