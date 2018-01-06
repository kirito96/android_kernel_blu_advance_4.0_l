#include <linux/delay.h>
#include <linux/fb.h>
#include "mtkfb.h"
#include <asm/uaccess.h>

#include "disp_drv.h"
#include "ddp_hal.h"
#include "disp_drv_platform.h"
#include "disp_drv_log.h"
#include "lcm_drv.h"
#include "debug.h"
//#ifdef MTK_DISP_CONFIG_SUPPORT
#include "fbconfig_kdebug.h"
//#endif

// Fence Sync Object
#if defined (MTK_FB_SYNC_SUPPORT)
#include "disp_sync.h"
#endif

#include <linux/disp_assert_layer.h>

#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include "mach/mt_clkmgr.h"
#include <linux/vmalloc.h>
#include "mtkfb_info.h"
#include <linux/dma-mapping.h>
#include <linux/rtpm_prio.h>
#include <linux/aee.h>
extern unsigned int lcd_fps;
extern BOOL is_early_suspended;
extern struct semaphore sem_early_suspend;

extern unsigned int EnableVSyncLog;

#define LCM_ESD_CHECK_MAX_COUNT 5

#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#ifndef disp_path_wait_mem_out_done3
	#define disp_path_wait_mem_out_done3		vvvvvvvvvvv31
#endif

#ifndef disp_path_clear_mem_out_done3
	#define disp_path_clear_mem_out_done3		vvvvvvvvvvv32
#endif




#ifndef DISP_Capture_Framebuffer3
	#define DISP_Capture_Framebuffer3		vvvvvvvvvvv87
#endif


extern int disp_path_wait_mem_out_done3(void);
extern int disp_path_clear_mem_out_done3(void);

extern BOOL disp_drv_init_context(void);

extern OVL_CONFIG_STRUCT cached_layer_config[];
extern struct mutex MemOutSettingMutex;
extern struct disp_path_config_mem_out_struct MemOutConfig;
extern LCM_PARAMS *lcm_params;
extern wait_queue_head_t reg_update_wq;
extern void * gv_TnRVa[2];


enum WDMA_OUTPUT_FORMAT {
    WDMA_OUTPUT_FORMAT_RGB565 = 0x00, //  // basic format
    WDMA_OUTPUT_FORMAT_RGB888 = 0x01,//
    WDMA_OUTPUT_FORMAT_ARGB = 0x02,//
    WDMA_OUTPUT_FORMAT_XRGB = 0x03,//
    WDMA_OUTPUT_FORMAT_UYVY = 0x04,//
    WDMA_OUTPUT_FORMAT_YUV444 = 0x05,//
    WDMA_OUTPUT_FORMAT_YUV420_P = 0x06,//???
    WDMA_OUTPUT_FORMAT_UYVY_BLK = 0x0c,//????

    WDMA_OUTPUT_FORMAT_BGR888 = 0xa0,//   // special format by swap
    WDMA_OUTPUT_FORMAT_BGRA = 0xa1,//
    WDMA_OUTPUT_FORMAT_ABGR = 0xa2,//
    WDMA_OUTPUT_FORMAT_RGBA = 0xa3,//
//4.4
    WDMA_OUTPUT_FORMAT_UNKNOWN = 0x100
//4.4
    //WDMA_OUTPUT_FORMAT_Y1V0Y0U0,   // UV format by swap
    //WDMA_OUTPUT_FORMAT_V0Y1U0Y0,
    //WDMA_OUTPUT_FORMAT_Y1U0Y0V0,
    //WDMA_OUTPUT_FORMAT_U0Y1V0Y0,//
};
//TN801031 resumed 2014.04.04


static unsigned int TnGetMemOutBpp(unsigned int f_Format)
{
    unsigned int lv_Bpp;

    switch(f_Format) 
	{
        case WDMA_OUTPUT_FORMAT_RGB565:
        case WDMA_OUTPUT_FORMAT_UYVY_BLK:
        case WDMA_OUTPUT_FORMAT_UYVY:
            lv_Bpp = 2;
            break;

        case WDMA_OUTPUT_FORMAT_YUV420_P:
            lv_Bpp = 1;
            break;
            
        case WDMA_OUTPUT_FORMAT_RGB888:
        case WDMA_OUTPUT_FORMAT_BGR888:
        case WDMA_OUTPUT_FORMAT_YUV444:
            lv_Bpp = 3;
            break;

        case WDMA_OUTPUT_FORMAT_ARGB:
        case WDMA_OUTPUT_FORMAT_XRGB:
        case WDMA_OUTPUT_FORMAT_BGRA:
        case WDMA_OUTPUT_FORMAT_ABGR:            
        case WDMA_OUTPUT_FORMAT_RGBA:
            lv_Bpp = 4;
            break;

        default:
            ASSERT(0);  // invalid format
    }// switch (f_Format) 

//    printk("TnGetMemOutBpp-20 lv_Bpp=%d,%x\n",lv_Bpp,f_Format); 
	return(lv_Bpp);
}


static int tn_mem_out(struct disp_path_config_mem_out_struct* pConfig)
{
//			printk("TN tn_mem_out_without_lcd-10\n");

	//if((pConfig->enable) && (pConfig->dstAddr))
	if(pConfig->dstAddr)
	{
			//unsigned int TnGetMemOutBpp(unsigned int f_Format)

			unsigned int lv_Size;

			lv_Size = pConfig->srcROI.width * pConfig->srcROI.height * TnGetMemOutBpp(pConfig->outFormat);
			//pConfig->enable = 0;

			memcpy((void *)pConfig->dstAddr,(void *)gv_TnRVa[0],lv_Size);

			//printk("TN tn_mem_out_without_lcd-30\n");
	}//if((gv_MemOutConfig.enable) && (gv_MemOutConfig.dstAddr))

	return 0;
}//int tn_mem_out(struct disp_path_config_mem_out_struct* pConfig)


DISP_STATUS DISP_Capture_Framebuffer3( unsigned int pvbuf, unsigned int bpp, unsigned int is_early_suspended )
{

    unsigned int ret = 0;

    int i;


    for (i=0; i<OVL_LAYER_NUM; i++)
    {
        if (cached_layer_config[i].layer_en && cached_layer_config[i].security)
		{
			break;
		}
    }

    if (i < OVL_LAYER_NUM)
    {
        // There is security layer.
        memset(pvbuf, 0, DISP_GetScreenHeight()*DISP_GetScreenWidth()*bpp/8);

        return DISP_STATUS_OK;
    }//if (i < OVL_LAYER_NUM)

    disp_drv_init_context();

    mutex_lock(&MemOutSettingMutex);
    if(bpp == 32)
	{
//4.2
        MemOutConfig.outFormat = WDMA_OUTPUT_FORMAT_ARGB;
//4.2

	}
    else if(bpp == 16)
	{
        MemOutConfig.outFormat = WDMA_OUTPUT_FORMAT_RGB565;

	}
    else if(bpp == 24)
	{
        MemOutConfig.outFormat = WDMA_OUTPUT_FORMAT_RGB888;
	}
    else
    {
        //printk("DSI_Capture_FB, fb color format not support\n");

        MemOutConfig.outFormat = WDMA_OUTPUT_FORMAT_RGB888;
		
    }

	//printk("TN DISP_Capture_Framebuffer-30,bpp=%d\n",bpp);

    MemOutConfig.enable = 1;

	MemOutConfig.dstAddr = pvbuf;

    MemOutConfig.srcROI.x = 0;
    MemOutConfig.srcROI.y = 0;
    MemOutConfig.srcROI.height= DISP_GetScreenHeight();
    MemOutConfig.srcROI.width= DISP_GetScreenWidth();

    if (is_early_suspended == 0)
	{
        MemOutConfig.dirty = 1;
	}
    
	// must clear mem_out_flag before config
    disp_path_clear_mem_out_done3();

    mutex_unlock(&MemOutSettingMutex);

    if (is_early_suspended)
    {
		//printk("TN DISP_Capture_Framebuffer-50\n");

		tn_mem_out(&MemOutConfig);

		//printk("TN DISP_Capture_Framebuffer-60\n");

    }//if (is_early_suspended)
    else
    {
		//printk("TN DISP_Capture_Framebuffer-70\n");

        // Wait for mem out done.
        ret = disp_path_wait_mem_out_done3();

		tn_mem_out(&MemOutConfig);

        // apply for video mode only
        if ((lcm_params->type == LCM_TYPE_DPI) || ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE)))
        {
            mutex_lock(&MemOutSettingMutex);
            MemOutConfig.enable = 0;
            MemOutConfig.dirty = 1;
            mutex_unlock(&MemOutSettingMutex);
        }//if ((lcm_params->type == LCM_TYPE_DPI) || ((lcm_params->type == LCM_TYPE_DSI) && (lcm_params->dsi.mode != CMD_MODE)))

        // Wait for reg update.
        wait_event_interruptible(reg_update_wq, !MemOutConfig.dirty);

		//printk("TN DISP_Capture_Framebuffer-80\n");//NO,??
    }//else !if (is_early_suspended)

    //BOOL deconfigWdma = TRUE;

//4.2
#if defined(MTK_HDMI_SUPPORT)
    deconfigWdma &= !is_hdmi_active();
#endif

#if defined(MTK_WFD_SUPPORT)
    deconfigWdma &= !is_wfd_active();
#endif
    return DISP_STATUS_OK;
}
//TN801031 added 2013.09.26


























