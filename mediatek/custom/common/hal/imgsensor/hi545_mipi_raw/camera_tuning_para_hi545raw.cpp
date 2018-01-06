#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_hi545raw.h"
#include "camera_info_hi545raw.h"
#include "camera_custom_AEPlinetable.h"
#include "camera_custom_tsf_tbl.h"
const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,
    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        }
    },
    ISPPca:{
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
        },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        66300,    // i4R_AVG
        15231,    // i4R_STD
        99725,    // i4B_AVG
        21766,    // i4B_STD
        {  // i4P00[9]
            3995000, -1215000, -220000, -605000, 3767500, -590000, -172500, -1910000, 4650000
        },
        {  // i4P10[9]
            690622, -1111173, 420550, -76327, -474845, 513827, 142686, 603561, -726494
        },
        {  // i4P01[9]
            705642, -866342, 160700, -251907, -380380, 594791, -5826, -39860, 69114
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1152,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            49,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            16,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            16,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            16,    // u4CapExpUnit 
            30,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            0    // u4FocusLength_100x
        },
        // rHistConfig
        {
            4,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {82, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            FALSE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            TRUE,    // bEnableCaptureThres
            TRUE,    // bEnableVideoThres
            TRUE,    // bEnableStrobeThres
            60,    // u4AETarget
            57,    // u4StrobeAETarget
            50,    // u4InitIndex
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            4,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -3,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            1,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            1,    // u4VideoFlareThres
            32,    // u4StrobeFlareOffset
            1,    // u4StrobeFlareThres
            160,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            160,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            75    // u4FlatnessStrength
        }
    },
    // AWB NVRAM
    {
        // AWB calibration data
        {
            // rUnitGain (unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rGoldenGain (golden sample gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                869,    // i4R
                512,    // i4G
                678    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                26,    // i4X
                -326    // i4Y
            },
            // Horizon
            {
                -472,    // i4X
                -241    // i4Y
            },
            // A
            {
                -341,    // i4X
                -284    // i4Y
            },
            // TL84
            {
                -195,    // i4X
                -303    // i4Y
            },
            // CWF
            {
                -142,    // i4X
                -380    // i4Y
            },
            // DNP
            {
                -178,    // i4X
                -278    // i4Y
            },
            // D65
            {
                92,    // i4X
                -299    // i4Y
            },
            // DF
            {
                -76,    // i4X
                -267    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                26,    // i4X
                -326    // i4Y
            },
            // Horizon
            {
                -472,    // i4X
                -241    // i4Y
            },
            // A
            {
                -341,    // i4X
                -284    // i4Y
            },
            // TL84
            {
                -195,    // i4X
                -303    // i4Y
            },
            // CWF
            {
                -142,    // i4X
                -380    // i4Y
            },
            // DNP
            {
                -178,    // i4X
                -278    // i4Y
            },
            // D65
            {
                92,    // i4X
                -299    // i4Y
            },
            // DF
            {
                -76,    // i4X
                -267    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                824,    // i4R
                512,    // i4G
                769    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                700,    // i4G
                1839    // i4B
            },
            // A 
            {
                512,    // i4R
                553,    // i4G
                1290    // i4B
            },
            // TL84 
            {
                593,    // i4R
                512,    // i4G
                1005    // i4B
            },
            // CWF 
            {
                707,    // i4R
                512,    // i4G
                1039    // i4B
            },
            // DNP 
            {
                587,    // i4R
                512,    // i4G
                949    // i4B
            },
            // D65 
            {
                869,    // i4R
                512,    // i4G
                678    // i4B
            },
            // DF 
            {
                663,    // i4R
                512,    // i4G
                814    // i4B
            }
        },
        // Rotation matrix parameter
        {
            0,    // i4RotationAngle
            256,    // i4Cos
            0    // i4Sin
        },
        // Daylight locus parameter
        {
            -112,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -245,    // i4RightBound
            -895,    // i4LeftBound
            -212,    // i4UpperBound
            -312    // i4LowerBound
            },
            // Warm fluorescent
            {
            -245,    // i4RightBound
            -895,    // i4LeftBound
            -312,    // i4UpperBound
            -432    // i4LowerBound
            },
            // Fluorescent
            {
            -228,    // i4RightBound
            -245,    // i4LeftBound
            -215,    // i4UpperBound
            -341    // i4LowerBound
            },
            // CWF
            {
            -228,    // i4RightBound
            -245,    // i4LeftBound
            -341,    // i4UpperBound
            -430    // i4LowerBound
            },
            // Daylight
            {
            117,    // i4RightBound
            -228,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Shade
            {
            477,    // i4RightBound
            117,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            117,    // i4RightBound
            -228,    // i4LeftBound
            -379,    // i4UpperBound
            -480    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            477,    // i4RightBound
            -895,    // i4LeftBound
            0,    // i4UpperBound
            -480    // i4LowerBound
            },
            // Daylight
            {
            142,    // i4RightBound
            -228,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Cloudy daylight
            {
            242,    // i4RightBound
            67,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Shade
            {
            342,    // i4RightBound
            67,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Twilight
            {
            -228,    // i4RightBound
            -388,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Fluorescent
            {
            142,    // i4RightBound
            -295,    // i4LeftBound
            -249,    // i4UpperBound
            -430    // i4LowerBound
            },
            // Warm fluorescent
            {
            -241,    // i4RightBound
            -441,    // i4LeftBound
            -249,    // i4UpperBound
            -430    // i4LowerBound
            },
            // Incandescent
            {
            -241,    // i4RightBound
            -441,    // i4LeftBound
            -219,    // i4UpperBound
            -379    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            724,    // i4R
            512,    // i4G
            813    // i4B
            },
            // Cloudy daylight
            {
            946,    // i4R
            512,    // i4G
            623    // i4B
            },
            // Shade
            {
            1012,    // i4R
            512,    // i4G
            582    // i4B
            },
            // Twilight
            {
            506,    // i4R
            512,    // i4G
            1165    // i4B
            },
            // Fluorescent
            {
            731,    // i4R
            512,    // i4G
            899    // i4B
            },
            // Warm fluorescent
            {
            511,    // i4R
            512,    // i4G
            1286    // i4B
            },
            // Incandescent
            {
            484,    // i4R
            512,    // i4G
            1218    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            6785    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5601    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1341    // i4OffsetThr
            },
            // Daylight WB gain
            {
            603,    // i4R
            512,    // i4G
            977    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: warm fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -564,    // i4RotatedXCoordinate[0]
                -433,    // i4RotatedXCoordinate[1]
                -287,    // i4RotatedXCoordinate[2]
                -270,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace

const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    #include INCLUDE_FILENAME_TSF_PARA
    #include INCLUDE_FILENAME_TSF_DATA
};


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T),
                                             0,
                                             sizeof(CAMERA_TSF_TBL_STRUCT)};

    if (CameraDataType > CAMERA_DATA_TSF_TABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        case CAMERA_DATA_TSF_TABLE:
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
            break;
        default:
            return 1;
    }
    return 0;
}}; // NSFeature


