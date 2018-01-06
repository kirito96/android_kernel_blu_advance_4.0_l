#ifdef IAP_PORTION
const u8 huaruichuan_fw[]=
{
//#include "S4050_hrc_v1208.i"
#include "S4050_hrc_v1209.i"
};

struct vendor_map
{
	int vendor_id;
	char vendor_name[30];
	uint8_t* fw_array;
};
const struct vendor_map g_vendor_map[]=
{
	{0x2ae0,"HUARUIC",huaruichuan_fw}
};

#endif/*IAP_PORTION*/
