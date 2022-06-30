#ifndef _DEVICE_INFO_H
#define _DEVICE_INFO_H

//dram type
enum{
	DRAM_TYPE0 = 0,
	DRAM_TYPE1,
	DRAM_TYPE2,
	DRAM_TYPE3,
	DRAM_UNKNOWN,
};

struct manufacture_info {
        char *version;
        char *manufacture;
        char *fw_path;
};

int register_device_proc(char *name, char *version, char *manufacture);
int register_devinfo(char *name, struct manufacture_info *info);


#endif /*_DEVICE_INFO_H*/
