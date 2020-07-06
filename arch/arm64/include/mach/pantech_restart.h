
//p14291_userdump
#define LOGSET_USERDUMP_DISABLE			0 //p11014 wks   sync with pantech_sys.h & pantech_sys_info.h
#define LOGSET_USERDUMP_COREDUMP    		1
//#define LOGSET_USERDUMP_FRAME_COREDUMP	2
#define LOGSET_USERDUMP_USER_RAMDUMP	3
#define LOGSET_USERDUMP_FRAME_RAMDUMP	4
extern int userdump;

// p15060
extern int user_fault;
