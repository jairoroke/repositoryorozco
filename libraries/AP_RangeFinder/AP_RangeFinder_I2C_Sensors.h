#include <stdint.h>
#include "linux/types.h"

// LIDAR-Lite default I2C device address
#define LIDARLITE_ADDR_DEFAULT 0x62

// LIDAR-Lite internal register addresses
#define LLv3_ACQ_CMD       0x00
#define LLv3_STATUS        0x01
#define LLv3_SIG_CNT_VAL   0x02
#define LLv3_ACQ_CONFIG    0x04
#define LLv3_DISTANCE      0x0f
#define LLv3_REF_CNT_VAL   0x12
#define LLv3_UNIT_ID_HIGH  0x16
#define LLv3_UNIT_ID_LOW   0x17
#define LLv3_I2C_ID_HIGH   0x18
#define LLv3_I2C_ID_LOW    0x19
#define LLv3_I2C_SEC_ADR   0x1a
#define LLv3_THRESH_BYPASS 0x1c
#define LLv3_I2C_CONFIG    0x1e
#define LLv3_COMMAND       0x40
#define LLv3_CORR_DATA     0x52
#define LLv3_ACQ_SETTINGS  0x5d

#define MAXSONAR_DEFAULTADDR    0x70
#define MAXSONAR_UPDATEREAD     0x51
#define MAXSONAR_GETREAD        0xE1


class I2CSensors
{
	//
	__u32		file_i2c;
public:
	__s32		i2c_init			(void);
	
	__s32		sonnar_connect		(__u8 sonarAddress = MAXSONAR_DEFAULTADDR);
	void		sonarTakeRange		(__u8 sonarAddress = MAXSONAR_DEFAULTADDR);
	__u16		sonarGetRangeRead	(__u8 sonarAddress = MAXSONAR_DEFAULTADDR);
	__u16		sonari2cRead		(__u8 sonarAddress = MAXSONAR_DEFAULTADDR);

	__s32     	garmin_connect 		(__u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	void      	garmin_configure   	(__u8 configuration = 0, __u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	__u16     	garminReadDistance	(__u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	void      	garminWaitForBusy 	(__u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	__u8      	garminGetBusyFlag 	(__u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	void      	garminTakeRange   	(__u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	__s32     	garmini2cWrite	    (__u8 regAddr, __u8 * dataBytes, __u8 numBytes, __u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	__s32     	garmini2cRead     	(__u8 regAddr, __u8 * dataBytes, __u8 numBytes, __u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
	void      	correlationRecordRead (__s16 * corrValues, __u16 numberOfReadings = 256, __u8 lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

};