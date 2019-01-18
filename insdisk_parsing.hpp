#ifndef __INSDISK_PARSING_H
#define __INSDISK_PARSING_H
#include <sys/types.h>



typedef unsigned long long uint64_t;


/** @brief Typedef defining 32 bit unsigned int type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned int uint32_t;

/** @brief Typedef defining 32 bit int type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef int int32_t;

/** @brief Typedef defining 16 bit unsigned short type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned short uint16_t;

/** @brief Typedef defining 16 bit short type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef short int16_t;

/** @brief Typedef defining 8 bit unsigned char type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned char uint8_t;

/** @brief Typedef defining 8 bit char type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef signed char int8_t;

/*insdisk calculat data*/
struct insdisk_cd
{
	float  yaw;  //偏航角
	float	 pitch;//俯仰
	float	 roll; //滚转
	float	 alt;  //高度
	float  tempr;//温度
	float  press;//气压
};

/*insdisk  original data*/
struct insdisk_od
{
	int16_t ax, ay, az;//加速度计
	int16_t gx, gy, gz;//陀螺仪
	int16_t hx, hy, hz;//磁力计
};
/*insdisk  gps data*/
struct insdisk_gps
{
	float GPS_Altitude; //天线离海平面的高度，-9999.9到9999.9米
	float Latitude_GPS; //纬度	 单位为度
	float Longitude_GPS; //经度  单位为度
	float Speed_GPS; //地面速率  单位  米每秒
	float Course_GPS ; //地面航向（000.0~359.9度，以真北为参考基准)
	unsigned char GPS_STA_Num ;
};
#define  PRA_CAL_DATA	0x01
#define  PRA_ORG_DATA	0x02
#define  PRA_GPS_DATA	0x04
/*insdisk data*/
struct insdisk_data
{
	unsigned char pra_case;//the current prasing data (PRA_CAL_DATA/PRA_ORG_DATA/PRA_GPS_DATA)
	struct insdisk_cd cal_data;
	struct insdisk_od org_data;
	struct insdisk_gps gps_data;
};

struct mag_calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};
/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
struct accel_calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};
/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
struct gyro_calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};
struct sensor_mag_s {
	uint64_t timestamp; // required for logger
	// uint64_t error_count;
	float x;
	float y;
	float z;
	// float range_ga;
	// float scaling;
	// float temperature;
	// uint32_t device_id;
	// int16_t x_raw;
	// int16_t y_raw;
	// int16_t z_raw;
	// uint8_t _padding0[6]; // required for logger

};
struct sensor_accel_s {
	uint64_t timestamp; // required for logger
	// uint64_t integral_dt;
	// uint64_t error_count;
	float x;
	float y;
	float z;
	// float x_integral;
	// float y_integral;
	// float z_integral;
	// float temperature;
	// float range_m_s2;
	// float scaling;
	// uint32_t device_id;
	// int16_t x_raw;
	// int16_t y_raw;
	// int16_t z_raw;
	// int16_t temperature_raw;
};
struct sensor_gyro_s {
	uint64_t timestamp; // required for logger
	// uint64_t integral_dt;
	// uint64_t error_count;
	float x;
	float y;
	float z;
	// float x_integral;
	// float y_integral;
	// float z_integral;
	// float temperature;
	// float range_rad_s;
	// float scaling;
	// uint32_t device_id;
	// int16_t x_raw;
	// int16_t y_raw;
	// int16_t z_raw;
	// int16_t temperature_raw;

};

#define Sensitive_Accel  13107.0f        //加速度灵敏度[1000mV/g]
#define Sensitive_Gyro   78.642f         //陀螺仪灵敏度[6mV/°/sec] 
#define GYROA_ID 12

#define ACCELA_ID 11

#define MAGA_ID 10

#define _ONE_G					9.80665f

extern "C" {
/*function define*/
int insdisk_parsing(unsigned int cnt, unsigned char *buf, struct insdisk_data *insdata);		
int insdisk_init(void);
}

#endif 