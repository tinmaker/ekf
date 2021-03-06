
/* 

float 	yaw,  //偏航角
		pitch,//俯仰
		roll, //滚转
		alt,  //高度
		tempr,//温度
		press;//气压
int16_t ax, ay, az;//加速度计
int16_t gx, gy, gz;//陀螺仪
int16_t hx, hy, hz;//磁力计
------------------------------------
*/
 
#include "insdisk_parsing.h"
#include "./filter/LowPassFilter2p.hpp"
#include <stdio.h>
#include<sys/time.h>

static long time_insdisk = 0;
static struct timeval t;

//uart reicer flag
#define b_uart_head  0x80  //收到A5 头 标志位
#define b_rx_over    0x40  //收到完整的帧标志
// USART Receiver buffer
#define RX_BUFFER_SIZE 100 //接收缓冲区字节数
#define HAS_INIT 0x20 //has init
unsigned char rx_buffer[RX_BUFFER_SIZE]; //接收数据缓冲区
unsigned char rx_wr_index = 0; //缓冲写指针
unsigned char RC_Flag = 0;  //接收状态标志字节

//解算后的角度值
float 	yaw,  //偏航角
		pitch,//俯仰
		roll, //滚转
		alt,  //高度
		tempr,//温度
		press;//气压
//ADC值
int16_t ax, ay, az;//加速度计
int16_t gx, gy, gz;//陀螺仪
int16_t hx, hy, hz;//磁力计
//GPS位置信息
float GPS_Altitude , //天线离海平面的高度，-9999.9到9999.9米
	  Latitude_GPS , //纬度	 单位为度
	  Longitude_GPS , //经度  单位为度
	  Speed_GPS , //地面速率  单位  米每秒
	  Course_GPS ; //地面航向（000.0~359.9度，以真北为参考基准)
unsigned char GPS_STA_Num ;


int _gyro_pub = 0;
int _accel_pub = 0;
int _mag_pub = 0;
struct gyro_calibration_s gyro_cal;
struct accel_calibration_s accel_cal;
struct mag_calibration_s mag_cal;

struct insdisk_cd cal_data;
struct insdisk_od org_data;
struct insdisk_gps gps_data;

#define _ACCEL_DEFAULT_RATE					1000
#define _ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30
#define _GYRO_DEFAULT_RATE					1000
#define _GYRO_DEFAULT_DRIVER_FILTER_FREQ		30
math::LowPassFilter2p   _accel_filter_x(_ACCEL_DEFAULT_RATE, _ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
math::LowPassFilter2p   _accel_filter_y(_ACCEL_DEFAULT_RATE, _ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
math::LowPassFilter2p   _accel_filter_z(_ACCEL_DEFAULT_RATE, _ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
math::LowPassFilter2p   _gyro_filter_x(_GYRO_DEFAULT_RATE, _GYRO_DEFAULT_DRIVER_FILTER_FREQ);
math::LowPassFilter2p   _gyro_filter_y(_GYRO_DEFAULT_RATE, _GYRO_DEFAULT_DRIVER_FILTER_FREQ);
math::LowPassFilter2p   _gyro_filter_z(_GYRO_DEFAULT_RATE, _GYRO_DEFAULT_DRIVER_FILTER_FREQ);

void Insdisk_Get_IMU(void);
void Insdisk_Get_Motion(void);
void Insdisk_Get_GPS(void);
unsigned char Sum_check(void);
int frame_parsing(struct insdisk_data *insdata);

long timeget(void)
{
	gettimeofday(&t, NULL );
	return (long)(t.tv_sec-time_insdisk)*1000000+t.tv_usec;
}

long timeinit()
{
	if(time_insdisk==0)
	{
		gettimeofday(&t, NULL );
		time_insdisk = t.tv_sec;
	}
    return  timeget();
}

//在接收完一帧IMU姿态报告后，调用这个子程序来取出姿态数据
void Insdisk_Get_IMU(void)
{
	int16_t temp;
	
	temp = 0;
	temp = rx_buffer[2];
	temp <<= 8;
	temp |= rx_buffer[3];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	yaw=(float)temp / 10.0f; //偏航角
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	pitch=(float)temp / 10.0f;//俯仰
	
	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	roll=(float)temp / 10.0f;//滚转

	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	alt=(float)temp / 10.0f;//高度
	
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	tempr=(float)temp / 10.0f;//温度
	
	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	press=(float)temp * 10.0f;//气压

}

//在接收一帧ReportMotion 后调用这个子程序来取出ADC数据
void Insdisk_Get_Motion(void)
{
	int16_t temp;
	
	temp = 0;
	temp = rx_buffer[2];
	temp <<= 8;
	temp |= rx_buffer[3];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	ax=temp;//加速度计 X轴的ADC值
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	ay=temp;//加速度计 Y轴的ADC值
	
	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	az=temp;//加速度计 Z轴的ADC值
	
	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gx=temp;//陀螺仪 X轴的ADC值
	
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gy=temp;//陀螺仪 Y轴的ADC值
	
	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gz=temp;//陀螺仪 Z轴的ADC值
	
	temp = 0;
	temp = rx_buffer[14];
	temp <<= 8;
	temp |= rx_buffer[15];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hx=temp;//磁力计 X轴的ADC值
	
	temp = 0;
	temp = rx_buffer[16];
	temp <<= 8;
	temp |= rx_buffer[17];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hy=temp;//磁力计 Y轴的ADC值
	
	temp = 0;
	temp = rx_buffer[18];
	temp <<= 8;
	temp |= rx_buffer[19];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hz=temp;//磁力计 Z轴的ADC值
}

//在接收完一帧GPS位置报告后，调用这个子程序来取出姿态数据
void Insdisk_Get_GPS(void)
{
	long int temp;
	//经度值，单位0.0001度。当传送的值为 1234567  表示 123.4567度
	temp = 0;
	temp = ((long int)rx_buffer[2]<<24);
	temp |= ((long int)rx_buffer[3]<<16);
	temp |= ((long int)rx_buffer[4]<<8);
	temp |= ((long int)rx_buffer[5]); 
	Longitude_GPS = ((float)temp) / ((float)10000.0);
	//纬度值，单位0.0001度。当传送的值为 123456  表示  12.4567度
	temp = 0;
	temp = ((long int)rx_buffer[6]<<24);
	temp |= ((long int)rx_buffer[7]<<16);
	temp |= ((long int)rx_buffer[8]<<8);
	temp |= ((long int)rx_buffer[9]);
	Latitude_GPS = ((float)temp) / ((float)10000.0);
	//GPS海拔高度值，单位0.1米。当传送值为 1623  表示当前海拔为 162.3米
	temp = 0;
	temp |= ((long int)rx_buffer[10]<<8);
	temp |= ((long int)rx_buffer[11]);
	GPS_Altitude = ((float)temp) / ((float)10.0);
	//锁定的卫星数量， 0 表示没有定位、
	GPS_STA_Num = rx_buffer[12];
	//GPS航向值，单位0.1度。当传送值为 125时，表示12.5度。
	temp = 0;
	temp |= ((long int)rx_buffer[13]<<8);
	temp |= ((long int)rx_buffer[14]);
	Course_GPS = ((float)temp) / ((float)10.0);
	//GPS速度，单位0.1米/S  当传送的值为 255时，表示 25.5M/S
	temp = 0;
	temp |= ((long int)rx_buffer[15]<<8);
	temp |= ((long int)rx_buffer[16]);
	Speed_GPS = ((float)temp) / ((float)10.0);

}

//--校验当前接收到的一帧数据是否 与帧校验字节一致
unsigned char Sum_check(void)
{ 
  unsigned char i;
  unsigned int checksum=0; 
  for(i=0;i<rx_buffer[0]-2;i++)
   checksum+=rx_buffer[i];
  if((checksum%256)==rx_buffer[rx_buffer[0]-2])
   return(0x01); //Checksum successful
  else
   return(0x00); //Checksum error
}

int frame_parsing(struct insdisk_data *insdata)
{
	int res = 0;
	//prasing one frame datas
	if((RC_Flag&b_rx_over) && (rx_wr_index!=0))
	{
				// printf(">6\n");
		if(Sum_check()){ 
			//校验通过
			if(rx_buffer[1]==0xA1){ 
				Insdisk_Get_IMU();	//取数据
				insdata->cal_data.yaw = yaw;
				insdata->cal_data.pitch = pitch;
				insdata->cal_data.roll = roll;
				insdata->cal_data.alt = alt;
				insdata->cal_data.tempr = tempr;
				insdata->cal_data.press = press;
				insdata->pra_case |= PRA_CAL_DATA;

				cal_data.yaw = yaw;
				cal_data.pitch = pitch;
				cal_data.roll = roll;
				cal_data.alt = alt;
				cal_data.tempr = tempr;
				cal_data.press = press;
			}
			if(rx_buffer[1]==0xA2){ 
				Insdisk_Get_Motion();	 //取数据
				insdata->org_data.ax = ax;
				insdata->org_data.ay = ay;
				insdata->org_data.az = az;
				insdata->org_data.gx = gx;
				insdata->org_data.gy = gy;
				insdata->org_data.gz = gz;
				insdata->org_data.hx = hx;
				insdata->org_data.hy = hy;
				insdata->org_data.hz = hz;
				insdata->pra_case |= PRA_ORG_DATA;

				org_data.ax = ax;
				org_data.ay = ay;
				org_data.az = az;
				org_data.gx = gx;
				org_data.gy = gy;
				org_data.gz = gz;
				org_data.hx = hx;
				org_data.hy = hy;
				org_data.hz = hz;
			} 
			if(rx_buffer[1]==0xA3){ 
				Insdisk_Get_GPS();	 //取数据
				insdata->gps_data.GPS_Altitude = GPS_Altitude;
				insdata->gps_data.Latitude_GPS = Latitude_GPS;
				insdata->gps_data.Longitude_GPS = Longitude_GPS;
				insdata->gps_data.Speed_GPS = Speed_GPS;
				insdata->gps_data.Course_GPS = Course_GPS;
				insdata->gps_data.GPS_STA_Num = GPS_STA_Num;
				insdata->pra_case |= PRA_GPS_DATA;

				gps_data.GPS_Altitude = GPS_Altitude;
				gps_data.Latitude_GPS = Latitude_GPS;
				gps_data.Longitude_GPS = Longitude_GPS;
				gps_data.Speed_GPS = Speed_GPS;
				gps_data.Course_GPS = Course_GPS;
				gps_data.GPS_STA_Num = GPS_STA_Num;
			} 
			res = 1;
		}else
		{
			res = -1;
		}
		insdisk_init();
	}
	return res;
}
#define FRAME_HEAD0 0XA5
#define FRAME_HEAD1 0X5A
// RC_Flag
int insdisk_parsing(unsigned int cnt, unsigned char *buf, struct insdisk_data *insdata)
{	
	int k = 0;
	int res = 0;
	insdata->pra_case = 0;

	if(!(RC_Flag & HAS_INIT))
	{
		insdisk_init();
		timeinit();
	}
	// for(k=0;k<cnt;k++)
	// {
	// 	printf("%x  ", (char)buf[k]);
	// }
	// printf(">0\n");
	//find one frame datas
	for(k=0;k<cnt;k++)
	{
		// printf(">k=%d,%x\n", k, buf[k]);
		if(buf[k]==FRAME_HEAD0)
		{
			RC_Flag|=b_uart_head; //如果接收到A5 置位帧头标专位
    		rx_buffer[rx_wr_index++]=buf[k]; //保存这个字节.
			// printf(">1\n");
			// return 2;

		}else if(buf[k]==FRAME_HEAD1)
		{
			if(RC_Flag&b_uart_head) //如果上一个字节是A5 那么认定 这个是帧起始字节
			{
				rx_wr_index = 0;  //重置 缓冲区指针
				RC_Flag&=~b_rx_over; //这个帧才刚刚开始收
				// printf(">2\n");
			}else //上一个字节不是A5
			{
				// printf(">3\n");
				rx_buffer[rx_wr_index++]=buf[k];
			}
			RC_Flag&=~b_uart_head; //清帧头标志
		}else
		{
			if(!(RC_Flag&b_rx_over))
			{
				// printf(">4\n");
				rx_buffer[rx_wr_index++]=buf[k];
				RC_Flag&=~b_uart_head;
				if(rx_wr_index==rx_buffer[0]) //收够了字节数.
				{  
				// printf(">5\n");
					RC_Flag|=b_rx_over; //置位 接收完整的一帧数据
					res = frame_parsing(insdata);
					return 3;
					break;
				}
			}
			
		}
		if(rx_wr_index>=RX_BUFFER_SIZE)
		{
			insdisk_init();
			res = -1;
			break;
		}
	}
	
	return res;
}
int insdisk_init(void)
{
	RC_Flag = 0;
	RC_Flag&=~b_uart_head; //清帧头标志 
	RC_Flag|=b_rx_over; //置位 接收完整的一帧数据
	RC_Flag |= HAS_INIT;//has init 
	rx_wr_index = 0;
	return 0;
}



void gyro_get(struct sensor_gyro_s  *gyro)
{
     // struct sensor_gyro_s  gyro = {};
	if (_gyro_pub == 0) {
        gyro_cal.x_offset = 0;
        gyro_cal.y_offset = 0;
        gyro_cal.z_offset = 0;
        gyro_cal.x_scale = 1;
        gyro_cal.y_scale = 1;
        gyro_cal.z_scale = 1;
    } 
     float gyro_[3];
     float x_in_new;
     float y_in_new;
     float z_in_new;
     gyro_[0] = org_data.gx/Sensitive_Gyro/360*2*3.14159f;
     gyro_[1] = org_data.gy/Sensitive_Gyro/360*2*3.14159f;
     gyro_[2] = org_data.gz/Sensitive_Gyro/360*2*3.14159f;

     //swap to NED frame
     gyro_[1] = -gyro_[1];
     gyro_[2] = -gyro_[2];

    gyro->timestamp = timeget(); // required for logger
    // gyro->device_id = GYROA_ID;

    x_in_new = (gyro_[0] - gyro_cal.x_offset) * gyro_cal.x_scale;
    y_in_new = (gyro_[1] - gyro_cal.y_offset) * gyro_cal.y_scale;
    z_in_new = (gyro_[2] - gyro_cal.z_offset) * gyro_cal.z_scale;

    gyro->x = _gyro_filter_x.apply(x_in_new);
    gyro->y = _gyro_filter_y.apply(y_in_new);
    gyro->z = _gyro_filter_z.apply(z_in_new);
    // printf("%d, %f, %f, %f\n", org_data.gx, gyro->x, gyro->y, gyro->z);
    // gyro->x_integral = 0;
    // gyro->y_integral = 0;
    // gyro->z_integral = 0;

    // gyro->error_count = 0;

    // gyro->temperature = 0;
    // gyro->range_rad_s = 0;
    // gyro->scaling = 0;
    // gyro->x_raw = gyro_[0];
    // gyro->y_raw = gyro_[1];
    // gyro->z_raw = gyro_[2];
    // gyro->temperature_raw = 0;

    
}

void accel_get(struct sensor_accel_s  *accel)
{
     // struct sensor_accel_s  accel = {};
	if (_accel_pub == 0) {
        accel_cal.x_offset = 0;
        accel_cal.y_offset = 0;
        accel_cal.z_offset = 0;
        accel_cal.x_scale = 1;
        accel_cal.y_scale = 1;
        accel_cal.z_scale = 1;
    } 
     float accel_[3];
     float x_in_new;
     float y_in_new;
     float z_in_new;
     accel_[0] = org_data.ax/Sensitive_Accel*_ONE_G;
     accel_[1] = org_data.ay/Sensitive_Accel*_ONE_G;
     accel_[2] = org_data.az/Sensitive_Accel*_ONE_G;

     //swap to NED frame
     accel_[0] = -accel_[0];

    accel->timestamp = timeget(); // required for logger
    // accel->device_id = ACCELA_ID;

    x_in_new = (accel_[0] - accel_cal.x_offset) * accel_cal.x_scale;
    y_in_new = (accel_[1] - accel_cal.y_offset) * accel_cal.y_scale;
    z_in_new = (accel_[2] - accel_cal.z_offset) * accel_cal.z_scale;

   	accel->x = _accel_filter_x.apply(x_in_new);
    accel->y = _accel_filter_y.apply(y_in_new);
    accel->z = _accel_filter_z.apply(z_in_new);
    // accel->x_integral = 0;
    // accel->y_integral = 0;
    // accel->z_integral = 0;

    // accel->error_count = 0;

    // accel->temperature = 0;
    // accel->scaling = 0;
    // accel->x_raw = accel_[0];
    // accel->y_raw = accel_[1];
    // accel->z_raw = accel_[2];
    // accel->temperature_raw = 0;

   
}

void mag_get(struct sensor_mag_s * mag)
{
     // struct sensor_mag_s  mag = {};
	if (_mag_pub == 0) {
        mag_cal.x_offset = 0;
        mag_cal.y_offset = 0;
        mag_cal.z_offset = 0;
        mag_cal.x_scale = 1;
        mag_cal.y_scale = 1;
        mag_cal.z_scale = 1;
    } 
     float mag_[3];
     mag_[0] = org_data.hx/1090.0f;//Gain(LSb/Gauss)
     mag_[1] = org_data.hy/1090.0f;
     mag_[2] = org_data.hz/1090.0f;

     mag_[1] = -mag_[1];
     mag_[2] = -mag_[2];

    mag->timestamp = timeget(); // required for logger
    // mag->device_id = MAGA_ID;

    mag->x = (mag_[0] - mag_cal.x_offset) * mag_cal.x_scale;
    mag->y = (mag_[1] - mag_cal.y_offset) * mag_cal.y_scale;
    mag->z = (mag_[2] - mag_cal.z_offset) * mag_cal.z_scale;


    // mag->error_count = 0;

    // mag->temperature = 0;
    // mag->scaling = 0;
    // mag->x_raw = mag_[0];
    // mag->y_raw = mag_[1];
    // mag->z_raw = mag_[2];
 
}
