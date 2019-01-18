
/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hil.cpp
 *
 * Driver/configurator for the virtual PWMOut port.
 *
 * This virtual driver emulates PWM / servo outputs for setups where
 * the connected hardware does not provide enough or no PWM outputs.
 *
 * Its only function is to take actuator_control uORB messages,
 * mix them with any loaded mixer and output the result to the
 * actuator_output uORB topic. PWMOut can also be performed with normal
 * PWM outputs, a special flag prevents the outputs to be operated
 * during PWMOut mode. If PWMOut is not performed with a standalone FMU,
 * but in a real system, it is NOT recommended to use this virtual
 * driver. Use instead the normal FMU or IO driver.
 */

#include "insdisk_parsing.h"
/*
 * This driver is supposed to run on Snapdragon. It sends actuator_controls (PWM)
 * to a Pixhawk/Pixfalcon/Pixracer over UART (mavlink) and receives RC input.
 */
#if 1
class Insdisk
{
public:
	Insdisk();

	~Insdisk();

	int start(int argc, char *argv[]);

	void usage();

	int send(const char* str);

	char _device[32] = {};
	int _baudrate;
	bool _is_running;
	uint32_t port_count = 0;

private:
	bool _task_should_exit;
	int _fd;
	int _data_bits;
	int _parity_bits;
	int _stop_bits;
	int _control_task;
	struct termios _t_bak;
	int open_uart(int baud, const char *uart_name, int data_bits, int parity_bits, int stop_bits);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main(int argc, char *argv[]);

	void gyro_publish(struct insdisk_od *orgdata);
	void accel_publish(struct insdisk_od *orgdata);
	void mag_publish(struct insdisk_od *orgdata);

	Integrator      _gyro_int;
	Integrator      _accel_int;
	orb_advert_t _gyro_pub ;
	orb_advert_t _mag_pub ;
	orb_advert_t _accel_pub ;
	int         _gyro_orb_class_instance;
	int         _mag_orb_class_instance;
	int         _accel_orb_class_instance;
	#define Sensitive_Accel  13107.0f        //加速度灵敏度[1000mV/g]
	#define Sensitive_Gyro   78.642f         //陀螺仪灵敏度[6mV/°/sec] 
	struct gyro_calibration_s gyro_cal;
	#define GYROA_ID 12

	struct accel_calibration_s accel_cal;
	#define ACCELA_ID 11

	struct mag_calibration_s mag_cal;
	#define MAGA_ID 10

	#define _ONE_G					9.80665f
};

namespace insdisk
{
	Insdisk *g_control;
}

Insdisk::Insdisk() : 
	_device("/dev/ttyS2"),
	_baudrate(921600),
	_is_running(false),
	_task_should_exit(false),
	_fd(-1),
	_data_bits(8),
	_parity_bits('N'),//
	_stop_bits(1),
	_control_task(-1),
	_t_bak{},
	_gyro_int{},
	_accel_int{},
	_gyro_pub(nullptr),
	_mag_pub(nullptr),
	_accel_pub(nullptr),
	_gyro_orb_class_instance(-1),
	_mag_orb_class_instance(-1),
	_accel_orb_class_instance(-1),
	gyro_cal{},
	accel_cal{},
	mag_cal{}
{

}

Insdisk::~Insdisk()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	insdisk::g_control = nullptr;
}

void
Insdisk::task_main_trampoline(int argc, char *argv[])
{
	insdisk::g_control->task_main(argc, argv);
}

int
Insdisk::open_uart(int baud, const char *uart_name, int data_bits, int parity_bits, int stop_bits)
{
	int fd;	
	struct termios t;

#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

	/* process baud rate */
	int speed;
	int databit, stopbit, paritybit;

	switch(data_bits)
	{
		case 5:
			databit = CS5;
			break;
		case 6:
			databit = CS6;
			break;
		case 7:
			databit = CS7;
			break;
		case 8:
		default:
			databit = CS8;
			break;
	}

	switch(stop_bits)
	{
		case 1:
			stopbit = 1;
			break;
	}
	(void)stopbit;
	(void)paritybit;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

#ifdef B100000
	case 100000: speed = B100000; break;
#endif

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;//

	case 1000000: speed = B1000000; break;

#ifdef B1500000
	case 1500000: speed = B1500000; break;
#endif

#ifdef B3000000
	case 3000000: speed = B3000000; break;
#endif

	default:
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n921600\n1000000\n",
			baud);
		return -EINVAL;
	}

	fd = ::open(uart_name, O_RDWR | O_NONBLOCK);

	tcsetattr(fd, TCSANOW, &_t_bak);

	memset(&t, 0, sizeof(struct termios));
	cfmakeraw(&t);

	t.c_ispeed = speed;                // set the baud

	t.c_cc[VTIME] = 0;
	t.c_cc[VMIN] = 0;

	t.c_cflag |= (t.c_cflag & ~CSIZE) | databit;                         // set data length
	 /*设置停止位*/
    if( stopbit == 1)/*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
    {
        t.c_cflag &= ~CSTOPB;/*默认为一位停止位； */
    }
    else if( stopbit == 2)
    {
        t.c_cflag |= CSTOPB;/*CSTOPB表示送两位停止位*/
    }
    /*设置奇偶校验位*/
     switch( parity_bits )
     {
         case 'O':  /*奇校验*/
             t.c_cflag |= PARENB;/*开启奇偶校验*/
             t.c_iflag |= (INPCK | ISTRIP);/*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
             t.c_cflag |= PARODD;/*启用奇校验(默认为偶校验)*/
             break;
         case 'E':/*偶校验*/
             t.c_cflag |= PARENB; /*开启奇偶校验  */
             t.c_iflag |= ( INPCK | ISTRIP);/*打开输入奇偶校验并去除字符第八个比特*/
             t.c_cflag &= ~PARODD;/*启用偶校验*/
             break;
         case 'N': /*无奇偶校验*/
             t.c_cflag &= ~PARENB;
             break;
     }
     PX4_INFO("uart set\n");
	
	int rv = tcsetattr(fd, TCSANOW, &t);
	if(rv < 0) {

		printf("tcsetattr failed\n");
		return -EINVAL;
	}
	return fd;
}

void Insdisk::gyro_publish(struct insdisk_od *orgdata)
{
     struct sensor_gyro_s  gyro = {};

     float gyro_[3];
     gyro_[0] = orgdata->gx/Sensitive_Gyro/360*2*3.14159f;
     gyro_[1] = orgdata->gy/Sensitive_Gyro/360*2*3.14159f;
     gyro_[2] = orgdata->gz/Sensitive_Gyro/360*2*3.14159f;

     //swap to NED frame
     gyro_[1] = -gyro_[1];
     gyro_[2] = -gyro_[2];

    gyro.timestamp = hrt_absolute_time(); // required for logger
    gyro.device_id = GYROA_ID;

    gyro.x = (gyro_[0] - gyro_cal.x_offset) * gyro_cal.x_scale;
    gyro.y = (gyro_[1] - gyro_cal.y_offset) * gyro_cal.y_scale;
    gyro.z = (gyro_[2] - gyro_cal.z_offset) * gyro_cal.z_scale;

    math::Vector<3> gval_integrated;
    math::Vector<3> gval(gyro.x, gyro.y, gyro.z);
    _gyro_int.put(gyro.timestamp, gval, gval_integrated, gyro.integral_dt);
    gyro.x_integral = gval_integrated(0);
    gyro.y_integral = gval_integrated(1);
    gyro.z_integral = gval_integrated(2);

    gyro.error_count = 0;

    gyro.temperature = 0;
    gyro.range_rad_s = 0;
    gyro.scaling = 0;
    gyro.x_raw = gyro_[0];
    gyro.y_raw = gyro_[1];
    gyro.z_raw = gyro_[2];
    gyro.temperature_raw = 0;

    if (_gyro_pub == nullptr) {
        _gyro_pub = orb_advertise_multi(ORB_ID(sensor_gyro), &gyro, &_gyro_orb_class_instance, ORB_PRIO_MAX);
        gyro_cal.x_offset = 0;
        gyro_cal.y_offset = 0;
        gyro_cal.z_offset = 0;
        gyro_cal.x_scale = 1;
        gyro_cal.y_scale = 1;
        gyro_cal.z_scale = 1;
    } else {
        orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &gyro);
        // printf(">>>>>\n");
    }
}

void Insdisk::accel_publish(struct insdisk_od *orgdata)
{
     struct sensor_accel_s  accel = {};

     float accel_[3];
     accel_[0] = orgdata->ax/Sensitive_Accel*_ONE_G;
     accel_[1] = orgdata->ay/Sensitive_Accel*_ONE_G;
     accel_[2] = orgdata->az/Sensitive_Accel*_ONE_G;

     //swap to NED frame
     accel_[0] = -accel_[0];

    accel.timestamp = hrt_absolute_time(); // required for logger
    accel.device_id = ACCELA_ID;

    accel.x = (accel_[0] - accel_cal.x_offset) * accel_cal.x_scale;
    accel.y = (accel_[1] - accel_cal.y_offset) * accel_cal.y_scale;
    accel.z = (accel_[2] - accel_cal.z_offset) * accel_cal.z_scale;

    math::Vector<3> gval_integrated;
    math::Vector<3> gval(accel.x, accel.y, accel.z);
    _accel_int.put(accel.timestamp, gval, gval_integrated, accel.integral_dt);
    accel.x_integral = gval_integrated(0);
    accel.y_integral = gval_integrated(1);
    accel.z_integral = gval_integrated(2);

    accel.error_count = 0;

    accel.temperature = 0;
    accel.scaling = 0;
    accel.x_raw = accel_[0];
    accel.y_raw = accel_[1];
    accel.z_raw = accel_[2];
    accel.temperature_raw = 0;

    if (_accel_pub == nullptr) {
        _accel_pub = orb_advertise_multi(ORB_ID(sensor_accel), &accel, &_accel_orb_class_instance, ORB_PRIO_MAX);
        accel_cal.x_offset = 0;
        accel_cal.y_offset = 0;
        accel_cal.z_offset = 0;
        accel_cal.x_scale = 1;
        accel_cal.y_scale = 1;
        accel_cal.z_scale = 1;
    } else {
        orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
        // printf("%f,%f,%f\n", (double)accel.x, (double)accel.y, (double)accel.z);
    }
}

void Insdisk::mag_publish(struct insdisk_od *orgdata)
{
     struct sensor_mag_s  mag = {};

     float mag_[3];
     mag_[0] = orgdata->hx/1090.0f;//Gain(LSb/Gauss)
     mag_[1] = orgdata->hy/1090.0f;
     mag_[2] = orgdata->hz/1090.0f;

     mag_[1] = -mag_[1];
     mag_[2] = -mag_[2];

    mag.timestamp = hrt_absolute_time(); // required for logger
    mag.device_id = MAGA_ID;

    mag.x = (mag_[0] - mag_cal.x_offset) * mag_cal.x_scale;
    mag.y = (mag_[1] - mag_cal.y_offset) * mag_cal.y_scale;
    mag.z = (mag_[2] - mag_cal.z_offset) * mag_cal.z_scale;


    mag.error_count = 0;

    mag.temperature = 0;
    mag.scaling = 0;
    mag.x_raw = mag_[0];
    mag.y_raw = mag_[1];
    mag.z_raw = mag_[2];

    if (_mag_pub == nullptr) {
        _mag_pub = orb_advertise_multi(ORB_ID(sensor_mag), &mag, &_mag_orb_class_instance, ORB_PRIO_MAX);
        mag_cal.x_offset = 0;
        mag_cal.y_offset = 0;
        mag_cal.z_offset = 0;
        mag_cal.x_scale = 1;
        mag_cal.y_scale = 1;
        mag_cal.z_scale = 1;
    } else {
        orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
    }
}

void
Insdisk::task_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(_device, device, strlen(device));
			break;
		case 'b':
			_baudrate = strtoul(myoptarg, NULL, 10);

			if (_baudrate < 9600 || _baudrate > 3000000) {
				PX4_WARN("invalid baud rate '%s'", myoptarg);
				return;
			}
			break;
		}
	}

	_is_running = true;

	_fd = open_uart(_baudrate, _device, _data_bits, _parity_bits, _stop_bits);

	if (_fd == -1) {
		PX4_ERR("Failed to open UART:%s.", _device);
		return;
	}
	else
	{
		PX4_INFO("open UART:%s at baudrate :%d  %d  %d %c\n", _device, _baudrate,_data_bits,_stop_bits,_parity_bits);
	}

	int cnt = 0;
	char recv[512];
	struct insdisk_data insdata;

	while (!_task_should_exit) 
	{
		cnt = read(_fd, recv, 512);
		// printf("cnt=%d\n", cnt);
		// for(int k=0;k<cnt;k++)
		// {
		// 	printf("%x  ", recv[k]);
		// }
		// printf("\n");
		port_count += cnt;
		static  uint32_t insdisk_count = 0;
        static  uint64_t insdisk_time = 0;
        if(insdisk_parsing(cnt, recv, &insdata)>0)
        {
        	insdisk_count++;
        	gyro_publish(&insdata.org_data);
        	accel_publish(&insdata.org_data);
        	mag_publish(&insdata.org_data);
        }
        if(hrt_absolute_time()-insdisk_time>=1000000)
        {
            insdisk_time = hrt_absolute_time();
            printf("[ insdisk rate:%d ]  \n", (int)insdisk_count);
			// printf("%f,%f,%f,%f,%f,%f\n", (double)insdata.org_data.ax, (double)insdata.org_data.ay, (double)insdata.org_data.az
			// 	, (double)insdata.org_data.gx, (double)insdata.org_data.gy, (double)insdata.org_data.gz);	

            insdisk_count = 0;
        }
		usleep(1000);
	}
	close(_fd);
	tcsetattr(_fd, TCSANOW, &_t_bak);
	_control_task = -1;
	_is_running = false;
}

int
Insdisk::start(int argc, char *argv[])
{
	ASSERT(_control_task == -1);

	_task_should_exit = false;


	/* start the task */
	_control_task = px4_task_spawn_cmd("Insdisk",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX + 5,
					   8192,
					   (px4_main_t)&Insdisk::task_main_trampoline,
					   (char *const *)argv);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
Insdisk::usage()
{
	PX4_INFO("usage: Insdisk start -d /dev/ttySx -b 115200");
	PX4_INFO("       Insdisk stop");
	PX4_INFO("       Insdisk status");
}
#endif

/* driver 'main' command */
extern "C" __EXPORT int insdisk_main(int argc, char *argv[]);

int insdisk_main(int argc, char *argv[])
{
	#if 1
	char *verb = nullptr;

	printf("Insdisk_main\n");

	if (argc >= 2) {
		verb = argv[1];
	}else{
		insdisk::g_control->usage();
		return 1;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		printf("Insdisk_main start\n");
		if (insdisk::g_control != nullptr) {
			PX4_WARN("Insdisk already running");
			return 1;
		}

		insdisk::g_control=new Insdisk();
		insdisk::g_control->start(argc, argv);
	}

	else if (!strcmp(verb, "stop")) {
		if (insdisk::g_control == nullptr) {
			PX4_WARN("Insdisk is not running");
			return 1;
		}

		delete insdisk::g_control;
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("Insdisk is %s\n", insdisk::g_control->_is_running ? "running" : "not running");
		return 0;

	} else {
		insdisk::g_control->usage();
		return 1;
	}
#endif
	return 0;
}

