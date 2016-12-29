
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "i9dof.h"

#define i9dof_debug(fmt, args ...)  warnx("I9DOF::%s, Line: %d; ----->  " fmt "\n",__FUNCTION__, __LINE__,## args)


#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

class I9DOF : public device::CDev
{
public:
	/* Constructor */
	I9DOF();

	/* Destructor, also kills the I9DOF task.*/
	virtual ~I9DOF();

	virtual int init();
	virtual ssize_t read(sturct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/* Start the I9DOF task */
	int start();
	void info();

private:

	volatile bool	 _task_should_exit;		/**< if true, sensor task should exit */
	int 			_i9dof_task;			/**< task handle for sensor task */
	int				_serial_fd;					///< serial interface to I9DOF
	unsigned		_baudrate;					///< current baudrate
	char			_port[20];					///< device / serial port path

	struct accel_report 	_i9dof_accel_report;
	struct gyro_report 		_i9dof_gyro_report;
	struct mag_report 		_i9dof_mag_report;

	int				_class_instance;
	int				_accel_class_instance;
	int				_gyro_class_instance;
	perf_counter_t	_bad_transfers;			/**< loop performance counter */
	perf_counter_t	_loop_perf;			/**< loop performance counter */


	i9dof_decode_state_e	_decode_state;
	uint16_t 		_rx_crc;
	uint16_t		_rx_msg;
	uint16_t		_rx_class;
	uint16_t		_rx_payload_length;
	uint16_t		_rx_payload_index;

    void 			stop();
	static void		task_main_trampoline(int argc, char *argv[]);
	void			task_main();
	int 			receive(const unsigned timeout);
	int				set_baudrate(const int &fd, unsigned baud);
	void 			decode_init(void);
	int 			parse_char(const uint8_t b);
	int 			payload_rx_imu_add(const uint8_t b);
	int 			payload_rx_att_add(const uint8_t b);
	int 			payload_rx_done(void);

	uint32 			get_baudrate(uint32 baudRate);
	I9dof_Error_Code_E 			Create_Serial(const char *deviceName,uint32 baudRate,int *fd);
	I9dof_Error_Code_E 			Destroy_Serial(int *fd);

	/* do not allow to copy due to ptr data members */
	I9DOF(const I9DOF&);
	I9DOF operator=(const I9DOF&);
}

namespace i9dof
{
	I9DOF *g_i9dof = nullptr;
}

I9DOF::I9DOF():
	CDev("I9DOF",PX4_I9DOF_DEVICE_PATH),
	_task_should_exit(true),
	_i9dof_task(-1),
	_bad_transfers(perf_alloc(PC_COUNT, "i9dof_bad_transfers")),
	_loop_perf(perf_alloc(PC_ELAPSED, "sensor task update"))
	{

	}

I9DOF::~I9DOF()
{
	if(_i9dof_task != -1){
		_task_should_exit = true;
		unsigned i = 0;
		do{
			/* wait 20ms*/
			usleep(20000);
			if(++i > 50){
				task_delete(_i9dof_task);
				break;
			}
		}while(_i9dof_task != -1);
	}
	perf_free(_bad_transfers);

	if(_class_instance != -1)
		unregister_class_devname(I9DOF0_DEVICE_PATH, _class_instance);
	if (_accel_class_instance != -1)
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	if (_gyro_class_instance != -1)
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);

	i9dof::g_i9dof = nullptr;
}

int 
I9DOF::init()
{
	int ret;
	ASSERT(_i9dof_task == -1);  // debug,print __FILE__ , __LINE__

	/* do regular cdev init*/
	ret = CDev::init();
	if(ret != OK)
		return ret;

	_class_instance = register_class_devname(I9DOF0_DEVICE_PATH)
	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);
	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	/* start the IO interface task  2000*/
	_i9dof_task = px4_task_spawn_cmd("i9dof",
		SCHED_FIFO,
		241,
		2000,
		(main_t)&I9DOF::task_main_trampoline,
		nullptr);

	if(_i9dof_task < 0){
		warnx("task start failed: %d",errno);
		return -errno;
	}

	return OK;
}

void
I9DOF::task_main_trampoline(int argc, char *argv[])
{
	i9dof::g_i9dof->task_main();
}

void
I9DOF::task_main()
{
	I9dof_Error_Code_E 		ret_tmp;
	const uint32_t baudRate = 115200;
	int rec = 0;
	uint64_t cnt = 0;

	ret_tmp = Create_Serial(I9DOF_SERIAL_DEVICE,baudRate,&_serial_fd);
	i9dof_debug("open:%s ,baudRate:%d ,serial_fd:%d ,errorcode:%d \n",I9DOF_SERIAL_DEVICE,baudRate,_serial_fd,ret_tmp);
	if(ret_tmp == I9DOF_NO_ERROR){
		decode_init();
		_task_should_exit = false;
	}
	while(!_task_should_exit){
		perf_begin(_loop_perf);
		cnt++;
		rec = receive(20);
		if(cnt % 400 == 0)
			i9dof_debug("i9dof_poll,receive:%d",rec);
	//	usleep(1000);
		perf_end(_loop_perf);
	}
	Destroy_Serial(&_serial_fd);
	i9dof_debug("i9dof_poll EXIT");
	_i9dof_task = -1;
}
uint32 I9DOF::get_baudrate(uint32 baudRate)
{
	uint32 baudRateConst;
	
	//
	// Create the right baud rate value for unix platforms
	//
	switch (baudRate)
	{
		case 9600:
			baudRateConst = B9600;
			break;
		case 19200:
			baudRateConst = B19200;
			break;
#ifdef B38400
		case 38400:
			baudRateConst = B38400;
			break;
#endif
#ifdef B57600
		case 57600:
			baudRateConst = B57600;
			break;
#endif
#ifdef B115200
		case 115200:
			baudRateConst = B115200;
			break;
#endif
#ifdef B230400
		case 230400:
			baudRateConst = B230400;
			break;
#endif
#ifdef B460800
		case 460800:
			baudRateConst = B460800;
			break;
#endif
#ifdef B921600
		case 921600:
			baudRateConst = B921600;
			break;
#endif
		default:
			baudRateConst = baudRate;
	}

	return baudRateConst;
}

/**
 * [I9DOF::Create_Serial description]
 * @author Brown 2016-12-28
 * @param  deviceName [Serial interface location , /dev/ttyS6]
 * @param  baudRate   [Serial interface baud rate in bps]
 * @param  fd         [Opne return fd]
 * @return            [I9DOF_NO_ERROR if the interface has been created.]
 */
I9dof_Error_Code_E I9DOF::Create_Serial(const char *deviceName,uint32 baudRate,int *fd)
{
	int32_t 		hSerialHandle;
	struct termios 	options;
	uint32 			baudRateConst;

	if(deviceName)
	{
		baudRateConst = get_baudrate(baudRate);
		i9dof_debug("serial baudrate:%d",baudRateConst);

		hSerialHandle = open(deviceName,O_RDWR|O_NOCTTY|O_NDELAY)；
     //   *fd = hSerialHandle;
		i9dof_debug("open serial: %s,hSerialHandle:%d",deviceName,hSerialHandle);

		if(hSerialHandle != -1)
		{
			if(fcntl(hSerialHandle,F_SETFL,O_NONBLOCK) != -1)
			{
				if(tcgetattr(hSerialHandle,&options) != -1)
				{
					/* Define serial port options */
					options.c_cflag |= (CLOCAL | CREAD);  		// Enable the receiver and seet local mode...
					options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);	// No parity,1 stop bit, mask characher size bits
					options.c_cflag |= CS8; 					// Select 8 data bits
					options.c_cfalg &= ~CRTSCTS; 				// Disable Hardware flow control

					/* Disalbe software flow control */
					optons.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|IXON);

					/* We would like raw input */
					options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG/* | IEXTEN|ECHONL */);
					options.c_oflag &= ~OPOST;

					/* Set our timeout to 0 */
					options.c_cc[VMIN]  = 0;
					options.c_cc[VTIME] = 1;

					/* Set both input and output baud */
					if((cfsetispeed(&options,baudRateConst)!= -1) && (cfsetospeed(&options,baudRateConst) != -1))
					{
						/* Define options */
						if(tcsetattr(hSerialHandle,TCSANOW,&options) != -1)
						{
							*fd = hSerialHandle;
							return I9DOF_NO_ERROR;
						}else{
							i9dof_debug("error: serial creat : tcsetattr fails. \n");
						}
					}else{
						i9dof_debug("error: serial creat : Unable to set speed. \n");
					}

				}else{
					i9dof_debug("error: serial creat : tcgetattr fails. \n");
				}
			}else{
				i9dof_debug("error: serial creat : fcntl fails. \n");
			}
		}else{
			i9dof_debug("error: serial creat : Unable to open the serial port: %s \n",deviceName);
		}
		return I9DOF_ERROR;
	}else{
		return I9DOF_INVALID_PARAMETER;
	}

}

I9dof_Error_Code_E I9DOF::Destroy_Serial(int *fd)
{
	int32_t 	hSerialHandle;
	if(fd)
	{
		/* Get the internal serial fd */
		hSerialHandle = *fd;

		close(hSerialHandle);
		fd = NULL;

		return i9dof I9DOF_NO_ERROR;
	}else{
		return I9DOF_NULL_POINTER;
	}

}

I9dof_Error_Code_E I9DOF::Serial_Read(int *pHandle, void *pBuffer, uint32_t *pReadBytes, uint32_t bytesToRead)
{
	I9dof_Error_Code_E 	errorCode;
	int32_t 			hSerialHandle;
	int32_t 			numBytesRead;

	/* Test input parameters */
	if((pHandle)&&(pBuffer)&&(pReadBytes))
	{
		hSerialHandle = *pHandle;
		numBytesRead = read(hSerialHandle,pBuffer,bytesToRead); 	// Read our buffer

		if(numBytesRead >0)				// Check if we have at least one byte
		{
			errorCode = I9DOF_NO_ERROR;
		}else{
			errorCode = I9DOF_READ_ERROR;
			numBytesRead = 0;
		}

		if(pReadBytes)					// If we can, returns the number of read bytes
		{
			*pReadBytes = numBytesRead;
		}

	}else{
		errorCode = I9DOF_NULL_POINTER;
	}

	return errorCode;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
I9DOF::receive(const unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	static uint64_t cnt = 0;

	uint8_t buf[256];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	uint32_t count = 0;

	int handled = 0;

	while (true) {

		/* poll for new data, wait for only UBX_PACKET_TIMEOUT (2ms) if something already received */
	//	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 10);

		cnt++;
		/*
		 * We are here because poll says there is some data, so this
		 * won't block even on a blocking device. But don't read immediately
		 * by 1-2 bytes, wait for some more data to save expensive read() calls.
		 * If more bytes are available, we'll go back to poll() again.
		 */
		usleep(5 * 1000);
		Serial_Read(&_serial_fd,buf,&count,sizeof(buf));
		
		if(cnt%400==0)	
			i9dof_debug("sbgInterfaceSerialRead bytes: %d, count down: %lld",count,hrt_absolute_time());

		for (uint32_t i = 0; i < count; i++) {
			handled  |= parse_char(buf[i]);
		}
		if(handled)
			return handled;

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			return -1;
		}
	}

}

/* Decoder state */
void
I9DOF::decode_init(void)
{
	_decode_state = I9DOF_DECODE_SYNC1;
	_rx_crc = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
}
/*	
I9DOF_DECODE_SYNC1 = 0,
	I9DOF_DECODE_SYNC2,
	I9DOF_DECODE_LENGTH,
	I9DOF_DECODE_CLASS,
	I9DOF_DECODE_PAYLOAD,
	I9DOF_DECODE_CRC,
	I9DOF_DECODE_ETX
	*/
int
I9DOF::parse_char(const uint8_t byte)
{
	int ret = 0;
	static uint32_t i9dof_crc;
	switch(_decode_state){
		case I9DOF_DECODE_SYNC1:
			if(byte == I9DOF_SYNC1){
				decode_init();
				_decode_state = I9DOF_DECODE_SYNC2;
			}		
			break;
		case I9DOF_DECODE_SYNC2:
			if(byte == I9DOF_SYNC2){
				_decode_state = I9DOF_DECODE_LENGTH;				
			}else{
				decode_init();
			}
			break;
		case I9DOF_DECODE_LENGTH:
			_rx_payload_length = byte;
			_decode_state = I9DOF_DECODE_CLASS;				
			break;
		case I9DOF_DECODE_CLASS:
			_rx_msg = byte;
			_decode_state = I9DOF_DECODE_PAYLOAD;				
		
			break;
		case I9DOF_DECODE_PAYLOAD:
			switch(_rx_msg){
				case I9DOF_IMU_DATA: 
					ret = payload_rx_att_add(byte);
				break;
				case I9DOF_ATT_DATA: 
					ret = payload_rx_imu_add(byte);					
				break;
			}
			if
			break;
		case I9DOF_DECODE_CRC:
			if(_rx_crc != byte){
				i9dof_debug("i9dof checksum err, rx_crc:0x%04x, b:0x%04x",_rx_crc,byte);
			}else{
				ret = payload_rx_done();
			}
				decode_init();				
			break;
		default:
			break;

	}

	return ret;

}

/**
 * [I9DOF::payload_rx_att_add description]
 * @author Brown 2016-12-23
 * @param  byte [serial receive byte add to buffer]
 * @return      [-1 = error, 0 = ok, 1 = payload completed]
 */
int I9DOF::payload_rx_att_add(const uint8_t byte)
{
	int ret = 0;
	i9dof_att_u.raw[_rx_payload_index] = byte;
	if(_rx_payload_length >12){
		i9dof_debug("i9dof att data length error, length:%d,require length:12",_rx_payload_length);
		ret = -1;
	}
	if(++_rx_payload_index >= _rx_payload_length){
		ret = 1;
	}
	return ret;
}

/**
 * [I9DOF::payload_rx_imu_add description]
 * @author Brown 2016-12-23
 * @param  byte [serial receive byte add to buffer]
 * @return      [-1 = error, 0 = ok, 1 = payload completed]
 */
int	I9DOF::payload_rx_imu_add(const uint8_t byte)
{
	int ret = 0;
	i9dof_imu_u.raw[_rx_payload_index] = byte;
	if(_rx_payload_length >16){
		i9dof_debug("i9dof imu data length error, length:%d,require length:16",_rx_payload_length);
		ret = -1;
	}
	if(++_rx_payload_index >= _rx_payload_length){
		ret = 1;
	}
	return ret;
}

/**
 * [I9DOF::payload_rx_done description]
 * @author Brown 2016-12-23
 * @return  [0 = no message handled, 1 = message handled, 2 = sat info message handled]
 */
int I9DOF::payload_rx_done(void)
{
	int ret = 0;
	static uint64_t ptr_cnt;
	ptr_cnt++;

	switch(_rx_msg){
		case I9DOF_IMU_DATA:

			_i9dof_accel_report.x = i9dof_imu_u.imu_data.acc_x/Sensitive_Accel;
			_i9dof_accel_report.y = i9dof_imu_u.imu_data.acc_y/Sensitive_Accel;
			_i9dof_accel_report.z = i9dof_imu_u.imu_data.acc_z/Sensitive_Accel;
			_i9dof_gyro_report.x = i9dof_imu_u.imu_data.gyro_x/Sensitive_Gyro;
			_i9dof_gyro_report.y = i9dof_imu_u.imu_data.gyro_y/Sensitive_Gyro;
			_i9dof_gyro_report.z = i9dof_imu_u.imu_data.gyro_z/Sensitive_Gyro;
			_i9dof_mag_report.x = i9dof_imu_u.imu_data.mag_x;
			_i9dof_mag_report.y = i9dof_imu_u.imu_data.mag_y;
			_i9dof_mag_report.z = i9dof_imu_u.imu_data.mag_z;
	
			_i9dof_accel_report.timestamp = _i9dof_gyro_report.timestamp = _i9dof_mag_report.timestamp = hrt_absolute_time();

			if(ptr_cnt%200 == 0){
				i9dof_debug("receive IMU data\n")
			}

			break；

		case I9DOF_ATT_DATA:

			float yaw = (float)i9dof_imu_u.att_data.yaw;
			float roll = (float)i9dof_imu_u.att_data.roll;
			float pitch = (float)i9dof_imu_u.att_data.pitch;
			float altitude = (float)i9dof_imu_u.att_data.altitude;
			float temperature = (float)i9dof_imu_u.att_data.temperature;
			float pressure = (float)i9dof_imu_u.att_data.pressure;
			int16_t calc_time = i9dof_imu_u.att_data.calc_time;
	
			if(ptr_cnt%200 == 0){
				i9dof_debug("receive ATT data\n")
				warnx("roll: %2.2f , pitch: %2.2f , yaw: %3.2f \n",yaw,roll,pitch);
				warnx("temperature: %2.2f , altitude: %2.2f , pressure: %3.2f , calc_time: %5d \n",temperature,altitude,pressure,calc_time);
			}

			break；
			
	}
}











