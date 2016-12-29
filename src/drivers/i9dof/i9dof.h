
#ifndef __I9DOF_H
#define __I9DOF_H

#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>

#define I9DOF_DEVICE_PATH "/dev/i9dof"
#define PX4_I9DOF_DEVICE_PATH	"/dev/px4_i9dof"

#define I9DOF0_DEVICE_PATH	"/dev/i9dof"
#define I9DOF_DEFAULT_UART_PORT "/dev/ttyS4"

<<<<<<< HEAD
=======
#define I9DOF_SERIAL_DEVICE	"/dev/ttyS6"

>>>>>>> i9dof_imu-1
#define I9DOF_SYNC1  0xA5
#define I9DOF_SYNC2  0x5A
#define I9DOF_EXT    0xAA

#define I9DOF_IMU_DATA 0xA2
#define I9DOF_ATT_DATA 0xA1

//基准电压+5.000V  16位ADC ,对应 [13.107  LBS/mV]
#define Sensitive_Accel  13107.0f        //加速度灵敏度[1000mV/g]
#define Sensitive_Gyro   78.642f         //陀螺仪灵敏度[6mV/°/sec] 

/* Decoder state */
typedef enum{
	I9DOF_DECODE_SYNC1 = 0,
	I9DOF_DECODE_SYNC2,
	I9DOF_DECODE_LENGTH,
	I9DOF_DECODE_CLASS,
	I9DOF_DECODE_PAYLOAD,
	I9DOF_DECODE_CRC,
	I9DOF_DECODE_ETX
}i9dof_decode_state_t;

/* ACK state */
typedef enum {
	I9DOF_ACK_IDLE = 0,
	I9DOF_ACK_WAITING,
	I9DOF_ACK_GOT_ACK,
	I9DOF_ACK_GOT_NAK
} i9dof_ack_state_e;

typedef enum {
	I9DOF_NO_ERROR = 0,
	I9DOF_ERROR,
	I9DOF_NULL_POINTER,
<<<<<<< HEAD
	I9DOF_INVALID_PARAMETER

=======
	I9DOF_INVALID_PARAMETER,
	
	I9DOF_READ_ERROR
>>>>>>> i9dof_imu-1
}I9dof_Error_Eode_E;

struct PACKED i9dof_imu_s{
	int16_t 	acc_x;
	int16_t 	acc_y;
	int16_t 	acc_z;
	int16_t 	acc_z;
	int16_t 	gyro_x;
	int16_t 	gyro_y;
	int16_t 	gyro_z;
	int16_t 	mag_x;
	int16_t 	mag_y;
	int16_t 	mag_z;
};
union PACKED{
	i9dof_imu_s 	imu_data;
	uint8_t 		raw[];
}i9dof_imu_u;

struct PACKED i9dof_att_s{
	int16_t 	yaw;
	int16_t 	pitch;
	int16_t 	roll;
	int16_t 	altitude;
	int16_t 	temperature;
	int16_t 	pressure;
	int16_t 	calc_time;	// calculat attitude frequency,per second
};
union PACKED{
	i9dof_att_s 	att_data;
	uint8_t 		raw[];
}i9dof_att_u;

#endif  
