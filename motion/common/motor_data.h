#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H
extern "C"{
typedef struct{
	double* x_speed;
	double* y_speed;
	double* yaw_speed;
	int* shoot_power;
    unsigned char* hold_ball;
}robot_command;

typedef struct{
	unsigned char* head1;
	unsigned char* head2;
	unsigned char* w1;
	unsigned char* w2;
	unsigned char* w3;
	unsigned char* w4;
	unsigned char* enable_and_stop;
	unsigned char* shoot;
	unsigned char* checksum;
	unsigned char* safe1;
	unsigned char* safe2;
	unsigned char* safe3;
	unsigned char* crc_16_1;
	unsigned char* crc_16_2;
}serial_tx;

typedef struct{
	unsigned char* head1;
	unsigned char* head2;
	unsigned char* w1;
	unsigned char* w2;
	unsigned char* w3;
	unsigned char* w4;
	unsigned char* enable_and_stop;
	unsigned char* shoot;
	unsigned char* checksum;
	unsigned char* safe1;
	unsigned char* safe2;
	unsigned char* safe3;
	unsigned char* batery;
	unsigned char* infrared;
	
}serial_rx;

//typedef struct MOTOR_FEEDBACK motor_feedback;
//typedef struct MOTOR_COMMAND motor_command;
}
#endif 

