#include "base_control.h"
#define MECANUM
// #define DEBUG_CSSL
//#include "../common/cssl/cssl.h"
#include "cssl.c"
#include "port.h"

serial_rx* Base_Control::base_RX = NULL;
Base_Control::Base_Control()
{
	/*
		Declare and initalize the serial rx and tx data 
	*/
	this->base_robotFB = new robot_command;
	this->base_robotFB->x_speed = new double;
	this->base_robotFB->y_speed = new double;
	this->base_robotFB->yaw_speed = new double;
	this->base_robotFB->shoot_power = new int;
	std::memset(this->base_robotFB->x_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->y_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->yaw_speed, 0, sizeof(double));
	std::memset(this->base_robotFB->shoot_power, 0, sizeof(int));

	Base_Control::base_RX = new serial_rx;

	Base_Control::base_RX->head1 			= new unsigned char;
	Base_Control::base_RX->head2 			= new unsigned char;
	Base_Control::base_RX->w1	 			= new unsigned char;
	Base_Control::base_RX->w2	 			= new unsigned char;
	Base_Control::base_RX->w3   			= new unsigned char;
	Base_Control::base_RX->w4 				= new unsigned char;
	Base_Control::base_RX->enable_and_stop 	= new unsigned char;
	Base_Control::base_RX->shoot			= new unsigned char;
	Base_Control::base_RX->checksum 		= new unsigned char;
	Base_Control::base_RX->safe1 			= new unsigned char;
	Base_Control::base_RX->safe2 			= new unsigned char;
	Base_Control::base_RX->safe3 			= new unsigned char;
	Base_Control::base_RX->infrared			= new unsigned char;
	// Base_Control::base_RX->batery = new unsigned char;

	memset(Base_Control::base_RX->head1, 			0, sizeof(unsigned char));
	memset(Base_Control::base_RX->head2, 			0, sizeof(unsigned char));
	memset(Base_Control::base_RX->w1, 				0, sizeof(unsigned char));
	memset(Base_Control::base_RX->w2, 				0, sizeof(unsigned char));
	memset(Base_Control::base_RX->w3, 				0, sizeof(unsigned char));
	memset(Base_Control::base_RX->w4, 				0, sizeof(unsigned char));
	memset(Base_Control::base_RX->enable_and_stop, 	0, sizeof(unsigned char));
	memset(Base_Control::base_RX->shoot, 			0, sizeof(unsigned char));
	memset(Base_Control::base_RX->checksum, 		0, sizeof(unsigned char));
	memset(Base_Control::base_RX->safe1, 			0, sizeof(unsigned char));
	memset(Base_Control::base_RX->safe2, 			0, sizeof(unsigned char));
	memset(Base_Control::base_RX->safe3, 			0, sizeof(unsigned char));
	memset(Base_Control::base_RX->infrared, 			0, sizeof(unsigned char));
	// memset(Base_Control::base_RX->batery, 0, sizeof(unsigned char));

	this->base_TX = new serial_tx;
	this->base_TX->head1 = new unsigned char;
	this->base_TX->head2 = new unsigned char;
	this->base_TX->w1 = new unsigned char;
	this->base_TX->w2 = new unsigned char;
	this->base_TX->w3 = new unsigned char;
	this->base_TX->w4 = new unsigned char;
	this->base_TX->enable_and_stop = new unsigned char;
	this->base_TX->shoot = new unsigned char;
	this->base_TX->checksum = new unsigned char;
	this->base_TX->safe1 = new unsigned char;
	this->base_TX->safe2 = new unsigned char;
	this->base_TX->safe3 = new unsigned char;
	this->base_TX->crc_16_1 = new unsigned char;
	this->base_TX->crc_16_2 = new unsigned char;

	memset(this->base_TX->head1, 0x55, sizeof(unsigned char));
	memset(this->base_TX->head2, 0xaa, sizeof(unsigned char));
	// memset(this->base_TX->head1, 0xff, sizeof(unsigned char));
	// memset(this->base_TX->head2, 0xfa, sizeof(unsigned char));
	memset(this->base_TX->w1, 0, sizeof(unsigned char));
	memset(this->base_TX->w2, 0, sizeof(unsigned char));
	memset(this->base_TX->w3, 0, sizeof(unsigned char));
	memset(this->base_TX->w4, 0, sizeof(unsigned char));
	memset(this->base_TX->enable_and_stop, 0, sizeof(unsigned char));
	memset(this->base_TX->shoot, 0, sizeof(unsigned char));
	memset(this->base_TX->checksum, 0, sizeof(unsigned char));
	memset(this->base_TX->safe1, 0, sizeof(unsigned char));
	memset(this->base_TX->safe2, 0, sizeof(unsigned char));
	memset(this->base_TX->safe3, 0, sizeof(unsigned char));
	memset(this->base_TX->crc_16_1, 0, sizeof(unsigned char));
	memset(this->base_TX->crc_16_2, 0, sizeof(unsigned char));
	
	this->x_CMD = 0;
	this->y_CMD = 0;
	this->yaw_CMD = 0;
	this->serial = NULL;
#ifdef DEBUG
	std::cout << "Base_Control(DEBUG)\n";
	std::cout << "Init base control\n";
	/*std::cout << (int)*this->base_TX->head1 << std::endl;
	std::cout << (int)*this->base_TX->head2 << std::endl;
	std::cout << (int)*this->base_TX->w1 << std::endl;
	std::cout << (int)*this->base_TX->w2 << std::endl;
	std::cout << (int)*this->base_TX->w3 << std::endl;
	std::cout << (int)*this->base_TX->enable_and_stop << std::endl;
	std::cout << (int)*this->base_TX->shoot << std::endl;
	std::cout << (int)*this->base_TX->checksum << std::endl;*/
#endif
	mcssl_init();
}

Base_Control::~Base_Control()
{
#ifdef DEBUG
	std::cout << "~Base_Control(DEBUG)\n";
#endif
	mcssl_finish();
}

int Base_Control::mcssl_init()
{
	/*
		start using cssl, check and open the serial port.
	*/
	const char *devs;
#ifdef DEBUG_CSSL
	std::cout << "mcssl_init(DEBUG_CSSL)\n";
#else
	cssl_start();
	if(!serial){
		devs = "/dev/wrs/mobile";
		serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	}
	// if(!serial){
	// 	devs = "/dev/ttyUSB0";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB1";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB2";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB3";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB4";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB5";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB6";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB7";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB8";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB9";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	// if(!serial){
	// 	devs = "/dev/ttyUSB10";
	// 	serial = cssl_open(devs, mcssl_Callback/*NULL*/, 0, 115200, 8, 0, 1);
	// }
	if(!serial){
		std::cout << cssl_geterrormsg() << std::endl;
		std::cout << "----> ATTACK MOTION RS232 OPEN FAILED <----\n";
		fflush(stdout);
		//return 0;
		std::cout << devs << std::endl;
		exit(EXIT_FAILURE);
	}else{
		std::cout << "----> ATTACK MOTION RS232 OPEN SUCCESSFUL <----\n";
		std::cout << "Initialize attack motion with port = "<< devs << "...\n";
		cssl_setflowcontrol(serial, 0, 0);
	}
	//serial->callback = &Base_Control::mcssl_Callback;
#endif
	return 1;
}

void Base_Control::mcssl_finish()
{
	/*
		Close the serial port when the process done.
	*/
#ifdef DEBUG_CSSL
	std::cout << "mcssl_finish(DEBUG_CSSL)\n";
#else
	cssl_close(serial);
	cssl_stop();
#endif
}

void Base_Control::mcssl_Callback(int id, uint8_t* buf, int length)
{
	/*
		Receive the motor feed back;
	*/
	// printf("call_back\n");
#ifdef OMNIDIRECTIONAL
	static unsigned char cssl_buffer[50]={0};
	static int count_buffer=0;
	cssl_buffer[count_buffer++] = *buf;
	count_buffer = (count_buffer)%50;
	unsigned char checksum;
	bool error = true;
	int i;
	for(i=0; i<30; i++){
		if((cssl_buffer[i]==0xff)&&(cssl_buffer[i+1]==0xfa)&&(cssl_buffer[i+15]==0xff)&&(cssl_buffer[i+16])==0xfa){
			checksum = cssl_buffer[i+2]+cssl_buffer[i+3]+cssl_buffer[i+4]+cssl_buffer[i+5]+cssl_buffer[i+6]+cssl_buffer[i+7]+cssl_buffer[i+8]+cssl_buffer[i+9]+cssl_buffer[i+10]+cssl_buffer[i+11]+cssl_buffer[i+12]+cssl_buffer[i+13];
			if(cssl_buffer[i+14]==checksum){
				*(base_RX->head1) = cssl_buffer[i];
				*(base_RX->head2) = cssl_buffer[i+1];
				*(base_RX->w1) = (cssl_buffer[i+2]<<24)+(cssl_buffer[i+3]<<16)+(cssl_buffer[i+4]<<8)+(cssl_buffer[i+5]);
				*(base_RX->w2) = (cssl_buffer[i+6]<<24)+(cssl_buffer[i+7]<<16)+(cssl_buffer[i+8]<<8)+(cssl_buffer[i+9]);
				*(base_RX->w3) = (cssl_buffer[i+10]<<24)+(cssl_buffer[i+11]<<16)+(cssl_buffer[i+12]<<8)+(cssl_buffer[i+13]);
				*(base_RX->shoot) = 0;
				// *(base_RX->batery) = 0;
				error = false;
				break;
			}else{
				continue;
			}
		}else{
			continue;
		}
	}
#endif
	static unsigned char cssl_buffer[50]={0};
	static int count_buffer=0;
	cssl_buffer[count_buffer++] = *buf;
	count_buffer = (count_buffer)%50;
	for (int i=0;i<50;i++){
		if((cssl_buffer[i]==0xff) && (cssl_buffer[i+1]==0xfa) && (cssl_buffer[i+19]==0xff) && (cssl_buffer[i+20])==0xfa){
			unsigned char checksum = cssl_buffer[i+2]+cssl_buffer[i+3]+cssl_buffer[i+4]+cssl_buffer[i+5]+\
			cssl_buffer[i+6]+cssl_buffer[i+7]+cssl_buffer[i+8]+cssl_buffer[i+9]+cssl_buffer[i+10]+\
			cssl_buffer[i+11]+cssl_buffer[i+12]+cssl_buffer[i+13]+cssl_buffer[i+14]+cssl_buffer[i+15]\
			+cssl_buffer[i+16]+cssl_buffer[i+17]+cssl_buffer[i+18]+cssl_buffer[i+19];
			if(cssl_buffer[i+20]==checksum){
				*(base_RX->head1) = cssl_buffer[i];
				*(base_RX->head2) = cssl_buffer[i+1];
				*(base_RX->w1) = (cssl_buffer[i+2]<<24)+(cssl_buffer[i+3]<<16)+(cssl_buffer[i+4]<<8)+(cssl_buffer[i+5]);
				*(base_RX->w2) = (cssl_buffer[i+6]<<24)+(cssl_buffer[i+7]<<16)+(cssl_buffer[i+8]<<8)+(cssl_buffer[i+9]);
				*(base_RX->w3) = (cssl_buffer[i+10]<<24)+(cssl_buffer[i+11]<<16)+(cssl_buffer[i+12]<<8)+(cssl_buffer[i+13]);
				*(base_RX->w4) = (cssl_buffer[i+14]<<24)+(cssl_buffer[i+15]<<16)+(cssl_buffer[i+16]<<8)+(cssl_buffer[i+17]);
				*(base_RX->infrared) = (cssl_buffer[i+18]);
				// printf("head1: %x\nhead2: %x\nw1: %d\nw2: %d\nw3: %d\nw4: %d\ninfrared: %x\n", *(base_RX->head1), *(base_RX->head2), int(*(base_RX->w1)), \
				int(*(base_RX->w2)), int(*(base_RX->w3)), int(*(base_RX->w4)), *(base_RX->infrared));
				break;
			}
			else{
				continue;
			}
		}else{
			continue;
		}
	}
#ifdef DEBUG_CSSLCALLBACK_TEST
	double x,y,z,yaw;
	int round;
	x = (*(base_RX->w1)*(-0.3333) + *(base_RX->w2)*(-0.3333) + *(base_RX->w3)*(0.6667))*2*M_PI*0.0508/(26)/2000;
    y = (*(base_RX->w1)*(0.5774) + *(base_RX->w2)*(-0.5774) + *(base_RX->w3)*(0))*2*M_PI*0.0508/26/2000;
	z = (*(base_RX->w1)*(2.3251) + *(base_RX->w2)*(2.3251) + *(base_RX->w3)*(2.3251))*2*M_PI*0.0508/2000/26;
	round = yaw/(2*M_PI);
	yaw = (z - round*2*M_PI);
	if(x>1){
		std::cout << "mcssl_Callback(DEBUG_CSSLCALLBACK)\n";
		std::cout << "Forward Kinematics\n";
		std::cout << std::dec;
		std::cout << "x: " << x << "\t";
		std::cout << "y: " << y << "\t";
		std::cout << "yaw: "<< yaw << std::endl;
		std::cout << std::endl;
		for(int j=0; j<50; j++){
			std::cout << std::hex << (int)cssl_buffer[j] << " ";
		}
		std::cout << std::endl;
		std::cout << std::dec << "RX(" << i << "): ";
		std::cout << std::hex;
		for(int j=i; j<i+15; j++){
			std::cout << std::hex << (int)cssl_buffer[j] << " ";
		}
		std::cout << std::endl << std::hex;
		std::cout << "head1: " << (int)*(base_RX->head1) << "\n";
		std::cout << "head2: " << (int)*(base_RX->head2) << "\n";
		std::cout << "w1: " << (int)*(base_RX->w1) << "\n";
		std::cout << "w2: " << (int)*(base_RX->w2) << "\n";
		std::cout << "w3: " << (int)*(base_RX->w3) << "\n";
		std::cout << "shoot: " << (int)*(base_RX->shoot) << "\n";
		std::cout << "batery: " << (int)*(base_RX->batery) << "\n";
		std::cout << std::endl;
		exit(1);
	}
#endif
#ifdef DEBUG_CSSLCALLBACK
	std::cout << "mcssl_Callback(DEBUG_CSSLCALLBACK)\n";
	std::cout << std::hex;
	std::cout << "buf: " << (int)*(buf) << "\n";
	std::cout << "buf: " << (int)*(buf) << "\n";
	std::cout << "RX SERIAL: ";
	for(int j=0; j<50; j++){
		std::cout << (int)cssl_buffer[j] << " ";
	}
	std::cout << std::endl;
	std::cout << std::dec << "RX(" << i << "): ";
	if(!error){
		std::cout << std::hex;
		for(int j=i; j<i+15; j++){
			std::cout << std::hex << (int)cssl_buffer[j] << " ";
		}
	}else{
		std::cout << "=========->ERROR<========";
	}
	std::cout << std::endl << std::dec;
	std::cout << "head1: " << (int)*(base_RX->head1) << "\n";
	std::cout << "head2: " << (int)*(base_RX->head2) << "\n";
	std::cout << "w1: " << (int)*(base_RX->w1) << "\n";
	std::cout << "w2: " << (int)*(base_RX->w2) << "\n";
	std::cout << "w3: " << (int)*(base_RX->w3) << "\n";
	std::cout << "shoot: " << (int)*(base_RX->shoot) << "\n";
	std::cout << "batery: " << (int)*(base_RX->batery) << "\n";
	std::cout << std::endl;
#else
#endif

}

void Base_Control::mcssl_send2motor()
{	
	/*
		Send the motor speed control to the control board through RS232 by using libcssl
	*/
	unsigned char com_data[]={	*(this->base_TX->head1), *(this->base_TX->head2),
								*(this->base_TX->w1), *(this->base_TX->w2), 
								*(this->base_TX->w3), *(this->base_TX->w4), 
								*(this->base_TX->enable_and_stop)};
	int size = int(sizeof(com_data)/sizeof(unsigned char));
	Crc_16 crc16(com_data, size);
	unsigned short crc = crc16.getCrc();
	this->base_TX->crc_16_1 = ((unsigned char*)(&crc) + 1);
	this->base_TX->crc_16_2 = ((unsigned char*)(&crc) + 0);
	*this->base_TX->checksum = *(this->base_TX->head1) + *(this->base_TX->head2) + *(this->base_TX->w1) + *(this->base_TX->w2) + *(this->base_TX->w3) + *(this->base_TX->w4) + *(this->base_TX->enable_and_stop) + *(this->base_TX->crc_16_1) + *(this->base_TX->crc_16_2);
	int aaa(*(this->base_TX->head1) + *(this->base_TX->head2) + *(this->base_TX->w1) + *(this->base_TX->w2) + *(this->base_TX->w3) + *(this->base_TX->w4) + *(this->base_TX->enable_and_stop) + *(this->base_TX->crc_16_1) + *(this->base_TX->crc_16_2));
#ifdef DEBUG_CSSL
	std::cout << "mcssl_send2motor(DEBUG_CSSL)\n";
	std::cout << std::hex;
	std::cout << "head1: " 				<< (int)*(this->base_TX->head1) 			<< std::endl;
	std::cout << "head2: " 				<< (int)*(this->base_TX->head2) 			<< std::endl;
	std::cout << "w1: " 				<< (int)*(this->base_TX->w1) 				<< std::endl;
	std::cout << "w2: " 				<< (int)*(this->base_TX->w2) 				<< std::endl;
	std::cout << "w3: " 				<< (int)*(this->base_TX->w3) 				<< std::endl;
	std::cout << "w4: " 				<< (int)*(this->base_TX->w4) 				<< std::endl;
	std::cout << "enable_and_stop: " 	<< (int)*(this->base_TX->enable_and_stop) 	<< std::endl;
	std::cout << "shoot: " 				<< (int)*(this->base_TX->shoot) 			<< std::endl;
	std::cout << "checksum: " 			<< (int)*(this->base_TX->checksum) 			<< std::endl;
	std::cout << "safe1: " 				<< (int)*(this->base_TX->safe1) 			<< std::endl;
	std::cout << "safe2: "				<< (int)*(this->base_TX->safe2) 			<< std::endl;
	std::cout << "safe3: "				<< (int)*(this->base_TX->safe3) 			<< std::endl;
	std::cout << "cssl error: "			<< cssl_geterrormsg() 						<< std::endl;
#else
#ifdef DEBUG
	std::cout << "mcssl_send2motor(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "w1: " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "w2: " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "w3: " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "w4: " << (int)*(this->base_TX->w4) << std::endl;
	std::cout << "checksum: " << (int)*(this->base_TX->checksum) << std::endl;
	std::cout << std::endl;
#endif
	uint8_t buffer[]={	*(this->base_TX->head1), *(this->base_TX->head2),
						*(this->base_TX->w1), *(this->base_TX->w2),
						*(this->base_TX->w3), *(this->base_TX->w4),
						*(this->base_TX->enable_and_stop), *(this->base_TX->crc_16_1),
						*(this->base_TX->crc_16_2), *(this->base_TX->checksum)};
	cssl_putdata(serial,buffer,int(sizeof(buffer)/sizeof(uint8_t)));
	cssl_putdata(serial,buffer,int(sizeof(buffer)/sizeof(uint8_t)));
	// cssl_putchar(serial, *(this->base_TX->head1));
	// cssl_putchar(serial, *(this->base_TX->head2));
	// cssl_putchar(serial, *(this->base_TX->w1));
	// cssl_putchar(serial, *(this->base_TX->w2));
	// cssl_putchar(serial, *(this->base_TX->w3));
	// cssl_putchar(serial, *(this->base_TX->w4));
	// cssl_putchar(serial, *(this->base_TX->enable_and_stop));
	// cssl_putchar(serial, *(this->base_TX->crc_16_1));
	// cssl_putchar(serial, *(this->base_TX->crc_16_2));
	// cssl_putchar(serial, *(this->base_TX->checksum));

	printf("**************************\n");
	printf("* mcssl_send(DEBUG_CSSL) *\n");
	printf("**************************\n");
	printf("head1: %x\n", *(this->base_TX->head1));
	printf("head2: %x\n", *(this->base_TX->head2));
	printf("w1: %x\n", *(this->base_TX->w1));
	printf("w2: %x\n", *(this->base_TX->w2));
	printf("w3: %x\n", *(this->base_TX->w3));
	printf("w4: %x\n", *(this->base_TX->w4));
	printf("enable_and_stop: %x\n", *(this->base_TX->enable_and_stop));
	printf("crc16-1: %x\n", *(this->base_TX->crc_16_1));
	printf("crc16-2: %x\n", *(this->base_TX->crc_16_2));
	printf("checksum: %x\n", *(this->base_TX->checksum));
	
#endif
}

void Base_Control::shoot_regularization()
{
	if(*(this->base_robotCMD->shoot_power)==0){
		*(this->base_TX->shoot) = 1;
	}else if(*(this->base_robotCMD->shoot_power)>=100){
		*(this->base_TX->shoot) = 255;
	}
	else{
		*(this->base_TX->shoot) = (255**(this->base_robotCMD->shoot_power)/100);
	}
#ifdef DEBUG
	std::cout << "shoot_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "shoot byte(hex): " << (int)*(this->base_TX->shoot) << std::endl;
	std::cout << std::endl;
#endif
}

void Base_Control::speed_regularization(double w1, double w2, double w3, double w4)
{
	/*
		Regular the speed and set the minimum speed 
		param:
			speed_max: the limit of wheel 				(default: 2000)
			min_scale:	the percent of min wheel speed	(default: 5%)
			acc_scale: the accelerate limit of wheel	(default: 1  velocity/loop_rate)
			acc_limit: the accelerate limit of wheel	(default: 1  velocity/loop_rate)
 	*/
	int speed_max = 2000;
	int min_scale = 5;
	int speed_min = (speed_max/100)*min_scale;
	int acc_scale(3), acc_limit((speed_max/100)*acc_scale);
	static double w1_old(w1), w2_old(w2), w3_old(w3), w4_old(w4);

	if(fabs(w1-w1_old)>acc_limit) (w1-w1_old > 0)? w1 = w1_old+acc_limit : w1 = w1_old-acc_limit;
	if(fabs(w2-w2_old)>acc_limit) (w2-w2_old > 0)? w2 = w2_old+acc_limit : w2 = w2_old-acc_limit;
	if(fabs(w3-w3_old)>acc_limit) (w3-w3_old > 0)? w3 = w3_old+acc_limit : w3 = w3_old-acc_limit;
	if(fabs(w4-w4_old)>acc_limit) (w4-w4_old > 0)? w4 = w4_old+acc_limit : w4 = w4_old-acc_limit;

	unsigned char w1_dir = (w1<0)? 0x80 : 0;
	unsigned char w2_dir = (w2<0)? 0 : 0x80;	// accroding to the direction of motor
	unsigned char w3_dir = (w3<0)? 0x80 : 0;
	unsigned char w4_dir = (w4<0)? 0 : 0x80;	// accroding to the direction of motor
	w1_old = w1;
	w2_old = w2;
	w3_old = w3;
	w4_old = w4;
	
#ifdef OMNIDIRECTIONAL
	if((w1_speed_percent>0.1) && (w1_speed_percent<5))w1_speed_percent=5;
	if((w2_speed_percent>0.1) && (w2_speed_percent<5))w2_speed_percent=5;
	if((w3_speed_percent>0.1) && (w3_speed_percent<5))w3_speed_percent=5;
	if((w4_speed_percent>0.1) && (w4_speed_percent<5))w4_speed_percent=5;
	if((w1_speed_percent>=100))w1_speed_percent=100;
	if((w2_speed_percent>=100))w2_speed_percent=100;
	if((w3_speed_percent>=100))w3_speed_percent=100;
	if((w4_speed_percent>=100))w4_speed_percent=100;
	*(this->base_TX->w1) = (w1_speed_percent>0)? (unsigned char)((127*w1_speed_percent/speed_max) + w1_dir) : 0x80;
	*(this->base_TX->w2) = (w2_speed_percent>0)? (unsigned char)((127*w2_speed_percent/speed_max) + w2_dir) : 0x80;
	*(this->base_TX->w3) = (w3_speed_percent>0)? (unsigned char)((127*w3_speed_percent/speed_max) + w3_dir) : 0x80;
	*(this->base_TX->w4) = (w4_speed_percent>0)? (unsigned char)((127*w4_speed_percent/speed_max) + w4_dir) : 0x80;
#endif
#ifdef MECANUM
	double w1_speed_percent = (fabs(w1)<0.1)? 0 : fabs(w1);
	double w2_speed_percent = (fabs(w2)<0.1)? 0 : fabs(w2);
	double w3_speed_percent = (fabs(w3)<0.1)? 0 : fabs(w3);
	double w4_speed_percent = (fabs(w4)<0.1)? 0 : fabs(w4);
	if((w1_speed_percent>0.1) && (w1_speed_percent<speed_min))w1_speed_percent = speed_min;
	if((w2_speed_percent>0.1) && (w2_speed_percent<speed_min))w2_speed_percent = speed_min;
	if((w3_speed_percent>0.1) && (w3_speed_percent<speed_min))w3_speed_percent = speed_min;
	if((w4_speed_percent>0.1) && (w4_speed_percent<speed_min))w4_speed_percent = speed_min;
#endif
	this->en1 = (w1_speed_percent > 0)? 1 : 0;
	this->en2 = (w2_speed_percent > 0)? 1 : 0;
	this->en3 = (w3_speed_percent > 0)? 1 : 0;
	this->en4 = (w3_speed_percent > 0)? 1 : 0;
	w1 == 0 ? this->stop1 = 1 : this->stop1 = 0;
	w1 == 0 ? this->stop2 = 1 : this->stop2 = 0;
	w1 == 0 ? this->stop3 = 1 : this->stop3 = 0;
	w1 == 0 ? this->stop4 = 1 : this->stop4 = 0; 

	*(this->base_TX->w1) = (w1_speed_percent>0)? (unsigned char)((127*w1_speed_percent/speed_max) + w1_dir) : 0x80;
	*(this->base_TX->w2) = (w2_speed_percent>0)? (unsigned char)((127*w2_speed_percent/speed_max) + w2_dir) : 0x80;
	*(this->base_TX->w3) = (w3_speed_percent>0)? (unsigned char)((127*w3_speed_percent/speed_max) + w3_dir) : 0x80;
	*(this->base_TX->w4) = (w4_speed_percent>0)? (unsigned char)((127*w4_speed_percent/speed_max) + w4_dir) : 0x80;

	*(this->base_TX->enable_and_stop) = (this->en1<<7)+(this->en2<<6)+(this->en3<<5)+
									(this->en4<<4)+(this->stop1<<3)+(this->stop2<<2)+
									(this->stop3<<1)+(this->stop4);
#ifdef DEBUG
	std::cout << "speed_regularization(DEBUG)\n";
	std::cout << std::hex;
	std::cout << "motor1 speed(hex): " << (int)*(this->base_TX->w1) << std::endl;
	std::cout << "motor2 speed(hex): " << (int)*(this->base_TX->w2) << std::endl;
	std::cout << "motor3 speed(hex): " << (int)*(this->base_TX->w3) << std::endl;
	std::cout << "motor4 speed(hex): " << (int)*(this->base_TX->w4) << std::endl;
	std::cout << std::hex;
	std::cout << std::endl;

#endif
}

void Base_Control::forwardKinematics()
{
	/*
		Mecanum forward kinematics
		math:
			[v_x				[1			1			1			1			[w1
			 v_y		=(r/4)*	-1			1			1			-1			 w2
			 v_yaw]				-1/(lx+ly)	1/(lx+ly)	-1/(lx+ly)	1/(lx+ly)]	 w3
																				 w4]
	*/
#ifdef MECANUM
// inverseKinematics for mecanum wheeled platform
	double Lx = 0.3;	// platform x size 
	double Ly = 0.4;	// platform y size
	double Rw = 0.05;	// platform wheeled radius
	*(this->base_robotFB->x_speed) = *(base_RX->w1) + *(base_RX->w2) + *(base_RX->w3) + *(base_RX->w4);
	*(this->base_robotFB->y_speed) = -*(base_RX->w1) + *(base_RX->w2) + *(base_RX->w3) - *(base_RX->w4);
	*(this->base_robotFB->yaw_speed) = -(1/(Lx+Ly)) * *(base_RX->w1) + (1/(Lx+Ly)) * *(base_RX->w2) - (1/(Lx+Ly)) * *(base_RX->w3) + (1/(Lx+Ly)) * *(base_RX->w4);
#endif
}

void Base_Control::inverseKinematics()
{
	/*
		Mecanum inverse kinematics
	*/
	double w1_speed, w2_speed, w3_speed, w4_speed;
	x_CMD = *(this->base_robotCMD->x_speed);
	y_CMD = *(this->base_robotCMD->y_speed);
	yaw_CMD = *(this->base_robotCMD->yaw_speed);
#ifdef OMNIDIRECTIONAL
	double x_error = *(this->base_robotCMD->x_speed) - x_CMD;
	double y_error = *(this->base_robotCMD->y_speed) - y_CMD;
	double yaw_error = *(this->base_robotCMD->yaw_speed) - yaw_CMD;
	if(x_error >= 0) x_CMD = (x_error>4)? x_CMD+4 :  *(this->base_robotCMD->x_speed);
	else x_CMD = (x_error<(-4))? x_CMD-4 :  *(this->base_robotCMD->x_speed);
	if(y_error >= 0) y_CMD = (y_error>4)? y_CMD+4 :  *(this->base_robotCMD->y_speed);
	else y_CMD = (y_error<(-4))? y_CMD-4 :  *(this->base_robotCMD->y_speed);
	if(yaw_error >= 0) yaw_CMD = (yaw_error>4)? yaw_CMD+4 :  *(this->base_robotCMD->yaw_speed);
	else yaw_CMD = (yaw_error<(-4))? yaw_CMD-4 :  *(this->base_robotCMD->yaw_speed);
#endif
	/*
		speed control
	*/

#ifdef OMNIDIRECTIONAL
// inverseKinematics for omnidirectional wheeled platform
	w1_speed = x_CMD*cos(m1_Angle)+y_CMD*sin(m1_Angle)+yaw_CMD*robot_radius*(-1);
	w2_speed = x_CMD*cos(m2_Angle)+y_CMD*sin(m2_Angle)+yaw_CMD*robot_radius*(-1);
	w3_speed = x_CMD*cos(m3_Angle)+y_CMD*sin(m3_Angle)+yaw_CMD*robot_radius*(-1);
	w4_speed = x_CMD*cos(m4_Angle)+y_CMD*sin(m4_Angle)+yaw_CMD*robot_radius*(-1);
	for(int i=0;i<10;i++){
		if(fabs(w1_speed)>100||fabs(w2_speed)>100||fabs(w3_speed>100)||fabs(w4_speed)>100){
			w1_speed = w1_speed*0.9;
			w2_speed = w2_speed*0.9;
			w3_speed = w3_speed*0.9;
			w4_speed = w4_speed*0.9;
		}else{
			w1_speed = w1_speed*7/5;
			w2_speed = w2_speed*7/5;
			w3_speed = w3_speed*7/5;
			w4_speed = w4_speed*7/5;
			break;
		}
	}
#endif
#ifdef MECANUM
// inverseKinematics for mecanum wheeled platform
	double Lx = 0.3;	// platform x size 
	double Ly = 0.4;	// platform y size
	double Rw = 0.05;	// platform wheeled radius
	w1_speed = (1/Rw)*(x_CMD-y_CMD-yaw_CMD*(Lx+Ly));
	w2_speed = (1/Rw)*(x_CMD+y_CMD+yaw_CMD*(Lx+Ly));
	w3_speed = (1/Rw)*(x_CMD+y_CMD-yaw_CMD*(Lx+Ly));
	w4_speed = (1/Rw)*(x_CMD-y_CMD+yaw_CMD*(Lx+Ly));
#endif

	speed_regularization(w1_speed, w2_speed, w3_speed, w4_speed);
#ifdef DEBUG
	std::cout << "Inverse kinematics(DEBUG)\n";
	std::cout << std::dec;
	std::cout << "x_speed CMD: " << *(base_robotCMD->x_speed) << std::endl;
	std::cout << "y_speed CMD: " << *(base_robotCMD->y_speed) << std::endl;
	std::cout << "yaw_speed CMD: " << *(base_robotCMD->yaw_speed) << std::endl;
	std::cout << "w1_speed(%): " << w1_speed << std::endl;
	std::cout << "w2_speed(%): " << w2_speed << std::endl;
	std::cout << "w3_speed(%): " << w3_speed << std::endl;
	std::cout << "w4_speed(%): " << w4_speed << std::endl;
	std::cout << std::endl;
#endif
}
void Base_Control::curveFunction(double& vx_cmd, double& vy_cmd, double& vyaw_cmd){
	/*
		using s function to regularize robot velocity 

		math:
			(v_max-v_min)*(cos(pi*((t-t_min)/(t_max-t_min)-1))+1)/2+v_min;
		param:
			v_max:		upper bound of the robot velocity		(default: 80 percent)
			v_min: 		lower bound of the robot velocity		(default: 10 percent)
			angle_max:	upper bound of the angular velocity		(default: 144 degree)
			angle_min:	lower bound of the angular velocity		(default: 5.0 degree)
			dis_max:	upper bound of the robot distance		(default: 2.0 m)
			dis_min:	lower bound of the robot distance		(default: 0.3 m)
			omega_max:  upper bound of the angular velocity		(default: 80 percent)
			omega_min:	lower bound of the angular velocity		(default: 5 percent)
	*/
	double v_max(80), v_min(10);
	double dis_max(2), dis_min(0.3);
	double angle_max(144), angle_min(5);
	double omega_max(80), omega_min(5);
	double v = hypot(vx_cmd, vy_cmd);
	double alpha = atan2(vy_cmd, vx_cmd)*rad2deg;
	double angle = vyaw_cmd*rad2deg;
	double t = ros::Time::now().toSec();
	double angle_(angle), v_(v);

	// velocity 
	if(v > v_max)
		v_ = v_max;
	else if(v < v_min)
		v_ = v_min;
	else if(v != 0)
		v_ = (v_max-v_min)*(cos(M_PI*((v-dis_min)/(dis_max-dis_min)-1))+1)/2+v_min;
	// angular velocity
	if(fabs(angle) > angle_max)
		angle_ = angle_max;
	else if(angle < angle_min)
		angle_ = angle_min;
	else if (angle != 0)
		angle_ = (angle_max-angle_min)*(cos(M_PI*((fabs(angle)-omega_min)/(omega_max-omega_min)-1))+1)/2+angle_min;
	if(angle<0)
		angle_ = -angle_;
	// detect the diff between velocity_cmd
	 
	// output 
	vx_cmd = v_*cos(alpha*deg2rad);
	vy_cmd = v_*sin(alpha*deg2rad);
	vyaw_cmd = angle_;
}
void Base_Control::send(robot_command* CMD)
{
	this->base_robotCMD = CMD;
	inverseKinematics();
	mcssl_send2motor();
}

robot_command* Base_Control::get_feedback()
{
	forwardKinematics();
	return base_robotFB;
}
