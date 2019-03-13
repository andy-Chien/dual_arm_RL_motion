/*
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Savage on 27/01/11.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************
 Modifications:
 
 25/07/2011 - Eliminado la modificacion serial para ser modificada dentro del mismo Hardware Serial.
 25/07/2011 - Modificado la funcion setBD() para aceptar todas la velocidades sin PDF.
 25/07/2011 - Agregada la funcion de Rotacion Continua.
 26/07/2011 - Agregada la funcion begin sin seteo de Direction_Pin.
 25/07/2011 - Agregada la funcion Reset.
 26/07/2011 - Agregada la funcion Reg_Write en move y moveSpeed.
 26/07/2011 - Agregada la funcion Action.
 13/12/2011 - Arreglado el manejo y envio de variables.
 22/12/2011 - Compatible con la actualizacion Arduino 1.0.
 10/01/2012 - Utilizacion de Macros y eliminacion codigo no necesario.
 11/01/2012 - Agregadas las funciones:
              int setTempLimit(unsigned char ID, unsigned char Temperature);
              int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
              int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
			  int setMaxTorque(unsigned char ID, int MaxTorque);
              int setSRL(unsigned char ID, unsigned char SRL);
              int setRDT(unsigned char ID, unsigned char RDT);
              int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
              int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
              int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
			  int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
 15/01/2012 - Agregadas las funciones:             
              int setPunch(unsigned char ID, int Punch);
              int moving(unsigned char ID);
              int lockRegister(unsigned char ID);
			  int RWStatus(unsigned char ID);
              int readSpeed(unsigned char ID);
              int readLoad(unsigned char ID);
 
 TODO:
 
 FUNCION SYNCWRITE.
 
 *****************************************************************************
 
 Contact: savageelectronics@gmail.com 
 Web:     http://savageelectrtonics.blogspot.com/
 Autor:   Josue Alejandro Savage
 
 */

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "DynamixelSerial1.h"

// Macro for the selection of the Serial Port

//#define sendData(args)  (Serial1.write(args))    // Write Over Serial
//#define availableData() (Serial1.available())    // Check Serial Data Available
//#define readData()      (Serial1.read())         // Read Serial Data
//#define peekData()      (Serial1.peek())         // Peek Serial Data
//#define beginCom(args)  (Serial1.begin(args))    // Begin Serial Comunication
//#define endCom()        (Serial1.end())          // End Serial Comunication
/* 將以上巨集改成 this->serial.method() */

// Macro for Timing

#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

// Macro for Comunication Flow Control

#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

// Private Methods //////////////////////////////////////////////////////////////

int DynamixelClass::read_error(void)
{
	Time_Counter = 0;
	while((this->serial.available() < 5) & (Time_Counter < TIME_OUT)) {  // Wait for Data
		Time_Counter++;
		delayus(1000);
	}
	
	while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ) {
			this->serial.read();                                    // Start Bytes
			this->serial.read();                                    // Ax-12 ID
			this->serial.read();                                    // Length
			Error_Byte = this->serial.read();                       // Error
				return (Error_Byte);
		}
	}
	return (-1);											 // No Ax Response
}

// Public Methods //////////////////////////////////////////////////////////////

void DynamixelClass::begin(long baud, unsigned char directionPin)
{	
	Direction_Pin = directionPin;
	setDPin(Direction_Pin,OUTPUT);
	this->serial.begin(baud);
}	

void DynamixelClass::begin(long baud)
{	
	this->serial.begin(baud);
}	

void DynamixelClass::end()
{
	this->serial.end();
}

int DynamixelClass::reset(unsigned char ID)
{
	Checksum = (~(ID + AX_RESET_LENGTH + AX_RESET))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_RESET_LENGTH);
	this->serial.write(AX_RESET);
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);

    return (read_error());  
}

int DynamixelClass::ping(unsigned char ID)
{
	Checksum = (~(ID + AX_READ_DATA + AX_PING))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);                     
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_READ_DATA);
	this->serial.write(AX_PING);    
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());              
}

int DynamixelClass::setID(unsigned char ID, unsigned char newID)
{    
	Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_ID_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_ID);
    this->serial.write(newID);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::setBD(unsigned char ID, long baud)
{    
	unsigned char Baud_Rate = (2000000/baud) - 1;
    Checksum = (~(ID + AX_BD_LENGTH + AX_WRITE_DATA + AX_BAUD_RATE + Baud_Rate))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                 // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_BD_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_BAUD_RATE);
    this->serial.write(Baud_Rate);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::move(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                 // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_GOAL_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_GOAL_POSITION_L);
    this->serial.write(Position_L);
    this->serial.write(Position_H);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);

    return (read_error());                 // Return the read error
}

int DynamixelClass::moveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
	Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
 
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_GOAL_SP_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_GOAL_POSITION_L);
    this->serial.write(Position_L);
    this->serial.write(Position_H);
    this->serial.write(Speed_L);
    this->serial.write(Speed_H);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());               // Return the read error
}

int DynamixelClass::setEndless(unsigned char ID, bool Status)
{
 if ( Status ) {	
	  char AX_CCW_AL_LT = 0;     // Changing the CCW Angle Limits for Full Rotation.
	  Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L))&0xFF;
	
	  switchCom(Direction_Pin,Tx_MODE);
      this->serial.write(AX_START);                // Send Instructions over Serial
      this->serial.write(AX_START);
      this->serial.write(ID);
      this->serial.write(AX_GOAL_LENGTH);
      this->serial.write(AX_WRITE_DATA);
      this->serial.write(AX_CCW_ANGLE_LIMIT_L );
      this->serial.write(AX_CCW_AL_LT);
      this->serial.write(AX_CCW_AL_LT);
      this->serial.write(Checksum);
      delayus(TX_DELAY_TIME);
	  switchCom(Direction_Pin,Rx_MODE);

	  return(read_error());
 }
 else
 {
	 turn(ID,0,0);
	 Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H))&0xFF;
	
	 switchCom(Direction_Pin,Tx_MODE);
	 this->serial.write(AX_START);                 // Send Instructions over Serial
	 this->serial.write(AX_START);
	 this->serial.write(ID);
	 this->serial.write(AX_GOAL_LENGTH);
	 this->serial.write(AX_WRITE_DATA);
	 this->serial.write(AX_CCW_ANGLE_LIMIT_L);
	 this->serial.write(AX_CCW_AL_L);
	 this->serial.write(AX_CCW_AL_H);
	 this->serial.write(Checksum);
	 delayus(TX_DELAY_TIME);
	 switchCom(Direction_Pin,Rx_MODE);
	 
	 return (read_error());                 // Return the read error
  }
 } 

int DynamixelClass::turn(unsigned char ID, bool SIDE, int Speed)
{		
		if (SIDE == 0){                          // Move Left///////////////////////////
			
			char Speed_H,Speed_L;
			Speed_H = Speed >> 8;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
			
			switchCom(Direction_Pin,Tx_MODE);
			this->serial.write(AX_START);                // Send Instructions over Serial
			this->serial.write(AX_START);
			this->serial.write(ID);
			this->serial.write(AX_SPEED_LENGTH);
			this->serial.write(AX_WRITE_DATA);
			this->serial.write(AX_GOAL_SPEED_L);
			this->serial.write(Speed_L);
			this->serial.write(Speed_H);
			this->serial.write(Checksum);
			delayus(TX_DELAY_TIME);
			switchCom(Direction_Pin,Rx_MODE);
			
			return(read_error());               // Return the read error		
		}
		else
		{                                            // Move Rigth////////////////////
			char Speed_H,Speed_L;
			Speed_H = (Speed >> 8) + 4;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H))&0xFF;
			
			switchCom(Direction_Pin,Tx_MODE);
			this->serial.write(AX_START);                // Send Instructions over Serial
			this->serial.write(AX_START);
			this->serial.write(ID);
			this->serial.write(AX_SPEED_LENGTH);
			this->serial.write(AX_WRITE_DATA);
			this->serial.write(AX_GOAL_SPEED_L);
			this->serial.write(Speed_L);
			this->serial.write(Speed_H);
			this->serial.write(Checksum);
			delayus(TX_DELAY_TIME);
			switchCom(Direction_Pin,Rx_MODE);
			
			return(read_error());                // Return the read error	
		}
}

int DynamixelClass::moveRW(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;
    Checksum = (~(ID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                 // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_GOAL_LENGTH);
    this->serial.write(AX_REG_WRITE);
    this->serial.write(AX_GOAL_POSITION_L);
    this->serial.write(Position_L);
    this->serial.write(Position_H);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error());                 // Return the read error
}

int DynamixelClass::moveSpeedRW(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
    Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_GOAL_SP_LENGTH);
    this->serial.write(AX_REG_WRITE);
    this->serial.write(AX_GOAL_POSITION_L);
    this->serial.write(Position_L);
    this->serial.write(Position_H);
    this->serial.write(Speed_L);
    this->serial.write(Speed_H);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());               // Return the read error
}

void DynamixelClass::action()
{	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(BROADCAST_ID);
    this->serial.write(AX_ACTION_LENGTH);
    this->serial.write(AX_ACTION);
    this->serial.write(AX_ACTION_CHECKSUM);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
}

int DynamixelClass::torqueStatus( unsigned char ID, bool Status)
{
    Checksum = (~(ID + AX_TORQUE_LENGTH + AX_WRITE_DATA + AX_TORQUE_ENABLE + Status))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);              // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_TORQUE_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_TORQUE_ENABLE);
    this->serial.write(Status);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());              // Return the read error
}

int DynamixelClass::ledStatus(unsigned char ID, bool Status)
{    
    Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);              // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_LED_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_LED);
    this->serial.write(Status);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());              // Return the read error
}

int DynamixelClass::readTemperature(unsigned char ID)
{	
    Checksum = (~(ID + AX_TEM_LENGTH  + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_TEM_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_PRESENT_TEMPERATURE);
    this->serial.write(AX_BYTE_READ);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Temperature_Byte = -1;
    Time_Counter = 0;
    while((this->serial.available() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
			Temperature_Byte = this->serial.read();         // Temperature
		}
    }
	return (Temperature_Byte);               // Returns the read temperature
}

int DynamixelClass::readPosition(unsigned char ID)
{	
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;
  
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_POS_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_PRESENT_POSITION_L);
    this->serial.write(AX_BYTE_READ_POS);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Position_Long_Byte = -1;
	Time_Counter = 0;
    while((this->serial.available() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
    
			Position_Low_Byte = this->serial.read();            // Position Bytes
			Position_High_Byte = this->serial.read();
			Position_Long_Byte = Position_High_Byte << 8; 
			Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
		}
    }
	return (Position_Long_Byte);     // Returns the read position
}

int DynamixelClass::readVoltage(unsigned char ID)
{    
    Checksum = (~(ID + AX_VOLT_LENGTH  + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_VOLT_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_PRESENT_VOLTAGE);
    this->serial.write(AX_BYTE_READ);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Voltage_Byte = -1;
	Time_Counter = 0;
    while((this->serial.available() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
			Voltage_Byte = this->serial.read();             // Voltage
		}
    }
	return (Voltage_Byte);               // Returns the read Voltage
}

int DynamixelClass::setTempLimit(unsigned char ID, unsigned char Temperature)
{
	Checksum = (~(ID + AX_TL_LENGTH +AX_WRITE_DATA+ AX_LIMIT_TEMPERATURE + Temperature))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);                     
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_TL_LENGTH);
	this->serial.write(AX_WRITE_DATA);
	this->serial.write(AX_LIMIT_TEMPERATURE);
    this->serial.write(Temperature);
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelClass::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage)
{
	Checksum = (~(ID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_DOWN_LIMIT_VOLTAGE + DVoltage + UVoltage))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);                     
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_VL_LENGTH);
	this->serial.write(AX_WRITE_DATA);
	this->serial.write(AX_DOWN_LIMIT_VOLTAGE);
    this->serial.write(DVoltage);
    this->serial.write(UVoltage);
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelClass::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit)
{
	char CW_H,CW_L,CCW_H,CCW_L;
    CW_H = CWLimit >> 8;    
    CW_L = CWLimit;                // 16 bits - 2 x 8 bits variables
    CCW_H = CCWLimit >> 8;
    CCW_L = CCWLimit;  
	Checksum = (~(ID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_CW_ANGLE_LIMIT_L + CW_H + CW_L + AX_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);                     
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_CCW_CW_LENGTH);
	this->serial.write(AX_WRITE_DATA);
	this->serial.write(AX_CW_ANGLE_LIMIT_L);
    this->serial.write(CW_L);
	this->serial.write(CW_H);
	this->serial.write(AX_CCW_ANGLE_LIMIT_L);
    this->serial.write(CCW_L);
	this->serial.write(CCW_H);
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelClass::setMaxTorque(unsigned char ID, int MaxTorque)
{
    char MaxTorque_H,MaxTorque_L;
    MaxTorque_H = MaxTorque >> 8;           // 16 bits - 2 x 8 bits variables
    MaxTorque_L = MaxTorque;
	Checksum = (~(ID + AX_MT_LENGTH + AX_WRITE_DATA + AX_MAX_TORQUE_L + MaxTorque_L + MaxTorque_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                 // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_MT_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_MAX_TORQUE_L);
    this->serial.write(MaxTorque_L);
    this->serial.write(MaxTorque_H);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error());                 // Return the read error
}

int DynamixelClass::setSRL(unsigned char ID, unsigned char SRL)
{    
	Checksum = (~(ID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + SRL))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_SRL_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_RETURN_LEVEL);
    this->serial.write(SRL);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::setRDT(unsigned char ID, unsigned char RDT)
{    
	Checksum = (~(ID + AX_RDT_LENGTH + AX_WRITE_DATA + AX_RETURN_DELAY_TIME + (RDT/2)))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_RDT_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_RETURN_DELAY_TIME);
    this->serial.write((RDT/2));
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm)
{    
	Checksum = (~(ID + AX_LEDALARM_LENGTH + AX_WRITE_DATA + AX_ALARM_LED + LEDAlarm))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_LEDALARM_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_ALARM_LED);
    this->serial.write(LEDAlarm);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::setShutdownAlarm(unsigned char ID, unsigned char SALARM)
{    
	Checksum = (~(ID + AX_SALARM_LENGTH + AX_ALARM_SHUTDOWN + AX_ALARM_LED + SALARM))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_SALARM_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_ALARM_SHUTDOWN);
    this->serial.write(SALARM);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
	Checksum = (~(ID + AX_CM_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_MARGIN + CWCMargin + AX_CCW_COMPLIANCE_MARGIN + CCWCMargin))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);                     
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_CM_LENGTH);
	this->serial.write(AX_WRITE_DATA);
	this->serial.write(AX_CW_COMPLIANCE_MARGIN);
    this->serial.write(CWCMargin);
	this->serial.write(AX_CCW_COMPLIANCE_MARGIN);
    this->serial.write(CCWCMargin);
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelClass::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
	Checksum = (~(ID + AX_CS_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_SLOPE + CWCSlope + AX_CCW_COMPLIANCE_SLOPE + CCWCSlope))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
	this->serial.write(AX_START);                     
	this->serial.write(AX_START);
	this->serial.write(ID);
	this->serial.write(AX_CS_LENGTH);
	this->serial.write(AX_WRITE_DATA);
	this->serial.write(AX_CW_COMPLIANCE_SLOPE);
    this->serial.write(CWCSlope);
	this->serial.write(AX_CCW_COMPLIANCE_SLOPE);
    this->serial.write(CCWCSlope);
	this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error()); 
}

int DynamixelClass::setPunch(unsigned char ID, int Punch)
{
    char Punch_H,Punch_L;
    Punch_H = Punch >> 8;           // 16 bits - 2 x 8 bits variables
    Punch_L = Punch;
	Checksum = (~(ID + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L + Punch_L + Punch_H))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                 // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_PUNCH_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_PUNCH_L);
    this->serial.write(Punch_L);
    this->serial.write(Punch_H);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    return (read_error());                 // Return the read error
}

int DynamixelClass::moving(unsigned char ID)
{	
    Checksum = (~(ID + AX_MOVING_LENGTH  + AX_READ_DATA + AX_MOVING + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_MOVING_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_MOVING);
    this->serial.write(AX_BYTE_READ);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Moving_Byte = -1;
    Time_Counter = 0;
    while((this->serial.available() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
			Moving_Byte = this->serial.read();         // Temperature
		}
    }
	return (Moving_Byte);               // Returns the read temperature
}

int DynamixelClass::lockRegister(unsigned char ID)
{    
	Checksum = (~(ID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + LOCK))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);                // Send Instructions over Serial
    this->serial.write(AX_START);
    this->serial.write(ID);
	this->serial.write(AX_LR_LENGTH);
    this->serial.write(AX_WRITE_DATA);
    this->serial.write(AX_LOCK);
    this->serial.write(LOCK);
    this->serial.write(Checksum);
	delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
    
    return (read_error());                // Return the read error
}

int DynamixelClass::RWStatus(unsigned char ID)
{	
    Checksum = (~(ID + AX_RWS_LENGTH  + AX_READ_DATA + AX_REGISTERED_INSTRUCTION + AX_BYTE_READ))&0xFF;
    
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_RWS_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_REGISTERED_INSTRUCTION);
    this->serial.write(AX_BYTE_READ);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    RWS_Byte = -1;
    Time_Counter = 0;
    while((this->serial.available() < 6) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
			RWS_Byte = this->serial.read();         // Temperature
		}
    }
	return (RWS_Byte);               // Returns the read temperature
}

int DynamixelClass::readSpeed(unsigned char ID)
{	
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_POS))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_POS_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_PRESENT_SPEED_L);
    this->serial.write(AX_BYTE_READ_POS);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Speed_Long_Byte = -1;
	Time_Counter = 0;
    while((this->serial.available() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
			
			Speed_Low_Byte = this->serial.read();            // Position Bytes
			Speed_High_Byte = this->serial.read();
			Speed_Long_Byte = Speed_High_Byte << 8; 
			Speed_Long_Byte = Speed_Long_Byte + Speed_Low_Byte;
		}
    }
	return (Speed_Long_Byte);     // Returns the read position
}

int DynamixelClass::readLoad(unsigned char ID)
{	
    Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS))&0xFF;
	
	switchCom(Direction_Pin,Tx_MODE);
    this->serial.write(AX_START);
    this->serial.write(AX_START);
    this->serial.write(ID);
    this->serial.write(AX_POS_LENGTH);
    this->serial.write(AX_READ_DATA);
    this->serial.write(AX_PRESENT_LOAD_L);
    this->serial.write(AX_BYTE_READ_POS);
    this->serial.write(Checksum);
    delayus(TX_DELAY_TIME);
	switchCom(Direction_Pin,Rx_MODE);
	
    Load_Long_Byte = -1;
	Time_Counter = 0;
    while((this->serial.available() < 7) & (Time_Counter < TIME_OUT)){
		Time_Counter++;
		delayus(1000);
    }
	
    while (this->serial.available() > 0){
		Incoming_Byte = this->serial.read();
		if ( (Incoming_Byte == 255) & (this->serial.peek() == 255) ){
			this->serial.read();                            // Start Bytes
			this->serial.read();                            // Ax-12 ID
			this->serial.read();                            // Length
			if( (Error_Byte = this->serial.read()) != 0 )   // Error
				return (Error_Byte*(-1));
			
			Load_Low_Byte = this->serial.read();            // Position Bytes
			Load_High_Byte = this->serial.read();
			Load_Long_Byte = Load_High_Byte << 8; 
			Load_Long_Byte = Load_Long_Byte + Load_Low_Byte;
		}
    }
	return (Load_Long_Byte);     // Returns the read position
}

DynamixelClass Dynamixel(Serial1);