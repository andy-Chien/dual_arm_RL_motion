#include <ros.h>
#include <vacuum_cmd_msg/VacuumCmd.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <EEPROM.h>
#include <DynamixelSerial1.h>
#include <SPI.h>
#include <MFRC522.h>     

constexpr uint8_t RST_PIN = 49;     // Configurable, see typical pin layout above
constexpr uint8_t SS_1_PIN = 47;   // Configurable, take a unused pin, only HIGH/LOW required, must be diffrent to SS 2
constexpr uint8_t SS_2_PIN = 48;    // Configurable, take a unused pin, only HIGH/LOW required, must be diffrent to SS 1
constexpr uint8_t NR_OF_READERS = 2;

byte ssPins[] = {SS_1_PIN, SS_2_PIN};

MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instan/ce./

std_msgs::String str_msg;
std_msgs::Int32 int32;
ros::Publisher chatter("rfid", &str_msg);
ros::Publisher costII("consume", &str_msg);

#define ID_right  1
#define ID_left   2
#define UPSPEED   100
#define DOWNSPEED 100
#define ADJ_STEP  4
#define POS_LMT   1024

// Marco for debug
//#define SERIAL_PRINT

ros::NodeHandle  nh;
using vacuum_cmd_msg::VacuumCmd;

std_msgs::Bool is_grip_msg;
std_msgs::Bool is_stop_msg;
ros::Publisher isGripR("right/is_grip", &is_grip_msg);
ros::Publisher isGripL("left/is_grip", &is_grip_msg);
ros::Publisher isStop("robot/is_stop", &is_stop_msg);

void callback(const VacuumCmd::Request& , VacuumCmd::Response& ,bool);
void callback_right(const VacuumCmd::Request& , VacuumCmd::Response& );
void callback_left(const VacuumCmd::Request& , VacuumCmd::Response& );
void armTaskCallback(const std_msgs::Bool& msg);
ros::ServiceServer<VacuumCmd::Request, VacuumCmd::Response> vac_srv_right("right/suction_cmd", &callback_right);
ros::ServiceServer<VacuumCmd::Request, VacuumCmd::Response> vac_srv_left("left/suction_cmd", &callback_left);
ros::Subscriber<std_msgs::Bool> armTask_sub("/arduino/mode", &armTaskCallback);

DynamixelClass Dxl_right(Serial1);
DynamixelClass Dxl_left(Serial3);

bool armTask = false;
int MaxPos;
int MinPos;
int MaxPos_right;
int MinPos_right;
int MaxPos_left;
int MinPos_left;
byte MinPos_H_right;
byte MinPos_L_right; 
byte MaxPos_H_right; 
byte MaxPos_L_right; 
byte MinPos_H_left;
byte MinPos_L_left; 
byte MaxPos_H_left; 
byte MaxPos_L_left; 
int addressMin_L_right = 3;
int addressMin_H_right = 5;
int addressMax_L_right = 7;
int addressMax_H_right = 9;
int addressMin_L_left = 13;
int addressMin_H_left = 15;
int addressMax_L_left = 17;
int addressMax_H_left = 19;

const int is_grip_left  = 37;
const int is_grip_right = 39;
const int is_stop       = 35;

const int led_pin = 13;
const int vac_pin_right = 33;
const int vac_pin_left  = 31;
int ID = 0;
int vac_pin = 0;


void setup() 
{
  nh.initNode();
  nh.advertise(isGripR);
  nh.advertise(isGripL);
  nh.advertise(isStop);
  nh.subscribe(armTask_sub);
  
  nh.advertiseService(vac_srv_right);
  nh.advertiseService(vac_srv_left);
  
  pinMode(led_pin, OUTPUT);
  pinMode(vac_pin_right, OUTPUT);
  pinMode(vac_pin_left, OUTPUT);
  
  pinMode(is_grip_left, INPUT_PULLUP);
  pinMode(is_grip_right, INPUT_PULLUP);
  pinMode(is_stop, INPUT_PULLUP);

  Dxl_right.begin(1000000, 2);
  delay(1000);
  Dxl_left.begin(1000000, 2);
  delay(1000);
  Dxl_right.setEndless(ID_right, OFF);
  Dxl_left.setEndless(ID_left, OFF);
  Dxl_right.torqueStatus(ID_right, 0);
  Dxl_left.torqueStatus(ID_left, 0);

  MaxPos_L_right = EEPROM.read(addressMax_L_right);
  MaxPos_H_right = EEPROM.read(addressMax_H_right);
  MaxPos_right = MaxPos_H_right << 8 | MaxPos_L_right;

  MinPos_L_right = EEPROM.read(addressMin_L_right);
  MinPos_H_right = EEPROM.read(addressMin_H_right);
  MinPos_right = MinPos_H_right << 8 | MinPos_L_right;

  MaxPos_L_left = EEPROM.read(addressMax_L_left);
  MaxPos_H_left = EEPROM.read(addressMax_H_left);
  MaxPos_left = MaxPos_H_left << 8 | MaxPos_L_left;

  MinPos_L_left = EEPROM.read(addressMin_L_left);
  MinPos_H_left = EEPROM.read(addressMin_H_left);
  MinPos_left = MinPos_H_left << 8 | MinPos_L_left;

#ifdef SERIAL_PRINT
  Serial.begin(57600);
  while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
#endif

  SPI.begin(); // Init SPI bus
  //mfrc522.PCD_Init(); // Init MFRC522 
  nh.advertise(chatter);
  nh.advertise(costII);

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
    
#ifdef SERIAL_PRINT
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
#endif
  }
}

void armTaskCallback(const std_msgs::Bool& msg)
{
  armTask = (bool)msg.data;
}

DynamixelClass wDxl(bool isRight)
{
  return (isRight)? Dxl_right: Dxl_left;
}

void callback_right(const VacuumCmd::Request& req, VacuumCmd::Response& res)
{
  bool isRight = true;
  ID = ID_right;
  vac_pin = vac_pin_right;
  MaxPos = MaxPos_right;
  MinPos = MinPos_right;
  callback(req, res, isRight);
}

void callback_left(const VacuumCmd::Request& req, VacuumCmd::Response& res)
{
  bool isRight = false;
  ID = ID_left;
  vac_pin = vac_pin_left;
  MaxPos = MaxPos_left;
  MinPos = MinPos_left;
  callback(req, res, isRight);
}

void callback(const VacuumCmd::Request& req, VacuumCmd::Response& res, bool isRight)
{
  if (strcmp(req.cmd, "setMaxPos") == 0)
  {
    MaxPos = wDxl(isRight).readPosition(ID) - ADJ_STEP;
    MaxPos = MaxPos > 0 ? MaxPos : 0;
    if (MaxPos < 0)
    {
      res.success = false;
      return;
    }
    if (isRight)
    {
      MaxPos_L_right = MaxPos;
      MaxPos_H_right = MaxPos >> 8;
      MaxPos_right = MaxPos;
      EEPROM.write(addressMax_H_right, MaxPos_H_right);
      EEPROM.write(addressMax_L_right, MaxPos_L_right);
    } else {
      MaxPos_L_left = MaxPos;
      MaxPos_H_left = MaxPos >> 8;
      MaxPos_left = MaxPos;
      EEPROM.write(addressMax_H_left, MaxPos_H_left);
      EEPROM.write(addressMax_L_left, MaxPos_L_left);
    }
  }
  else if (strcmp(req.cmd, "setMinPos") == 0)
  {
    MinPos = wDxl(isRight).readPosition(ID) + ADJ_STEP;
    MinPos = MinPos < POS_LMT ? MinPos : POS_LMT - 1;
    if (MinPos < 0)
    {
      res.success = false;
      return;
    }

    if (isRight)
    {
      MinPos_L_right = MinPos;
      MinPos_H_right = MinPos >> 8;
      MinPos_right = MinPos;
      EEPROM.write(addressMin_H_right, MinPos_H_right);
      EEPROM.write(addressMin_L_right, MinPos_L_right);
    } else {
      MinPos_L_left = MinPos;
      MinPos_H_left = MinPos >> 8;
      MinPos_left = MinPos;
      EEPROM.write(addressMin_H_left, MinPos_H_left);
      EEPROM.write(addressMin_L_left, MinPos_L_left);
    }
  }
  else if (strcmp(req.cmd, "suctionUp") == 0)
  {
    int count = 0;
    while (wDxl(isRight).moveSpeed(ID, MaxPos, UPSPEED) != 0)
    {
      delay(10);
      if (count++ >=10)
      {
        res.success = false;
        return;
      }
    }
  }
  else if (strcmp(req.cmd, "suctionDown") == 0)
  {   
    int count = 0;
    while (wDxl(isRight).moveSpeed(ID, MinPos, DOWNSPEED) != 0)
    {
      delay(10);
      if (count++ >=10)
      {
        res.success = false;
        return;
      }
    }
  }
  else if (strcmp(req.cmd, "calibration") == 0)
  {
    int count = 0;
    while (wDxl(isRight).torqueStatus(ID, 0) != 0)
    {
      delay(10);
      if (count++ >=10)
      {
        res.success = false;
        return;
      }
    }
  }
  else if (strcmp(req.cmd, "vacuumOn") == 0)
  {   
    digitalWrite(vac_pin, HIGH);
    digitalWrite(led_pin, HIGH);
  }
  else if (strcmp(req.cmd, "vacuumOff") == 0)
  {   
    digitalWrite(vac_pin, LOW);
    digitalWrite(led_pin, LOW);
  }
  else
  {
    String cmd(req.cmd);
    double angle = cmd.toDouble();

    int pos = map(angle, -90.0, 0.0, MaxPos, MinPos);
    int count = 0;
    while (wDxl(isRight).moveSpeed(ID, pos, DOWNSPEED) != 0)
    {
      delay(10);
      if (count++ >=10)
      {
        res.success = false;
        return;
      }
    }
  }
  res.success = true;
}

void RFID()
{
  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) 
  {
    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) 
    {
      byte* id = mfrc522[reader].uid.uidByte;

#ifdef SERIAL_PRINT
      dump_byte_array(id, mfrc522[reader].uid.size);
#endif

      pub_topic(id, "90", 101, 183, 231, 43);
      pub_topic(id, "91", 167, 227, 232, 43);
      pub_topic(id, "90", 176, 53, 41, 164);
      pub_topic(id, "91", 80, 16, 112, 163);
      pub_topic(id, "0", 16, 200, 16, 168);
      pub_topic(id, "1", 192, 110, 209, 87);
      pub_topic(id, "2", 96, 191, 31, 168);
      
      pub_topic(id, "90", 98, 181, 38, 65);
      pub_topic(id, "91", 162, 200, 147, 65);
      pub_topic(id, "90", 226, 254, 152, 65);
      pub_topic(id, "91", 146, 199, 139, 65);
      pub_topic(id, "0", 50, 232, 139, 65);
      pub_topic(id, "1", 18, 247, 135, 65);
      pub_topic(id, "2", 128, 198, 44, 164);
      
      pub_topic2(id, "consumeA", 242, 90, 144, 65);
      pub_topic2(id, "consumeB", 226, 233, 127, 65);
      pub_topic2(id, "consumeA", 98, 80, 27, 65);
      pub_topic2(id, "consumeB", 89, 10, 232, 43);
      
      pub_topic2(id, "spare", 98, 80, 27, 65);

#ifdef SERIAL_PRINT
      Serial.print(F("Reader "));
      Serial.print(reader);
      Serial.print(F(": Card UID:"));
      Serial.println();
#endif
    }
  }
}

void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i] , DEC);
  }
  Serial.println();
}

void pub_topic(byte *buffer, const char out_str[], byte arg0, byte arg1, byte arg2, byte arg3) {
  if ( buffer[0] == arg0
    && buffer[1] == arg1
    && buffer[2] == arg2
    && buffer[3] == arg3) {
      str_msg.data = out_str;
      chatter.publish(&str_msg);
    }
}

void pub_topic2(byte *buffer, const char out_str[], byte arg0, byte arg1, byte arg2, byte arg3) {
  if ( buffer[0] == arg0
    && buffer[1] == arg1
    && buffer[2] == arg2
    && buffer[3] == arg3) {
      str_msg.data = out_str;
      costII.publish(&str_msg);
    }
}

void loop()
{ 
  if (armTask)
  {
    is_grip_msg.data = (bool)digitalRead(is_grip_right);
    isGripR.publish(&is_grip_msg);

    is_grip_msg.data = (bool)digitalRead(is_grip_left);
    isGripL.publish(&is_grip_msg);
  }
  else
  {
    RFID();
  }
  is_stop_msg.data = !digitalRead(is_stop);
  isStop.publish(&is_stop_msg);

  nh.spinOnce();
  delay(10);
}
