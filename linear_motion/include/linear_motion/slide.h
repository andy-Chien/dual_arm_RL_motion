#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <boost/thread.hpp>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "linear_motion/Slide_Feedback.h"
#include "manipulator_h_base_module_msgs/SlideCommand.h"   //new

#include "modbus/modbus.h"

// #define MODE_GET_CURR_POS 291
#define ADDRESS_CMD  88
#define ADDRESS_FDB  204
#define MAX_SPEED    20000
#define ACCELERATION 40000
#define DECELERATION 40000

#define CMD_LENGTH 16
#define FDB_LENGTH 6

uint16_t cmd_arr[CMD_LENGTH] = {0, 0, 0, 1, 0, 0, 0, MAX_SPEED, 0, ACCELERATION, 0, DECELERATION, 0, 800, 0, 1};
uint16_t fdb_val[FDB_LENGTH] = {0, 0, 0, 0, 0, 0};

modbus_t *ct    = nullptr;
int goal_pos    = 0;
int curr_pos    = 0;
int curr_speed  = 0; 
double smp_time = 0.01;

boost::thread  *com_driver_thread_;
linear_motion::Slide_Feedback msg_fdb;
