#include "linear_motion/slide.h"

void slide_callback(const manipulator_h_base_module_msgs::SlideCommand::ConstPtr& msg)
{
    goal_pos = -(double)100000.0*msg->pos;

    cmd_arr[4] = goal_pos>>16;
    cmd_arr[5] = goal_pos;
}

void read_feedback()
{
    int rc = modbus_read_registers(ct, ADDRESS_FDB, FDB_LENGTH, fdb_val);
    if (rc != FDB_LENGTH)
    {
        fprintf(stderr, "modbus read failed: %d %s\n", errno, modbus_strerror(errno));
        errno = 0;
    }

    curr_pos   = fdb_val[0]<<16 | fdb_val[1];
    curr_speed = fdb_val[4]<<16 | fdb_val[5];
}

void write_command()
{
    int rc = modbus_write_registers(ct, ADDRESS_CMD, CMD_LENGTH, cmd_arr);
    if (rc != CMD_LENGTH)
    {
        fprintf(stderr, "modbus write failed: %d %s\n", errno, modbus_strerror(errno));
        errno = 0;
    }
}

modbus_t* init_modbus_rtu(int id, std::string port, int baud_rate)
{
    modbus_t* ct = modbus_new_rtu(port.c_str(), baud_rate, 'E', 8, id);
    modbus_set_slave(ct, id);
    if (modbus_connect(ct) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n",
            modbus_strerror(errno));
        std::cout << "Error connect" << std::endl;
        modbus_free(ct);
        return nullptr;
    }
    std::cout << "Init success" << std::endl;
    // modbus_set_debug(ct, true);

    return ct;
}

void send_cmd()
// for communication with driver
{
    int speed = 0;
    int smp_deleration = DECELERATION * smp_time;

    ros::Rate loop_rate(1 / smp_time);
    while (ros::ok())
    {
        if (goal_pos != curr_pos)
        {
            int diff_pos = abs(goal_pos - curr_pos);
            int speed_tmp = diff_pos / smp_time;
            speed_tmp = (speed_tmp < MAX_SPEED) ? speed_tmp : MAX_SPEED;
            speed = (speed_tmp < speed) ? (
                ((speed - speed_tmp) < smp_deleration) ? speed_tmp : speed - smp_deleration
                ) : speed_tmp;
            // speed = (speed_tmp < speed) ? speed : speed_tmp;
            // cmd_arr[6] = speed>>16;
            cmd_arr[7]  = speed;
            cmd_arr[9]  = exp((speed / MAX_SPEED)*4 - 2) * 5410;
            cmd_arr[11] = speed*2;
            // cmd_arr[9] = speed*2;

            std::cout << "speed = " << (cmd_arr[7] | cmd_arr[6]<<16) <<std::endl;
            write_command();
        }
        else
            speed = 0;

        read_feedback();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    //========================= Initialize ROS =============================
    ros::init(argc, argv, "linear_z");

    int baud_rate;
    std::string side_str;

    ros::NodeHandle nh_param("~");
    nh_param.param<int>("baud", baud_rate, 9600);
    nh_param.param<std::string>("side", side_str, "");

    //========================= Initialize Modbus_RTU ============================= 
    std::cout << "Preparing connection slide" << std::endl;

    int id = side_str == "right" ? 1 : 3;
    ct = init_modbus_rtu(id, "/dev/wrs/slide_" + side_str, baud_rate);
    if (!ct)
    {
        std::cout << "Connect " + side_str + " fail!!!" << std::endl;
        return -1;
    }
    std::cout << side_str + " slide connect ok" << std::endl;

    read_feedback();
    goal_pos = curr_pos;
    std::cout << side_str + " slide connect ok " << goal_pos<<std::endl;


    // generate thread to communicate with driver
    com_driver_thread_ = new boost::thread(boost::bind( &send_cmd ));

    // ============================= Subscribe message =============================
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/slide_command_msg", 1, slide_callback);
    ros::Publisher  pub = n.advertise<linear_motion::Slide_Feedback>("/slide_feedback_msg", 1);
    ros::Rate loop_rate(125);

    // ============================= ROS Loop =============================
    // main thread to communicate with other node
    while (ros::ok())
    {
        msg_fdb.curr_pos = curr_pos;
        msg_fdb.is_busy  = abs(curr_speed) > 10;
        pub.publish(msg_fdb);
        ros::spinOnce();

        loop_rate.sleep();
    }

    delete com_driver_thread_;
    return 0;
}
