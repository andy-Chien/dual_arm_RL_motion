[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/wrs/arm_left | 1000000  | l_joint1

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME | BULK READ ITEMS
dynamixel | /dev/wrs/arm_left | 1   | H54-200-S500-R | 2.0      | l_joint1   | present_position, present_voltage
dynamixel | /dev/wrs/arm_left | 2   | H54-200-S500-R | 2.0      | l_joint2   | present_position, present_voltage
dynamixel | /dev/wrs/arm_left | 3   | H54-100-S500-R | 2.0      | l_joint3   | present_position, present_voltage
dynamixel | /dev/wrs/arm_left | 4   | H54-100-S500-R | 2.0      | l_joint4   | present_position, present_voltage
dynamixel | /dev/wrs/arm_left | 5   | H54-100-S500-R | 2.0      | l_joint5   | present_position, present_voltage
dynamixel | /dev/wrs/arm_left | 6   | H54-100-S500-R | 2.0      | l_joint6   | present_position, present_voltage
dynamixel | /dev/wrs/arm_left | 7   | H42-20-S300-R  | 2.0      | l_joint7   | present_position, present_voltage
