#!/usr/bin/env python

import rospy
from math import radians, degrees, pi
from vacuum_cmd_msg.srv import VacuumCmd
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty


# 2017/07/19 Gripper Parameter Backup
# cam2tool_y = -0.025
# cam2tool_z = 0.14
# gripper_length = 0.04
# cam2center_y = 0.025
# cam2center_y_4_tote = 0.05

# NOW
cam2tool_z = 0.18 #0.27 #0.26
gripper_length = 0.04
cam2center_y = 0.035
cam2center_y_4_tote = 0.035#0.06 #0.05


class SuctionTask:

    switch_mode_pub = rospy.Publisher(
                '/arduino/mode',
                Bool,
                queue_size=1
            )

    def __init__(self, _name='/robotis'):
        """Inital object."""
        self.name    = _name
        self.fail_cnt = 0
        self.gripped = False
        print 'name = ', self.name
        if 'gazebo' in self.name:
            if 'right' in self.name:
                self.suction_pub = rospy.Publisher(
                    '/mobile_dual_arm/r_suction_joint_position/command',
                    Float64,
                    queue_size=1
                )
                self.is_grip_sub = rospy.Subscriber(
                    '/robot/right_gazebo/vacuum_gripper1/grasping',
                    Bool,
                    self.is_grip_callback,
                    queue_size=1
                )
                print 'nameeeee = ', self.name
            elif 'left' in self.name :
                self.suction_pub = rospy.Publisher(
                    '/mobile_dual_arm/l_suction_joint_position/command',
                    Float64,
                    queue_size=1
                )
                self.is_grip_sub = rospy.Subscriber(
                    '/robot/left_gazebo/vacuum_gripper1/grasping',
                    Bool,
                    self.is_grip_callback,
                    queue_size=1
                )
        else:
            self.is_grip_sub = rospy.Subscriber(
                self.name + '/is_grip',
                Bool,
                self.is_grip_callback,
                queue_size=1
            )
    
    @staticmethod
    def switch_mode(enable):
        SuctionTask.switch_mode_pub.publish(enable)

    def robot_cmd_client(self, cmd):
        if 'gazebo' in self.name:
            for i in range(1, 5):
                if 'On' in cmd:
                    suction_service = '/robot/' + self.name + '/vacuum_gripper' + str(i) + '/on'
                elif 'Off' in cmd:
                    suction_service = '/robot/' + self.name + '/vacuum_gripper' + str(i) + '/off'
                rospy.wait_for_service(suction_service)
                try:
                    client = rospy.ServiceProxy(
                        suction_service,
                        Empty
                    )
                    client()
                except rospy.ServiceException, e:
                    print "Service call (Vacuum) failed: %s" % e
        else:
            suction_service = self.name + '/suction_cmd'
            try:
                rospy.wait_for_service(suction_service, timeout=1.)
            except rospy.ROSException as e:
                rospy.logwarn('wait_for_service timeout')
                self.robot_cmd_client(cmd)
                
            try:
                client = rospy.ServiceProxy(
                    suction_service,
                    VacuumCmd
                )
                res = client(cmd)
                print res
            except rospy.ServiceException, e:
                print "Service call (Vacuum) failed: %s" % e

    def gripper_vaccum_on(self):
        self.robot_cmd_client('vacuumOn')
        print('Vaccum On', self.gripped)
        rospy.sleep(0.5)

    def gripper_vaccum_off(self):
        self.robot_cmd_client('vacuumOff')
        print('Vaccum Off')
        rospy.sleep(0.3)

    def gripper_calibration(self):
        if 'gazebo' in self.name:
            pass
        else:
            self.robot_cmd_client('calibration')
        print('calibration')

    def gripper_set_max(self):
        if 'gazebo' in self.name:
            pass
        else:
            self.robot_cmd_client('setMaxPos')
        print('setMaxPos')
    
    def gripper_set_min(self):
        if 'gazebo' in self.name:
            pass
        else:
            self.robot_cmd_client('setMinPos')
        print('setMinPos')

    def gripper_suction_up(self):
        if 'gazebo' in self.name:
            msg = 0.0
            self.suction_pub.publish(msg)
        else:
            self.robot_cmd_client('suctionUp')
        print('Suction Up')

    def gripper_suction_down(self):
        if 'gazebo' in self.name:
            msg = -pi/2
            self.suction_pub.publish(msg)
        else:
            self.robot_cmd_client('suctionDown')
        print('Suction Down')

    def gripper_suction_deg(self, deg):
        str_deg = '{}'.format(deg)
        if 'gazebo' in self.name:
            msg = radians(deg)
            self.suction_pub.publish(msg)
        else:
            self.robot_cmd_client(str_deg)

        print('Suction Move : ' + str_deg)

    def is_grip_callback(self, msg):
        if not msg.data:
            if self.fail_cnt >= 100:
                self.gripped = False
                self.fail_cnt = 100
            else:
                self.fail_cnt += 1
        else:
            self.fail_cnt = 0
            self.gripped = True

    @property
    def is_grip(self):
        return self.gripped


if __name__ == '__main__':
    rospy.init_node('test_gripper')
    print('test_gripper')

    SuctionTask.switch_mode(True)
    right_gripper = SuctionTask(_name='right')
    left_gripper = SuctionTask(_name='left')

    while not rospy.is_shutdown():
        for gripper in [right_gripper, left_gripper]:
            gripper.gripper_vaccum_on()
            print('is grip: {}'.format(gripper.gripped))
            rospy.sleep(2)

            gripper.gripper_vaccum_off()

            gripper.gripper_suction_up()
            rospy.sleep(2)

            gripper.gripper_suction_down()
            rospy.sleep(2)

            gripper.gripper_suction_deg(-90)
            rospy.sleep(2)

    # right_gripper.gripper_vaccum_on()
    # right_gripper.gripper_vaccum_off()

    # right_gripper.gripper_calibration()
    # right_gripper.gripper_set_max()
    # right_gripper.gripper_set_min()

    # right_gripper.gripper_suction_up()
    # right_gripper.gripper_suction_down()
    # right_gripper.gripper_suction_deg(0)

    SuctionTask.switch_mode(False)
