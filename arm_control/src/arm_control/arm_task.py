#!/usr/bin/env python
#-*- coding: utf-8 -*-

"""Use to generate arm task and run."""

import rospy
import tf
# import object_distribtion

from math import radians, degrees, sin, cos, pi
from numpy import multiply

import numpy as np

from std_msgs.msg import String, Float64, Bool
from robotis_controller_msgs.msg import StatusMsg
# from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
from manipulator_h_base_module_msgs.msg import P2PPose, JointPose, KinematicsPose
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse
from manipulator_h_base_module_msgs.srv import GetJointPose, GetJointPoseResponse
from vacuum_cmd_msg.srv import VacuumCmd


_POS = (0, 0, 0)
_ORI = (0, 0, 0)
_PHI = 45
_suction_angle = 0


class ArmTask:
    """Running arm task class."""

    def __init__(self, _name = '/robotis'):
        """Inital object."""
        self.name = _name
        self.init()

    def init(self):
        self.__set_pubSub()
        #rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__is_busy = False
        self.__ik_fail = False
        self.__is_stop = False
        self.__is_wait = False
        self.__speed = 50

    def __set_pubSub(self):
        print "[Arm] name space : " + str(self.name) 
        self.__set_mode_pub = rospy.Publisher(
            str(self.name) + '/set_mode_msg',
            String,
            # latch=True,
            queue_size=1
        )
        self.__wait_pub = rospy.Publisher(
            str(self.name) + '/wait',
            Bool,
            # latch=True,
            queue_size=1
        )
        self.__clear_pub = rospy.Publisher(
            str(self.name) + '/clear_cmd',
            Bool,
            # latch=True,
            queue_size=1
        )
        self.__joint_pub = rospy.Publisher(
            str(self.name) + '/joint_pose_msg',
            JointPose,
            # latch=True,
            queue_size=1
        )
        self.__p2p_pub = rospy.Publisher(
            str(self.name) + '/p2p_pose_msg',
            P2PPose,
            # latch=True,
            queue_size=1
        )
        self.__line_pub = rospy.Publisher(
            str(self.name) + '/kinematics_pose_msg',
            KinematicsPose,
            # latch=True,
            queue_size=1
        )
        self.__status_sub = rospy.Subscriber(
            str(self.name) + '/status',
            StatusMsg,
            self.__status_callback,
            queue_size=1
        )
        self.__stop_sub = rospy.Subscriber(
            'robot/is_stop',
            Bool,
            self.__stop_callback,
            queue_size=5
        )
        # Waiting for topic enable
        rospy.sleep(0.3)

    def __status_callback(self, msg):
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.__ik_fail = True

        elif 'End Trajectory' in msg.status_msg:
            self.__is_busy = False
            print('Arm task receive End Trajectory')

    def __stop_callback(self, msg):
        if msg.data:
            self.__is_stop = True
        
    def back_home(self):
        self.jointMove(0,(0, 0, 0, 0, 0, 0, 0))

    @property
    def is_busy(self):
        return self.__is_busy

    @property
    def wait(self):
        return self.__is_wait

    @wait.setter
    def wait(self, state):
        if type(state) is bool:
            self.__is_wait = state
        else:
            err_msg = 'Type Error'
            print(err_msg)
            raise Exception(err_msg)

    @property
    def is_ikfail(self):
        return self.__ik_fail

    @property
    def is_stop(self):
        return self.__is_stop

    def set_speed(self,i_speed):
        self.__speed = i_speed

    def jointMove(self, slide_pos = 0,cmd=[0, 0, 0, 0, 0, 0, 0]):
        """Publish msg of joint cmd (rad) to manager node."""
        name  = list()
        value = list()
        speed = self.__speed
        
        for i, val in enumerate(cmd):
            name.append('joint{}'.format(i+1))
            value.append(val)

        self.__joint_pub.publish(JointPose(name, value, slide_pos, speed))
        self.__is_busy = True

    def singleJointMove(self, index=-1, pos=0):
        fb = self.get_joint()
        slide_pos = fb.slide_pos
        joint_pos = list(fb.joint_value)
        if index == 0:
            slide_pos = slide_pos + pos
            self.jointMove(slide_pos ,joint_pos)
        elif 0 < index <= 7:
            joint_pos[index-1] = joint_pos[index-1] + pos
            self.jointMove(slide_pos ,joint_pos)

    def euler2rotation(self, euler):
        roll, pitch, yaw = euler

        origin    = np.matrix([[1, 0, 0],
                               [0, -1, 0],
                               [0, 0, -1]])

        rotationX = np.matrix([[1.0,      0.0,       0.0],
                               [0.0, cos(yaw), -sin(yaw)],
                               [0.0, sin(yaw),  cos(yaw)]])

        rotationY = np.matrix([[cos(pitch),  0.0, sin(pitch)],
                               [0.0,         1.0,        0.0],
                               [-sin(pitch), 0.0, cos(pitch)]])

        rotationZ = np.matrix([[cos(roll), -sin(roll), 0.0],
                               [sin(roll),  cos(roll), 0.0],
                               [0.0,           0.0,    1.0]])
        return origin * rotationY * rotationX * rotationZ

    def euler2quaternion(self, euler):
        roll, pitch, yaw = euler
        quaternion = tf.transformations.quaternion_from_euler(-pitch+pi, -yaw, roll-pi, 'ryxz')
        return (quaternion)

    def ikMove(self, mode='line', pos=_POS, euler=_ORI, phi=_PHI):
        """Publish msg of ik cmd (deg) to manager node."""
        roll, pitch, yaw = euler
        roll  = roll * pi/ 180
        pitch = pitch* pi/ 180
        yaw   = yaw  * pi/ 180

        self.__is_busy = True

        msg = KinematicsPose()
        msg.name = 'arm'
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        quaternion = self.euler2quaternion((roll, pitch, yaw))
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        msg.speed = self.__speed
        msg.phi = radians(phi)
       
        #rospy.loginfo('Sent:{}'.format(cmd))

        if mode == 'line':
            self.__line_pub.publish(msg)
        elif mode == 'p2p':
            self.__p2p_pub.publish(msg)


    def quaternion2euler(self, ori):
        quaternion = (
            ori.x,
            ori.y,
            ori.z,
            ori.w
        )
        pitch, yaw, roll = tf.transformations.euler_from_quaternion(quaternion, 'ryxz')
        # euler = roll+pi ,-pitch+pi, -yaw
        euler = -roll, -pitch, yaw
        return euler

    def rotation2vector(self, rot):
        vec_n = [rot[0, 0], rot[1, 0], rot[2, 0]]
        vec_s = [rot[0, 1], rot[1, 1], rot[2, 1]]
        vec_a = [rot[0, 2], rot[1, 2], rot[2, 2]]
        return vec_n, vec_s, vec_a 

    def get_fb(self):
        rospy.wait_for_service(self.name + '/get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                self.name + '/get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
    
    def get_joint(self):
        rospy.wait_for_service(self.name + '/get_joint_pose')
        try:
            joint = rospy.ServiceProxy(
                self.name + '/get_joint_pose',
                GetJointPose
            )
            name  = list()
            for i in range(1, 8):
                name.append('joint{}'.format(i))
                
            res = joint(name)
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def noa_move_suction(self, mode='p2p', suction_angle=_suction_angle, n=0, o=0, a=0):
        suction_angle = suction_angle * pi/180
        suction_rot = np.matrix([[cos(suction_angle),  0.0, sin(suction_angle)],
                               [0.0,                 1.0,                0.0],
                               [-sin(suction_angle), 0.0, cos(suction_angle)]])
        fb = self.get_fb()
        pos = fb.group_pose.position
        phi = fb.phi
        euler = fb.euler
        # euler = self.quaternion2euler(ori)
    
        rot = self.euler2rotation(euler) * suction_rot
        vec_n, vec_o, vec_a = self.rotation2vector(rot) #for suction
        move = [0, 0, 0]

        if n > 1e-10:
            move += multiply(vec_n, n)
        if o != 0:
            move += multiply(vec_o, o)
        if a != 0:
            move += multiply(vec_a, a)
        self.ikMove(
            mode,
            (pos.x + move[0], pos.y + move[1], pos.z + move[2]),
            (degrees(euler[0]), degrees(euler[1]), degrees(euler[2])),
            degrees(phi)
        )

    def noa_relative_pos(self, mode='p2p', pos=_POS, euler=_ORI, phi=_PHI, suction_angle=_suction_angle, n=0, o=0, a=0):
        #由a點移動至基於b點延noa向量移動後的c點
        suction_angle = suction_angle * pi/180
        suction_rot = np.matrix([[cos(suction_angle),  0.0, sin(suction_angle)],
                               [0.0,                 1.0,                0.0],
                               [-sin(suction_angle), 0.0, cos(suction_angle)]])
        euler[0], euler[1], euler[2] = radians(euler[0]), radians(euler[1]), radians(euler[2])
        rot = self.euler2rotation(euler) * suction_rot
        vec_n, vec_o, vec_a = self.rotation2vector(rot) #for suction
        move = [0, 0, 0]

        if n > 1e-10:
            move += multiply(vec_n, n)
        if o != 0:
            move += multiply(vec_o, o)
        if a != 0:
            move += multiply(vec_a, a)

        self.ikMove(
            mode,
            (pos[0] + move[0], pos[1] + move[1], pos[2] + move[2]),
            (degrees(euler[0]), degrees(euler[1]), degrees(euler[2])),
            phi
        )

    def relative_move_pose(self, mode='p2p', pos = _POS):
        fb = self.get_fb()
        curr_pos = fb.group_pose.position
        phi = fb.phi
        euler = fb.euler

        pos[0] += curr_pos.x
        pos[1] += curr_pos.y
        pos[2] += curr_pos.z

        self.ikMove(
            mode,
            (pos[0], pos[1], pos[2]),
            (degrees(euler[0]), degrees(euler[1]), degrees(euler[2])),
            degrees(phi)
        )

    def relative_move(self, mode='p2p', euler=_ORI, pos = _POS, phi=_PHI):
        fb = self.get_fb()
        curr_pos = fb.group_pose.position
        pos[0] += curr_pos.x
        pos[1] += curr_pos.y
        pos[2] += curr_pos.z

        self.ikMove(
            mode,
            (pos[0], pos[1], pos[2]),
            (euler[0], euler[1], euler[2]),
            phi
        )

    def move_euler(self, mode='p2p', euler=_ORI):
        fb = self.get_fb()
        pos = fb.group_pose.position
        phi = fb.phi
        self.ikMove(
            mode,
            (pos.x, pos.y, pos.z),
            euler,
            degrees(phi)
        )

    def move_to_vector_point(sef, mode='p2p', pos=_POS, vector=[1,0,0], phi=0): # This funthion will move arm and return suction angle 
    # Only for left arm Euler (0 0 30)
        goal_vec = -vector
        a = 0.866
        b = 0.5
        x, y, z = goal_vec[0], goal_vec[1], goal_vec[2]
        roll_angle = 0.0
        suc_angle = -acos((b*y - a*z) / (a*a + b*b))
        roll_angle_c = acos(x / sin(suc_angle))
        roll_angle_s = asin(-((a*y + b*z)/(a*a + b*b)) / sin(suc_angle))
        if (roll_angle_c*roll_angle_s) >= 0:
            roll_angle = roll_angle_c
        else:
            roll_angle = -roll_angle_c

        pos[0] += vector[0]*0.065
        pos[1] += vector[1]*0.065
        pos[2] += vector[2]*0.065
        euler[0] = roll_angle
        euler[1] = 0
        euler[2] = 30
        self.ikMove(
            mode,
            (pos[0], pos[1], pos[2]),
            (euler[0], euler[1], euler[2]),
            phi
        )
        return suc_angle

    def wait_busy(self):
        """This is blocking method."""
        while self.is_busy:
            rospy.sleep(0.1)

    def freeze(self,  enable):
        self.__wait_pub.publish(enable)

    def clear_cmd(self):
        self.__clear_pub.publish(True)

# if __name__ == '__main__':
#     rospy.init_node('test_arm_task')
#     print("Test arm task script")
    
#     a = ArmTask('right_arm')
#     rospy.sleep(0.3)

#     a.set_speed(100)
#     a.jointMove(0, (0, -1, 0, 1, 0, 0, 0))
#     a.set_speed(20)
#     a.wait_busy()
    
#     a.ikMove('p2p', (0, -0.3, -0.9), (0, 0, 0), 30) 
#     a.set_speed(100)
#     a.wait_busy()
    
#     a.noa_move_suction('p2p', -45, n=0, s=0, a=-0.1)
#     a.wait_busy()
        
#     a.singleJointMove(0,-0.2)
#     a.wait_busy()
        
#     a.jointMove(0, (0, -1, 0, 1, 0, 0, 0))
#     a.wait_busy()
        
#     a.singleJointMove(2,0.5)
#     a.wait_busy()
        
#     a.relative_move_pose('p2p', (0, 0.1, 0) )
#     a.wait_busy()
    
#     a.back_home()
#     a.wait_busy()
 
