#!/usr/bin/env python3

"""classic Acrobot task"""
import numpy as np
from numpy import sin, cos, pi
from gym import core, spaces
from gym.utils import seeding
import rospy
import math
import time
from train.srv import get_state, move_cmd, set_goal, set_start, move_init
from CheckCollision_v1 import CheckCollision
from gazebo_msgs.msg import ModelState
# from CheckCollision_tensor import CheckCollision
# from vacuum_cmd_msg.srv import VacuumCmd
class Test(core.Env):
    ACTION_VEC_TRANS = 1/300
    ACTION_ORI_TRANS = 1/100
    ACTION_PHI_TRANS = 1/100

    NAME = ['/right_', '/left_', '/right_']

    def __init__(self, name, workers):
        self.__name = self.NAME[name%2]
        self.__obname = self.NAME[name%2 + 1]
        if workers == 0:
            self.workers = 'arm'
        else:
            self.workers = str(workers)

        high = np.array([1.,1.,1.,1.,1.,1.,1.,1.,  #8
                         1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,     #7
                         0.,0.,0.,0.,0.,0.,         #6
                         0.,0.,0.,0.,0.,0.,0.,0.,0.,#9
                         0.,0.,0.,0.,0.,0.,0.,0.,0.,                  #2
                         0.,0.,0.,0.,0.,0.])                 #1
                                                    #24
        low = -1*high 
                    # ox,oy,oz,oa,ob,oc,od,of,
                    # vx,vy,vz,va,vb,vc,vd,vf
                    # dis of Link(15)
                    # 
                    # joint_angle(7), 
                    # limit(1), rate(3)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(3)
        self.act_dim=8
        self.obs_dim=61
        self.state = []
        self.action = []
        self.cmd = []
        self.point = []
        self.goal = []
        self.goal_pos = []
        self.goal_quat = []
        self.goal_phi = 0
        self.goal_rpy = []
        self.old = []
        self.old_pos = []
        self.old_quat = []
        self.old_phi = 0
        self.joint_pos = []
        self.joint_angle = []
        self.limit = []
        self.s_jointpos = []
        # self.dis_pos
        self.cc = CheckCollision()
        self.collision = False
        self.range_cnt = 0.7
        self.rpy_range = 1
        self.done = True
        self.s_cnt = 0
        self.goal_err = 0.03
        self.ori_err = 0.25
        self.quat_inv = False
        self.goal_angle = []
        self.object_pub = 0
        self.curr_angle = []
        self.move_speed = 0.1
        self.set_mode_pub = rospy.Publisher(
            '/gazebo/set_model_state',
            ModelState,
            queue_size=1,
            latch=True
        )
        self.seed(254*(name+1))
    
    @property
    def is_success(self):
        return self.done

    @property
    def success_cnt(self):
        return self.s_cnt

    @property
    def get_goal(self):
        return self.goal
        
    def get_state_client(self, name):
        service = name+self.workers+'/get_state'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.get_state_client(name)
            
        client = rospy.ServiceProxy(
            service,
            get_state
        )
        # res = client(cmd)
        res = client.call()
        return res

    def move_cmd_client(self, cmd, name):
        service = name+self.workers+'/move_cmd'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.move_cmd_client(cmd, name)
            
        client = rospy.ServiceProxy(
            service,
            move_cmd
        )
        # res = client(cmd)
        res = client.call(cmd)
        return res

    def move_init_client(self, cmd, name):
        service = name+self.workers+'/move_init'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.move_cmd_client(cmd, name)
            
        client = rospy.ServiceProxy(
            service,
            move_init
        )
        # res = client(cmd)
        res = client.call(cmd)
        return res

    def set_start_client(self, cmd, rpy, name):
        service = name+self.workers+'/set_start'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.set_start_client(cmd, rpy, name)
            
        client = rospy.ServiceProxy(
            service,
            set_start
        )
        # res = client(cmd)
        res = client(action=cmd, rpy=rpy)
        return res

    def set_goal_client(self, cmd, rpy, name):
        service = name+self.workers+'/set_goal'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.set_goal_client(cmd, rpy, name)
            
        client = rospy.ServiceProxy(
            service,
            set_goal
        )
        # res = client(cmd)
        res = client(action=cmd, rpy=rpy)
        return res

    def set_object(self, name, pos, ori):
        msg = ModelState()
        msg.model_name = name+self.workers
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.w = ori[0]
        msg.pose.orientation.x = ori[1]
        msg.pose.orientation.y = ori[2]
        msg.pose.orientation.z = ori[3]
        msg.reference_frame = 'world'
        self.set_mode_pub.publish(msg)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def quat_angle(self):
        cos_q = np.dot(self.goal[3:7],  self.old[3:7])
        if (self.quat_inv and cos_q>0) or ((not self.quat_inv) and cos_q<=0):
            cos_q = np.dot(-1*self.goal[3:7],  self.old[3:7])
        return math.acos(cos_q)/pi

    def reset(self, cmd):
        self.move_speed = cmd[8]/100
        cmd[7] = cmd[7]*2/pi
        res = self.move_init_client(cmd, self.__name)
        if not res.enable:
            return [], False, True, res.enable 
        _res = self.get_state_client(self.__name)
        res_ = self.get_state_client(self.__obname)
        self.goal, self.goal_angle, = np.array(res.state)[:7], res.joint_angle
        self.old, self.joint_pos[:15], self.joint_pos[15:30] = _res.state, _res.joint_pos, res_.joint_pos
        self.joint_angle, self.limit, self.goal_quat =  res.joint_angle, res.limit, res.quaterniond
   
        linkPosM, linkPosS = self.collision_init()
        alarm, Link_dis = self.cc.checkCollision(linkPosM, linkPosS)
        alarm_cnt = 0
        for i in alarm:
            alarm_cnt += i
        
        self.state = np.append(self.old, np.subtract(self.goal[:3], self.old[:3]))
        self.state = np.append(self.state, self.goal_quat)

        self.state = np.append(self.state, Link_dis)
        self.state = np.append(self.state, self.joint_angle)
        self.state = np.append(self.state, self.limit)
        self.dis_pos = np.linalg.norm(self.goal[:3] - self.old[:3])
        self.dis_ori = math.sqrt(np.linalg.norm(self.goal[3:7] - self.old[3:7]) + np.linalg.norm(-1*self.goal[3:7] - self.old[3:7]) - 2)
        self.state = np.append(self.state, self.dis_pos)
        self.state = np.append(self.state, self.dis_ori)
        self.state = np.append(self.state, self.joint_pos[6:12])
        self.state = np.append(self.state, self.goal_angle)
        self.collision = False
        self.done = False
        if alarm_cnt > 0:
            self.collision = True
        return self.state, res.success, self.collision, res.enable

    def collision_init(self):
        linkPosM = np.array(self.joint_pos[:15])
        linkPosS = np.array(self.joint_pos[15:])
        linkPosM = np.append([0.,0.,-0.8], linkPosM)
        linkPosS = np.append([0.,0.,-0.8], linkPosS)
        linkPosM = linkPosM.reshape(6,3)
        linkPosS = linkPosS.reshape(6,3)
        return linkPosM, linkPosS

    def move_arm(self, cmd):
        self.move_cmd_client(cmd, self.__name)

    def step(self, a):
        alarm = []
        Link_dis = []
        s = self.state
        self.collision = False
        action_vec = a[:3]* self.ACTION_VEC_TRANS *self.move_speed
        action_ori = a[3:7]*self.ACTION_ORI_TRANS *self.move_speed
        action_phi = a[7]*  self.ACTION_PHI_TRANS *self.move_speed
        self.action = np.append(action_vec, action_ori)
        self.action = np.append(self.action, action_phi)
        self.cmd = np.add(s[:8], self.action)
        self.cmd[3:7] /= np.linalg.norm(self.cmd[3:7])

        res = self.move_cmd_client(self.cmd, self.__name)
        res_ = self.get_state_client(self.__obname)
        if res.success:
            self.old, self.joint_pos[:15], self.joint_angle = np.array(res.state), res.joint_pos, res.joint_angle
            self.limit, self.goal_quat, self.quat_inv, self.joint_pos[15:30] = res.limit, res.quaterniond, res.quat_inv, res_.joint_pos
            linkPosM, linkPosS = self.collision_init()
            alarm, Link_dis = self.cc.checkCollision(linkPosM, linkPosS)
            s = np.append(self.old, np.subtract(self.goal[:3], self.old[:3]))
            s = np.append(s, self.goal_quat)
            s = np.append(s, Link_dis)
            s = np.append(s, self.joint_angle)
            s = np.append(s, self.limit)
            self.dis_pos = np.linalg.norm(self.goal[:3] - s[:3])
            self.dis_ori = math.sqrt(np.linalg.norm(self.goal[3:7] - s[3:7]) + np.linalg.norm(-1*self.goal[3:7] - s[3:7]) - 2)
            s = np.append(s, self.dis_pos)
            s = np.append(s, self.dis_ori)
            s = np.append(s, self.joint_pos[6:12])
            s = np.append(s, self.goal_angle)
   
        terminal = self._terminal(s, res.success, alarm)

        if (not self.collision) and math.fabs(s[7])<0.9:
            self.state = s
        if self.workers == 'arm':
            if self.object_pub == 0:
                self.set_object(self.__name, (self.goal[0]-0.08, self.goal[1], self.goal[2]+1.45086), (0, 0, 0, 0))
                self.object_pub = 1
            else:
                self.set_object(self.__name+'q', (self.goal[0]-0.08, self.goal[1], self.goal[2]+1.45086), self.goal[3:7])
                self.object_pub = 0

        return self.state, terminal, self.collision, res.success, res.singularity

    def _terminal(self, s, ik_success, alarm):
        alarm_cnt = 0
        for i in alarm:
            alarm_cnt += i
        if alarm_cnt>0.4:
            self.collision = True
        if ik_success and not self.collision:
            if self.dis_pos < self.goal_err and self.dis_ori < self.ori_err:
                if not self.done:
                    self.done = True
                    self.s_cnt += 1
                return True
            else:
                return False
        else:
            return False

    # def check_singularity(self):