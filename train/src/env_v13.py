#!/usr/bin/env python3

"""classic Acrobot task"""
import numpy as np
from numpy import sin, cos, pi
from gym import core, spaces
from gym.utils import seeding
import rospy
import math
import time
from train.srv import get_state, move_cmd, set_goal, set_start
from CheckCollision_v1 import CheckCollision
from gazebo_msgs.msg import ModelState
# from CheckCollision_tensor import CheckCollision
# from vacuum_cmd_msg.srv import VacuumCmd
class Test(core.Env):
    ACTION_VEC_TRANS = 1/240
    ACTION_ORI_TRANS = 1/80
    ACTION_PHI_TRANS = 1/80

    NAME = ['/right_arm', '/left_arm', '/right_arm']

    def __init__(self, name):
        self.__name = self.NAME[name%2]
        self.__obname = self.NAME[name%2 + 1]
        self.viewer = None

        high = np.array([1.,1.,1.,1.,1.,1.,1.,1.,  #8
                         1.,1.,1.,1.,1.,1.,1.,     #7
                         0.,0.,0.,0.,0.,0.,         #6
                         0.,0.,0.,0.,0.,0.,0.,0.,0.,#9
                         0.,0.,0.,                  #2
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
        self.obs_dim=42
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
        self.range_cnt = 0.6
        self.rpy_range = 0.1
        self.done = True
        self.s_cnt = 0
        self.goal_err = 0.08
        self.ori_err = 0.24
        self.quat_inv = False
        self.set_mode_pub = rospy.Publisher(
            '/gazebo/set_model_state',
            ModelState,
            queue_size=1,
            latch=True
        )
        self.seed()
        self.reset()
    
    @property
    def is_success(self):
        return self.done

    @property
    def success_cnt(self):
        return self.s_cnt
        
    def get_state_client(self, name):
        service = name+'/get_state'
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
        service = name+'/move_cmd'
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

    def set_start_client(self, cmd, rpy, name):
        service = name+'/set_start'
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
        service = name+'/set_goal'
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
        msg.model_name = name
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


    def move(self, goal):
        self.goal = goal
        res = self.get_state_client(self.__name)
        res_ = self.get_state_client(self.__obname)
        self.old, self.joint_pos[:15], self.joint_angle = np.array(res.state), res.joint_pos, res.joint_angle
        self.limit, self.goal_quat, self.quat_inv, self.joint_pos[15:30] = res.limit, res.quaterniond, res.quat_inv, res_.joint_pos
        linkPosM, linkPosS = self.collision_init()
        _, Link_dis = self.cc.checkCollision(linkPosM, linkPosS)
        s = np.append(self.old[:3], np.subtract(self.goal[:3], self.old[:3]))
        s = np.append(s, Link_dis)
        s = np.append(s, self.joint_angle)
        s = np.append(s, self.limit[0])
        self.dis_pos = np.linalg.norm(self.goal[:3] - s[:3])
        s = np.append(s, self.dis_pos)
        self.state = s
        return s

    def reset(self):
        self.goal = self.set_goal()
        self.old, self.joint_pos[:15], self.joint_pos[15:30], self.joint_angle, self.limit, self.goal_quat, self.quat_inv = self.set_old()
        linkPosM, linkPosS = self.collision_init()
        alarm, Link_dis = self.cc.checkCollision(linkPosM, linkPosS)
        alarm_cnt = 0
        for i in alarm:
            alarm_cnt += i
        if alarm_cnt>0:
            return self.reset()
        self.state = np.append(self.old, np.subtract(self.goal[:3], self.old[:3]))
        self.state = np.append(self.state, self.goal_quat)
        self.state = np.append(self.state, Link_dis)
        self.state = np.append(self.state, self.joint_angle)
        self.state = np.append(self.state, self.limit)
        self.dis_pos = np.linalg.norm(self.goal[:3] - self.old[:3])
        # self.dis_ori = np.linalg.norm(self.goal[3:7] - self.old[3:7])
        self.angle_ori = self.quat_angle()
        self.state = np.append(self.state, self.dis_pos)
        self.state = np.append(self.state, self.angle_ori)
        self.state = np.append(self.state, self.joint_pos[6:12])
        self.collision = False
        self.done = False
        return self.state

    def set_goal(self):
        self.goal = self.np_random.uniform(low=0., high=self.range_cnt, size=(8,))
        rpy = self.np_random.uniform(low=-1*self.rpy_range, high=self.rpy_range, size=(3,))
        # print('self.goal = ', self.goal)
        self.goal[0] = 0
        # self.goal[3] = self.range_cnt/2
        self.goal = np.append(self.goal, self.range_cnt)
        res = self.set_goal_client(self.goal, rpy, self.__name)
        goal_pos = np.array(res.state)
        if not res.success:
            return self.set_goal()
        if np.linalg.norm(goal_pos[:2])>0.2:
            return goal_pos[:7]
        else:
            return self.set_goal()

    def set_old(self):
        self.start = self.np_random.uniform(low=0., high=self.range_cnt, size=(8,))
        rpy = self.np_random.uniform(low=-1*self.rpy_range, high=self.rpy_range, size=(3,))
       
        self.start[0] = 0
        # self.start[3] = self.range_cnt/2
        self.start = np.append(self.start, self.range_cnt)
        res = self.set_start_client(self.start, rpy, self.__name)
        res_ = self.get_state_client(self.__obname)
        old_pos = np.array(res.state)
        if not res.success:
            return self.set_old()
        if np.linalg.norm(np.subtract(old_pos[:3], self.goal[:3])) > 0.1:
            return old_pos, res.joint_pos, res_.joint_pos, res.joint_angle, res.limit, res.quaterniond, res.quat_inv
        else:
            return self.set_old()

    def collision_init(self):
        linkPosM = np.array(self.joint_pos[:15])
        linkPosS = np.array(self.joint_pos[15:])
        linkPosM = np.append([0.,0.,-0.8], linkPosM)
        linkPosS = np.append([0.,0.,-0.8], linkPosS)
        linkPosM = linkPosM.reshape(6,3)
        linkPosS = linkPosS.reshape(6,3)
        return linkPosM, linkPosS

    def step(self, a):
        alarm = []
        Link_dis = []
        s = self.state
        self.collision = False
        action_vec = a[:3]*self.ACTION_VEC_TRANS
        action_ori = a[3:7]*self.ACTION_ORI_TRANS
        action_phi = a[7]*self.ACTION_PHI_TRANS
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
            self.angle_ori = self.quat_angle()
            s = np.append(s, self.dis_pos)
            s = np.append(s, self.angle_ori)
            s = np.append(s, self.joint_pos[6:12])
   
        terminal = self._terminal(s, res.success, alarm)
        reward = self.get_reward(s, res.success, terminal)

        if not self.collision and math.fabs(s[7])<0.9:
            self.state = s

        self.set_object(self.__name, (self.goal[0]-0.08, self.goal[1], self.goal[2]+1.45086), (0, 0, 0, 0))
        return self.state, reward, terminal, self.collision

    def _terminal(self, s, ik_success, alarm):
        alarm_cnt = 0
        for i in alarm:
            alarm_cnt += i
        if alarm_cnt>0.5:
            self.collision = True
        if ik_success and not self.collision:
            if self.dis_pos < self.goal_err and self.angle_ori < self.ori_err:
                if not self.done:
                    self.done = True
                    self.s_cnt += 1
                    self.range_cnt = self.range_cnt + 0.003 if self.range_cnt < 0.95 else 0.95
                    self.rpy_range = self.rpy_range + 0.0005 if self.rpy_range < 0.8 else 0.8
                    self.goal_err = self.goal_err*0.999 if self.goal_err > 0.01 else 0.01
                    self.ori_err = self.ori_err*0.999 if self.ori_err > 0.05 else 0.05
                return True
            else:
                return False
        else:
            return False
        

    def get_reward(self, s, ik_success, terminal):
        reward = 0.

        if not ik_success:
            return -4
        if self.collision:
            return -5
        if math.fabs(s[7])>0.9:
            return -3

        reward -= self.dis_pos
        reward -= self.angle_ori
        reward += 0.4
        
        if reward > 0:
            reward *= 3
        d = np.linalg.norm([self.joint_pos[6], self.joint_pos[8]])
        if d<0.1:
            reward -= (0.1-d)*20

        cos_vec = np.dot(self.action[:3],  self.state[8:11])/(np.linalg.norm(self.action[:3]) *np.linalg.norm(self.state[8:11]))
        
        reward += (cos_vec*self.dis_pos - self.dis_pos)
        # reward += (cos_ori*self.dis_ori/2 - self.dis_ori/2)
        
        

        reward -= 2
        # if terminal and ik_success and not self.collision:
        #     reward += 1
        # if self.dis_pos < 0.04:
        #     reward += 1
        
        # if self.dis_pos < 0.1:
        #     reward += 1
        return reward
        #==================================================================================


