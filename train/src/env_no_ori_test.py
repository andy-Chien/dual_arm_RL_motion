#!/usr/bin/env python3

"""classic Acrobot task"""
import numpy as np
from numpy import sin, cos, pi
from gym import core, spaces
from gym.utils import seeding
import rospy
import math
from train.srv import environment
from CheckCollision_v1 import CheckCollision
# from CheckCollision_tensor import CheckCollision
# from vacuum_cmd_msg.srv import VacuumCmd
class Test(core.Env):

    dt = .2

    LINK_LENGTH_1 = 1.  # [m]
    LINK_LENGTH_2 = 1.  # [m]
    LINK_MASS_1 = 1.  #: [kg] mass of link 1
    LINK_MASS_2 = 1.  #: [kg] mass of link 2
    LINK_COM_POS_1 = 0.5  #: [m] position of the center of mass of link 1
    LINK_COM_POS_2 = 0.5  #: [m] position of the center of mass of link 2
    LINK_MOI = 1.  #: moments of inertia for both links

    MAX_VEL_1 = 2.
    MAX_VEL_2 = 2.
    MAX_VEL_3 = 2.

    ACTION_VEC_TRANS = 1/60
    ACTION_ORI_TRANS = 1/10
    ACTION_PHI_TRANS = 1/10

    NAME = ['/right_arm', '/left_arm', '/right_arm']

    torque_noise_max = 0.

    #: use dynamics equations from the nips paper or the book
    book_or_nips = "book"
    action_arrow = None
    domain_fig = None
    actions_num = 8

    def __init__(self, name):
        self.__name = self.NAME[name%2]
        self.__obname = self.NAME[name%2 + 1]
        self.viewer = None

        high = np.array([1.,1.,1.,#1.,1.,1.,1.,1.,  #8
                         1.,1.,1.,#1.,1.,1.,1.,1.,  #8
                         0.,0.,0.,0.,0.,0.,         #6
                         0.,0.,0.,0.,0.,0.,0.,0.,0.,#9
                         0.,0.,                     #2
                         0.])                       #1
                                                    #24
        low = -high 
                    # ox,oy,oz,oa,ob,oc,od,of,
                    # vx,vy,vz,va,vb,vc,vd,vf
                    # dis of Link(15)
                    # 
                    # joint_angle(7), 
                    # limit(1), rate(3)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(3)
        self.act_dim=3
        self.obs_dim=24
        self.state = []
        self.action = []
        self.cmd = []
        self.point = []
        self.goal = []
        self.goal_pos = []
        self.goal_quat = []
        self.goal_phi = 0
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
        self.range_cnt = 0.2
        self.s_cnt = 0
        self.done = True
        self.goal_err = 0.05
        self.seed()
        self.reset()
        
        
    def get_state_client(self, cmd, name):
        ik_service = name+'/train_env'
        try:
            rospy.wait_for_service(ik_service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.get_state_client(cmd, name)
            
        client = rospy.ServiceProxy(
            ik_service,
            environment
        )
        # res = client(cmd)
        res = client.call(cmd)
        return res

    def env_reset_client(self, cmd, name):
        reset_service = name+'/env_reset'
        try:
            rospy.wait_for_service(reset_service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.env_reset_client(cmd, name)
            
        client = rospy.ServiceProxy(
            reset_service,
            environment
        )
        # res = client(cmd)
        res = client.call(cmd)
        return res

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def get_state_jointpos(self):
        self.s_jointpos = []
        self.s_jointpos = np.append(self.joint_pos[6:12], self.joint_pos[18:27])

    def reset(self):
        # if self.done:
        self.goal = self.set_goal()
        self.old, self.joint_pos[:12], self.joint_pos[12:24], self.joint_pos[24:27],self.joint_angle, self.limit = self.set_old()
        linkPosM, linkPosS = self.collision_init(self.old[:3])
        alarm, Link_dis = self.cc.checkCollision(linkPosM, linkPosS)
        self.state = np.append(self.old[:3], np.subtract(self.goal, self.old[:3]))
        self.state = np.append(self.state, Link_dis)
        self.state = np.append(self.state, self.joint_angle)
        self.state = np.append(self.state, self.limit[0])

        self.collision = False
        self.done = False
        alarm_cnt = 0
        for i in alarm:
            alarm_cnt += i
        if alarm_cnt>0:
            return self.reset()
        else:
            return self.state

    def set_goal(self):
        self.goal = self.np_random.uniform(low=0., high=self.range_cnt, size=(8,))
        # print('self.goal = ', self.goal)
        self.goal[0] = 0
        self.goal[3] = self.range_cnt/2
        self.goal = np.append(self.goal, self.range_cnt)
        res = self.env_reset_client(self.goal, self.__name)
        goal_pos = np.array(res.state)

        if res.success and np.linalg.norm(goal_pos[:2])>0.2:
            return goal_pos[:3]
        else:
            return self.set_goal()

    def set_old(self):
        # if self.done:
        self.start = self.np_random.uniform(low=0., high=self.range_cnt, size=(8,))
        # print('self.old = ', self.old)
        self.start[0] = 0
        self.start[3] = self.range_cnt/2
        self.start = np.append(self.start, self.range_cnt)
        res = self.env_reset_client(self.start, self.__name)
        res_ = self.env_reset_client([0], self.__obname)
        old_pos = np.array(res.state)
        if np.linalg.norm(np.subtract(old_pos[:3], self.goal[:3])) > 0.2:
            return np.array(res.state), res.joint_pos, res_.joint_pos,[res_.state[0], res_.state[1], res_.state[2]], res.joint_angle, res.limit
        else:
            return self.set_old()

    def collision_init(self, endpos):
        linkPosM = np.array(self.joint_pos[0:12])
        linkPosS = np.array(self.joint_pos[12:27])
        linkPosM = np.append(linkPosM, endpos)
        linkPosM = np.append([0.,0.,-0.8], linkPosM)
        linkPosS = np.append([0.,0.,-0.8], linkPosS)
        linkPosM = linkPosM.reshape(6,3)
        linkPosS = linkPosS.reshape(6,3)
        return linkPosM, linkPosS

    def step(self, a):
        alarm = []
        Link_dis = []
        s = self.state
        action_vec = a[:3]*self.ACTION_VEC_TRANS
        # action_ori = a[3:7]*self.ACTION_ORI_TRANS
        # action_phi = a[7]*self.ACTION_PHI_TRANS
        self.action = action_vec
        self.cmd = np.add(s[:3], self.action)
        self.cmd = np.append(self.cmd, self.old[3:])
        # if self.__name == '/right_arm':
            # print(self.cmd)
        res = self.get_state_client(self.cmd, self.__name)
        res_ = self.get_state_client([0], self.__obname)
        # if res.success:
        #     self.old, self.joint_pos[:12], self.joint_angle, self.limit = np.array(res.state), res.joint_pos, res.joint_angle, res.limit
        #     self.joint_pos[12:24] = res_.joint_pos
        #     self.joint_pos[24:27] = [res_.state[0], res_.state[1], res_.state[2]]
        #     linkPosM, linkPosS = self.collision_init(self.old[:3])
        #     alarm, Link_dis = self.cc.checkCollision(linkPosM, linkPosS)
        #     s = np.append(self.old[:3], np.subtract(self.goal, self.old[:3]))
        #     s = np.append(s, Link_dis)
        #     s = np.append(s, self.joint_angle)
        #     s = np.append(s, self.limit[0])
        #     self.dis_pos = np.linalg.norm(self.goal[:3] - s[:3])
        # else:
        #     print(res.joint_angle, res.limit)
        s[:3] = self.cmd[:3]
        self.dis_pos = np.linalg.norm(self.goal[:3] - s[:3])
        terminal = self._terminal(s, res.success, alarm)
        reward = self.get_reward(s, res.success, terminal)
        # if self.__name == '/right_arm':
        #     print('cmdcmdcmdcmdcmd', self.cmd)
        #     print('oldoldoldoldold', self.old)
            # print(reward)
        if not self.collision:
            self.state = s
        # else:
        #     print(Link_dis)
        return self.state, reward, terminal, 1

    def _terminal(self, s, ik_success, alarm):
        alarm_cnt = 0
        for i in alarm:
            alarm_cnt += i
        if alarm_cnt>0:
            self.collision = True
        if ik_success and not self.collision:
            if self.dis_pos < self.goal_err:
                if not self.done:
                    self.done = True
                    self.s_cnt += 1
                    self.range_cnt = self.range_cnt + 0.003 if self.range_cnt < 0.95 else 0.95
                    self.goal_err = self.goal_err*0.999 if self.goal_err > 0.001 else 0.001
                    print('ssssssuuuuuuccccccccceeeeeeeesssssssssss' , self.s_cnt)
                return True
            else:
                return False
        else:
            return True
        

    def get_reward(self, s, ik_success, terminal):
        reward = 0

        if terminal and ik_success and not self.collision:
            return 3
        if not ik_success:
            return -3
        if self.collision:
            return -3

        reward -= self.dis_pos
        # if self.dis_pos < 0.04:
        #     reward += 1
        reward += 0.05

        return reward
        #==================================================================================


