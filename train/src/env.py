#!/usr/bin/env python3

"""classic Acrobot task"""
import numpy as np
from numpy import sin, cos, pi
from gym import core, spaces
from gym.utils import seeding
import rospy
from train.srv import environment
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

    ACTION_VEC_TRANS = 1/1000
    ACTION_ORI_TRANS = 1/300
    ACTION_PHI_TRANS = 1/36

    AVAIL_TORQUE = [-1., 0., +1]

    torque_noise_max = 0.

    #: use dynamics equations from the nips paper or the book
    book_or_nips = "book"
    action_arrow = None
    domain_fig = None
    actions_num = 8

    def __init__(self):
        rospy.init_node('envtest')
        self.viewer = None
        high = np.array([1.,1.,1.,1.,1.,1.,1.,1.,
                         1.,1.,1.,1.,1.,1.,1.,1.,
                         0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
        low = -high # gx,gy,gz,ga,gb,gc,gd,gf,
                    #ox,oy,oz,oa,ob,oc,od,of,
                    # 1xyz,2xyz,4xyz,6xyz,
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(8)
        self.state = []
        self.action = []
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
        self.seed()
        
    def get_state_client(self, cmd):
        ik_service = '/train_env'
        try:
            rospy.wait_for_service(ik_service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.get_state_client(cmd)
            
        client = rospy.ServiceProxy(
            ik_service,
            environment
        )
        # res = client(cmd)
        res = client.call(cmd)
        return res

    def env_reset_client(self, cmd):
        reset_service = '/env_reset'
        try:
            rospy.wait_for_service(reset_service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.env_reset_client(cmd)
            
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

    def reset(self):
        self.goal = self.set_goal()
        self.old, self.joint_pos = self.set_old()
        self.state = np.append(self.goal, self.old)
        self.state = np.append(self.state, self.joint_pos)#.tolist()
        return self.state

    def set_goal(self):
        # self.goal_pos = self.np_random.uniform(low=-1., high=1., size=(3,))
        # self.goal_quat = self.np_random.uniform(low=-1., high=1., size=(4,))
        # self.goal_phi = self.np_random.uniform(low=-1., high=1., size=(1,))
        # self.goal_quat /= np.linalg.norm(self.goal_quat)
        # self.goal = np.append(self.goal_pos, self.goal_quat)
        # self.goal = np.append(self.goal, self.goal_phi)#.tolist()
        # self.goal.append(self.goal_pos)
        # self.goal.append(self.goal_quat)
        # self.goal.append(self.goal_phi)
        self.goal = self.np_random.uniform(low=0., high=1., size=(8,))
        res = self.env_reset_client(self.goal)
        if res.success:
            return res.state
        else:
            return self.set_goal()

    def set_old(self):
        # self.old_pos = self.np_random.uniform(low=-1., high=1., size=(3,))
        # self.old_quat = self.np_random.uniform(low=-1., high=1., size=(4,))
        # self.old_phi = self.np_random.uniform(low=-1., high=1., size=(1,))
        # self.old_quat /= np.linalg.norm(self.old_quat)
        # self.old = np.append(self.old_pos, self.old_quat)
        # self.old = np.append(self.old, self.old_phi)#.tolist()
        # self.old.append(self.old_pos))
        # self.old.append(self.old_quat)
        # self.old.append(self.old_phi)
        self.old = self.np_random.uniform(low=0., high=1., size=(8,))
        res = self.env_reset_client(self.old)
        if res.success:
            return res.state, res.joint_pos
        else:
            return self.set_old()

    def step(self, a):
        s = self.state
        action_vec = a[:3]*self.ACTION_VEC_TRANS
        action_ori = (a[3:7]/np.linalg.norm(a[3:7]))*self.ACTION_ORI_TRANS
        action_phi = a[7]*np.pi*self.ACTION_PHI_TRANS
        self.action = np.append(action_vec, action_ori)
        self.action = np.append(self.action, action_phi)#.tolist()
        # self.action.append(action_vec)
        # self.action.append(action_ori)
        # self.action.append(action_phi)
        self.action = np.add(s[8:16], self.action)
        self.action[3:7] /= np.linalg.norm(self.action[3:7])
        res = self.get_state_client(self.action)
        if res.success:
            self.old, self.joint_pos = res.state, res.joint_pos
            s = np.append(self.goal, self.old)
            s = np.append(s, self.joint_pos)#.tolist()
            # s.append(self.goal)
            # s.append(self.old)
            # s.append(self.joint_pos)
        
        terminal = self._terminal(s, res.success)
        reward = self.get_reward(s, res.success, terminal)
        self.state = s
        return self.state, reward, terminal, -1

    def _terminal(self, s, ik_success):
        if ik_success:
            dis_pos = np.linalg.norm(self.goal[:3] - s[8:11])
            dis_ori = np.linalg.norm(self.goal[3:7] - s[3:7])
            dis_phi = np.fabs(self.goal[7] - s[7])
            if dis_pos < 0.01 and dis_ori < 0.1 and dis_phi < 0.1:
                return True
            else:
                return False
        else:
            return True
        

    def get_reward(self, s, ik_success, terminal):
        goal_vec = self.goal[:3] - self.state[8:11]
        goal_ori = self.goal[3:7]- self.state[11:15]
        goal_phi = self.goal[7]  - self.state[15]

        cos_vec = np.dot(self.action[:3],  goal_vec)/(np.linalg.norm(self.action[:3]) *np.linalg.norm(goal_vec))
        cos_ori = np.dot(self.action[3:7], goal_ori)/(np.linalg.norm(self.action[3:7])*np.linalg.norm(goal_ori))
       
        goal_dis = np.linalg.norm(goal_vec)
        a_leng = np.linalg.norm(self.action[:3]/self.ACTION_VEC_TRANS)

        reward = 0
        if terminal:
            if ik_success:
                reward += 500
            else:
                reward += -2000
            return reward
        if a_leng<0.2 or a_leng>2:
            reward += -2
        if cos_vec > np.math.cos(30*np.pi/180):
            r = (cos_vec*cos_vec*cos_vec)/(goal_dis**0.5)
            reward += 3 if r > 3 else r
        if cos_vec < np.math.cos(60*np.pi/180):
            r = -goal_dis/(cos_vec+1)
            reward += -1 if r<-1 else r
        if cos_ori > np.math.cos(30*np.pi/180):
            reward += 1
        # if cos_ori < np.math.cos(60*np.pi/180):
        #     reward += -1
        if goal_phi*self.action[7] > 0:
            reward += 0.5
        return reward/5

    def _get_ob(self):
        return self.state
   
    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None

