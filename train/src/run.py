#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
from ddpg import DDPG
from env_v4 import Test 

MAX_EPISODES = 100000
MAX_EP_STEPS = 500
MEMORY_CAPACITY = 10000
SIDE = ['right_', 'left_']
GOAL_REWARD = 1000
LOAD = False

def train(nameIndx):
    T_REWARD = []
    MU_REWARD = 0
    BEST_R = -1000
    env = Test(nameIndx) #0 = right

    # agent = DDPG(a_dim, s_dim, a_bound, SIDE[nameIndx])
    agent = DDPG(act_dim=8, obs_dim=42,
                    lr_actor=0.0001, lr_q_value=0.001, gamma=0.99, tau=0.01, action_noise_std=1, name=SIDE[nameIndx])

    var = 0.8  # control exploration
    rar = 0.3
    cnt = 0
    
    t1 = time.time()
    for i in range(MAX_EPISODES):
        t2, t3, t23, t32 = 0., 0., 0., 0.
        s = env.reset() 
        ep_reward = 0
        for j in range(MAX_EP_STEPS):
            a = agent.choose_action(s)
            # a = np.clip(np.random.normal(a, var), -1, 1)    # add randomness to action selection for exploration
            t2 = time.time()
            if t3 != 0:t32 += (t2-t3)
            s_, r, done, info = env.step(a)
            t3 = time.time()
            t23 += (t3-t2)
            agent.memory.store_transition(s, a, r/10, s_, done)
  
            if cnt > MEMORY_CAPACITY:
                if cnt%100 == 0:
                    agent.learn(cnt)
                    # agent.learn()
                else:
                    agent.learn(False)
                    # agent.learn()
            s = s_
            ep_reward += r
            cnt+=1
        
        
        if len(T_REWARD) >= 100:
            T_REWARD.pop(0)
        T_REWARD.append(ep_reward)
        r_sum = 0
        for k in T_REWARD:
            r_sum += k
        MU_REWARD = r_sum/100
        BEST_R = MU_REWARD if MU_REWARD>BEST_R else BEST_R
        print('Episode:', i, ' Reward: %i' % int(ep_reward), 'MU_REWARD: ', int(MU_REWARD),'BEST_R: ', int(BEST_R), 'cnt = ',j , 't_step:', int(t23), 't_learn: ', int(t32)) #'var: %.3f' % var, 'rar: %.3f' % rar)
        if MU_REWARD > GOAL_REWARD:
            break

    if os.path.isdir(agent.path): shutil.rmtree(agent.path)
    os.mkdir(agent.path)
    ckpt_path = os.path.join(agent.path, 'DDPG.ckpt')
    save_path = agent.saver.save(agent.sess, ckpt_path, write_meta_graph=False)
    print("\nSave Model %s\n" % save_path)
    print('Running time: ', time.time() - t1)

def action_sample(s):
    a = s[8:16]
    a[:3] /= np.linalg.norm(a[:3])
    a[3:7]/= np.linalg.norm(a[3:7])
    a[7]  /= math.fabs(a[7])
    a[:3] *= s[-3]
    a[3:7]*= s[-2]
    a[7]  *= s[-1]
    return a


if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    
    for i in range(2):
        t = threading.Thread(target=train, args=(i,))
        threads.append(t)
    for i in range(2):
        threads[i].start()