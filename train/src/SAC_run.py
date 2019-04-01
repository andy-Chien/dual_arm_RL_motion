#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
from sac_v6 import SAC
from env_v7 import Test 

MAX_EPISODES = 100000
MAX_EP_STEPS = 800
MEMORY_CAPACITY = 10000
BATTH_SIZE = 256
SIDE = ['right_', 'left_']
GOAL_REWARD = 800
LOAD = False

def train(nameIndx):
    global r_run, l_run
    T_REWARD = []
    MU_REWARD = 0
    BEST_R = 0
    env = Test(nameIndx) #0 = right

    # agent = DDPG(a_dim, s_dim, a_bound, SIDE[nameIndx])
    # agent = PPO(act_dim=8, obs_dim=39,
    #             lr_actor=0.0001, lr_value=0.0002, gamma=0.9, clip_range=0.2, name=SIDE[nameIndx])
    agent = SAC(act_dim=8, obs_dim=39,
            lr_actor=1e-3, lr_value=1e-3, gamma=0.99, tau=0.995, name=SIDE[nameIndx])

    var = 0.8  # control exploration
    rar = 0.3
    cnt = 0
    if nameIndx == 0:
        r_run = False
    elif nameIndx ==1:
        l_run = False
    while r_run or l_run:
        time.sleep(0)
    time.sleep(0.5)
    t1 = time.time()

    for i in range(MAX_EPISODES):

        if nameIndx == 0:
            r_run = False
        elif nameIndx ==1:
            l_run = False
        while r_run or l_run:
            time.sleep(0)

        s = env.reset()

        time.sleep(0.1)
        if nameIndx == 0:
            r_run = True
        elif nameIndx ==1:
            l_run = True
        
        ep_reward = 0
        for j in range(MAX_EP_STEPS):
            cnt+=1
            a = agent.choose_action(s)
            # a = np.clip(np.random.normal(a, var), -1, 1)    # add randomness to action selection for exploration
            # if (i+1)%20 == 0 and np.random.rand(1) < 0.5:
            #     a = action_sample(s)
            s_, r, done, info = env.step(a)
            
            agent.replay_buffer.store_transition(s, a, r/10, s_, done)
            
            if cnt >= BATTH_SIZE * 3:
                if cnt%50 == 0:
                    agent.learn(cnt)
                else:
                    agent.learn(0)

            s = s_
            ep_reward += r
            
        
        
        if len(T_REWARD) >= 100:
            T_REWARD.pop(0)
        T_REWARD.append(ep_reward)
        r_sum = 0
        for k in T_REWARD:
            r_sum += k
        MU_REWARD = r_sum/100
        BEST_R = MU_REWARD if MU_REWARD>BEST_R else BEST_R
        print('Episode:', i, ' Reward: %i' % int(ep_reward), 'MU_REWARD: ', int(MU_REWARD),'BEST_R: ', int(BEST_R), 'cnt = ',j)# , 't_step:', int(t23), 't_learn: ', int(t32)) #'var: %.3f' % var, 'rar: %.3f' % rar)
        if MU_REWARD > GOAL_REWARD:
            break

    if os.path.isdir(agent.path): shutil.rmtree(agent.path)
    os.mkdir(agent.path)
    ckpt_path = os.path.join(agent.path, 'SAC.ckpt')
    save_path = agent.saver.save(agent.sess, ckpt_path, write_meta_graph=False)
    print("\nSave Model %s\n" % save_path)
    print('Running time: ', time.time() - t1)

def action_sample(s):
    a = s[8:16]
    al = max([np.linalg.norm(a[:3]), 0.04])
    rl = max([np.linalg.norm(a[3:7]), 0.01])
    pl = max([math.fabs(a[7]), 0.01])
    a[:3] /= al
    a[3:7]/= rl
    a[7]  /= pl
    a[3:7] *= (1/6)*(rl/al)
    a[7]   *= (1/6)*(pl/al)
    return a


if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    l_run = True
    r_run = True
    for i in range(2):
        t = threading.Thread(target=train, args=(i,))
        threads.append(t)
    for i in range(2):
        threads[i].start()