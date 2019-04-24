#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
from sac_v11 import SAC
from env_v15 import Test
from manipulator_h_base_module_msgs.msg import P2PPose

MAX_EPISODES = 100000
MAX_EP_STEPS =  600
MEMORY_CAPACITY = 10000
BATTH_SIZE = 256
SIDE = ['right_', 'left_']
GOAL_REWARD = 800
LOAD = False
SAVE = [False, False]
COUNTER = 0
WORKS = 4
SUCCESS_ARRAY = np.zeros([2,1000])
GOAL_RATE = [60, 60]
ACTION_FLAG = [False, False]

def worker(name, workers):
    global agent
    SUCCESS_RATE = 0
    COLLISION = False

    env = Test(name, workers) #0 = right
    run_flag[name, workers] = False
    while run_flag[0, workers] or run_flag[1, workers]:
        time.sleep(0)
    time.sleep(0.5)

    for i in range(MAX_EPISODES):
        run_flag[name, workers] = False
        while run_flag[0, workers] or run_flag[1, workers]:
            time.sleep(0.0001)

        s = env.reset()

        time.sleep(0.1)
        run_flag[name, workers] = True
        
        ep_reward = 0
        done_cnt = 0

        SUCCESS_ARRAY[name, COUNTER%1000] = 0.
        COLLISION = False
        for j in range(MAX_EP_STEPS):
            cnt+=1
            while ACTION_FLAG[name]:
                time.sleep(0.01)
            ACTION_FLAG[name] = True
            a = agent[name].choose_action(s)
            ACTION_FLAG[name] = False
            s_, r, done, collision = env.step(a)
            agent[name].replay_buffer[workers].store_transition(s, a, r, s_, done)
            done_cnt += int(done)
            if collision:
                COLLISION = True
            s = s_
            ep_reward += r
            if done_cnt > 32:
                if not COLLISION:
                    SUCCESS_ARRAY[name, COUNTER%1000] = 1.
                break

        SUCCESS_RATE = 0
        for z in SUCCESS_ARRAY[name]:
            SUCCESS_RATE += z/10
        if SUCCESS_RATE >= GOAL_RATE[name]:
            SAVE[name] = True
            SUCCESS_ARRAY = np.zeros([2,1000])
        else:
            SAVE[name] = False
        agent[name].replay_buffer[workers].store_eprwd(ep_reward*j/100)
        
        if env.is_success:
            print('Eps:', COUNTER, ' Reward: %i' % int(ep_reward), 'cnt: ',j, 's_rate: ', int(SUCCESS_RATE), 'sssuuucccccceeessssss ', env.success_cnt)
        else:
            print('Eps:', COUNTER, ' Reward: %i' % int(ep_reward), 'cnt: ',j, 's_rate: ', int(SUCCESS_RATE))
        

def train(name):
    global agent
    threads_worker = []
    t1 = time.time()
    for i in range(WORKS):
        t = threading.Thread(target=train, args=(name, i,))
        threads_worker.append(t)
    for i in range(WORKS):
        threads_worker[i].start()

    while True:
        if COUNTER >= BATTH_SIZE*10:
                agent[name].learn(COUNTER)

        if SAVE[name]:
            print(agent[name].path)
            if os.path.isdir(agent[name].path+str(GOAL_RATE[name])): shutil.rmtree(agent[name].path+str(GOAL_RATE[name]))
            os.mkdir(agent[name].path+str(GOAL_RATE[name]))
            ckpt_path = os.path.join(agent[name].path+str(GOAL_RATE[name]), 'SAC.ckpt')
            save_path = agent[name].saver.save(agent[name].sess, ckpt_path, write_meta_graph=False)
            print("\nSave Model %s\n" % save_path)
            print('Running time: ', time.time() - t1)
            if GOAL_RATE[name] < 90:
                GOAL_RATE[name] += 5
            else:
                GOAL_RATE[name] += 2
            if GOAL_RATE[name] > 100:
                break
        time.sleep(0.0001)

if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    agent = []
    run_flag = np.full((2, WORKS), False, dtype=bool)
    
    env = Test(0, 0)
    agent[0] = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim,
            lr_actor=1e-3, lr_value=1e-3, gamma=0.99, tau=0.995, name=SIDE[0])
    agent[1] = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim,
            lr_actor=1e-3, lr_value=1e-3, gamma=0.99, tau=0.995, name=SIDE[1])
    env = None
    for i in range(2):
        t = threading.Thread(target=train, args=(i,))
        threads.append(t)
    for i in range(2):
        threads[i].start()
