#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
import tensorflow as tf
from sac_v12 import SAC
from env_v18 import Test
from manipulator_h_base_module_msgs.msg import P2PPose

MAX_EPISODES = 100000
MAX_EP_STEPS =  600
MEMORY_CAPACITY = 10000
BATTH_SIZE = 512
SIDE = ['right_', 'left_']
SIDE_ = ['R', 'L']
GOAL_REWARD = 800
LOAD = False
SAVE = [False, False]
COUNTER = [1, 1]
TRAIN_CNT = [0, 0]
EP = [0, 0]
WORKS = 4
SUCCESS_ARRAY = np.zeros([2,500])
GOAL_RATE = [40, 40]
ACTION_FLAG = [False, False]

def worker(name, workers, agent):
    global SUCCESS_ARRAY, ACTION_FLAG, SAVE, COUNTER, EP
    SUCCESS_RATE = 0
    COLLISION = False
    
    env = Test(name, workers) #0 = right
    time.sleep(0.5)
    print(threading.current_thread())
    print('name', name, 'workers', workers, 'agentID', id(agent))
    cmd = [0.,0.3,-0.5,0,1,0,0,0]
    if name == 0:
        cmd[1] *= -1
    env.move_cmd_client(cmd, '/'+SIDE[name])

    t1 = time.time()
    while (not COORD.should_stop()) and (not rospy.is_shutdown()):
        
        s = env.reset()
        
        ep_reward = 0
        done_cnt = 0
        EP[name] += 1
        ep = EP[name]
        SUCCESS_ARRAY[name, ep%500] = 0.
        COLLISION = False
        for j in range(MAX_EP_STEPS):
            WORKER_EVENT[name].wait()
            
            a = agent.choose_action(s)
            s_, r, done, collision = env.step(a)
            agent.replay_buffer[workers].store_transition(s, a, r, s_, done)
            done_cnt += int(done)
            if collision:
                COLLISION = True
            s = s_
            ep_reward += r

            COUNTER[name]+=1
            if COUNTER[name] >= BATTH_SIZE*32 and COUNTER[name]%(8*WORKS) == 0:
                WORKER_EVENT[name].clear()
                for _ in range(8):
                    agent.learn(TRAIN_CNT[name])
                    TRAIN_CNT[name]+=1
                WORKER_EVENT[name].set()
                
                # LEARN_EVENT[name].set()
            if done_cnt > 10:
                if not COLLISION:
                    SUCCESS_ARRAY[name, ep%500] = 1.
                break

        SUCCESS_RATE = 0
        for z in SUCCESS_ARRAY[name]:
            SUCCESS_RATE += z/5
        if SUCCESS_RATE >= GOAL_RATE[name]:
            SAVE[name] = True
        else:
            SAVE[name] = False
        agent.replay_buffer[workers].store_eprwd(ep_reward*j/100)
        
        if workers == 0 and SAVE[name]:
            SUCCESS_ARRAY[name] = np.zeros([500])
            save(agent, name)
            print('Running time: ', time.time() - t1)
        if env.is_success:
            print(SIDE_[name]+str(workers), ep, ' Reward: %i' % int(ep_reward), 'cnt: ',j, 's_rate: ', int(SUCCESS_RATE), 'sssuuucccccceeessssss ', env.success_cnt)
        else:
            print(SIDE_[name]+str(workers), ep, ' Reward: %i' % int(ep_reward), 'cnt: ',j, 's_rate: ', int(SUCCESS_RATE))

def save(agent, name):
    print(agent.path)
    if os.path.isdir(agent.path+str(GOAL_RATE[name])): shutil.rmtree(agent.path+str(GOAL_RATE[name]))
    os.mkdir(agent.path+str(GOAL_RATE[name]))
    ckpt_path = os.path.join(agent.path+str(GOAL_RATE[name]), 'SAC.ckpt')
    save_path = agent.saver.save(agent.sess, ckpt_path, write_meta_graph=False)
    print("\nSave Model %s\n" % save_path)
    if GOAL_RATE[name] < 90:
        GOAL_RATE[name] += 5
    else:
        GOAL_RATE[name] += 2
    if GOAL_RATE[name] > 100:
        COORD.request_stop()

def train(name):
    global SAVE, COUNTER
    threads_ = []
    print(threading.current_thread())
    env = Test(name, 0)
    agent = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim,
            lr_actor=1e-3, lr_value=1e-3, gamma=0.99, tau=0.995, buffers = WORKS, name=SIDE[name])
    env = None
    print('name', name, 'agentID', id(agent))

    for j in range(WORKS):
        t = threading.Thread(target=worker, args=(name, j, agent,))
        threads_.append(t)
    time.sleep(1)
    for i in threads_:
        i.start()
        time.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    # RUN_FLAG = []
    WORKER_EVENT = [threading.Event(), threading.Event()]
    COORD = tf.train.Coordinator()

    for i in range(2):
        t = threading.Thread(target=train, args=(i,))
        threads.append(t)
        WORKER_EVENT[i].set()

    for i in threads:
        i.start()
        
    print(threading.current_thread())
    COORD.join(threads)
