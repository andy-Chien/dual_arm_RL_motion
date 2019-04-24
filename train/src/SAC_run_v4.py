#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
from sac_v10 import SAC
from env_v14 import Test
from manipulator_h_base_module_msgs.msg import P2PPose

MAX_EPISODES = 100000
MAX_EP_STEPS =  600
MEMORY_CAPACITY = 10000
BATTH_SIZE = 256
SIDE = ['right_', 'left_']
GOAL_REWARD = 800
LOAD = False
SAVE = [False, False]

def train(nameIndx):
    global r_run, l_run, SAVE
    T_REWARD = []
    T_REWARD.append(-2000)
    MU_REWARD = 0
    BEST_R = -999
    SUCCESS_ARRAY = np.zeros([1000])
    SUCCESS_RATE = 0
    COLLISION = False
    GOAL_RATE = 60

    env = Test(nameIndx) #0 = right

    # agent = DDPG(a_dim, s_dim, a_bound, SIDE[nameIndx])
    # agent = PPO(act_dim=8, obs_dim=39,
    #             lr_actor=0.0001, lr_value=0.0002, gamma=0.9, clip_range=0.2, name=SIDE[nameIndx])
    agent = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim,
            lr_actor=1e-3, lr_value=1e-3, gamma=0.99, tau=0.995, name=SIDE[nameIndx])
    print(agent.path)

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
            time.sleep(0.0001)

        s = env.reset()

        time.sleep(0.1)
        if nameIndx == 0:
            r_run = True
        elif nameIndx ==1:
            l_run = True
        
        ep_reward = 0
        done_cnt = 0

        SUCCESS_ARRAY[i%1000] = 0.
        COLLISION = False
        for j in range(MAX_EP_STEPS):
            cnt+=1
            a = agent.choose_action(s)
            s_, r, done, collision = env.step(a)
            agent.replay_buffer.store_transition(s, a, r, s_, done)
            done_cnt += int(done)
            if collision:
                COLLISION = True
            if cnt >= BATTH_SIZE*10:
                if cnt%50 == 0:
                    agent.learn(cnt)
                elif cnt%5 == 0:
                    agent.learn(0)

            s = s_
            ep_reward += r
            if done_cnt > 32:
                if not COLLISION:
                    SUCCESS_ARRAY[i%1000] = 1.
                break

        SUCCESS_RATE = 0
        for z in SUCCESS_ARRAY:
            SUCCESS_RATE += z/10
        if SUCCESS_RATE >= GOAL_RATE:
            SAVE[nameIndx] = True
        else:
            SAVE[nameIndx] = False
        if len(T_REWARD) >= 1000:
            T_REWARD.pop(0)
        T_REWARD.append(ep_reward)
        agent.replay_buffer.store_eprwd(ep_reward*j/100)
        r_sum = 0
        for k in T_REWARD:
            r_sum += k
        MU_REWARD = r_sum/1000
        BEST_R = MU_REWARD if MU_REWARD>BEST_R else BEST_R

        if env.is_success:
            print('Eps:', i, ' Reward: %i' % int(ep_reward), 'MU_R: ', int(MU_REWARD), 'cnt: ',j, 's_rate: ', int(SUCCESS_RATE), 'sssuuucccccceeessssss ', env.success_cnt)
        else:
            print('Eps:', i, ' Reward: %i' % int(ep_reward), 'MU_R: ', int(MU_REWARD), 'cnt: ',j, 's_rate: ', int(SUCCESS_RATE))
        if SAVE[nameIndx]:
            SUCCESS_ARRAY = np.zeros([1000])
            print(agent.path)
            if os.path.isdir(agent.path+str(GOAL_RATE)): shutil.rmtree(agent.path+str(GOAL_RATE))
            os.mkdir(agent.path+str(GOAL_RATE))
            ckpt_path = os.path.join(agent.path+str(GOAL_RATE), 'SAC.ckpt')
            save_path = agent.saver.save(agent.sess, ckpt_path, write_meta_graph=False)
            print("\nSave Model %s\n" % save_path)
            print('Running time: ', time.time() - t1)
            if GOAL_RATE < 90:
                GOAL_RATE += 5
            else:
                GOAL_RATE += 2
            if GOAL_RATE > 100:
                break

def run(nameIndx):
    global cmd, move
    
    SUCCESS_ARRAY = np.zeros([100])
    SUCCESS_RATE = 0
    COLLISION = False

    env = Test(nameIndx) #0 = right

    agent = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim,
            lr_actor=1e-3, lr_value=1e-3, gamma=0.99, tau=0.995, name=SIDE[nameIndx])

    cnt = 0

    while not rospy.is_shutdown() :
        time.sleep(0.1)
        ep_reward = 0
        done_cnt = 0
        if cnt > 999999:
            cnt = 0

        SUCCESS_ARRAY[cnt%100] = 0
        COLLISION = False
        if move[nameIndx]:
            move[nameIndx] = False
            s = env.move(cmd[nameIndx][:])
            while True:
                t1 = time.time()
                a = agent.choose_action(s)
                t2 = time.time()
                s, r, done, collision = env.step(a)
                t3 = time.time()
                done_cnt += int(done)
                if collision:
                    COLLISION = True
                print('t1 ',t2-t1, 't2 ', t3-t2)
                ep_reward += r
                if done_cnt > 10:
                    if not COLLISION:
                        SUCCESS_ARRAY[cnt%100] = 1
                    break
            cnt+=1

            SUCCESS_RATE = 0
            for z in SUCCESS_ARRAY:
                SUCCESS_RATE += z
            print('Episode:', cnt, ' Reward: %i' % int(ep_reward), 's_rate = ', SUCCESS_RATE)
            

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

def right_callback(msg):
    global cmd, move
    cmd[0][0] = msg.pose.position.x
    cmd[0][1] = msg.pose.position.y
    cmd[0][2] = msg.pose.position.z
    cmd[0][3] = msg.pose.orientation.w
    cmd[0][4] = msg.pose.orientation.x
    cmd[0][5] = msg.pose.orientation.y
    cmd[0][6] = msg.pose.orientation.z
    move[0] = True

def left_callback(msg):
    global cmd, move
    cmd[1][0] = msg.pose.position.x
    cmd[1][1] = msg.pose.position.y
    cmd[1][2] = msg.pose.position.z
    cmd[1][3] = msg.pose.orientation.w
    cmd[1][4] = msg.pose.orientation.x
    cmd[1][5] = msg.pose.orientation.y
    cmd[1][6] = msg.pose.orientation.z
    move[1] = True

if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    l_run = True
    r_run = True
    cmd = np.zeros([2,7])
    move = [False, False]
    
    for i in range(2):
        if LOAD:
            t = threading.Thread(target=run, args=(i,))
        else:
            t = threading.Thread(target=train, args=(i,))
        threads.append(t)
    for i in range(2):
        threads[i].start()

    if LOAD:
        rospy.Subscriber('right_arm/drl_pose_msg', P2PPose, right_callback)
        rospy.Subscriber('left_arm/drl_pose_msg', P2PPose, left_callback)
    rospy.spin()