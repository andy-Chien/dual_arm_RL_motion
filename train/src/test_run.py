#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
import tensorflow as tf
from test_sac import SAC
from test_env import Test
from arm_control.arm_task import ArmTask
from manipulator_h_base_module_msgs.msg import P2PPose

MAX_EPISODES = 100000
MAX_EP_STEPS =  600
MEMORY_CAPACITY = 10000
BATTH_SIZE = 256
SIDE = ['right_', 'left_']
GOAL_REWARD = 800
LOAD = False
SAVE = [False, False]

def run(nameIndx):
    global cmd, move
    
    SUCCESS_ARRAY = np.zeros([1000])
    SUCCESS_ARRAY_P2P = np.zeros([1000])
    SUCCESS_ARRAY_LINE = np.zeros([1000])
    COLLISION_ARRAY = np.zeros([1000])
    COLLISION_ARRAY_P2P = np.zeros([1000])
    COLLISION_ARRAY_LINE = np.zeros([1000])
    IKFAIL_ARRAY = np.zeros([1000])
    IKFAIL_ARRAY_P2P = np.zeros([1000])
    IKFAIL_ARRAY_LINE = np.zeros([1000])
    SINGULARITY_ARRAY = np.zeros([1000])
    SINGULARITY_ARRAY_P2P = np.zeros([1000])
    SINGULARITY_ARRAY_LINE = np.zeros([1000])
    S_RATE = 0
    S_RATE_P2P = 0
    S_RATE_LINE = 0
    C_RATE = 0
    C_RATE_P2P = 0
    C_RATE_LINE = 0
    I_RATE = 0
    I_RATE_P2P = 0
    I_RATE_LINE = 0
    COLLISION = False
    IKFAIL = False
    SINGULARITY = False

    env = Test(nameIndx, 0) #0 = right
    arm = ArmTask(SIDE[nameIndx]+'arm')

    agent = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim, name=SIDE[nameIndx])
    reset_start = False

    for cnt in range(1000):
        done_cnt = 0
        COLLISION = False
        IKFAIL = False
        SINGULARITY = False

        s = env.reset(reset_start)
        reset_start = True
        goal = env.get_goal
        goal = np.append(goal, 0)
        start = (s[:8])
        for __ in range(1000):
            a = agent.choose_action(s)
            s, done, collision, ik_success, singularity = env.step(a)
            done_cnt += int(done)
            if COLLISION and collision:
                COLLISION_ARRAY[cnt%1000] = 1
            elif collision:
                COLLISION = True
            else:
                COLLISION = False
            if IKFAIL and not ik_success:
                IKFAIL_ARRAY[cnt%1000] = 1
            elif not ik_success:
                IKFAIL = True
            else:
                IKFAIL = False
            if __ > 5:
                if SINGULARITY:
                    SINGULARITY_ARRAY[cnt%1000] = 1
                elif singularity:
                    SINGULARITY = True
            if done_cnt > 20:    
                SUCCESS_ARRAY[cnt%1000] = 1
                reset_start = False
                # COLLISION_ARRAY[cnt%1000] = 0
                # IKFAIL_ARRAY[cnt%1000] = 0
                break
            if __ == 999:
                reset_start = True
        arm.clear_cmd()
        # COLLISION = False
        # IKFAIL = False
        # SINGULARITY = False
        # time.sleep(0.5)
        # env.move_arm(start)
        # time.sleep(1)
        # arm.ikMove_quat('p2p', goal[:3], goal[3:7], goal[7])
        # while arm.is_busy:
        #     if env.check_collision():
        #         COLLISION = True
        #         COLLISION_ARRAY_P2P[cnt%1000] = 1
        #     if arm.is_ikfail:
        #         IKFAIL = True
        #         IKFAIL_ARRAY_P2P[cnt%1000] = 1
        #         arm.clear_cmd()
        #         break
        #     if arm.singularity:
        #         SINGULARITY = True
        #         SINGULARITY_ARRAY_P2P[cnt%1000] = 1
        # if not COLLISION and not IKFAIL:
        #     SUCCESS_ARRAY_P2P[cnt%1000] = 1
        # COLLISION = False
        # IKFAIL = False
        # SINGULARITY = False
        # time.sleep(0.5)
        # env.move_arm(start)
        # time.sleep(1)
        # arm.ikMove_quat('line', goal[:3], goal[3:7], goal[7])
        # while arm.is_busy:
        #     if env.check_collision():
        #         COLLISION = True
        #         COLLISION_ARRAY_LINE[cnt%1000] = 1
        #     if arm.is_ikfail:
        #         IKFAIL = True
        #         IKFAIL_ARRAY_LINE[cnt%1000] = 1
        #         arm.clear_cmd()
        #         break
        #     if arm.singularity:
        #         SINGULARITY = True
        #         SINGULARITY_ARRAY_LINE[cnt%1000] = 1
        # if not COLLISION and not IKFAIL:
        #     SUCCESS_ARRAY_LINE[cnt%1000] = 1

        S_RATE = 0
        S_RATE_P2P = 0
        S_RATE_LINE = 0
        C_RATE = 0
        C_RATE_P2P = 0
        C_RATE_LINE = 0
        I_RATE = 0
        I_RATE_P2P = 0
        I_RATE_LINE = 0
        SI_RATE = 0
        SI_RATE_P2P = 0
        SI_RATE_LINE = 0
        for z in SUCCESS_ARRAY:
            S_RATE += z
        for z in SUCCESS_ARRAY_P2P:
            S_RATE_P2P += z
        for z in SUCCESS_ARRAY_LINE:
            S_RATE_LINE += z
        for z in COLLISION_ARRAY:
            C_RATE += z
        for z in COLLISION_ARRAY_P2P:
            C_RATE_P2P += z
        for z in COLLISION_ARRAY_LINE:
            C_RATE_LINE += z
        for z in IKFAIL_ARRAY:
            I_RATE += z
        for z in IKFAIL_ARRAY_P2P:
            I_RATE_P2P += z
        for z in IKFAIL_ARRAY_LINE:
            I_RATE_LINE += z
        for z in SINGULARITY_ARRAY:
            SI_RATE += z
        for z in SINGULARITY_ARRAY_P2P:
            SI_RATE_P2P += z
        for z in SINGULARITY_ARRAY_LINE:
            SI_RATE_LINE += z
        print('Ep:', cnt, 's_rate:', S_RATE, S_RATE_P2P, S_RATE_LINE, '    c_rate:', C_RATE, C_RATE_P2P, C_RATE_LINE, \
                      '    i_rate', I_RATE, I_RATE_P2P, I_RATE_LINE, '    si_rate:', SI_RATE, SI_RATE_P2P, SI_RATE_LINE)

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
    rospy.init_node('aL')
    threads = []
    cmd = np.zeros([2,7])
    move = [False, False]
    COORD = tf.train.Coordinator()
    
    for i in range(2):
        t = threading.Thread(target=run, args=(i,))
        threads.append(t)
    COORD.join(threads)
    for i in range(2):
        threads[i].start()
        time.sleep(10)
    
    rospy.Subscriber('right_arm/drl_pose_msg', P2PPose, right_callback)
    rospy.Subscriber('left_arm/drl_pose_msg', P2PPose, left_callback)
    rospy.spin()