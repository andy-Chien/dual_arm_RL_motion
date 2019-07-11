#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
from execute_sac import SAC
from execute_env import Test
from arm_control.arm_task import ArmTask
from manipulator_h_base_module_msgs.msg import P2PPose

SIDE = ['right_', 'left_']

def run(nameIndx):
    global cmd, move
    
    env = Test(nameIndx, 0) #0 = right
    arm = ArmTask(SIDE[nameIndx]+'arm')

    agent = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim, name=SIDE[nameIndx])
    
    # t = threading.currentThread()
    # while getattr(t, "do_run", True):
    while not rospy.is_shutdown():
        MOVE_EVENT[nameIndx].wait()
        time.sleep(0.1)
        if rospy.is_shutdown(): break
        s, ik_success, collision, arm_enable = env.reset(cmd[nameIndx])
        if not arm_enable:
            MOVE_EVENT[nameIndx].clear()
            print("ffffffffffffffffffffffffffffffffffffffffffffffffff")
            continue

        done_cnt = 0
        for __ in range(2000):
            if done_cnt > 5 or collision or not ik_success:
                arm.clear_cmd()
                if not ik_success:
                    print("!!!!!!!!!!!!!!!!=IK FAIL=!!!!!!!!!!!!!!!!")
                elif collision:
                    print("!!!!!!!!!!!!!!!=COLLISION=!!!!!!!!!!!!!!!")
                else:
                    print("!!!!!!!!!!!!!!!=FUCK DONE=!!!!!!!!!!!!!!!")
                break

            a = agent.choose_action(s)
            if arm.is_stop: break
            s, done, collision, ik_success, _ = env.step(a)
            done_cnt += int(done)
            if __ == 1999:
                print('no good done')
        move[nameIndx] = False
        MOVE_EVENT[nameIndx].clear()

def right_callback(msg):
    global cmd, move
    cmd[0][0] = msg.pose.position.x
    cmd[0][1] = msg.pose.position.y
    cmd[0][2] = msg.pose.position.z
    cmd[0][3] = msg.pose.orientation.w
    cmd[0][4] = msg.pose.orientation.x
    cmd[0][5] = msg.pose.orientation.y
    cmd[0][6] = msg.pose.orientation.z
    cmd[0][7] = msg.phi
    cmd[0][8] = msg.speed
    move[0] = True
    MOVE_EVENT[0].set()
    print("got it")

def left_callback(msg):
    global cmd, move
    cmd[1][0] = msg.pose.position.x
    cmd[1][1] = msg.pose.position.y
    cmd[1][2] = msg.pose.position.z
    cmd[1][3] = msg.pose.orientation.w
    cmd[1][4] = msg.pose.orientation.x
    cmd[1][5] = msg.pose.orientation.y
    cmd[1][6] = msg.pose.orientation.z
    cmd[1][7] = msg.phi
    cmd[1][8] = msg.speed
    move[1] = True
    MOVE_EVENT[1].set()
    print("got it")

def stop_thread():
    MOVE_EVENT[0].set()
    MOVE_EVENT[1].set()

if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    cmd = np.zeros([2,9])
    move = [False, False]
    MOVE_EVENT = [threading.Event(), threading.Event()]
       
    for i in range(2):
        t = threading.Thread(target=run, args=(i,))
        threads.append(t)
        MOVE_EVENT[i].clear()
    for i in range(2):
        threads[i].start()
        time.sleep(3)
    # time.sleep(5)
    # for i in range(2):
    #     threads[i].join()
      
    # rospy.on_shutdown(threads[0].do_run = False)
    rospy.on_shutdown(stop_thread)
    # rospy.on_shutdown(threads[1].stop())
    rospy.Subscriber('right_arm/drl_pose_msg', P2PPose, right_callback)
    rospy.Subscriber('left_arm/drl_pose_msg', P2PPose, left_callback)
    rospy.spin()