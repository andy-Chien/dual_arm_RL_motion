#!/usr/bin/env python3

"""
Note: This is a updated version from my previous code,
for the target network, I use moving average to soft replace target parameters instead using assign function.
By doing this, it has 20% speed up on my machine (CPU).

Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
Pendulum example.

View more on my tutorial page: https://morvanzhou.github.io/tutorials/

Using:
tensorflow 1.0
gym 0.8.0
"""

import tensorflow as tf
import numpy as np
import gym
import time
from env import Test 
import threading, queue
import rospy
import os
import shutil
import math


#####################  hyper parameters  ####################

MAX_EPISODES = 100000
MAX_EP_STEPS = 2000
LR_A = 0.0001    # learning rate for actor
LR_C = 0.0005    # learning rate for critic
GAMMA = 0.95     # reward discount
TAU = 0.001      # soft replacement
MEMORY_CAPACITY = 10000
BATCH_SIZE = 128
SIDE = ['right_arm', 'left_arm']
GOAL_REWARD = 1500
NAME = 'DDPG_v1'
LOAD = False


###############################  DDPG  ####################################


class DDPG(object):
    def __init__(self, a_dim, s_dim, a_bound, names):
        self.name = names
        self.memory = np.zeros((MEMORY_CAPACITY, s_dim * 2 + a_dim + 1), dtype=np.float32)
        self.pointer = 0
        self.sess = tf.Session()
        self.r_flag = False

        self.a_dim, self.s_dim, self.a_bound = a_dim, s_dim, a_bound,
        self.S = tf.placeholder(tf.float32, [None, s_dim], 's')
        self.S_ = tf.placeholder(tf.float32, [None, s_dim], 's_')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')

        self.a = self._build_a(self.S,)
        q = self._build_c(self.S, self.a, )
        a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Actor'+self.name)
        c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Critic'+self.name)
        ema = tf.train.ExponentialMovingAverage(decay=1 - TAU)          # soft replacement

        def ema_getter(getter, name, *args, **kwargs):
            return ema.average(getter(name, *args, **kwargs))

        target_update = [ema.apply(a_params), ema.apply(c_params)]      # soft update operation
        a_ = self._build_a(self.S_, reuse=True, custom_getter=ema_getter)   # replaced target parameters
        q_ = self._build_c(self.S_, a_, reuse=True, custom_getter=ema_getter)

        a_loss = - tf.reduce_mean(q)  # maximize the q
        self.atrain = tf.train.AdamOptimizer(LR_A).minimize(a_loss, var_list=a_params)

        with tf.control_dependencies(target_update):    # soft replacement happened at here
            q_target = self.R + GAMMA * q_
            td_error = tf.losses.mean_squared_error(labels=q_target, predictions=q)
            tf.Print(td_error, [td_error])
            self.ctrain = tf.train.AdamOptimizer(LR_C).minimize(td_error, var_list=c_params)

        self.sess.run(tf.global_variables_initializer())

        self.saver = tf.train.Saver()
        self.path = './'+ NAME + self.name
        if LOAD:
            self.saver.restore(self.sess, tf.train.latest_checkpoint(self.path))
        else:
            self.sess.run(tf.global_variables_initializer())

    def choose_action(self, s):
        return self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]

    def learn(self):
        indices = np.random.choice(MEMORY_CAPACITY, size=BATCH_SIZE)
        bt = self.memory[indices, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]
        bs_ = bt[:, -self.s_dim:]

        self.sess.run(self.atrain, {self.S: bs})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % MEMORY_CAPACITY  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.pointer += 1

    def _build_a(self, s, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Actor'+self.name, reuse=reuse, custom_getter=custom_getter):
            net = tf.layers.dense(s, 3000, activation=tf.nn.relu, name='l1', trainable=trainable)
            net = tf.layers.dense(net, 3000, activation=tf.nn.relu, name='l2', trainable=trainable)
            # net = tf.layers.dense(net, 1000, activation=tf.nn.relu, name='l3', trainable=trainable)
            # t1 = tf.layers.dense(net, 300, activation=tf.nn.tanh, name='t1', trainable=trainable)
            # t2 = tf.layers.dense(net, 400, activation=tf.nn.tanh, name='t2', trainable=trainable)
            # t3 = tf.layers.dense(net, 100, activation=tf.nn.tanh, name='t3', trainable=trainable)
            # a1 = tf.layers.dense(t1, 3, activation=tf.nn.tanh, name='a1', trainable=trainable)
            # a2 = tf.layers.dense(t2, 4, activation=tf.nn.tanh, name='a2', trainable=trainable)
            # a3 = tf.layers.dense(t3, 1, activation=tf.nn.tanh, name='a3', trainable=trainable)
            a = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            # a = [a1[0],a2[0],a3[0]]
            # a = tf.reshape(a,[-1])
            # a = tf.concat([a1[0],a2[0]], 0)
            # a = tf.concat([a[0], a3[0]], 0)
            return a#tf.multiply(a, self.a_bound, name='scaled_a')

    def _build_c(self, s, a, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Critic'+self.name, reuse=reuse, custom_getter=custom_getter):
            n_l1 = 3000
            w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            net = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            net = tf.layers.dense(net, 3000, activation=tf.nn.relu, name='l2', trainable=trainable)
            # net = tf.layers.dense(net, 1000, activation=tf.nn.relu, name='l3', trainable=trainable)
            return tf.layers.dense(net, 1, trainable=trainable)  # Q(s,a)


###############################  training  ####################################
def train(nameIndx):
    T_REWARD = []
    MU_REWARD = 0
    BEST_R = 0
    env = Test(nameIndx) #0 = right
    s_dim = env.observation_space.shape[0]
    a_dim = 8#env.action_space.shape[0]
    a_bound = 1#env.action_space.high

    ddpg = DDPG(a_dim, s_dim, a_bound, SIDE[nameIndx])

    var = 0.8  # control exploration
    rar = 0.2
    
    t1 = time.time()
    for i in range(MAX_EPISODES):
        s = env.reset() 
        ep_reward = 0
        cnt = 0
        for j in range(MAX_EP_STEPS):
            # Add exploration noise
            a = ddpg.choose_action(s)
            a = np.clip(np.random.normal(a, var), -1, 1)    # add randomness to action selection for exploration
            if (np.random.rand(1) < 50/(i+1) or np.random.rand(1) < rar) and i > 50:
                a = action_sample(s)
                rar *= .9999999
            s_, r, done, info = env.step(a)
            if r < -5000:
                r /= math.sqrt(j+1)
            if r > 0 or ddpg.r_flag:
                ddpg.r_flag = not ddpg.r_flag
                ddpg.store_transition(s, a, r, s_)
            cnt += info
            if ddpg.pointer > MEMORY_CAPACITY:
                var *= .9999999    # decay the action randomness
                ddpg.learn()

            s = s_
            ep_reward += r
            if j == MAX_EP_STEPS-1 or done or ep_reward < -10000:
                # print('state = ', s)
                if len(T_REWARD) >= 100:
                    T_REWARD.pop(0)
                T_REWARD.append(ep_reward)
                r_sum = 0
                for k in T_REWARD:
                    r_sum += k
                MU_REWARD = r_sum/100
                BEST_R = MU_REWARD if MU_REWARD>BEST_R else BEST_R
                print('Episode:', i, ' Reward: %i' % int(ep_reward), 'MU_REWARD: ', int(MU_REWARD),'BEST_R: ', int(BEST_R), 'cnt = ',cnt, 'var: %.3f' % var, 'rar: %.3f' % rar)
                break
        if MU_REWARD > GOAL_REWARD:
            break
    if os.path.isdir(ddpg.path): shutil.rmtree(ddpg.path)
    os.mkdir(ddpg.path)
    ckpt_path = os.path.join('./'+NAME+ddpg.name, 'DDPG.ckpt')
    save_path = ddpg.saver.save(ddpg.sess, ckpt_path, write_meta_graph=False)
    print("\nSave Model %s\n" % save_path)
    print('Running time: ', time.time() - t1)

def action_sample(s):
    a = s[:8] - s[8:16]
    a[:3] /= np.linalg.norm(a[:3])
    a[3:7] /= np.linalg.norm(a[3:7])
    a[7] /= math.fabs(a[7])
    return a

if __name__ == '__main__':
    rospy.init_node('a')
    threads = []
    
    for i in range(2):
        t = threading.Thread(target=train, args=(i,))
        threads.append(t)
    for i in range(2):
        threads[i].start()
    # add a PPO updating thread
    # threads.append(threading.Thread(target=GLOBAL_PPO.update,))
    # threads[-1].start()
    # COORD.join(threads)