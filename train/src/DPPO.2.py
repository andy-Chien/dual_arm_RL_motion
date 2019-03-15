#!/usr/bin/env python3

"""
A simple version of OpenAI's Proximal Policy Optimization (PPO). [http://adsabs.harvard.edu/abs/2017arXiv170706347S]

Distributing workers in parallel to collect data, then stop worker's roll-out and train PPO on collected data.
Restart workers once PPO is updated.

The global PPO updating rule is adopted from DeepMind's paper (DPPO):
Emergence of Locomotion Behaviours in Rich Environments (Google Deepmind): [http://adsabs.harvard.edu/abs/2017arXiv170702286H]

View more on my tutorial website: https://morvanzhou.github.io/tutorials

Dependencies:
tensorflow r1.2
gym 0.9.2
"""

import tensorflow as tf
from tensorflow.contrib.distributions import Normal
import numpy as np
import matplotlib.pyplot as plt
import threading, queue
import os
import shutil
import time
# from arm_env import ArmEnv
import gym
from env import Test 
env = Test(0)
# env = gym.make('Acrobot-v2')
print(env.__str__())

EP_MAX = 500000
EP_LEN = 3000
N_WORKER = 1                # parallel workers
GAMMA = 0.9                 # reward discount factor
A_LR = 0.00005               # learning rate for actor
C_LR = 0.0002                # learning rate for critic
MIN_BATCH_SIZE = 64         # minimum batch size for updating PPO
UPDATE_STEP = 5             # loop update operation n-steps
EPSILON = 0.2               # Clipped surrogate objective
# MODE = ['easy', 'hard']
LOAD = False
NAME = 'DPPO_test_5'
GOAL_REWARD = 50
# n_model = 1

# env = ArmEnv(mode=MODE[n_model])
S_DIM = env.observation_space.shape[0]
A_DIM = 8#env.action_dim
# A_BOUND = env.action_bound[1]


class PPO(object):
    def __init__(self):
        self.sess = tf.Session()

        self.tfs = tf.placeholder(tf.float32, [None, S_DIM], 'state')

        # critic
        l1 = tf.layers.dense(self.tfs, 300, tf.nn.relu)
        l1 = tf.layers.dense(l1, 300, tf.nn.relu)
        # l1 = tf.layers.dense(l1, 300, tf.nn.relu)
        # l1 = tf.layers.dense(l1, 300, tf.nn.relu)
        # l1 = tf.layers.dense(l1, 300, tf.nn.relu)
        self.v = tf.layers.dense(l1, 1)
        self.tfdc_r = tf.placeholder(tf.float32, [None, 1], 'discounted_r')
        self.advantage = self.tfdc_r - self.v
        self.closs = tf.reduce_mean(tf.square(self.advantage))
        self.ctrain_op = tf.train.AdamOptimizer(C_LR).minimize(self.closs)

        # actor
        pi, pi_params = self._build_anet('pi', trainable=True)
        oldpi, oldpi_params = self._build_anet('oldpi', trainable=False)
        self.sample_op = tf.squeeze(pi.sample(1), axis=0)  # choosing action
        self.update_oldpi_op = [oldp.assign(p) for p, oldp in zip(pi_params, oldpi_params)]

        self.tfa = tf.placeholder(tf.float32, [None, A_DIM], 'action')
        self.tfadv = tf.placeholder(tf.float32, [None, 1], 'advantage')
        # ratio = tf.exp(pi.log_prob(self.tfa) - oldpi.log_prob(self.tfa))
        ratio = pi.prob(self.tfa) / (oldpi.prob(self.tfa) + 1e-5)
        surr = ratio * self.tfadv   # surrogate loss

        self.aloss = -tf.reduce_mean(tf.minimum(
            surr,
            tf.clip_by_value(ratio, 1. - EPSILON, 1. + EPSILON) * self.tfadv))

        self.atrain_op = tf.train.AdamOptimizer(A_LR).minimize(self.aloss)
        # self.sess.run(tf.global_variables_initializer())
        self.saver = tf.train.Saver()
        self.path = './'+NAME
        if LOAD:
            self.saver.restore(self.sess, tf.train.latest_checkpoint(self.path))
        else:
            self.sess.run(tf.global_variables_initializer())

    def update(self):
        global GLOBAL_UPDATE_COUNTER
        while not COORD.should_stop():
            if GLOBAL_EP < EP_MAX:
                UPDATE_EVENT.wait()         # wait until get batch of data
                self.sess.run(self.update_oldpi_op)   # old pi to pi
                data = [QUEUE.get() for _ in range(QUEUE.qsize())]
                data = np.vstack(data)
                s, a, r = data[:, :S_DIM], data[:, S_DIM: S_DIM + A_DIM], data[:, -1:]
                adv = self.sess.run(self.advantage, {self.tfs: s, self.tfdc_r: r})
                [self.sess.run(self.atrain_op, {self.tfs: s, self.tfa: a, self.tfadv: adv}) for _ in range(UPDATE_STEP)]
                [self.sess.run(self.ctrain_op, {self.tfs: s, self.tfdc_r: r}) for _ in range(UPDATE_STEP)]
                UPDATE_EVENT.clear()        # updating finished
                GLOBAL_UPDATE_COUNTER = 0   # reset counter
                ROLLING_EVENT.set()         # set roll-out available

    def _build_anet(self, name, trainable):
        with tf.variable_scope(name):
            l1 = tf.layers.dense(self.tfs, 300, tf.nn.relu, trainable=trainable)
            l1 = tf.layers.dense(l1, 300, tf.nn.relu, trainable=trainable)
            # l1 = tf.layers.dense(l1, 300, tf.nn.relu6, trainable=trainable)
            # l1 = tf.layers.dense(l1, 300, tf.nn.relu6, trainable=trainable)
            # l1 = tf.layers.dense(l1, 300, tf.nn.relu6, trainable=trainable)
            mu = tf.layers.dense(l1, A_DIM, tf.nn.tanh, trainable=trainable)
            sigma = tf.layers.dense(l1, A_DIM, tf.nn.softplus, trainable=trainable)
            # sigma = [0.01,0.01,0.01]
            norm_dist = Normal(loc=mu, scale=sigma)
        params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=name)
        return norm_dist, params

    def choose_action(self, s):
        s = np.array(s)
        s = s[np.newaxis, :]
        a = self.sess.run(self.sample_op, {self.tfs: s})[0]
        return np.clip(a, -1, 1)

    def get_v(self, s):
        if s.ndim < 2: s = s[np.newaxis, :]
        return self.sess.run(self.v, {self.tfs: s})[0, 0]


class Worker(object):
    def __init__(self, wid):
        self.wid = wid
        # self.env = ArmEnv(mode=MODE[n_model])
        # self.env = gym.make('Acrobot-v2')
        self.env = Test(0)
        self.ppo = GLOBAL_PPO

    def work(self):
        global GLOBAL_EP, GLOBAL_RUNNING_R, GLOBAL_UPDATE_COUNTER, T_REWARD, MU_REWARD, BEST_R
        try:
            while not COORD.should_stop():
                s = self.env.reset()
                ep_r = 0
                cnt = 0
                buffer_s, buffer_a, buffer_r = [], [], []
                for t in range(EP_LEN):
                    # if self.wid == 0:
                    #     self.env.render()
                    if not ROLLING_EVENT.is_set():                  # while global PPO is updating
                        ROLLING_EVENT.wait()                        # wait until PPO is updated
                        buffer_s, buffer_a, buffer_r = [], [], []   # clear history buffer
                    a = self.ppo.choose_action(s)
                    s_, r, done, _ = self.env.step(a)
                    buffer_s.append(s)
                    buffer_a.append(a)
                    buffer_r.append(r)                    # normalize reward, find to be useful
                    s = s_
                    ep_r += r
                    cnt += _

                    GLOBAL_UPDATE_COUNTER += 1                      # count to minimum batch size
                    if t == EP_LEN - 1 or GLOBAL_UPDATE_COUNTER >= MIN_BATCH_SIZE or done:
                        # if done and ep_r<0:
                        #     ep_r = 0

                        v_s_ = self.ppo.get_v(s_)
                        discounted_r = []                           # compute discounted reward
                        for r in buffer_r[::-1]:
                            v_s_ = r + GAMMA * v_s_
                            discounted_r.append(v_s_)
                        discounted_r.reverse()

                        bs, ba, br = np.vstack(buffer_s), np.vstack(buffer_a), np.array(discounted_r)[:, np.newaxis]
                        buffer_s, buffer_a, buffer_r = [], [], []
                        QUEUE.put(np.hstack((bs, ba, br)))
                        if GLOBAL_UPDATE_COUNTER >= MIN_BATCH_SIZE:
                            ROLLING_EVENT.clear()       # stop collecting data
                            UPDATE_EVENT.set()          # globalPPO updat
                        if GLOBAL_EP >= EP_MAX or MU_REWARD > GOAL_REWARD:         # stop training
                            COORD.request_stop()
                            break
                        
                        if t == EP_LEN - 1 or done:
                            if len(T_REWARD) > 100:
                                T_REWARD.pop(0)
                            T_REWARD.append(ep_r)
                            r_sum = 0
                            for i in T_REWARD:
                                r_sum += i
                            MU_REWARD = r_sum/len(T_REWARD)
                            BEST_R = MU_REWARD if MU_REWARD>BEST_R else BEST_R
                            break

                # record reward changes, plot later
                if len(GLOBAL_RUNNING_R) == 0: GLOBAL_RUNNING_R.append(ep_r)
                else: GLOBAL_RUNNING_R.append(GLOBAL_RUNNING_R[-1]*0.9+ep_r*0.1)
                GLOBAL_EP += 1
                print('{0:.1f}%'.format(GLOBAL_EP/EP_MAX*100), '|W%i' % self.wid,  '|Ep_r: %.2f' % cnt, 'reward:', ep_r, 'm_reward: ', MU_REWARD, 'BEST_R: ', BEST_R)
        except KeyboardInterrupt:
            return

def train():
    tStart = time.time()
    workers = [Worker(wid=i) for i in range(N_WORKER)]
    threads = []
    
    for worker in workers:  # worker threads
        t = threading.Thread(target=worker.work, args=())
        t.start()
        threads.append(t)
    # add a PPO updating thread
    threads.append(threading.Thread(target=GLOBAL_PPO.update,))
    threads[-1].start()
    COORD.join(threads)

    # plot reward change and testing
    plt.plot(np.arange(len(GLOBAL_RUNNING_R)), GLOBAL_RUNNING_R)
    plt.xlabel('Episode'); plt.ylabel('Moving reward'); plt.ion(); plt.show()
    # env.set_fps(30)
    if os.path.isdir(GLOBAL_PPO.path): shutil.rmtree(GLOBAL_PPO.path)
    os.mkdir(GLOBAL_PPO.path)
    ckpt_path = os.path.join('./'+NAME, 'DPPO.ckpt')
    save_path = GLOBAL_PPO.saver.save(GLOBAL_PPO.sess, ckpt_path, write_meta_graph=False)
    tEnd = time.time()
    _time = tEnd - tStart
    print("\nSave Model %s\n" % save_path, 'time:', _time )

def eval():
    s = env.reset()
    cnt = 0
    fail = 0
    time_mul = 0
    _time = 0
    while cnt<200:
        cnt+=1
        s = env.reset()
        for t in range(100):
            env.render()
            tStart = time.time()
            s, r, done, _  = env.step(GLOBAL_PPO.choose_action(s))
            tEnd = time.time()
            _time = tEnd - tStart
            time_mul = (time_mul+_time)/2
            if done:
                break
            if t == 99:
                fail += 1
        print('success: ',(200-fail)/200, '|| time: ', time_mul)

if __name__ == '__main__':
    GLOBAL_PPO = PPO()
    UPDATE_EVENT, ROLLING_EVENT = threading.Event(), threading.Event()
    UPDATE_EVENT.clear()    # no update now
    ROLLING_EVENT.set()     # start to roll out
    GLOBAL_UPDATE_COUNTER, GLOBAL_EP = 0, 0
    GLOBAL_RUNNING_R = [] 
    COORD = tf.train.Coordinator()
    QUEUE = queue.Queue()
    MU_REWARD = 0.
    BEST_R = 0.
    T_REWARD = []
    if LOAD:
        eval()
    else:
        train()