#!/usr/bin/env python3

import gym
import numpy as np
import tensorflow as tf
import random

NOISE_DECAY = 0.99999
NOISE_MIN = 0.001
NAME = 'DDPG_v3'#dont care phi
LOAD = False
BATCH_SIZE = 512
MEMORY_CAPACITY = 4096

class ReplayBuffer(object):
    def __init__(self, capacity):
        self.buffer = []
        self.capacity = capacity
        self.index = 0

    def store_transition(self, obs0, act, rwd, obs1, done):
        data = (obs0, act, rwd, obs1, done)
        if self.index >= len(self.buffer):
            self.buffer.append(data)
        else:
            self.buffer[self.index] = data
        self.index = (self.index + 1) % self.capacity

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        obs0, act, rwd, obs1, done = map(np.stack, zip(*batch))
        return obs0, act, rwd[:, np.newaxis], obs1, done[:, np.newaxis]


class ActorNetwork(object):
    def __init__(self, act_dim, name):
        self.act_dim = act_dim
        self.name = name

    def step(self, obs, reuse):
        with tf.variable_scope(self.name, reuse=reuse):
            h1 = tf.layers.dense(obs, 256, tf.nn.leaky_relu,
                                 kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            h2 = tf.layers.dense(h1, 256, tf.nn.leaky_relu,
                                 kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            # h3 = tf.layers.dense(h1, 256, tf.nn.leaky_relu,
            #                      kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            # h3 = tf.layers.dense(h2, 1024, tf.nn.leaky_relu,
            #                      kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            action = tf.layers.dense(h2, self.act_dim, tf.nn.tanh,
                                 kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
        return action

    def choose_action(self, obs, reuse=False):
        action = self.step(obs, reuse)
        return action


class QValueNetwork(object):
    def __init__(self, name):
        self.name = name

    def step(self, obs, action, reuse):
        with tf.variable_scope(self.name, reuse=reuse):
            h1 = tf.layers.dense(obs, 256, tf.nn.leaky_relu,
                                 kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            h1 = tf.concat([h1, action], axis=-1)
            h2 = tf.layers.dense(h1, 256, tf.nn.leaky_relu,
                                 kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            h3 = tf.layers.dense(h2, 256, tf.nn.leaky_relu,
                                 kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            # h3 = tf.layers.dense(h2, 1024, tf.nn.leaky_relu,
            #                      kernel_initializer=tf.orthogonal_initializer(gain=np.sqrt(2)))
            value = tf.layers.dense(h3, 1,
                                    kernel_initializer=tf.orthogonal_initializer(gain=0.01))
            return value

    def get_q_value(self, obs, action, reuse=False):
        q_value = self.step(obs, action, reuse)
        return q_value


class DDPG(object):
    def __init__(self, act_dim, obs_dim, lr_actor, lr_q_value, gamma,
                 tau, action_noise_std, name):
        self.act_dim = act_dim
        self.obs_dim = obs_dim
        self.lr_actor = lr_actor
        self.lr_q_value = lr_q_value
        self.gamma = gamma
        self.tau = tau
        self.action_noise_std = action_noise_std
        self.name = name

        self.OBS0 = tf.placeholder(tf.float32, [None, self.obs_dim], name=self.name+"obs0")
        self.OBS1 = tf.placeholder(tf.float32, [None, self.obs_dim], name=self.name+"obs1")
        self.ACT = tf.placeholder(tf.float32, [None, self.act_dim], name=self.name+"action")
        self.RWD = tf.placeholder(tf.float32, [None, 1], name=self.name+"reward")
        self.TARGET_Q = tf.placeholder(tf.float32, [None, 1], name=self.name+"target_q_value")
        self.DONE = tf.placeholder(tf.float32, [None, 1], name=self.name+"done")
        self.q_value_loss = tf.placeholder(tf.float32, [None, 1], name=self.name+"_q_loss")
        self.actor_loss = tf.placeholder(tf.float32, [None, 1], name=self.name+"_a_loss")

        actor = ActorNetwork(self.act_dim, self.name+'actor')
        q_value = QValueNetwork(self.name+'q_value')

        target_actor = ActorNetwork(self.act_dim, self.name+'target_actor')
        target_q_value = QValueNetwork(self.name+'target_q_value')

        self.memory = ReplayBuffer(capacity = MEMORY_CAPACITY)

        self.action = actor.choose_action(self.OBS0)
        self.q_value0_with_actor = q_value.get_q_value(self.OBS0, self.action)

        q_value0 = q_value.get_q_value(self.OBS0, self.ACT, reuse=True)
        target_action1 = target_actor.choose_action(self.OBS1)
        self.target_q_value1 = self.RWD + (1. - self.DONE) * self.gamma \
                               * target_q_value.get_q_value(self.OBS1, target_action1)

        self.q_value_loss = tf.reduce_mean(tf.square(q_value0 - self.TARGET_Q))
        self.q_value_train_op = tf.train.AdamOptimizer(learning_rate=self.lr_q_value).minimize(self.q_value_loss)

        self.actor_loss = -tf.reduce_mean(self.q_value0_with_actor)
        self.actor_train_op = tf.train.AdamOptimizer(learning_rate=self.lr_actor).minimize(self.actor_loss)

        self.actor_params = tf.global_variables(self.name+'actor')
        self.target_actor_params = tf.global_variables(self.name+'target_actor')
        self.q_value_params = tf.global_variables(self.name+'q_value')
        self.target_q_value_params = tf.global_variables(self.name+'target_q_value')

        self.target_init_updates = \
            [[tf.assign(ta, a), tf.assign(tq, q)]
             for ta, a, tq, q in zip(self.target_actor_params, self.actor_params,
                                     self.target_q_value_params, self.q_value_params)]

        self.target_soft_updates = \
            [[tf.assign(ta, (1 - tau) * ta + tau * a), tf.assign(tq, (1 - tau) * tq + tau * q)]
             for ta, a, tq, q in zip(self.target_actor_params, self.actor_params,
                                     self.target_q_value_params, self.q_value_params)]

        self.sess = tf.Session()
        
        tf.summary.scalar(self.name+'_q_loss', self.q_value_loss)
        tf.summary.scalar(self.name+'_a_loss', self.actor_loss)
        self.merged = tf.summary.merge_all()
        self.writer = tf.summary.FileWriter('/home/andy/collision_ws/src/Collision_Avoidance/train/logs/'+NAME+'/'+self.name+'/', self.sess.graph)
        self.saver = tf.train.Saver()
        self.path = './'+ NAME + self.name
        if LOAD:
            self.saver.restore(self.sess, tf.train.latest_checkpoint(self.path))
        else:
            self.sess.run(tf.global_variables_initializer())
            self.sess.run(self.target_init_updates)

    def choose_action(self, obs):
        if obs.ndim < 2: obs = obs[np.newaxis, :]
        action = self.sess.run(self.action, feed_dict={self.OBS0: obs})
        action = action + np.random.normal(0, self.action_noise_std)
        action = np.clip(action, -2, 2).squeeze()
        return action

    def learn(self, indx):
        obs0, act, rwd, obs1, done = self.memory.sample(batch_size=BATCH_SIZE)

        target_q_value1 = self.sess.run(self.target_q_value1, feed_dict={self.OBS1: obs1, self.RWD: rwd,
                                                               self.DONE: np.float32(done)})

        self.sess.run(self.q_value_train_op,feed_dict={self.OBS0: obs0, self.ACT: act,
                                 self.TARGET_Q: target_q_value1})

        self.sess.run(self.actor_train_op, feed_dict={self.OBS0: obs0})

        self.sess.run(self.target_soft_updates)

        self.action_noise_std = max([self.action_noise_std * NOISE_DECAY, NOISE_MIN])

        if indx != 0:
            result = self.sess.run(self.merged, feed_dict={self.OBS0: obs0, self.ACT: act,
                                 self.TARGET_Q: target_q_value1})
            self.writer.add_summary(result, indx)                     


# env = gym.make('InvertedPendulum-v2')
# env.seed(1)
# env = env.unwrapped

# agent = DDPG(act_dim=8, obs_dim=63,
#                     lr_actor=0.0001, lr_q_value=0.001, gamma=0.99, tau=0.01, action_noise_std=1)

#  nepisode = 1000
# nstep = 200
# iteration = 0

# noise_decay = 0.9999
# noise_min = 0.001


# for i_episode in range(nepisode):
#     obs0 = env.reset()
#     ep_rwd = 0

#     for t in range(nstep):
#         act = agent.step(obs0)

#         obs1, rwd, done, _ = env.step(act)

#         agent.memory.store_transition(obs0, act, rwd/10, obs1, done)

#         obs0 = obs1
#         ep_rwd += rwd

#         if iteration >= 128 * 3:
#             agent.learn()

#         iteration += 1

#     print('Ep: %i' % i_episode, "|Ep_r: %i" % ep_rwd)
