import numpy as np
import tensorflow as tf
import gym
import random

NAME = 'SAC_v14_21'
EPS = 1e-8

class ActorNetwork(object):
    def __init__(self, act_dim, name):
        self.act_dim = act_dim
        self.name = name

    def step(self, obs, log_std_min=-5, log_std_max=0.5):
        with tf.variable_scope(self.name, reuse=tf.AUTO_REUSE):
            h1 = tf.layers.dense(obs, 512, tf.nn.leaky_relu, name='h1')
            h2 = tf.layers.dense(h1, 512, tf.nn.leaky_relu, name='h2')
            h3 = tf.layers.dense(h2, 512, tf.nn.leaky_relu, name='h3')
            h4 = tf.layers.dense(h3, 512, tf.nn.leaky_relu, name='h4')
            h5 = tf.layers.dense(h4, 512, tf.nn.leaky_relu, name='h5')
            mu = tf.layers.dense(h5, self.act_dim, None, name='mu')
            log_std = tf.layers.dense(h5, self.act_dim, tf.tanh, name='log_std')
            log_std = log_std_min + 0.5 * (log_std_max - log_std_min) * (log_std + 1)

            std = tf.exp(log_std)
            pi = mu + tf.random_normal(tf.shape(mu)) * std

            mu = tf.tanh(mu)
            pi = tf.tanh(pi)

            # mu = tf.tanh(mu)
        return mu, pi

    def evaluate(self, obs):
        mu, pi = self.step(obs)
        return mu


class SAC(object):
    def __init__(self, act_dim, obs_dim, name=None):
        # tf.reset_default_graph()

        self.act_dim = act_dim
        self.obs_dim = obs_dim
        self.name = name

        self.OBS0 = tf.placeholder(tf.float32, [None, self.obs_dim], name=self.name+"observations0")

        policy = ActorNetwork(self.act_dim, self.name+'Actor')

        self.mu = policy.evaluate(self.OBS0)
        if self.name == 'right_':
            self.path = '/home/andy/collision_ws/src/Collision_Avoidance/train/weights/'+ NAME +'/'+ self.name+'85'
        else:
            self.path = '/home/andy/collision_ws/src/Collision_Avoidance/train/weights/'+ NAME +'/'+ self.name+'85'

        self.sess = tf.Session()
        self.saver = tf.train.Saver()
        self.saver.restore(self.sess, tf.train.latest_checkpoint(self.path))

    def choose_action(self, obs):
        action = self.sess.run(self.mu, feed_dict={self.OBS0: obs.reshape(1, -1)})
        action = np.squeeze(action)
        return action