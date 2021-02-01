import numpy as np
import tensorflow as tf
import gym
import random
NAME = 'SAC_v15_1'
# SAC_v14_6 is the best no fail
# SAC_v14_7 1024 and last is 512 no fail
# SAC_v14_8 all of 512 has fail
# SAC_v14_9 all of 1024 has fail
# SAC_v14_10 512 and last is 1024 has fail
# SAC_v14_11 all of 512 no fail
# SAC_v14_12 all of 512 has fail
# SAC_v14_13 all of 512 has fail change rand place
# SAC_v14_14 all of 512 has fail no rand
# SAC_v14_15 all of 512 has fail has rand joint3 =0
# SAC_v14_16 range_cnt += 0.001 and the same
# SAC_v14_17 range_cnt max 0.95 -> 0.85 and the same
# SAC_v14_18 change kinematics and singularity reward and the same
# SAC_v14_19 change random been
# SAC_v14_20 change cos reward
# SAC_v14_21 no cube
# SAC_v14_22 change reward
# SAC_v14_23 no no fail
# SAC_v14_25 no rev quat
EPS = 1e-8
LOAD = False
BATCH_SIZE = 512
SEED = [123, 321]
class ReplayBuffer(object):
    def __init__(self, capacity, name):
        self.name = name
        self.buffer = []
        self.capacity = capacity
        self.index = 0
        self.ep_reward = -2000

    def store_transition(self, obs0, act, rwd, obs1, done):
        data = (obs0, act, rwd, obs1, done)
        if self.capacity >= len(self.buffer):
            self.buffer.append(data)
        else:
            self.buffer[self.index] = data
        self.index = (self.index + 1) % self.capacity
    
    def store_eprwd(self, rwd):
        self.ep_reward = rwd

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        obs0, act, rwd, obs1, done = map(np.stack, zip(*batch))
        return obs0, act, rwd, obs1, done, self.ep_reward

class ValueNetwork(object):
    def __init__(self, name):
        self.name = name

    def step(self, obs):
        with tf.compat.v1.variable_scope(self.name):
            h1 = tf.compat.v1.layers.dense(obs, 256, tf.nn.leaky_relu, name='h1')
            h2 = tf.compat.v1.layers.dense(h1, 256, tf.nn.leaky_relu, name='h2')
            h3 = tf.compat.v1.layers.dense(h2, 256, tf.nn.leaky_relu, name='h3')
            # h4 = tf.compat.v1.layers.dense(h3, 1024, tf.nn.leaky_relu)
            value = tf.compat.v1.layers.dense(h3, 1)
            value = tf.squeeze(value, axis=1)
            return value

    def get_value(self, obs):
        value = self.step(obs)
        return value


class QValueNetwork(object):
    def __init__(self, name):
        self.name = name

    def step(self, obs, action, reuse):
        with tf.compat.v1.variable_scope(self.name, reuse=reuse):
            input = tf.concat([obs, action], axis=-1)
            h1 = tf.compat.v1.layers.dense(input, 256, tf.nn.leaky_relu, name='h1')
            h2 = tf.compat.v1.layers.dense(h1, 256, tf.nn.leaky_relu, name='h2')
            h3 = tf.compat.v1.layers.dense(h2, 256, tf.nn.leaky_relu, name='h3')
            # h4 = tf.compat.v1.layers.dense(h3, 1024, tf.nn.leaky_relu)
            q_value = tf.compat.v1.layers.dense(h3, 1)
            q_value = tf.squeeze(q_value, axis=1)
            return q_value

    def get_q_value(self, obs, action, reuse=False):
        q_value = self.step(obs, action, reuse)
        return q_value


class ActorNetwork(object):
    def __init__(self, act_dim, name):
        self.act_dim = act_dim
        self.name = name

    def step(self, obs, log_std_min=-20, log_std_max=2):
        with tf.compat.v1.variable_scope(self.name, reuse=tf.compat.v1.AUTO_REUSE):
            h1 = tf.compat.v1.layers.dense(obs, 256, tf.nn.leaky_relu, name='h1')
            h2 = tf.compat.v1.layers.dense(h1, 256, tf.nn.leaky_relu, name='h2')
            h3 = tf.compat.v1.layers.dense(h2, 256, tf.nn.leaky_relu, name='h3')
            h4 = tf.compat.v1.layers.dense(h3, 256, tf.nn.leaky_relu, name='h4')
            h5 = tf.compat.v1.layers.dense(h4, 256, tf.nn.leaky_relu, name='h5')
            mu = tf.compat.v1.layers.dense(h5, self.act_dim, None, name='mu')
            log_std = tf.compat.v1.layers.dense(h5, self.act_dim, tf.tanh, name='log_std')
            log_std = log_std_min + 0.5 * (log_std_max - log_std_min) * (log_std + 1)

            std = tf.exp(log_std)
            pi = mu + tf.random.normal(tf.shape(mu)) * std

            pre_sum = -0.5 * (((pi - mu) / (tf.exp(log_std) + EPS)) ** 2 + 2 * log_std + np.log(2 * np.pi))
            logp_pi = tf.reduce_sum(pre_sum, axis=1)

            mu = tf.tanh(mu)
            pi = tf.tanh(pi)

            clip_pi = 1 - tf.square(pi)
            clip_up = tf.cast(clip_pi > 1, tf.float32)
            clip_low = tf.cast(clip_pi < 0, tf.float32)
            clip_pi = clip_pi + tf.stop_gradient((1 - clip_pi) * clip_up + (0 - clip_pi) * clip_low)

            logp_pi -= tf.reduce_sum(tf.math.log(clip_pi + 1e-6), axis=1)
        return mu, pi, logp_pi

    def evaluate(self, obs):
        mu, pi, logp_pi = self.step(obs)
        action_scale = 1.0 # env.action_space.high[0]
        mu *= action_scale
        pi *= action_scale
        return mu, pi, logp_pi


class SAC(object):
    def __init__(self, act_dim, obs_dim, lr_actor, lr_value, gamma, tau, buffers, alpha=0.2, name=None, seed=1):
        # tf.reset_default_graph()

        self.act_dim = act_dim
        self.obs_dim = obs_dim
        self.lr_actor = lr_actor
        self.lr_value = lr_value
        self.gamma = gamma
        self.tau = tau
        self.name = name
        self.replay_buffer = []
        self.buffers = buffers

        for i in range(buffers):
            b = ReplayBuffer(capacity=int(1e6), name=self.name+'buffer'+str(i))
            self.replay_buffer.append(b)

        self.OBS0 = tf.compat.v1.placeholder(tf.float32, [None, self.obs_dim], name=self.name+"observations0")
        self.OBS1 = tf.compat.v1.placeholder(tf.float32, [None, self.obs_dim], name=self.name+"observations1")
        self.ACT = tf.compat.v1.placeholder(tf.float32, [None, self.act_dim], name=self.name+"action")
        self.RWD = tf.compat.v1.placeholder(tf.float32, [None,], name=self.name+"reward")
        self.DONE = tf.compat.v1.placeholder(tf.float32, [None,], name=self.name+"done")
        self.EPRWD = tf.compat.v1.placeholder(tf.int32, [], name=self.name+"ep_reward")
        
        self.policy_loss = tf.compat.v1.placeholder(tf.float32, [None, 1], name=self.name+"policy_loss")
        self.q_value1_loss = tf.compat.v1.placeholder(tf.float32, [None, 1], name=self.name+"q_value1_loss")
        self.q_value2_loss = tf.compat.v1.placeholder(tf.float32, [None, 1], name=self.name+"q_value2_loss")
        self.value_loss = tf.compat.v1.placeholder(tf.float32, [None, 1], name=self.name+"value_loss")
        self.total_value_loss = tf.compat.v1.placeholder(tf.float32, [None, 1], name=self.name+"total_value_loss")

        policy = ActorNetwork(self.act_dim, self.name+'Actor')
        q_value_net_1 = QValueNetwork(self.name+'Q_value1')
        q_value_net_2 = QValueNetwork(self.name+'Q_value2')
        value_net = ValueNetwork(self.name+'Value')
        target_value_net = ValueNetwork(self.name+'Target_Value')

        mu, self.pi, logp_pi = policy.evaluate(self.OBS0)

        q_value1 = q_value_net_1.get_q_value(self.OBS0, self.ACT)
        q_value1_pi = q_value_net_1.get_q_value(self.OBS0, self.pi, reuse=True)
        q_value2 = q_value_net_2.get_q_value(self.OBS0, self.ACT)
        q_value2_pi = q_value_net_2.get_q_value(self.OBS0, self.pi, reuse=True)
        value = value_net.get_value(self.OBS0)
        target_value = target_value_net.get_value(self.OBS1)

        min_q_value_pi = tf.minimum(q_value1_pi, q_value2_pi)
        next_q_value = tf.stop_gradient(self.RWD + self.gamma * (1 - self.DONE) * target_value)
        next_value = tf.stop_gradient(min_q_value_pi - alpha * logp_pi)

        self.policy_loss = tf.reduce_mean(alpha * logp_pi - q_value1_pi)
        self.q_value1_loss = tf.reduce_mean(tf.math.squared_difference(next_q_value, q_value1))
        self.q_value2_loss = tf.reduce_mean(tf.math.squared_difference(next_q_value, q_value2))
        self.value_loss = tf.reduce_mean(tf.math.squared_difference(next_value, value))
        self.total_value_loss = self.q_value1_loss + self.q_value2_loss + self.value_loss

        actor_optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=self.lr_actor)
        actor_train_op = actor_optimizer.minimize(self.policy_loss, var_list=tf.compat.v1.global_variables(self.name+'Actor'))
        value_optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=self.lr_value)
        value_params = tf.compat.v1.global_variables(self.name+'Q_value1') + tf.compat.v1.global_variables(self.name+'Q_value2') + tf.compat.v1.global_variables(self.name+'Value')  #who is Q_value

        with tf.control_dependencies([actor_train_op]):
            value_train_op = value_optimizer.minimize(self.total_value_loss, var_list=value_params)
        with tf.control_dependencies([value_train_op]):
            self.target_update = [tf.compat.v1.assign(tv, self.tau * tv + (1 - self.tau) * v)
                             for v, tv in zip(tf.compat.v1.global_variables(self.name+'Value'), tf.compat.v1.global_variables(self.name+'Target_Value'))]

        target_init = [tf.compat.v1.assign(tv, v)
                       for v, tv in zip(tf.compat.v1.global_variables(self.name+'Value'), tf.compat.v1.global_variables(self.name+'Target_Value'))]
        
        gpu_options = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=0.4)
        self.sess = tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(gpu_options=gpu_options))
        # new_graph = tf.Graph()
        # new_graph.seed = SEED[seed]
        # self.sess = tf.Session()
        tf.compat.v1.summary.scalar(self.name+'policy_loss', self.policy_loss)
        tf.compat.v1.summary.scalar(self.name+'q_value1_loss', self.q_value1_loss)
        tf.compat.v1.summary.scalar(self.name+'q_value2_loss', self.q_value2_loss)
        tf.compat.v1.summary.scalar(self.name+'value_loss', self.value_loss)
        tf.compat.v1.summary.scalar(self.name+'total_value_loss', self.total_value_loss)
        tf.compat.v1.summary.scalar(self.name+'ep_reward', self.EPRWD)
        tf.compat.v1.summary.scalar(self.name+'rwd', self.RWD[0])
        self.merged = tf.compat.v1.summary.merge_all()
        self.writer = tf.compat.v1.summary.FileWriter('/home/iclab/c_a_ws/src/train/logs/'+NAME+'/'+self.name+'/', self.sess.graph)
        self.saver = tf.compat.v1.train.Saver()
        self.path = '/home/iclab/c_a_ws/src/train/weights/'+ NAME +'/'+ self.name
        # self.saver = tf.train.Saver()
        self.saver.restore(self.sess, tf.train.latest_checkpoint(self.path + '80'))
        # self.sess.run(tf.compat.v1.global_variables_initializer())
        self.sess.run(target_init)

    def choose_action(self, obs):
        action = self.sess.run(self.pi, feed_dict={self.OBS0: obs.reshape(1, -1)})
        action = np.squeeze(action)
        return action

    def learn(self, indx):
        obs0, act, rwd, obs1, done, eprwd = self.replay_buffer[indx%self.buffers].sample(batch_size=BATCH_SIZE)
        feed_dict = {self.OBS0: obs0, self.ACT: act,self.OBS1: obs1, self.RWD: rwd,
                     self.DONE: np.float32(done)}
        self.sess.run(self.target_update, feed_dict)
        
        if indx%50 == 0:
            obs0, act, rwd, obs1, done, eprwd = self.replay_buffer[indx%self.buffers].sample(batch_size=1)
            feed_dict = {self.OBS0: obs0, self.ACT: act,self.OBS1: obs1, self.RWD: rwd,
                     self.DONE: np.float32(done), self.EPRWD: eprwd}
            result = self.sess.run(self.merged, feed_dict)
            self.writer.add_summary(result, indx)
            