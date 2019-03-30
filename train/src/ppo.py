import gym
import numpy as np
import tensorflow as tf

NAME = 'PPO_v2'
LOAD = False

class Memory(object):
    def __init__(self):
        self.ep_obs, self.ep_act, self.ep_rwd, self.ep_neglogp = [], [], [], []

    def store_transition(self, obs0, act, rwd, neglogp):
        self.ep_obs.append(obs0)
        self.ep_act.append(act)
        self.ep_rwd.append(rwd)
        self.ep_neglogp.append(neglogp)

    def covert_to_array(self):
        array_obs = np.vstack(self.ep_obs)
        array_act = np.vstack(self.ep_act)
        array_rwd = np.array(self.ep_rwd)
        array_neglogp = np.array(self.ep_neglogp).squeeze(axis=1)
        return array_obs, array_act, array_rwd, array_neglogp

    def reset(self):
        self.ep_obs, self.ep_act, self.ep_rwd, self.ep_neglogp = [], [], [], []


class ActorNetwork(object):
    def __init__(self, act_dim, name):
        self.act_dim = act_dim
        self.name = name

    def step(self, obs, reuse):
        with tf.variable_scope(self.name, reuse=reuse):
            h1 = tf.layers.dense(obs, 4096, activation=tf.nn.relu)
            h2 = tf.layers.dense(h1, 4096, activation=tf.nn.relu)
            mu = 2 * tf.layers.dense(h2, self.act_dim, activation=tf.nn.tanh)
            sigma = tf.layers.dense(h2, self.act_dim, activation=tf.nn.softplus)
            pd = tf.distributions.Normal(loc=mu, scale=sigma)
        return pd

    def choose_action(self, obs, reuse=False):
        pd = self.step(obs, reuse)
        action = tf.squeeze(pd.sample(1), axis=0)
        action = tf.clip_by_value(action, -1, 1)
        return action

    def get_logp(self, obs, act, reuse=False):
        pd = self.step(obs, reuse)
        logp = pd.log_prob(act)
        return logp


class ValueNetwork(object):
    def __init__(self, name):
        self.name = name

    def step(self, obs, reuse):
        with tf.variable_scope(self.name, reuse=reuse):
            h1 = tf.layers.dense(inputs=obs, units=4096, activation=tf.nn.relu)
            h2 = tf.layers.dense(inputs=obs, units=4096, activation=tf.nn.relu)
            value = tf.layers.dense(inputs=h2, units=1)
            return value

    def get_value(self, obs, reuse=False):
        value = self.step(obs, reuse)
        return value


class PPO(object):
    def __init__(self, act_dim, obs_dim, lr_actor, lr_value, gamma, clip_range, name):
        self.name = name
        self.act_dim = act_dim
        self.obs_dim = obs_dim
        self.lr_actor = lr_actor
        self.lr_value = lr_value
        self.gamma = gamma
        self.clip_range = clip_range

        self.OBS = tf.placeholder(tf.float32, [None, self.obs_dim], name=self.name+"observation")
        self.ACT = tf.placeholder(tf.float32, [None, self.act_dim], name=self.name+"action")
        self.Q_VAL = tf.placeholder(tf.float32, [None, 1], name=self.name+"q_value")
        self.ADV = tf.placeholder(tf.float32, [None, 1], name=self.name+"advantage")
        self.NEGLOGP = tf.placeholder(tf.float32, [None, self.act_dim], name=self.name+"old_neglogp")
        self.value_loss = tf.placeholder(tf.float32, [None, 1], name=self.name+"_v_loss")
        self.actor_loss = tf.placeholder(tf.float32, [None, 1], name=self.name+"_a_loss")

        actor = ActorNetwork(self.act_dim, self.name+'actor')
        value = ValueNetwork(self.name+'critic')
        self.memory = Memory()

        self.action = actor.choose_action(self.OBS)
        self.neglogp = actor.get_logp(self.OBS, self.ACT, reuse=True)
        ratio = self.neglogp / self.NEGLOGP
        clip_ratio = tf.clip_by_value(ratio, 1. - self.clip_range, 1. + self.clip_range)
        self.actor_loss = -tf.reduce_mean((tf.minimum(ratio, clip_ratio)) * self.ADV)
        self.actor_train_op = tf.train.AdamOptimizer(self.lr_actor).minimize(self.actor_loss)

        self.value = value.get_value(self.OBS)
        self.advantage = self.Q_VAL - self.value
        self.value_loss = tf.reduce_mean(tf.square(self.advantage))
        self.value_train_op = tf.train.AdamOptimizer(self.lr_value).minimize(self.value_loss)

        self.sess = tf.Session()
        tf.summary.scalar(self.name+'_v_loss', self.value_loss)
        tf.summary.scalar(self.name+'_a_loss', self.actor_loss)
        self.merged = tf.summary.merge_all()
        self.writer = tf.summary.FileWriter('/home/ubuntu/Andy/collision_ws/src/Collision_Avoidance/train/logs/'+NAME+'/'+self.name+'/', self.sess.graph)
        self.saver = tf.train.Saver()
        self.path = './'+ NAME + self.name
        if LOAD:
            self.saver.restore(self.sess, tf.train.latest_checkpoint(self.path))
        else:
            self.sess.run(tf.global_variables_initializer())

    def choose_action(self, obs):
        if obs.ndim < 2: obs = obs[np.newaxis, :]
        action = self.sess.run(self.action, feed_dict={self.OBS: obs})
        # action = np.squeeze(action, 1).clip(-2, 2)
        action = np.squeeze(action)
        neglogp = self.sess.run(self.neglogp, feed_dict={self.OBS: obs, self.ACT: action[np.newaxis, :]})

        value = self.sess.run(self.value, feed_dict={self.OBS: obs})
        # value = np.squeeze(value, 1).squeeze(0)
        value = np.squeeze(value)
        return action, neglogp, value

    def learn(self, last_value, done, indx):
        obs, act, rwd, neglogp = self.memory.covert_to_array()
        q_value = self.compute_q_value(last_value, done, rwd)

        adv = self.sess.run(self.advantage, {self.OBS: obs, self.Q_VAL: q_value})

        [self.sess.run(self.value_train_op,
                       {self.OBS: obs, self.Q_VAL: q_value}) for _ in range(10)]
        [self.sess.run(self.actor_train_op,
                       {self.OBS: obs, self.ACT: act, self.ADV: adv, self.NEGLOGP: neglogp}) for _ in range(10)]
        result = self.sess.run(self.merged, 
                       {self.OBS: obs, self.ACT: act, self.ADV: adv, self.NEGLOGP: neglogp, self.Q_VAL: q_value, self.Q_VAL: q_value})
        self.writer.add_summary(result, indx)
        self.memory.reset()

    def compute_q_value(self, last_value, done, rwd):
        q_value_ = np.zeros_like(rwd)
        value = 0 if done else last_value
        for t in reversed(range(0, len(rwd))):
            value = value * self.gamma + rwd[t]
            q_value_[t] = value
        return q_value_[:, np.newaxis]


# env = gym.make('Pendulum-v0')
# env.seed(1)
# env = env.unwrapped

# # agent = PPO(act_dim=env.action_space.shape[0], obs_dim=env.observation_space.shape[0],
# #             lr_actor=0.0001, lr_value=0.0002, gamma=0.9, clip_range=0.2)

# nepisode = 1000
# nstep = 200

# for i_episode in range(nepisode):
#     obs0 = env.reset()
#     ep_rwd = 0

#     for t in range(nstep):
#         act, neglogp, _ = agent.choose_action(obs0)
#         obs1, rwd, done, _ = env.step(act)

#         agent.memory.store_transition(obs0, act, (rwd + 8) / 8, neglogp)

#         obs0 = obs1
#         ep_rwd += rwd

#         if (t + 1) % 32 == 0 or t == nstep - 1:
#             _, _, last_value = agent.choose_action(obs1)
#             agent.learn(last_value, done)

#     print('Ep: %i' % i_episode, "|Ep_r: %i" % ep_rwd)