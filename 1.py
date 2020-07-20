# coding=UTF-8
from ActorNetwork import ActorNetwork
import numpy as np
import time
import tensorflow as tf
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
from keras import backend as K
K.set_session(sess)

actor = ActorNetwork(sess)

t0 = time.clock()
s_t = np.random.randint(100,size=500)
a_t = actor.model.predict(s_t.reshape(1, s_t.shape[0]))[0]
t1 = time.clock()
print((t1-t0))
