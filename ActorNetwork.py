from keras.initializations import normal, glorot_normal
from keras.activations import relu
from keras.layers import Dense, Input, BatchNormalization
from keras.models import Model
from keras.regularizers import l2
import keras.backend as K
import tensorflow as tf


class ActorNetwork(object):
    def __init__(self, sess):
        self.HIDDEN1_UNITS = 500
        self.HIDDEN2_UNITS = 500
        self.action_size = 100  
        self.state_size = 500
        self.sess = sess
        self.BATCH_SIZE = 32
        self.TAU = 0.001
        self.LEARNING_RATE = 0.01
        self.acti = 'sigmoid'
        self.h_acti = relu

        K.set_session(sess)

        #Now create the model
        self.model, self.weights, self.state = self.create_actor_network(self.state_size, self.action_size)
        self.target_model, self.target_weights, self.target_state = self.create_actor_network(self.state_size, self.action_size)
        self.action_gradient = tf.placeholder(tf.float32, [None, self.action_size])
        self.params_grad = tf.gradients(self.model.output, self.weights, -self.action_gradient)
        grads = zip(self.params_grad, self.weights)
        self.optimize = tf.train.AdamOptimizer(self.LEARNING_RATE).apply_gradients(grads)
        self.sess.run(tf.global_variables_initializer())

    def train(self, states, action_grads):
        self.sess.run(self.optimize, feed_dict={
            self.state: states,
            self.action_gradient: action_grads
        })

    def target_train(self):
        actor_weights = self.model.get_weights()
        actor_target_weights = self.target_model.get_weights()
        for i in range(len(actor_weights)):
            actor_target_weights[i] = self.TAU * actor_weights[i] + (1 - self.TAU)* actor_target_weights[i]
        self.target_model.set_weights(actor_target_weights)

    def create_actor_network(self, state_size, action_dim):
        S = Input(shape=[state_size], name='a_S')
        h0 = Dense(self.HIDDEN1_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h0')(S)
#        h1 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h1')(h0)
#        h2 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h2')(h1)
#        h3 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h3')(h2)
#        h4 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h4')(h3)
#        h5 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h5')(h4)
#        h6 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h6')(h5)
#        h7 = Dense(self.HIDDEN2_UNITS, activation=self.h_acti, init=glorot_normal, name='a_h7')(h6)
        V = Dense(action_dim, activation=self.acti, init=glorot_normal, name='a_V')(h0)
        model = Model(input=S, output=V)
        return model, model.trainable_weights, S
