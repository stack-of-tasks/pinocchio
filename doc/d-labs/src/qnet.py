'''
Example of Q-table learning with a simple discretized 1-pendulum environment using a linear Q network.
'''

import numpy as np
import random
import tensorflow as tf
import matplotlib.pyplot as plt
from dpendulum import DPendulum
import signal
import time

### --- Random seed
RANDOM_SEED = int((time.time()%10)*1000)
print "Seed = %d" % RANDOM_SEED
np.random.seed(RANDOM_SEED)
tf.set_random_seed(RANDOM_SEED)

### --- Hyper paramaters
NEPISODES               = 500           # Number of training episodes
NSTEPS                  = 50            # Max episode length
LEARNING_RATE           = 0.1           # Step length in optimizer
DECAY_RATE              = 0.99          # Discount factor 

### --- Environment
env = DPendulum()
NX  = env.nx
NU  = env.nu

### --- Q-value networks
class QValueNetwork:
    def __init__(self):
        x               = tf.placeholder(shape=[1,NX],dtype=tf.float32)
        W               = tf.Variable(tf.random_uniform([NX,NU],0,0.01,seed=100))
        qvalue          = tf.matmul(x,W)
        u               = tf.argmax(qvalue,1)

        qref            = tf.placeholder(shape=[1,NU],dtype=tf.float32)
        loss            = tf.reduce_sum(tf.square(qref - qvalue))
        optim           = tf.train.GradientDescentOptimizer(LEARNING_RATE).minimize(loss)

        self.x          = x             # Network input
        self.qvalue     = qvalue        # Q-value as a function of x
        self.u          = u             # Policy  as a function of x
        self.qref       = qref          # Reference Q-value at next step (to be set to l+Q o f)
        self.optim      = optim         # Optimizer      

### --- Tensor flow initialization
tf.reset_default_graph()
qvalue  = QValueNetwork()
sess = tf.InteractiveSession()
tf.global_variables_initializer().run()

def onehot(ix,n=NX):
    '''Return a vector which is 0 everywhere except index <i> set to 1.'''
    return np.array([[ (i==ix) for i in range(n) ],],np.float)
   
def disturb(u,i):
    u += int(np.random.randn()*10/(i/50+10))
    return np.clip(u,0,NU-1)

def rendertrial(maxiter=100):
    x = env.reset()
    for i in range(maxiter):
        u = sess.run(qvalue.u,feed_dict={ qvalue.x:onehot(x) })
        x,r = env.step(u)
        env.render()
        if r==1: print 'Reward!'; break
signal.signal(signal.SIGTSTP, lambda x,y:rendertrial()) # Roll-out when CTRL-Z is pressed

### --- History of search
h_rwd = []                              # Learning history (for plot).

### --- Training
for episode in range(1,NEPISODES):
    x    = env.reset()
    rsum = 0.0

    for step in range(NSTEPS-1):
        u = sess.run(qvalue.u,feed_dict={ qvalue.x: onehot(x) })[0] # Greedy policy ...
        u = disturb(u,episode)                                      # ... with noise
        x2,reward = env.step(u)

        # Compute reference Q-value at state x respecting HJB
        Q2        = sess.run(qvalue.qvalue,feed_dict={ qvalue.x: onehot(x2) })
        Qref      = sess.run(qvalue.qvalue,feed_dict={ qvalue.x: onehot(x ) })
        Qref[0,u] = reward + DECAY_RATE*np.max(Q2)

        # Update Q-table to better fit HJB
        sess.run(qvalue.optim,feed_dict={ qvalue.x    : onehot(x),
                                          qvalue.qref : Qref       })

        rsum += reward
        x = x2
        if reward == 1: break

    h_rwd.append(rsum)
    if not episode%20: print 'Episode #%d done with %d sucess' % (episode,sum(h_rwd[-20:]))

print "Total rate of success: %.3f" % (sum(h_rwd)/NEPISODES)
rendertrial()
plt.plot( np.cumsum(h_rwd)/range(1,NEPISODES) )
plt.show()

