'''
Example of Q-table learning with a simple discretized 1-pendulum environment.
'''

import numpy as np
from dpendulum import DPendulum
import matplotlib.pyplot as plt
import signal
import time

### --- Random seed
RANDOM_SEED = int((time.time()%10)*1000)
print "Seed = %d" % RANDOM_SEED
np.random.seed(RANDOM_SEED)

### --- Hyper paramaters
NEPISODES               = 500           # Number of training episodes
NSTEPS                  = 50            # Max episode length
LEARNING_RATE           = 0.85          # 
DECAY_RATE              = 0.99          # Discount factor 

### --- Environment
env = DPendulum()
NX  = env.nx                            # Number of (discrete) states
NU  = env.nu                            # Number of (discrete) controls

Q     = np.zeros([env.nx,env.nu])       # Q-table initialized to 0

def rendertrial(maxiter=100):
    '''Roll-out from random state using greedy policy.'''
    s = env.reset()
    for i in range(maxiter):
        a = np.argmax(Q[s,:])
        s,r = env.step(a)
        env.render()
        if r==1: print 'Reward!'; break

signal.signal(signal.SIGTSTP, lambda x,y:rendertrial()) # Roll-out when CTRL-Z is pressed
h_rwd = []                              # Learning history (for plot).

for episode in range(1,NEPISODES):
    x    = env.reset()
    rsum = 0.0
    for steps in range(NSTEPS):
        u         = np.argmax(Q[x,:] + np.random.randn(1,NU)/episode) # Greedy action with noise
        x2,reward = env.step(u)

        # Compute reference Q-value at state x respecting HJB
        Qref = reward + DECAY_RATE*np.max(Q[x2,:])

        # Update Q-Table to better fit HJB
        Q[x,u] += LEARNING_RATE*(Qref-Q[x,u])
        x       = x2
        rsum   += reward
        if reward==1:             break

    h_rwd.append(rsum)
    if not episode%20: print 'Episode #%d done with %d sucess' % (episode,sum(h_rwd[-20:]))

print "Total rate of success: %.3f" % (sum(h_rwd)/NEPISODES)
rendertrial()
plt.plot( np.cumsum(h_rwd)/range(1,NEPISODES) )
plt.show()
