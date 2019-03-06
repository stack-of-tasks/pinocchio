'''
Example of optimal control resolution by direct optimization of a single trajectory.
'''

from pendulum import Pendulum
from scipy.optimize import *
from pinocchio.utils import *
import pinocchio as pin
from numpy.linalg import norm
import time
import signal
import matplotlib.pyplot as plt

env      = Pendulum(1)
NSTEPS   = 50
x0 = env.reset().copy()

def cost(U):
    '''Cost for a trajectory starting at state X0 with control U'''
    env.reset(x0)
    csum = 0.0
    for t in range(NSTEPS):
        u = U[env.nu*t:env.nu*(t+1)]  # Control at time step <t>
        _,r = env.step(u)             # Pendulum step, with reward r
        csum += r                     # Cumulative sum
    return -csum                      # Returns cost ie negative reward

def display(U,verbose=False):
    '''Display the trajectory on Gepetto viewer.'''
    x = x0.copy()
    if verbose: print "U = ", " ".join(map(lambda u:'%.1f'%u,np.asarray(U).flatten()))
    for i in range(len(U)/env.nu):
        env.dynamics(x,U[env.nu*i:env.nu*(i+1)],True)
        env.display(x)
        time.sleep(5e-2)
        if verbose: print "X%d"%i,x.T

class CallBack:
    '''Call back function used to follow optimizer steps.'''
    def __init__(self):
        self.iter = 0
        self.withdisplay = False
        self.h_rwd = []
    def __call__(self,U):
        print 'Iteration ',self.iter, " ".join(map(lambda u:'%.1f'%u,np.asarray(U).flatten()))
        self.iter += 1
        self.U = U.copy()
        self.h_rwd.append(cost(U))
        if self.withdisplay: r = display(U)        # Display if CTRL-Z has been pressed.
    def setWithDisplay(self,boolean = None):
        self.withdisplay = not self.withdisplay if boolean is None else boolean

callback = CallBack()
signal.signal(signal.SIGTSTP, lambda x,y:callback.setWithDisplay())

### --- OCP resolution
U0     = zero(NSTEPS*env.nu)-env.umax             # Initial guess for the control trajectory.
bounds = [ [-env.umax,env.umax], ]*env.nu*NSTEPS  # Set control bounds to environment umax.

# Start BFGS optimization routine 
U,c,info = fmin_l_bfgs_b(cost,x0=U0,callback=callback,
                         approx_grad=True,bounds=bounds)

# When done, display the trajectory in Gepetto-viewer
display(U,True)

plt.plot(callback.h_rwd)
plt.show()

