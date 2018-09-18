from pendulum import Pendulum
import numpy as np
from numpy import pi
import time

p = Pendulum(1)

NQ   = 20  # Discretization steps for position
NV   = 19  # Discretization steps for velocity
VMAX = 5   # Max velocity (v in [-vmax,vmax])
NU   = 9   # Discretization steps for torque
UMAX = 5   # Max torque (u in [-umax,umax])
DT   = 3e-1

DQ = 2*pi/NQ
DV = 2.0*(VMAX)/NV
DU = 2.0*(UMAX)/NU

# Continuous to discrete
def c2dq(q):
    q = (q+pi)%(2*pi)
    return int(round(q/DQ))  % NQ

def c2dv(v):
    v = np.clip(v,-VMAX+1e-3,VMAX-1e-3)
    return int(np.floor((v+VMAX)/DV))

def c2du(u):
    u = np.clip(u,-UMAX+1e-3,UMAX-1e-3)
    return int(np.floor((u+UMAX)/DU))

def c2d(qv):
    '''From continuous to discrete.'''
    return c2dq(qv[0]),c2dv(qv[1])

# Discrete to continuous
def d2cq(iq):
    iq = np.clip(iq,0,NQ-1)
    return iq*DQ - pi

def d2cv(iv):
    iv = np.clip(iv,0,NV-1) - (NV-1)/2
    return iv*DV

def d2cu(iu):
    iu = np.clip(iu,0,NU-1) - (NU-1)/2
    return iu*DU

def d2c(iqv):
    '''From discrete to continuous'''
    return d2cq(iqv[0]),d2cv(iqv[1])

def x2i(x): return x[0]+x[1]*NQ
def i2x(i): return [ i%NQ, i/NQ ]

# --- PENDULUM

class DPendulum:
    def __init__(self):
        self.pendulum = Pendulum(1)
        self.pendulum.DT  = DT
        self.pendulum.NDT = 5

    @property
    def nqv(self): return [NQ,NV]
    @property
    def nx(self): return NQ*NV
    @property
    def nu(self): return NU
    @property
    def goal(self): return x2i(c2d([0.,0.]))

    def reset(self,x=None):
        if x is None:
            x = [ np.random.randint(0,NQ), np.random.randint(0,NV) ]
        else: x = i2x(x)
        assert(len(x)==2)
        self.x = x
        return x2i(self.x)

    def step(self,iu):
        self.x     = self.dynamics(self.x,iu)
        reward     = 1 if x2i(self.x)==self.goal else 0
        return x2i(self.x),reward

    def render(self):
        q = d2cq(self.x[0])
        self.pendulum.display(np.matrix([q,]))
        time.sleep(self.pendulum.DT)

    def dynamics(self,ix,iu):
        x   = np.matrix(d2c (ix)).T
        u   = d2cu(iu)
        
        self.xc,_ = self.pendulum.dynamics(x,u)
        return c2d(x.T.tolist()[0])
        

'''
env = DPendulum()

print env.reset(x2i([14,11]))
hq = []
hv = []
hqc = []
hvc = []
u = 0
for i in range(100):
    ix,r=env.step(u)
    q,v = i2x(ix)
    env.render()
    if d2cv(v)==0.0: u = MAXU-1 if u==0 else 0
    hq.append( d2cq(env.x[0]) )
    hv.append( d2cv(env.x[1]) )
    hqc.append( env.xc[0,0] )
    hvc.append( env.xc[1,0] )

'''

'''


EPS = 1e-3
q = 0.0
v = -VMAX
hq = []
hv = []
hiq = []
hiv = []
hqa = []
hva = []

while q<2*pi:
    hq.append(q)
    iq = c2dq(q)
    hiq.append(iq)
    hqa.append(d2cq(iq))
    q += EPS
while v<VMAX:
    iv = c2dv(v)
    hv.append(v)
    hiv.append(iv)
    hva.append(d2cv(iv))
    v += EPS




'''
