#
# Copyright (c) 2016 CNRS
#

from __future__ import print_function

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from numpy.linalg import norm
import lambdas
from lambdas import Mcross,ancestors,parent,iv,td,quad,adj,adjdual

def hessian(robot,q,crossterms=False):
    '''
    Compute the Hessian tensor of all the robot joints.
    If crossterms, then also compute the Si x Si terms, 
    that are not part of the true Hessian but enters in some computations like VRNEA.
    '''
    lambdas.setRobotArgs(robot)
    H=np.zeros([6,robot.model.nv,robot.model.nv])
    pin.computeJointJacobians(robot.model,robot.data,q)
    J = robot.data.J
    skiplast = -1 if not crossterms else None
    for joint_i in range(1,robot.model.njoints):
        for joint_j in ancestors(joint_i)[:skiplast]: # j is a child of i
            for i in iv(joint_i):
                for j in iv(joint_j):
                    Si  = J[:,i]
                    Sj  = J[:,j]
                    H[:,i,j] = np.asarray(Mcross(Sj,    Si))[:,0]
    return H


# CRBA ====>       Mp[i,j] += Si.transpose()*Yk*Sj
# D-CRBA ==>   TSi Y Sj + Si Y TSj - Si YSdx Sj + Si SdxY Sj
# Term a = term d && term b = term c  --- or are null
#    -> term a is null if j<=diff
#    -> term d is null if diff>k
#    -> then a-d is nonzero only when k>=diff>j
# Due to simplification, terms cancel to the following:
#    if i<d (always true)    M[i,j,d]  =   Si.T Yd TjSd   (= -Si.T Yd Sd x Sj)
#    if j<d (possibly false) M[i,j,d] += TdSi.T Yd   Sd   (= +Si.T Sdx Yd Sj)
# where Yd is the composite inertia of the minimal subtree rooted either at j or d.

class DCRBA:
    '''
    Compute the derivative (tangent application) of the mass matrix M wrt q,
    which is a NVxNVxNV tensor.
    '''

    def __init__(self,robot):
        self.robot = robot
        lambdas.setRobotArgs(robot)

    def pre(self,q):
        robot = self.robot
        self.H = hessian(robot,q)
        self.J = robot.data.J.copy()
        pin.crba(robot.model,robot.data,q)
        self.Y =[ (robot.data.oMi[i]*robot.data.Ycrb[i]).matrix() for i in range(0,robot.model.njoints) ]

        self.dM = np.zeros([robot.model.nv,]*3)

    def __call__(self):
        robot = self.robot
        J     = self.J
        Y     = self.Y
        dM    = self.dM
        H     = self.H

        for j in range(robot.model.njoints-1,0,-1):
            for joint_diff in range(1,robot.model.njoints):  # diff should be a descendant or ancestor of j
                if j not in ancestors(joint_diff) and joint_diff not in ancestors(j): continue

                for i in ancestors(min(parent(joint_diff),j)):

                    i0,i1 = iv(i)[0],iv(i)[-1]+1
                    j0,j1 = iv(j)[0],iv(j)[-1]+1

                    Si  = J[:,i0:i1]
                    Sj  = J[:,j0:j1]

                    for d in iv(joint_diff):

                        T_iSd = np.matrix(H[:,d,i0:i1])   # this is 0 if d<=i (<=j)
                        T_jSd = np.matrix(H[:,d,j0:j1])   # this is 0 is d<=j

                        '''
                        assert( norm(T_iSd)<1e-6 or not joint_diff<i )  # d<i => TiSd=0
                        assert( norm(T_jSd)<1e-6 or not joint_diff<j )  # d<j => TjSd=0
                        assert( norm(T_jSd)<1e-6 or not norm(T_iSd)<1e-6 )  # TiSd=0 => TjSd=0
                        assert( norm(T_iSd)>1e-6 )
                        '''
                    
                        Yd = Y[max(j,joint_diff)]

                        dM    [i0:i1,j0:j1,d]  = T_iSd.T * Yd *    Sj
                        if j<joint_diff: 
                            dM[i0:i1,j0:j1,d] +=    Si.T * Yd * T_jSd
                            
                        # Make dM triangular by copying strict-upper triangle to lower one.
                        if i!=j: dM[j0:j1,i0:i1,d]      = dM[i0:i1,j0:j1,d].T
                        else:   dM[j0:j1,i0:i1,d]     += np.triu(dM[i0:i1,j0:j1,d],1).T

        return dM

# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
class VRNEA:
    '''
    Compute the C tensor so that nle(q,vq) = vq C vq.
    Noting Cv = C*vq for an arbitrary robot velocity vq:
    Since Mdot = (Cv+Cv')/2, then C = dCv/dvq and dM/dq = (Cv+Cv')

    Q is index by i,j,k i.e. tau_k = v.T*Q[:,:,k]*v
    At level k, Qk is a lower triangular matrix (same shape as H) where:
      * rows i s.t. i>k are equals to Sk.T*Ycrb_i*S_ixS_j (j the col index), 
      * rows i s.t. i<=k are equals to Sk.T*Ycrb_k*S_ixS_j (j the col index)
    To avoid the call to Ycrb_i>k, the first par is set up while computing Q_k
    '''

    def __init__(self,robot):
        self.robot = robot
        lambdas.setRobotArgs(robot)
        self.YJ = zero([6,robot.model.nv])
        self.Q  = np.zeros([robot.model.nv,]*3)

    def __call__(self,q):
        robot = self.robot
        H     = hessian(robot,q,crossterms=True)
        J     = robot.data.J
        YJ    = self.YJ
        Q     = self.Q

        # The terms in SxS YS corresponds to f = vxYv 
        # The terms in S YSxS corresponds to a = vxv  (==> f = Yvxv)
        for  k in range(robot.model.njoints-1,0,-1):
            k0,k1 = iv(k)[0],iv(k)[-1]+1
            Yk    = (robot.data.oMi[k]*robot.data.Ycrb[k]).matrix() 
            Sk    = J[:,k0:k1]
            for j in ancestors(k):  # Fill YJ = [ ... Yk*Sj ... ]_j
                j0,j1 = iv(j)[0],iv(j)[-1]+1
                YJ[:,j0:j1]    = Yk*J[:,j0:j1]

            # Fill the diagonal of the current level of Q = Q[k,:k,:k]
            for i in ancestors(k):
                i0,i1 = iv(i)[0],iv(i)[-1]+1
                Si      = J[:,i0:i1]

                Q[k0:k1,i0:i1,i0:i1] = -td(H[:,k0:k1,i0:i1],YJ[:,i0:i1],[0,0])    # = (Si x Sk)' * Yk Si    

                # Fill the nondiag of the current level Q[k,:k,:k]
                for j in ancestors(i)[:-1]:
                    j0,j1 = iv(j)[0],iv(j)[-1]+1
                    Sj      = J[:,j0:j1]

                    #  = Sk' * Yk * Si x Sj
                    Q[k0:k1,i0:i1,j0:j1]  = td(YJ[:,k0:k1],     H[:,i0:i1,j0:j1],[0,0])
                    #  = (Si x Sk)' * Yk Sj
                    Q[k0:k1,i0:i1,j0:j1] -= td(H[:,k0:k1,i0:i1],YJ[:,j0:j1],     [0,0])
                    #  = (Sj x Sk)' * Yk Si
                    Q[k0:k1,j0:j1,i0:i1] =- td(H[:,k0:k1,j0:j1],YJ[:,i0:i1],     [0,0])
                    
            # Fill the border elements of levels below k Q[kk,k,:] and Q[kk,:,k] with kk<k
            for kk in ancestors(k)[:-1]:
                kk0,kk1 = iv(kk)[0],iv(kk)[-1]+1
                Skk     = J[:,kk0:kk1]

                for j in ancestors(k):
                    j0,j1 = iv(j)[0],iv(j)[-1]+1
                    Sj      = J[:,j0:j1]

                    #  = Skk' Yk Sk x Sj  = (Yk Skk)' Sk x Sj
                    if k!=j:
                        Q[kk0:kk1,k0:k1,j0:j1]  = td(YJ[:,kk0:kk1],H[:,k0:k1,j0:j1 ],[0,0])
                    #  = (Sk x Skk)' Yk Sj
                    Q[kk0:kk1,k0:k1,j0:j1] += td(H[:,k0:k1,kk0:kk1].T,YJ[:,j0:j1],       [2,0])
                    #  = (Sj x Skk)' Yk Sk
                    # Switch because j can be > or < than kk
                    if j<kk:
                        Q[kk0:kk1,j0:j1,k0:k1] = -Q[j0:j1,kk0:kk1,k0:k1].transpose(1,0,2)
                    elif j>=kk:
                        Q[kk0:kk1,j0:j1,k0:k1] = td(H[:,j0:j1,kk0:kk1].T,YJ[:,k0:k1],    [2,0])

        return Q

# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
class Coriolis:
    def __init__(self,robot):
        self.robot = robot
        lambdas.setRobotArgs(robot)
        NV = robot.model.nv
        NJ = robot.model.njoints
        self.C = zero([NV,NV])
        self.YS = zero([6,NV])
        self.Sxv = zero([6,NV])

    def __call__(self,q,vq):
        robot = self.robot
        NV  = robot.model.nv
        NJ  = robot.model.njoints
        C   = self.C
        YS  = self.YS
        Sxv = self.Sxv
        H = hessian(robot,q)

        pin.computeAllTerms(robot.model,robot.data,q,vq)
        J = robot.data.J
        oMi=robot.data.oMi
        v = [ (oMi[i]*robot.data.v[i]).vector      for i in range(NJ) ]
        Y = [ (oMi[i]*robot.data.Ycrb[i]).matrix() for i in range(NJ) ]

        # --- Precomputations
        # Centroidal matrix
        for j in range(NJ):
            j0,j1 = iv(j)[0],iv(j)[-1]+1
            YS[:,j0:j1] = Y[j]*J[:,j0:j1]

        # velocity-jacobian cross products.
        for j in range(NJ):
            j0,j1 = iv(j)[0],iv(j)[-1]+1
            vj = v[j]
            Sxv[:,j0:j1] = adj(vj)*J[:,j0:j1]

        # --- Main loop
        for i in range(NJ-1,0,-1):
            i0,i1 = iv(i)[0],iv(i)[-1]+1
            Yi    = Y[i]
            Si    = J[:,i0:i1]
            vp = v[robot.model.parents[i]]

            SxvY = Sxv[:,i0:i1].T*Yi
            for j in ancestors(i):
                j0,j1 = iv(j)[0],iv(j)[-1]+1
                Sj      = J[:,j0: j1 ]

                # COR ===> Si' Yi Sj x vj
                C[i0:i1,j0:j1]  = YS[:,i0:i1].T*Sxv[:,j0:j1]
                # CEN ===> Si' vi x Yi Sj = - (Si x vi)' Yi Sj
                C[i0:i1,j0:j1] -= SxvY*Sj

            vxYS = adjdual(v[i])*Yi*Si
            YSxv = Yi*Sxv[:,i0:i1]
            Yv   = Yi*vp
            for ii in ancestors(i)[:-1]:
                ii0,ii1 = iv(ii)[0],iv(ii)[-1]+1
                Sii = J[:,ii0:ii1]
                # COR ===> Sii' Yi Si x vi
                C[ii0:ii1,i0:i1]  = Sii.T*YSxv
                # CEN ===> Sii' vi x Yi Si = (Sii x vi)' Yi Si
                C[ii0:ii1,i0:i1] += Sii.T*vxYS
                # CEN ===> Sii' Si x Yi vi = (Sii x Si)' Yi vi
                C[ii0:ii1,i0:i1] += np.matrix(td(H[:,i0:i1,ii0:ii1],Yv,[0,0])[:,:,0]).T

        return C

# --- DRNEA -------------------------------------------------------------------------
# --- DRNEA -------------------------------------------------------------------------
# --- DRNEA -------------------------------------------------------------------------

from lambdas import MCross,FCross


class DRNEA:
    def __init__(self,robot):
        self.robot = robot
        lambdas.setRobotArgs(robot)
        
    def __call__(self,q,vq,aq):
        robot   = self.robot
        pin.rnea(robot.model,robot.data,q,vq,aq)
        NJ      = robot.model.njoints
        NV      = robot.model.nv
        J       = robot.data.J
        oMi     = robot.data.oMi
        v       = [ (oMi[i]*robot.data.v   [i]).vector           for i in range(NJ) ]
        a       = [ (oMi[i]*robot.data.a_gf[i]).vector           for i in range(NJ) ]
        f       = [ (oMi[i]*robot.data.f   [i]).vector           for i in range(NJ) ]
        Y       = [ (oMi[i]*robot.model.inertias[i]).matrix() for i in range(NJ) ]
        Ycrb    = [ (oMi[i]*robot.data.Ycrb[i])     .matrix() for i in range(NJ) ]
        
        Tkf = [ zero([6,NV]) for i in range(NJ) ]
        Yvx = [ zero([6,6]) for i in range(NJ) ]
        adjf = lambda f: -np.bmat([ [zero([3,3]),skew(f[:3])] , [skew(f[:3]),skew(f[3:])] ])
        
        R = self.R = zero([NV,NV])

        for i in reversed(range(1,NJ)):           # i is the torque index :    dtau_i = R[i,:] dq
            i0,i1 = iv(i)[0],iv(i)[-1]+1
            Yi = Y[i]
            Yci = Ycrb[i]
            Si = J[:,i0:i1]
            vi = v[i]
            ai = a[i]
            fi = f[i]
            aqi = aq[i0:i1]
            vqi = vq[i0:i1]
            dvi = Si*vqi
            li = parent(i)

            Yvx[ i] += Y[i]*adj(v[i])
            Yvx[ i] -= adjf(Y[i]*v[i])
            Yvx[ i] -= adjdual(v[i])*Yi

            Yvx[li] += Yvx[i]

            for k in ancestors(i):      # k is the derivative index: dtau   = R[:,k] dq_k
                k0,k1 = iv(k)[0],iv(k)[-1]+1
                Sk = J[:,k0:k1]
                lk = parent(k)

                # Si' Yi Tk ai = Si' Yi ( Sk x (ai-alk) + (vi-vlk) x (Sk x vlk) )
                # Tk Si' Yi ai = Si' Yi ( - Sk x a[lk]  + (vi-vlk) x (Sk x vlk) ) 
                Tkf = Yci*(- MCross(Sk,a[lk]) + MCross(MCross(Sk,v[lk]),v[lk]))
        
                # Tk Si' fs = Tk Si' Ycrb[i] ai + Si' Ys (vs-vi) x (Sk x vlk)
                #           =      ""           + Si' Ys vs x (Sk x vlk) - Si' Ys vi x (Sk x vlk)
                Tkf += Yvx[i]*MCross(Sk,v[lk])

                R[i0:i1,k0:k1]      = Si.T*Tkf
                if i==k:
                    for kk in ancestors(k)[:-1]:
                        kk0,kk1 = iv(kk)[0],iv(kk)[-1]+1
                        Skk = J[:,kk0:kk1]
                        R[kk0:kk1,k0:k1]   =  Skk.T * FCross(Sk,f[i])
                        R[kk0:kk1,k0:k1]   += Skk.T * Tkf

        self.a = a
        self.v = v
        self.f = f
        self.Y = Y
        self.Ycrb = Ycrb
        self.Yvx = Yvx

        return R

# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# --- UNIT TEST ---
if __name__ == '__main__':
    np.random.seed(0)

    robot = RobotWrapper('/home/nmansard/src/pinocchio/pinocchio/models/romeo/urdf/romeo.urdf',
                         [ '/home/nmansard/src/pinocchio/pinocchio/models/romeo/', ],
                         pin.JointModelFreeFlyer()
                         )
    q  = rand(robot.model.nq); q[3:7] /= norm(q[3:7])
    vq = rand(robot.model.nv)
    
    # --- HESSIAN ---
    # --- HESSIAN ---
    # --- HESSIAN ---
    H = hessian(robot,q)

    # Compute the Hessian matrix H using RNEA, so that acor = v*H*v
    vq1 = vq*0
    Htrue = np.zeros([6,robot.model.nv,robot.model.nv])
    for joint_i in range(1,robot.model.njoints):
        for joint_j in ancestors(joint_i)[:-1]:
            for i in iv(joint_i):
                for j in iv(joint_j):
                    vq1 *= 0
                    vq1[i] = vq1[j] = 1.0
                    pin.computeAllTerms(robot.model,robot.data,q,vq1)
                    Htrue[:,i,j] = (robot.data.oMi[joint_i]*robot.data.a[joint_i]).vector.T
    
    print('Check hessian = \t\t', norm(H-Htrue))    

    # --- dCRBA ---
    # --- dCRBA ---
    # --- dCRBA ---

    dcrba = DCRBA(robot)
    dcrba.pre(q)
    Mp = dcrba()

    # --- Validate dM/dq by finite diff
    dM = np.zeros([robot.model.nv,]*3)
    eps = 1e-6
    dq = zero(robot.model.nv)
    
    for diff in range(robot.model.nv):
    
        dM[:,:,diff] = -pin.crba(robot.model,robot.data,q)
    
        dq *=0; dq[diff] = eps
        qdq = pin.integrate(robot.model,q,dq)
    
        dM[:,:,diff] += pin.crba(robot.model,robot.data,qdq)
    
    dM /= eps
    
    print('Check dCRBA = \t\t\t', max([ norm(Mp[:,:,diff]-dM[:,:,diff]) for diff in range(robot.model.nv) ]))
    
    
    # --- vRNEA ---
    # --- vRNEA ---
    # --- vRNEA ---
    
    vrnea = VRNEA(robot)
    Q = vrnea(q)

    # --- Compute C from rnea, for future check
    robot.model.gravity = pin.Motion.Zero()
    rnea0 = lambda q,vq: pin.nle(robot.model,robot.data,q,vq)
    vq1 = vq*0
    C = np.zeros([robot.model.nv,]*3)
    for i in range(robot.model.nv):
        vq1 *= 0; vq1[i] = 1
        C[:,i,i] = rnea0(q,vq1).T

    for i in range(robot.model.nv):
        for j in range(robot.model.nv):
            if i==j: continue
            vq1 *= 0
            vq1[i] = vq1[j] = 1.0
            C[:,i,j] = (rnea0(q,vq1).T-C[:,i,i]-C[:,j,j]) /2

    print("Check d/dv rnea = \t\t",norm(quad(Q,vq)-rnea0(q,vq)))
    print("Check C  = Q+Q.T = \t\t", norm((Q+Q.transpose(0,2,1))/2-C))
    print("Check dM = C+C.T /2 \t\t", norm( Mp - (C+C.transpose(1,0,2)) ))
    print("Check dM = Q+Q.T+Q.T+Q.T /2 \t", norm( Mp - 
                                            (Q+Q.transpose(0,2,1)+Q.transpose(1,0,2)+Q.transpose(2,0,1))/2 ))

    # --- CORIOLIS
    # --- CORIOLIS
    # --- CORIOLIS
    coriolis = Coriolis(robot)
    C = coriolis(q,vq)
    print("Check coriolis \t\t\t",norm(C*vq-rnea0(q,vq)))

    # --- DRNEA
    # --- DRNEA
    # --- DRNEA
    drnea = DRNEA(robot)
    aq    = rand(robot.model.nv)
    R = drnea(q,vq,aq)

    NV = robot.model.nv
    Rd = zero([NV,NV])
    eps = 1e-8
    r0 = pin.rnea(robot.model,robot.data,q,vq,aq).copy()
    for i in range(NV):
        dq = zero(NV); dq[i]=eps
        qdq = pin.integrate(robot.model,q,dq)
        Rd[:,i] = (pin.rnea(robot.model,robot.data,qdq,vq,aq)-r0)/eps
    print("Check drnea    \t\t\t",norm(Rd-R))


    
