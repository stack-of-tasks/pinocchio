import pinocchio as se3
from pinocchio.utils import *

# --- SE3
R = rand([3,3]); [R,tmp,tmp] = npl.svd(R)
p = rand(3)
m = se3.SE3(R,p)
X = np.vstack( [ np.hstack([ R, skew(p)*R ]), np.hstack([ zero([3,3]), R ]) ])
assert( isapprox(m.action(),X))
M = np.vstack( [ np.hstack([R,p]), np.matrix('0 0 0 1',np.double) ] )
assert( isapprox(m.homogeneous(),M) )
m2 = se3.SE3.Random()
assert(isapprox( (m*m2).homogeneous(),m.homogeneous()*m2.homogeneous() ))
assert(isapprox( (~m).homogeneous(),npl.inv(m.homogeneous()) ))

p = rand(3)
assert(isapprox(m*p,m.rotation*p+m.translation))
assert(isapprox(m.actInv(p),m.rotation.T*p-m.rotation.T*m.translation))

p = np.vstack([p,1])
assert(isapprox(m*p,m.homogeneous()*p))
assert(isapprox(m.actInv(p),npl.inv(m.homogeneous())*p))

p = rand(6)
assert(isapprox(m*p,m.action()*p))
assert(isapprox(m.actInv(p),npl.inv(m.action())*p))

# --- Motion
assert(isapprox(se3.Motion.Zero().vector(),zero(6)))
v = se3.Motion.Random()
assert(isapprox((m*v).vector(),m.action()*v.vector()))
assert(isapprox((m.actInv(v)).vector(),npl.inv(m.action())*v.vector()))
vv = v.linear; vw = v.angular
assert(isapprox( v.vector(),np.vstack([vv,vw]) ))
assert(isapprox((v**v).vector(),zero(6)))

# --- Force ---
assert(isapprox(se3.Force.Zero().vector(),zero(6)))
f = se3.Force.Random()
ff = f.linear; ft = f.angular
assert(isapprox( f.vector(),np.vstack([ff,ft]) ))

assert(isapprox((m*f).vector(),npl.inv(m.action().T)*f.vector()))
assert(isapprox((m.actInv(f)).vector(),m.action().T*f.vector()))
f = se3.Force( np.vstack([ v.vector()[3:], v.vector()[:3] ]) )
assert(isapprox( (v**f).vector(),zero(6) ))

# --- Inertia ---
Y1 = se3.Inertia.Random()
Y2 = se3.Inertia.Random()
Y=Y1+Y2
assert(isapprox(Y1.matrix()+Y2.matrix(),Y.matrix()))
assert(isapprox((Y*v).vector(),Y.matrix()*v.vector()))
assert(isapprox( (m*Y).matrix(),m.inverse().action().T*Y.matrix()*m.inverse().action())) 
assert(isapprox( (m.actInv(Y)).matrix(),m.action().T*Y.matrix()*m.action())) 
