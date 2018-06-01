# CheatSheet: SE(3) operations

<div class="center">

\f$
\newcommand{\BIN}{\begin{bmatrix}}
\newcommand{\BOUT}{\end{bmatrix}}
\newcommand{\calR}{\mathcal{R}}
\newcommand{\calE}{\mathcal{E}}
\newcommand{\repr}{\cong}
\newcommand{\dpartial}[2]{\frac{\partial{#1}}{\partial{#2}}}
\newcommand{\ddpartial}[2]{\frac{\partial^2{#1}}{\partial{#2}^2}}

\newcommand{\aRb}{\ {}^{A}R_B}
\newcommand{\aMb}{\ {}^{A}M_B}
\newcommand{\amb}{\ {}^{A}m_B}
\newcommand{\apb}{{\ {}^{A}{AB}{}}}
\newcommand{\aXb}{\ {}^{A}X_B}

\newcommand{\bRa}{\ {}^{B}R_A}
\newcommand{\bMa}{\ {}^{B}M_A}
\newcommand{\bma}{\ {}^{B}m_A}
\newcommand{\bpa}{\ {}^{B}{BA}{}}
\newcommand{\bXa}{\ {}^{B}X_A}

\newcommand{\ap}{\ {}^{A}p}
\newcommand{\bp}{\ {}^{B}p}

\newcommand{\afs}{\ {}^{A}\phi}
\newcommand{\bfs}{\ {}^{B}\phi}
\newcommand{\af}{\ {}^{A}f}
\renewcommand{\bf}{\ {}^{B}f}
\newcommand{\an}{\ {}^{A}\tau}
\newcommand{\bn}{\ {}^{B}\tau}

\newcommand{\avs}{\ {}^{A}\nu}
\newcommand{\bvs}{\ {}^{B}\nu}
\newcommand{\w}{\omega}
\newcommand{\av}{\ {}^{A}v}
\newcommand{\bv}{\ {}^{B}v}
\newcommand{\aw}{\ {}^{A}\w}
\newcommand{\bw}{\ {}^{B}\w}

\newcommand{\aI}{\ {}^{A}I}
\newcommand{\bI}{\ {}^{B}I}
\newcommand{\cI}{\ {}^{C}I}
\newcommand{\aY}{\ {}^{A}Y}
\newcommand{\bY}{\ {}^{B}Y}
\newcommand{\cY}{\ {}^{c}Y}
\newcommand{\aXc}{\ {}^{A}X_C}
\newcommand{\aMc}{\ {}^{A}M_C}
\newcommand{\aRc}{\ {}^{A}R_C}
\newcommand{\apc}{\ {}^{A}{AC}{}}
\newcommand{\bXc}{\ {}^{B}X_C}
\newcommand{\bRc}{\ {}^{B}R_C}
\newcommand{\bMc}{\ {}^{B}M_C}
\newcommand{\bpc}{\ {}^{B}{BC}{}}
\f$

## Rigid transformation

\f$m : p \in \calE(3) \rightarrow m(p) \in E(3)\f$

Transformation from B to A:

\f$\amb : \bp \in \calR^3 \repr \calE(3) \ \rightarrow\ \ap = \amb(\bp) = \aMb\ \bp\f$

\f$ \ap = \aRb \bp +  \apb\f$

\f$\aMb = \BIN \aRb & \apb \\ 0 & 1 \BOUT \f$

Transformation from A to B:

\f$\bp = \aRb^T \ap + \bpa, \quad\textrm{with }\bpa = - \aRb^T \apb\f$

\f$\bMa = \BIN \aRb^T & - \aRb^T \apb \\ 0 & 1 \BOUT \f$

For Featherstone, \f$E = \bRa =\aRb^T\f$ and \f$r = \apb\f$. Then:

\f$\bMa = \BIN \bRa & -\bRa \apb \\ 0 & 1 \BOUT = \BIN E & -E r \\ 0 & 1 \BOUT \f$

\f$\aMb = \BIN \bRa^T & \apb \\ 0 & 1 \BOUT = \BIN E^T & r \\ 0 & 1 \BOUT \f$

## Composition

\f$ \aMb \bMc = \BIN \aRb \bRc & \apb +  \aRb \bpc \\ 0 & 1 \BOUT \f$

\f$ \aMb^{-1} \aMc = \BIN \aRb^T \aRc & \aRb^T (\apc - \apb) \\ 0 & 1 \BOUT \f$



## Motion Application

\f$\avs = \BIN \av \\ \aw \BOUT\f$

\f$\bvs = \bXa\avs\f$

\f$ \aXb =  \BIN \aRb & \apb_\times \aRb \\ 0 & \aRb \BOUT \f$

\f$ \aXb^{-1} = \bXa =  \BIN \aRb^T & -\aRb^T \apb_\times \\ 0 & \aRb^T \BOUT \f$

For Featherstone, \f$E = \bRa =\aRb^T\f$ and \f$r = \apb\f$. Then:

\f$ \bXa = \BIN \bRa & - \bRa \apb_\times \\ 0 & \bRa \BOUT = \BIN E & -E r_\times \\ 0 & E \BOUT\f$

\f$ \aXb = \BIN \bRa^T & \apb_\times \bRa^T \\ 0 & \bRa^T \BOUT = \BIN E^T & r_\times E^T \\ 0 & E^T \BOUT\f$

## Force Application

\f$\afs = \BIN \af \\ \an \BOUT\f$

\f$\bfs = \bXa^* \afs\f$

For any \f$\phi,\nu\f$, \f$\phi\dot\nu = \afs^T \avs = \bfs^T \bvs\f$ and then:

\f$\aXb^* = \aXb^{-T} = \BIN \aRb & 0 \\ \apb_\times \aRb & \aRb \BOUT\f$

(because \f$\apb_\times^T = - \apb_\times\f$).

\f$\aXb^{-*} = \bXa^* = \BIN \aRb^T & 0 \\ -\aRb^T \apb_\times  & \aRb^T \BOUT\f$

For Featherstone, \f$E = \bRa =\aRb^T\f$ and \f$r = \apb\f$. Then:

\f$\bXa^* = \BIN \bRa & 0 \\ -\bRa \apb_\times & \bRa \BOUT = \BIN E & 0 \\ - E r_\times & E \BOUT \f$

\f$\aXb^* = \BIN \bRa^T & 0 \\  \apb_\times \bRa^T & \bRa^T \BOUT = \BIN E^T & 0 \\ r_\times E^T & E^T \BOUT \f$

## Inertia
### Inertia application

\f$\aY: \avs \rightarrow \afs = \aY \avs\f$

Coordinate transform:

\f$\bY = \bXa^{*} \aY \bXa^{-1}\f$

since:

\f$\bfs = \bXa^* \bfs = \bXa^* \aI \aXb \bvs\f$

Cannonical form. The inertia about the center of mass \f$c\f$ is:

\f$\cY = \BIN m & 0 \\ 0 & \cI \BOUT\f$

Expressed in any non-centered coordinate system \f$A\f$:
\f$\aY = \aXc^* \cI \aXc^{-1} = \BIN m & m\ ^AAC_\times^T \\  m\ ^AAC_\times & \aI + m \apc_\times \apc\times^T \BOUT
\f$

Changing the coordinates system from \f$B\f$ to \f$A\f$:

\f$\aY = \aXb^* \bXc^* \cI \bXc^{-1} \aXb^{-1} \f$
\f$ = \BIN m & m [\apb + \aRb \bpc]_\times^T \\  m [\apb + \aRb \bpc]_\times & \aRb \bI \aRb^T - m [\apb + \aRb
\bpc]_\times^2 \BOUT\f$

Representing the spatial inertia in \f$B\f$ by the triplet \f$(m,\bpc,\bI)\f$, the expression in \f$A\f$ is:

\f$ \amb: \bY = (m,\bpc,\bI) \rightarrow \aY = (m,\apb+\aRb \bpc,\aRb \bI \aRb^T)\f$

Similarly, the inverse action is:

\f$ \amb^{-1}: \aY \rightarrow \bY = (m,\aRb^T(^AAC-\apb),\aRb^T\aI \aRb) \f$

Motion-to-force map:

\f$ Y \nu = \BIN m & mc_\times^T \\ mc_\times & I+mc_\times c_\times^T \BOUT \BIN v \\ \omega \BOUT
 = \BIN m v - mc \times \omega \\ mc \times v + I \omega - mc \times ( c\times \omega) \BOUT\f$

Nota: the square of the cross product is:
\f$\BIN x\\y\\z\BOUT_ \times^2 = \BIN 0&-z&y \\ z&0&-x \\ -y&x&0 \BOUT^2 = \BIN -y^2-z^2&xy&xz \\ xy&-x^2-z^2&yz \\
xz&yz&-x^2-y^2 \BOUT\f$
There is no computational interest in using it.

### Inertia addition

\f$ Y_p = \BIN m_p &  m_p  p_\times^T \\ m_p p_\times &  I_p + m_p  p_\times p_\times^T \BOUT\f$

\f$ Y_q = \BIN m_q &  m_q  q_\times^T \\ m_q q_\times &  I_q + m_q  q_\times q_\times^T \BOUT\f$




## Cross products

Motion-motion product:

\f$\nu_1 \times \nu_2 = \BIN v_1\\\omega_1\BOUT \times \BIN v_2\\\omega_2\BOUT = \BIN  v_1 \times \omega_2 + \omega_1 \times v_2 \\ \omega_1 \times \omega_2 \BOUT \f$

Motion-force product:

\f$\nu \times \phi =  \BIN v\\\omega\BOUT \times \BIN f\\ \tau \BOUT = \BIN  \omega \times f \\ \omega \times \tau + v \times f \BOUT \f$

A special form of the motion-force product is often used:

\f$\begin{align*}\nu \times (Y \nu) &= \nu \times \BIN mv - mc\times \omega \\ mc\times v + I \omega - mc\times(c\times \omega) \BOUT \\&= \BIN m \omega\times v - \omega\times(mc\times \omega) \\ \omega \times ( mc \times v) + \omega \times (I\omega) -\omega \times(c \times( mc\times \omega)) -v\times(mc \times \omega)\BOUT\end{align*}\f$

Setting \f$\beta=mc \times \omega\f$, this product can be written:

\f$\nu \times (Y \nu) = \BIN \omega \times (m v - \beta) \\ \omega \times( c \times (mv-\beta)+I\omega) - v \times \beta \BOUT\f$

This last form cost five \f$\times\f$, four \f$+\f$ and one \f$3\times3\f$ matrix-vector multiplication.

## Joint

We denote by \f$1\f$ the coordinate system attached to the parent (predecessor) body at the joint input, ad by \f$2\f$
the coordinate system attached to the (child) successor body at the joint output. We neglect the possible time
variation of the joint model (ie the bias velocity \f$\sigma = \nu(q,0)\f$ is null).

The joint geometry is expressed by the rigid transformation from the input to the ouput, parametrized by the joint
coordinate system \f$q \in \mathcal{Q}\f$:

\f$ ^2m_1 \repr \ ^2M_1(q)\f$

The joint velocity (i.e. the velocity of the child wrt. the parent in the child coordinate system) is:

\f$^2\nu_{12} = \nu_J(q,v_q) = \ ^2S(q) v_q \f$

where \f$^2S\f$ is the joint Jacobian (or constraint matrix) that define the motion subspace allowed by the joint, and
\f$v_q\f$ is the joint coordinate velocity (i.e. an element of the Lie algebra associated with the joint coordinate
manifold), which would be \f$v_q=\dot q\f$ when \f$\dot q\f$ exists.

The joint acceleration is:

\f$^2\alpha_{12} = S \dot v_q + c_J + \ ^2\nu_{1} \times \ ^2\nu_{12}\f$

where \f$c_J = \sum_{i=1}^{n_q} \dpartial{S}{q_i} \dot q_i\f$ (null in the usual cases) and \f$^2\nu_{1}\f$ is the
velocity of the parent body with respect to an absolute (Galilean) coordinate system

NB: The abosulte velocity
\f$\nu_{1}\f$ is also the relative velocity wrt. the Galilean coordinate system \f$\Omega\f$. The exhaustive notation
should be \f$\nu_{\Omega1}\f$ but \f$\nu_1\f$ is prefered for simplicity.

The joint calculations take as input the joint position \f$q\f$ and velocity \f$v_q\f$ and should output \f$^2M_1\f$,
\f$^2\nu_{12}\f$ and \f$^2c\f$ (this last vector being often a trivial \f$0_6\f$ vector). In addition, the joint model
should store the position of the joint input in the central coordinate system of the previous joint \f$^0m_1\f$ which is a constant value.

The joint integrator computes the exponential map associated with the joint manifold. The function inputs are the
initial position \f$q_0\f$, the velocity \f$v_q\f$  and the length of the integration interval \f$t\f$. It computes \f$q_t\f$ as:

\f$ q_t = q_0 + \int_0^t v_q dt\f$

For the simple vectorial case where \f$v_q=\dot q\f$, we have \f$q_t=q_0 + t v_q\f$. Written in the more general case of a Lie groupe, we have \f$q_t = q_0 exp(t v_q)\f$ where \f$exp\f$ denotes the exponential map (i.e. integration of a constant vector field from the Lie algebra into the Lie group). This integration only consider first order explicit Euler. More general integrators (e.g. Runge-Kutta in Lie groupes) remains to be written. Adequate references are welcome.

## RNEA

### Initialization
\f$^0\nu_0 = 0 ; \ ^0\alpha_0 = -g\f$

In the following, the coordinate system \f$i\f$ is attached to the output of the joint (child body), while \f$lambda(i)\f$ is the central coordinate system attached to the parent joint. The coordinated system associated with the joint input is denoted by \f$i_0\f$. The constant rigid transformation from \f$\lambda(i)\f$ to the joint input is then \f$^{\lambda(i)}M_{i_0}\f$.


### Forward loop
For each joint \f$i\f$, update the joint calculation \f$\mathbf j_i\f$.calc(\f$q,v_q\f$). This compute \f$\mathbf{j}.M = \ ^{\lambda(i)}M_{i_0}(q)\f$, \f$\mathbf{j}.\nu = \ ^i\nu_{{\lambda(i)}i}(q,v_q)\f$, \f$\mathbf{j}.S = \ ^iS(q)\f$  and \f$\mathbf{j}.c = \sum_{k=1}^{n_q} \dpartial{^iS}{q_k} \dot q_k\f$. Attached to the joint is also its placement in body \f$\lambda(i)\f$ denoted by \f$\mathbf{j}.M_0 =\ ^{\lambda(i)}M_{i_0}\f$. Then:

\f$^{\lambda(i)}M_i = \mathbf{j}.M_0 \ \mathbf{j}.M \f$

\f$^0M_i = \ ^0M_{\lambda(i)} \ ^{\lambda(i)}M_i\f$

\f$^i\nu_{i}= \ ^{\lambda(i)}X_i^{-1} \ ^{\lambda(i)}\nu_{{\lambda(i)}} + \mathbf{j}.\nu\f$

\f$^i\alpha_{i}= \ ^{\lambda(i)}X_i^{-1} \  ^{\lambda(i)}\alpha_{{\lambda(i)}} + \mathbf{j}.S \dot v_q + \mathbf{j}.c + \ ^i\nu_{i} \times  \mathbf{j}.\nu\f$

\f$^i\phi_i= \ ^iY_i \ ^i\alpha_i + \ ^i\nu_i \times \ ^iY_i \ ^i\nu_i - \ ^0X_i^{-*}\ ^0\phi_i^{ext}\f$

### Backward loop
For each joint \f$i\f$ from leaf to root, do:

\f$\tau_i = \mathbf{j}.S^T \ ^i\phi_i\f$

\f$^{\lambda(i)}\phi_{\lambda(i)} \ +\!\!= \ ^{\lambda(i)}X_i^{*} \ ^i\phi_i\f$

### Nota
It is more efficient to apply \f$X^{-1}\f$ than \f$X\f$. Similarly, it is more efficient to apply \f$X^{-*}\f$ than \f$X^*\f$. Therefore, it is better to store the transformations \f$^{\lambda(i)}m_i\f$ and \f$^0m_i\f$ than \f$^im_{\lambda(i)}\f$ and \f$^im_0\f$.

</div>
