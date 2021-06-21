from sympy import sin, cos, Matrix, sqrt, pi
from sympy.core.symbol import symbols
from sympy.physics.mechanics import dynamicsymbols, Body, PinJoint, SlidingJoint
from sympy.physics.mechanics.joint import Joint
from sympy.physics.vector import Vector, ReferenceFrame
from sympy.testing.pytest import raises

t = dynamicsymbols._t


def test_Joint():
    parent = Body('parent')
    child = Body('child')
    raises(TypeError, lambda: Joint('J', parent, child))


def test_pinjoint():
    P = Body('P')
    C = Body('C')
    l, m = symbols('l m')
    theta, omega = dynamicsymbols('J_theta, J_omega')

    Pj = PinJoint('J', P, C )
    assert Pj._name == 'J'
    assert Pj.parent() == P
    assert Pj.child() == C
    assert Pj.coordinates() == [theta]
    assert Pj.speeds() == [omega]
    assert Pj.kdes() == [omega - theta.diff(t)]
    assert Pj._parent_axis == P.frame.x
    assert Pj._child_joint.pos_from(C.masscenter) == Vector(0)
    assert Pj._parent_joint.pos_from(P.masscenter) == Vector(0)
    assert Pj._parent_joint.pos_from(Pj._child_joint) == Vector(0)
    assert C.masscenter.pos_from(P.masscenter) == Vector(0)

    P1 = Body('P1')
    C1 = Body('C1')
    J1 = PinJoint('J1', P1, C1, parent_joint_pos= l * P1.frame.x, child_joint_pos= m * C1.frame.y,
        parent_axis = P1.frame.z)

    assert J1._parent_axis == P1.frame.z
    assert J1._child_joint.pos_from(C1.masscenter) == m * C1.frame.y
    assert J1._parent_joint.pos_from(P1.masscenter) == l * P1.frame.x
    assert J1._parent_joint.pos_from(J1._child_joint) == Vector(0)
    assert P1.masscenter.pos_from(C1.masscenter) == - l*P1.frame.x + m*C1.frame.y

def test_pin_joint_double_pendulum():
    q1, q2 = dynamicsymbols('q1 q2')
    u1, u2 = dynamicsymbols('u1 u2')
    m, l = symbols('m l')
    N = ReferenceFrame('N')
    A = ReferenceFrame('A')
    B = ReferenceFrame('B')
    C = Body('C', frame=N) #ceiling
    PartP = Body('P', frame=A, mass=m)
    PartR = Body('R', frame=B, mass=m)

    J1 = PinJoint('J1', C, PartP, speeds=u1, coordinates=q1, child_joint_pos=-l*A.x, \
        parent_axis=C.frame.z, child_axis=PartP.frame.z)
    J2 = PinJoint('J2', PartP, PartR, speeds=u2, coordinates=q2, child_joint_pos=-l*B.x, \
        parent_axis=PartP.frame.z, child_axis=PartR.frame.z)

    #Check orientation
    assert N.dcm(A) == Matrix([[cos(q1), -sin(q1), 0], [sin(q1), cos(q1), 0], [0, 0, 1]])
    assert A.dcm(B) == Matrix([[cos(q2), -sin(q2), 0], [sin(q2), cos(q2), 0], [0, 0, 1]])
    assert N.dcm(B).simplify() == Matrix([[cos(q1 + q2), -sin(q1 + q2), 0], \
        [sin(q1 + q2), cos(q1 + q2), 0], [0, 0, 1]])

    #Check Angular Velocity
    assert A.ang_vel_in(N) == u1 * N.z
    assert B.ang_vel_in(A) == u2 * A.z
    assert B.ang_vel_in(N) == u1 * N.z + u2 * A.z

    #Check kde
    assert J1.kdes() == [u1 - q1.diff(t)]
    assert J2.kdes() == [u2 - q2.diff(t)]

    #Check Linear Velocity
    assert PartP.masscenter.vel(N) == l*u1*A.y
    assert PartR.masscenter.vel(A) == l*u2*B.y

def test_pin_joint_chaos_pendulum():
    mA, mB, lA, lB, h = symbols('mA, mB, lA, lB, h')
    theta, phi, omega, alpha = dynamicsymbols('theta phi omega alpha')
    N = ReferenceFrame('N')
    A = ReferenceFrame('A')
    B = ReferenceFrame('B')
    lA = (lB - h / 2) / 2
    lC = (lB/2 + h/4)
    rod = Body('rod', frame=A, mass=mA)
    plate = Body('plate', mass=mB, frame=B)
    C = Body('C', frame=N)
    J1 = PinJoint('J1', C, rod, coordinates=theta, speeds=omega, child_joint_pos=lA*A.z,\
        parent_axis=N.y, child_axis=A.y)
    J2 = PinJoint('J2',rod, plate, coordinates=phi, speeds=alpha, parent_joint_pos=lC*A.z,\
        parent_axis=A.z, child_axis=B.z)

    #Check orientation
    assert A.dcm(N) == Matrix([[cos(theta), 0, -sin(theta)], [0, 1, 0], [sin(theta), 0, cos(theta)]])
    assert A.dcm(B) == Matrix([[cos(phi), -sin(phi), 0], [sin(phi), cos(phi), 0], [0, 0, 1]])
    assert B.dcm(N) == Matrix([[cos(phi)*cos(theta), sin(phi), -sin(theta)*cos(phi)], \
        [-sin(phi)*cos(theta), cos(phi), sin(phi)*sin(theta)], [sin(theta), 0, cos(theta)]])

    #Check Angular Velocity
    assert A.ang_vel_in(N) == omega * N.y
    assert A.ang_vel_in(B) == - alpha * A.z
    assert N.ang_vel_in(B) == - omega * N.y - alpha * A.z

    #Check kde
    assert J1.kdes() == [omega - theta.diff(t)]
    assert J2.kdes() == [alpha - phi.diff(t)]

    #Check pos of masscenters
    assert C.masscenter.pos_from(rod.masscenter) == lA*A.z
    assert rod.masscenter.pos_from(plate.masscenter) == - lC * A.z

    #Check Linear Velocities
    assert rod.masscenter.vel(N) == (h/4 - lB/2)*omega*A.x
    assert plate.masscenter.vel(N) == ((h/4 - lB/2)*omega + (h/4 + lB/2)*omega)*A.x

def test_pinjoint_arbitrary_axis():
    def generate_body():
        N = ReferenceFrame('N')
        A = ReferenceFrame('A')
        P = Body('P', frame=N)
        C = Body('C', frame=A)
        return N, A, P, C

    theta, omega = dynamicsymbols('J_theta, J_omega')

    #When the bodies are attached though masscenters but axess are opposite.
    N, A, P, C = generate_body()
    PinJoint('J', P, C, child_axis=-A.x)

    assert -A.x.angle_between(N.x) == 0
    assert A.dcm(N) == Matrix([[1, 0, 0], [0, cos(theta), sin(theta)], [0, -sin(theta), cos(theta)]])
    assert A.ang_vel_in(N) == omega*N.x
    assert C.masscenter.pos_from(P.masscenter) == 0
    assert C.masscenter.vel(N) == 0

    #When axes are different and parent joint is at masscenter but child joint is at a unit vector from
    #child masscenter.
    N, A, P, C = generate_body()
    PinJoint('J', P, C, child_axis=A.y, child_joint_pos=A.x)

    assert A.y.angle_between(N.x) == 0 #Axis are aligned
    assert A.dcm(N) == Matrix([[0, -cos(theta), -sin(theta)], [1, 0, 0], [0, -sin(theta), cos(theta)]])
    assert A.ang_vel_in(N) == omega*N.x
    assert C.masscenter.vel(N) == omega*A.z
    assert C.masscenter.pos_from(P.masscenter) == -A.x

    #Similar to previous case but wrt parent body
    N, A, P, C = generate_body()
    PinJoint('J', P, C, parent_axis=N.y, parent_joint_pos=N.x)

    assert N.y.angle_between(A.x) == 0 #Axis are aligned
    assert A.dcm(N) == Matrix([[0, 1, 0], [-cos(theta), 0, sin(theta)], [sin(theta), 0, cos(theta)]])
    assert A.ang_vel_in(N) == omega*N.y
    assert C.masscenter.vel(N).simplify() == - omega*N.z
    assert C.masscenter.pos_from(P.masscenter) == N.x

    #Both joint pos id defined but different axes
    N, A, P, C = generate_body()
    PinJoint('J', P, C, parent_joint_pos=N.x, child_joint_pos=A.x, child_axis=A.x+A.y)
    assert N.x.angle_between(A.x + A.y).simplify() == 0 #Axis are aligned
    assert A.dcm(N).simplify() == Matrix([[sqrt(2)/2, -sqrt(2)*cos(theta)/2, -sqrt(2)*sin(theta)/2], \
        [sqrt(2)/2, sqrt(2)*cos(theta)/2, sqrt(2)*sin(theta)/2], [0, -sin(theta), cos(theta)]])
    assert A.ang_vel_in(N) == omega*N.x
    assert C.masscenter.vel(N).simplify() == (omega * A.z)/sqrt(2)
    assert C.masscenter.pos_from(P.masscenter) == N.x - A.x

    N, A, P, C = generate_body()
    PinJoint('J', P, C, parent_joint_pos=N.x, child_joint_pos=A.x, child_axis=A.x+A.y-A.z)
    assert N.x.angle_between(A.x + A.y - A.z).simplify() == 0 #Axis are aligned
    assert A.dcm(N).simplify() == Matrix([[sqrt(3)/3, -sqrt(6)*sin(theta + pi/4)/3, sqrt(6)*cos(theta + pi/4)/3], \
        [sqrt(3)/3, sqrt(6)*cos(theta + pi/12)/3, sqrt(6)*sin(theta + pi/12)/3], \
        [-sqrt(3)/3, sqrt(6)*cos(theta + 5*pi/12)/3, sqrt(6)*sin(theta + 5*pi/12)/3]])
    assert A.ang_vel_in(N) == omega*N.x
    assert C.masscenter.vel(N).simplify() == (omega * A.y + omega * A.z)/sqrt(3)
    assert C.masscenter.pos_from(P.masscenter) == N.x - A.x

def test_slidingjoint():
    P = Body('P')
    C = Body('C')
    x, v = dynamicsymbols('S_x, S_v')
    S = SlidingJoint('S', P, C)
    assert S._name == 'S'
    assert S.parent() == P
    assert S.child() == C
    assert S.coordinates() == [x]
    assert S.speeds() == [v]
    assert S.kdes() == [v - x.diff(t)]
    assert S._parent_axis == P.frame.x
    assert S._child_axis == C.frame.x
    assert S._child_joint.pos_from(C.masscenter) == Vector(0)
    assert S._parent_joint.pos_from(P.masscenter) == Vector(0)
    assert S._parent_joint.pos_from(S._child_joint) == - x * P.frame.x

    l, m = symbols('l m')
    S = SlidingJoint('S', P, C, parent_joint_pos= l * P.frame.x, child_joint_pos= m * C.frame.y,
        parent_axis = P.frame.z)

    assert S._parent_axis == P.frame.z
    assert S._child_joint.pos_from(C.masscenter) == m * C.frame.y
    assert S._parent_joint.pos_from(P.masscenter) == l * P.frame.x
    assert S._parent_joint.pos_from(S._child_joint) == - x * P.frame.z
