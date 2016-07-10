from sympy import Symbol
from sympy.physics.continuum_mechanics.beam import Beam, PointLoad, DistributedLoad
from sympy.physics.mechanics import Point
from sympy.printing import sstr
from sympy.utilities.pytest import XFAIL

x = Symbol('x')


def test_Beam():
    E = Symbol('E')
    E_1 = Symbol('E_1')
    I = Symbol('I')
    I_1 = Symbol('I_1')
    b = Beam(1, E, I)
    assert b._length == 1
    assert b._elastic_modulus == E
    assert b._second_moment == I

    # Test the length setter
    b.length = 4
    assert b.length == 4

    # Test the E setter
    b.elastic_modulus = E_1
    assert b.elastic_modulus == E_1

    # Test the I setter
    b.second_moment = I_1
    assert b.second_moment is I_1

    # Test for all boundary conditions.
    bcs = b.apply_boundary_conditions(moment = [(0, 4), (4, 0)], deflection = [(0, 2)], slope = [(0, 1)])
    assert bcs == {'deflection': [(0, 2)], 'moment': [(0, 4), (4, 0)], 'slope': [(0, 1)]}

    # Test for moment boundary condition method
    m_bcs = b.apply_moment_boundary_conditions()
    assert m_bcs == bcs['moment']
    m_bcs = b.apply_moment_boundary_conditions((4, 3), (5, 0))
    assert m_bcs == [(0, 4), (4, 0), (4, 3), (5, 0)]

    # Test for slope boundary condition method
    s_bcs = b.apply_slope_boundary_conditions()
    assert s_bcs == bcs['slope']
    s_bcs = b.apply_slope_boundary_conditions((4, 3), (5, 0))
    assert s_bcs == [(0, 1), (4, 3), (5, 0)]

    # Test for deflection boundary condition method
    d_bcs = b.apply_deflection_boundary_conditions()
    assert d_bcs == bcs['deflection']
    d_bcs = b.apply_deflection_boundary_conditions((4, 3), (5, 0))
    assert d_bcs == [(0, 2), (4, 3), (5, 0)]

    # Test for updated boundary conditions
    bcs_new = b.boundary_conditions()
    assert bcs_new == {
                       'deflection': [(0, 2), (4, 3), (5, 0)],
                           'moment': [(0, 4), (4, 0), (4, 3), (5, 0)],
                            'slope': [(0, 1), (4, 3), (5, 0)]}

@XFAIL
def test_apply():
    E = Symbol('E')
    I = Symbol('I')
    b = Beam(4, E, I)
    Load_1 = PointLoad(location = 0, value = -3, moment = True)
    Load_2 = PointLoad(location = 2, value = 4)
    Load_3 = DistributedLoad(start = 3, order = 2, value = -2)
    p = b.apply(Load_1, Load_2, Load_3)
    q = -3*SingularityFunction(x, 0, -2) + 4*SingularityFunction(x, 2, -1) - 2*SingularityFunction(x, 3, 2)
    assert p == q


def test_PointLoad():
    P1 = Point('P1')
    P2 = Point('P2')
    Load_1 = PointLoad(location = P1, value = -4)
    assert Load_1.location == P1
    assert Load_1.value == -4
    assert Load_1.moment is False

    # Test the location setter
    Load_1.location = P2
    assert Load_1.location == P2

    # Test the value setter
    Load_1.value = 4
    assert Load_1.value == 4

    # Test the moment setter
    Load_1.moment = True
    assert Load_1.moment is True

    Load_2 = PointLoad(location = P1, value = 5, moment=True)
    assert Load_2.location == P1
    assert Load_2.value == 5
    assert Load_2.moment is True


def test_DistributedLoad():
    P1 = Point('P1')
    P2 = Point('P2')
    Load_1 = DistributedLoad(start = P1, order = 2, value = -4)
    assert Load_1.start == P1
    assert Load_1.order == 2
    assert Load_1.value == -4

    # Test the start setter
    Load_1.start = P2
    assert Load_1.start == P2

    # Test the order setter
    Load_1.order = 4
    assert Load_1.order == 4

    # Test the value setter
    Load_1.value = 4
    assert Load_1.value == 4
