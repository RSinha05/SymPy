from sympy.stats import Poisson, Beta, Exponential, P, simplify, sqrt, exp, pi
from sympy.stats.rv import pspace, density
from sympy.stats.drv_types import PoissonDistribution
from sympy.stats.crv import Normal
from sympy.stats.joint_rv import JointPSpace
from sympy import Symbol, Eq, Ne

def test_density():
    x = Symbol('x')
    l = Symbol('l', positive=True)
    rate = Beta(l, 2, 3)
    X = Poisson(x, rate)
    assert isinstance(pspace(X), JointPSpace)
    assert density(X, Eq(rate, rate.symbol)) == PoissonDistribution(l)
    N1 = Normal('N1', 0, 1)
    N2 = Normal('N2', N1, 2)
    assert simplify(density(N2, Eq(N1, 1))(x)) == \
        sqrt(2)*exp(-(x - 1)**2/8)/(4*sqrt(pi))

def test_mix_expression():
    Y, E = Poisson('Y', 1), Exponential('E', 1)
    assert P(Eq(Y + E, 1)) == 0
    assert P(Ne(Y + E, 2)) == 1
    assert str(P(E + Y < 2, evaluate=False)) == """Integral(Sum(exp(-1)*Integral"""\
+"""(exp(-E)*DiracDelta(-_z + E + Y - 2), (E, 0, oo))/factorial(Y), (Y, 0, oo)), (_z, -oo, 0))"""
    assert str(P(E + Y > 2, evaluate=False)) == """Integral(Sum(exp(-1)*Integral"""\
+"""(exp(-E)*DiracDelta(-_z + E + Y - 2), (E, 0, oo))/factorial(Y), (Y, 0, oo)), (_z, 0, oo))"""
