from sympy import Symbol, exp, log
from sympy.calculus.singularities import (singularities, is_increasing,
        is_strictly_increasing, is_decreasing, is_strictly_decreasing, is_monotonic,
        order)
from sympy.sets import Interval, FiniteSet, EmptySet
from sympy import oo, S, I, sqrt, sin, cos, pi

from sympy.utilities.pytest import XFAIL

from sympy.abc import x, y, z
a = Symbol('a', negative=True)
b = Symbol('b', positive=True)


def test_singularities():
    x = Symbol('x')
    y = Symbol('y')

    assert singularities(x**2, x) == S.EmptySet
    assert singularities(x/(x**2 + 3*x + 2), x) == FiniteSet(-2, -1)
    assert singularities(1/(x**2 + 1), x) == FiniteSet(I, -I)
    assert singularities(x/(x**3 + 1), x) == FiniteSet(-1, (1 - sqrt(3)*I)/2,
                                                       (1 + sqrt(3)*I)/2)
    assert singularities(1/(y**2 + 2*I*y + 1), y) == FiniteSet(-I + sqrt(2)*I, -I - sqrt(2)*I)
    assert singularities(x**2/(x**2 - 4), x) == FiniteSet(-2, 2)


def test_singularities_non_rational():
    assert singularities(exp(1/z), z) == FiniteSet(0)
    assert singularities(log((z - 2)**2), z) == FiniteSet(2)
    assert singularities(exp(1/log(z + 1)), z) == FiniteSet(0)
    assert singularities(z*sin(1/z).rewrite(exp), z) == FiniteSet(0)
    assert singularities(exp(1/z)*z/(z**4 - 4), z) == \
        FiniteSet(0, -sqrt(2), sqrt(2), -I*sqrt(2), I*sqrt(2))
    assert singularities(sin(1/(z - 1)), z) == FiniteSet(1)
    assert singularities(cos(1/z) + sin(z**2), z) == FiniteSet(0)


def test_order():
    assert order(log(z - 1)/(z - 2)**5, z, 2) == -4
    assert order(z - 1, z, 1) == 1
    assert order(exp(z - 1), z, 1) == S.Infinity
    assert order(sin(z)/z, z, 0) == 0
    assert order(cos(z)/(z - pi/2), z, pi/2) == 0


def test_is_increasing():
    assert is_increasing(x**3 - 3*x**2 + 4*x, S.Reals)
    assert is_increasing(-x**2, Interval(-oo, 0))
    assert is_increasing(-x**2, Interval(0, oo)) is False
    assert is_increasing(4*x**3 - 6*x**2 - 72*x + 30, Interval(-2, 3)) is False
    assert is_increasing(x**2 + y, Interval(1, oo), x) is True
    assert is_increasing(-x**2*a, Interval(1, oo), x) is True
    assert is_increasing(1) is True


def test_is_strictly_increasing():
    assert is_strictly_increasing(4*x**3 - 6*x**2 - 72*x + 30, Interval.Ropen(-oo, -2))
    assert is_strictly_increasing(4*x**3 - 6*x**2 - 72*x + 30, Interval.Lopen(3, oo))
    assert is_strictly_increasing(4*x**3 - 6*x**2 - 72*x + 30, Interval.open(-2, 3)) is False
    assert is_strictly_increasing(-x**2, Interval(0, oo)) is False
    assert is_strictly_decreasing(1) is False


def test_is_decreasing():
    assert is_decreasing(1/(x**2 - 3*x), Interval.open(1.5, 3))
    assert is_decreasing(1/(x**2 - 3*x), Interval.Lopen(3, oo))
    assert is_decreasing(1/(x**2 - 3*x), Interval.Ropen(-oo, S(3)/2)) is False
    assert is_decreasing(-x**2, Interval(-oo, 0)) is False
    assert is_decreasing(-x**2*b, Interval(-oo, 0), x) is False


def test_is_strictly_decreasing():
    assert is_strictly_decreasing(1/(x**2 - 3*x), Interval.open(1.5, 3))
    assert is_strictly_decreasing(1/(x**2 - 3*x), Interval.Lopen(3, oo))
    assert is_strictly_decreasing(1/(x**2 - 3*x), Interval.Ropen(-oo, S(3)/2)) is False
    assert is_strictly_decreasing(-x**2, Interval(-oo, 0)) is False
    assert is_strictly_decreasing(1) is False


def test_is_monotonic():
    assert is_monotonic(1/(x**2 - 3*x), Interval.open(1.5, 3))
    assert is_monotonic(1/(x**2 - 3*x), Interval.Lopen(3, oo))
    assert is_monotonic(x**3 - 3*x**2 + 4*x, S.Reals)
    assert is_monotonic(-x**2, S.Reals) is False
    assert is_monotonic(x**2 + y + 1, Interval(1, 2), x) is True
