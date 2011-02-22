from sympy import symbols, log, Real, nan, oo, I, pi, E, exp, Symbol, \
        LambertW, sqrt, Rational, sin, expand_log
from sympy.utilities.pytest import XFAIL

def test_exp():

    x, y = symbols('xy')

    k = Symbol('k', integer=True)

    assert exp(nan) == nan

    assert exp(oo) == oo
    assert exp(-oo) == 0

    assert exp(0) == 1
    assert exp(1) == E

    assert exp(pi*I/2) == I
    assert exp(pi*I) == -1
    assert exp(3*pi*I/2) == -I
    assert exp(2*pi*I) == 1

    assert exp(pi*I*2*k) == 1
    assert exp(pi*I*2*(k+Rational(1,2))) == -1
    assert exp(pi*I*2*(k+Rational(1,4))) == I
    assert exp(pi*I*2*(k+Rational(3,4))) == -I

    assert exp(log(x)) == x
    assert exp(2*log(x)) == x**2
    assert exp(pi*log(x)) == x**pi

    assert exp(17*log(x) + E*log(y)) == x**17 * y**E

    assert exp(x*log(x)) != x**x
    assert exp(sin(x)*log(x)) != x

    assert exp(x).as_base_exp() == (E, x)
    assert exp(-x).as_base_exp() == (E, -x)

def test_log():

    assert log(nan) == nan

    assert log(oo) == oo
    assert log(-oo) == oo

    assert log(0) == -oo

    assert log(1) == 0
    assert log(-1) == I*pi

    assert log(E) == 1
    assert log(-E).expand() == 1 + I*pi

    assert log(pi) == log(pi)
    assert log(-pi).expand() == log(pi) + I*pi

    assert log(17) == log(17)
    assert log(-17) == log(17) + I*pi

    assert log(I) == I*pi/2
    assert log(-I) == -I*pi/2

    assert log(17*I) == I*pi/2 + log(17)
    assert log(-17*I).expand() == -I*pi/2 + log(17)

    assert log(oo*I) == oo
    assert log(-oo*I) == oo

    assert exp(-log(3))**(-1) == 3

    assert log(1, 2) == 0
    assert log(2, 2) == 1
    assert log(3, 2) == log(3)/log(2)
    assert log(6, 2) == 1 + log(3)/log(2)
    assert log(6, 3) == 1 + log(2)/log(3)
    assert log(2**3, 2) == 3
    assert log(3**3, 3) == 3

    x, y = symbols('xy')

    assert log(x, exp(1)) == log(x)
    assert log(exp(x)) != x

    assert log(x) == log(x)
    assert log(x, exp(1)) == log(x)
    assert log(x*y) != log(x) + log(y)

    assert log(x, 2) == log(x)/log(2)
    assert log(E, 2) == 1/log(2)


    p, q = symbols('pq', positive=True)
    r = Symbol('r', real=1)

    assert log(p**2) != 2*log(p)
    assert log(p**2).expand() == 2*log(p)
    assert log(p**q) != q*log(p)
    assert log(exp(p)) == p
    assert log(p*q) != log(p) + log(q)
    assert log(p*q).expand() == log(p) + log(q)

    assert log(-exp(x)) != x + I*pi
    assert log(-exp(x)).expand() != x + I*pi
    assert log(-exp(r)).expand() == r + I*pi

    k = Symbol('k', positive=True)

    assert log(-x) == log(-x)
    assert log(-k) == log(-k)

def test_log_expand_complex():
    assert log(1+I).expand(complex=True) == log(2)/2 + I*pi/4
    assert log(1 - sqrt(2)).expand(complex=True) == log(sqrt(2)-1) + I*pi

def test_log_apply_evalf():
    value = (log(3)/log(2)-1).evalf()
    assert value.epsilon_eq(Real("0.58496250072115618145373"))

def test_lambertw():
    x = Symbol('x')
    assert LambertW(x) == LambertW(x)
    assert LambertW(0) == 0
    assert LambertW(E) == 1
    assert LambertW(-1/E) == -1
    assert LambertW(-log(2)/2) == -log(2)
    assert LambertW(oo) == oo
    assert LambertW(x**2).diff(x) == 2*LambertW(x**2)/x/(1+LambertW(x**2))
    assert LambertW(sqrt(2)).evalf(30).epsilon_eq(
        Real("0.701338383413663009202120278965",30),1e-29)

def test_log_expand():
    w = Symbol("w", positive=True)
    e = log(w**(log(5)/log(3)))
    assert e.expand() == log(5)/log(3) * log(w)
    x, y, z = symbols('xyz', positive=True)
    assert log(x*(y+z)).expand(mul=False) == log(x)+log(y+z)
    assert log(log(x**2)*log(y*z)).expand() == log(2*log(x)*log(y) + 2*log(x)*log(z))
    assert log(x**log(x**2)).expand(deep=False) == log(x)*log(x**2)
    assert log(x**log(x**2)).expand() == 2*log(x)**2
    assert (log(x*(y+z))*(x+y)),expand(mul=True, log=True) == y*log(x) + y*log(y + z) + z*log(x) + z*log(y + z)

def test_log_simplify():
    x = Symbol("x", positive=True)
    assert log(x**2).expand() == 2*log(x)
    assert expand_log(log(x**(2+log(2)))) == (2+log(2))*log(x)


def test_exp__as_base_exp():
    x,y = symbols('xy')

    assert exp(x)   .as_base_exp()  == (E, x)
    assert exp(2*x) .as_base_exp()  == (E, 2*x)
    assert exp(x*y) .as_base_exp()  == (E, x*y)

    # Pow( *expr.as_base_exp() ) == expr    invariant should hold
    assert E**x     == exp(x)
    assert E**(2*x) == exp(2*x)
    assert E**(x*y) == exp(x*y)

def test_infinity():
    y = Symbol('y')
    assert exp(I*y) != nan
    assert exp(I*oo) == nan
    assert exp(y*I*oo) == nan

def test_exp_expand():
    A,B,C = symbols('ABC', commutative=False)
    x,y,z = symbols('xyz')

    assert exp(A+B).expand() == exp(A+B)
    assert exp(A+B+C).expand() == exp(A+B+C)
    assert exp(x+y).expand() == exp(x)*exp(y)
    assert exp(x+y+z).expand() == exp(x)*exp(y)*exp(z)
