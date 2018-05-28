from __future__ import print_function, division

from sympy import sqrt
from sympy.core import S, Symbol, I
from sympy.core.compatibility import range
from sympy.discrete import fft, ifft, ntt, intt
from sympy.utilities.pytest import raises


def test_fft_ifft():
    assert all(tf(ls) == ls for tf in (fft, ifft)
                            for ls in ([], [S(5)/3]))

    ls = list(range(6))
    fls = [15, -7*sqrt(2)/2 - 4 - sqrt(2)*I/2 + 2*I, 2 + 3*I,
             -4 + 7*sqrt(2)/2 - 2*I - sqrt(2)*I/2, -3,
             -4 + 7*sqrt(2)/2 + sqrt(2)*I/2 + 2*I,
              2 - 3*I, -7*sqrt(2)/2 - 4 - 2*I + sqrt(2)*I/2]

    assert fft(ls) == fls
    assert ifft(fls) == ls + [S.Zero]*2

    ls = [1 + 2*I, 3 + 4*I, 5 + 6*I]
    ifls = [S(9)/4 + 3*I, -7*I/4, S(3)/4 + I, -2 - I/4]

    assert ifft(ls) == ifls
    assert fft(ifls) == ls + [S.Zero]

    x = Symbol('x', real=True)
    raises(TypeError, lambda: fft(x))
    raises(ValueError, lambda: ifft([x, 2*x, 3*x**2, 4*x**3]))


def test_ntt_intt():
    # prime moduli of the form (m*2**k + 1), sequence length
    # should be a divisor of 2**k
    p = 7*17*2**23 + 1
    q = 2*500000003 + 1 # only for sequences of length 1 or 2
    r = 2*3*5*7 # composite modulus

    assert all(tf(ls, p) == ls for tf in (ntt, intt)
                                for ls in ([], [5]))

    ls = list(range(6))
    nls = [15, 801133602, 738493201, 334102277, 998244350, 849020224,
            259751156, 12232587]

    assert ntt(ls, p) == nls
    assert intt(nls, p) == ls + [0]*2

    ls = [1 + 2*I, 3 + 4*I, 5 + 6*I]
    x = Symbol('x', integer=True)

    raises(TypeError, lambda: ntt(x, p))
    raises(ValueError, lambda: intt([x, 2*x, 3*x**2, 4*x**3], p))
    raises(ValueError, lambda: intt(ls, p))
    raises(ValueError, lambda: ntt([1.2, 2.1, 3.5], p))
    raises(ValueError, lambda: ntt([3, 5, 6], q))
    raises(ValueError, lambda: ntt([4, 5, 7], r))

    assert ntt([1.0, 2.0, 3.0], p) == ntt([1, 2, 3], p)
