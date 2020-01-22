from sympy.core.singleton import S
from sympy.core.symbol import Symbol
from sympy.ntheory.modular import crt, crt1, crt2, solve_congruence, crt_cartesian
from sympy.utilities.pytest import raises


def test_crt():
    def mcrt(m, v, r, symmetric=False):
        assert crt(m, v, symmetric)[0] == r
        mm, e, s = crt1(m)
        assert crt2(m, v, mm, e, s, symmetric) == (r, mm)

    mcrt([2, 3, 5], [0, 0, 0], 0)
    mcrt([2, 3, 5], [1, 1, 1], 1)

    mcrt([2, 3, 5], [-1, -1, -1], -1, True)
    mcrt([2, 3, 5], [-1, -1, -1], 2*3*5 - 1, False)

    assert crt([656, 350], [811, 133], symmetric=True) == (-56917, 114800)


def test_modular():
    assert solve_congruence(*list(zip([3, 4, 2], [12, 35, 17]))) == (1719, 7140)
    assert solve_congruence(*list(zip([3, 4, 2], [12, 6, 17]))) is None
    assert solve_congruence(*list(zip([3, 4, 2], [13, 7, 17]))) == (172, 1547)
    assert solve_congruence(*list(zip([-10, -3, -15], [13, 7, 17]))) == (172, 1547)
    assert solve_congruence(*list(zip([-10, -3, 1, -15], [13, 7, 7, 17]))) is None
    assert solve_congruence(
        *list(zip([-10, -5, 2, -15], [13, 7, 7, 17]))) == (835, 1547)
    assert solve_congruence(
        *list(zip([-10, -5, 2, -15], [13, 7, 14, 17]))) == (2382, 3094)
    assert solve_congruence(
        *list(zip([-10, 2, 2, -15], [13, 7, 14, 17]))) == (2382, 3094)
    assert solve_congruence(*list(zip((1, 1, 2), (3, 2, 4)))) is None
    raises(
        ValueError, lambda: solve_congruence(*list(zip([3, 4, 2], [12.1, 35, 17]))))
    assert crt_cartesian([[3, 5], [3, 7]], [7, 11]) == [3, 73, 47, 40]
    assert crt_cartesian([[1, 5], [4, 7], [6, 8]], [6, 11, 13]) == [565, 697, 799, 73, 851, 125, 227, 359]
    assert crt_cartesian([[5, 3], [7, 9], [3, 7]], [7, 11, 13]) == [887, 579, 614, 306, 458, 150, 185, 878]
    assert crt_cartesian([[11, 51], [54, 72], [16, 38]], [67, 79, 43]) == [196053, 111365, 129790, 45102, 189259, 104571, 122996, 38308]
    raises(ValueError, lambda: crt_cartesian([[2, 3], [3, 4]], [4, 5, 6]))
    raises(ValueError, lambda: crt_cartesian([[4, 7], [3, 5]], []))
