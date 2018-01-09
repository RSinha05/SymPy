# -*- coding: utf-8 -*-

from sympy import symbols, sin, cos, sqrt, Function
from sympy.core.compatibility import u_decode as u
from sympy.physics.vector import ReferenceFrame, dynamicsymbols
from sympy.physics.vector.printing import (VectorLatexPrinter, vpprint)
from sympy.physics.vector.printing import vlatex

a, b, c, d = symbols('a, b, c, d')
alpha, omega, beta, theta, q = dynamicsymbols('alpha, omega, beta, theta, q')
phi1, phi2, phi3 = dynamicsymbols('phi1, phi2, phi3')
theta1 = symbols('theta1')
A = ReferenceFrame('A')
N = ReferenceFrame('N')

"""
Expressions whose pretty-printing is tested here:

VectorStrPrinter

a ** 2 * N.x + b * N.y + c * sin(alpha) * N.z
alpha * N.x + sin(omega) * N.y + alpha * beta * N.z
a/b * N.x + (c+b)/a * N.y + c**2/b * N.z

Dyadic:
a ** 2 * (N.x | N.y) + b * (N.y | N.y) + c * sin(alpha) * (N.z | N.y)
alpha * (N.x | N.x) + sin(omega) * (N.y | N.z) + alpha * beta * (N.z | N.x)


VectorLatexPrinter

(a ** 2 + b / c) * A.x + sqrt(d) * A.y + cos(omega) * A.z
theta * A.x + omega * omega * A.y + (q * alpha) * A.z
sin(theta1) * A.x + cos(phi1) * cos(phi2) * A.y + cos(theta1 + phi3) * A.z
(a ** 2 + b / c) * N.x + sqrt(d) * N.y + cos(omega) * N.z

Custom unit vectors:
(a ** 2 + b / c) * _N.x + sqrt(d) * _N.y + cos(omega) * _N.z

Functions of time (dynamicsymbols):
omega.diff() * N.x
omega.diff() ** alpha * N.x

Dyadic:
a ** 2 * (N.x | N.y) + b * (N.y | N.y) + c * sin(alpha) * (N.z | N.y)
alpha * (N.x | N.x) + sin(omega) * (N.y | N.z) + alpha * beta * (N.z | N.x)

"""


def vpretty(expr):
    """ASCII pretty-printing"""
    return vpprint(expr, use_unicode=False, wrap_line=False)


def uvpretty(expr):
    """Unicode pretty-printing"""
    return vpprint(expr, use_unicode=True, wrap_line=False)


def test_vpretty_basic():
    """ASCII, Unicode basic testing"""

    # TODO : The unit vectors should print with subscripts but they just
    # print as `n_x` instead of making `x` a subscript with unicode.

    expr = a ** 2 * N.x + b * N.y + c * sin(alpha) * N.z
    ascii_str = """\
 2
a  n_x + b n_y + c*sin(alpha) n_z\
"""
    ucode_str = u("""\
 2
a  n_x + b n_y + c⋅sin(α) n_z\
""")
    assert vpretty(expr) == ascii_str
    assert uvpretty(expr) == ucode_str

    expr = alpha * N.x + sin(omega) * N.y + alpha * beta * N.z
    ascii_str = """\
alpha n_x + sin(omega) n_y + alpha*beta n_z\
"""
    ucode_str = u("""\
α n_x + sin(ω) n_y + α⋅β n_z\
""")
    assert vpretty(expr) == ascii_str
    assert uvpretty(expr) == ucode_str

    expr = a/b * N.x + (c+b)/a * N.y + c**2/b * N.z
    ascii_str = """\
                     2
a       b + c       c
- n_x + ----- n_y + -- n_z
b         a         b\
"""
    ucode_str = u("""\
                    2
a      b + c       c
─ n_x + ───── n_y + ── n_z
b        a         b\
""")
    assert vpretty(expr) == ascii_str
    assert uvpretty(expr) == ucode_str

    expr = alpha * N.x + sin(omega) * N.y + alpha / beta * N.z
    ascii_str = """\
                             alpha
alpha n_x + sin(omega) n_y + ----- n_z
                              beta\
"""
    ucode_str = u("""\
                    α
α nₓ + sin(ω) n_y + ─ n_z
                    β\
""")
    assert vpretty(expr) == ascii_str
    assert uvpretty(expr) == ucode_str


def test_latex_printer():
    r = Function('r')('t')
    assert VectorLatexPrinter().doprint(r ** 2) == "r^{2}"


def test_vector_latex():
    expr = (a ** 2 + b / c) * A.x + sqrt(d) * A.y + cos(omega) * A.z
    assert vlatex(expr) == (r'(a^{2} + \frac{b}{c})\mathbf{\hat{a}_x} + '
                            r'\sqrt{d}\mathbf{\hat{a}_y} + '
                            r'\operatorname{cos}\left(\omega\right)'
                            r'\mathbf{\hat{a}_z}')

    expr = theta * A.x + omega * omega * A.y + (q * alpha) * A.z
    assert vlatex(expr) == (r'\theta\mathbf{\hat{a}_x} + '
                            r'\omega^{2}\mathbf{\hat{a}_y} + '
                            r'\alpha q\mathbf{\hat{a}_z}')

    expr = (sin(theta1) * A.x +
            cos(phi1) * cos(phi2) * A.y +
            cos(theta1 + phi3) * A.z)
    assert vlatex(expr) == (r'\operatorname{sin}\left(\theta_{1}\right)'
                            r'\mathbf{\hat{a}_x} + \operatorname{cos}'
                            r'\left(\phi_{1}\right) \operatorname{cos}'
                            r'\left(\phi_{2}\right)\mathbf{\hat{a}_y} + '
                            r'\operatorname{cos}\left(\theta_{1} + '
                            r'\phi_{3}\right)\mathbf{\hat{a}_z}')

    expr = (a ** 2 + b / c) * N.x + sqrt(d) * N.y + cos(omega) * N.z
    assert vlatex(expr) == (r'(a^{2} + \frac{b}{c})\mathbf{\hat{n}_x} + '
                            r'\sqrt{d}\mathbf{\hat{n}_y} + '
                            r'\operatorname{cos}\left(\omega\right)'
                            r'\mathbf{\hat{n}_z}')

    # Try custom unit vectors.

    _N = ReferenceFrame('N', latexs=(r'\hat{i}', r'\hat{j}', r'\hat{k}'))

    expr = (a ** 2 + b / c) * _N.x + sqrt(d) * _N.y + cos(omega) * _N.z
    assert vlatex(expr) == (r'(a^{2} + \frac{b}{c})\hat{i} + '
                            r'\sqrt{d}\hat{j} + '
                            r'\operatorname{cos}\left(\omega\right)\hat{k}')


def test_vector_latex_with_functions():
    expr = omega.diff() * N.x
    assert vlatex(expr) == r'\dot{\omega}\mathbf{\hat{n}_x}'

    expr = omega.diff() ** alpha * N.x
    assert vlatex(expr) == (r'\dot{\omega}^{\alpha}'
                            r'\mathbf{\hat{n}_x}')


def test_dyadic_pretty_print():
    expr = a ** 2 * (N.x | N.y) + b * (N.y | N.y) + c * sin(alpha) * (N.z | N.y)
    ascii_str = """\
 2
a  n_x|n_y + b n_y|n_y + c*sin(alpha) n_z|n_y\
"""
    ucode_str = u("""\
 2
a  n_x⊗n_y + b n_y⊗n_y + c⋅sin(α) n_z⊗n_y\
""")
    assert vpretty(expr) == ascii_str
    assert uvpretty(expr) == ucode_str

    expr = (alpha * (N.x | N.x) + sin(omega) * (N.y | N.z) +
            alpha * beta * (N.z | N.x))
    ascii_str = u('alpha n_x|n_x + sin(omega) n_y|n_z + alpha*beta n_z|n_x')
    ucode_str = u('α n_x⊗n_x + sin(ω) n_y⊗n_z + α⋅β n_z⊗n_x')
    assert vpretty(expr) == ascii_str
    assert uvpretty(expr) == ucode_str


def test_dyadic_latex():
    expr1 = a ** 2 * (N.x | N.y) + b * (N.y | N.y) + c * sin(alpha) * (N.z | N.y)
    latex_str1 = (r'a^{2}\mathbf{\hat{n}_x}\otimes \mathbf{\hat{n}_y} + '
                  r'b\mathbf{\hat{n}_y}\otimes \mathbf{\hat{n}_y} + '
                  r'c \operatorname{sin}\left(\alpha\right)'
                  r'\mathbf{\hat{n}_z}\otimes \mathbf{\hat{n}_y}')

    assert vlatex(expr1) == latex_str1

    expr2 = (alpha * (N.x | N.x) + sin(omega) * (N.y | N.z) +
            alpha * beta * (N.z | N.x))
    latex_str2 = (r'\alpha\mathbf{\hat{n}_x}\otimes \mathbf{\hat{n}_x} + '
                 r'\operatorname{sin}\left(\omega\right)\mathbf{\hat{n}_y}'
                 r'\otimes \mathbf{\hat{n}_z} + '
                 r'\alpha \beta\mathbf{\hat{n}_z}\otimes \mathbf{\hat{n}_x}')
    assert vlatex(expr2) == latex_str2


def test_vlatex(): # vlatex is broken #12078
    from sympy.physics.vector import vlatex

    x = symbols('x')
    J = symbols('J')

    f = Function('f')
    g = Function('g')
    h = Function('h')

    expected = r'J \left(\frac{d}{d x} g{\left (x \right )} - \frac{d}{d x} h{\left (x \right )}\right)'

    expr = J*f(x).diff(x).subs(f(x), g(x)-h(x))

    assert vlatex(expr) == expected
