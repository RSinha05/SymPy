import random
from sympy import symbols, Derivative
from sympy.printing.torch import torch_code
from sympy import (eye, MatrixSymbol, Matrix)
from sympy.tensor.array import NDimArray
from sympy.codegen.array_utils import (
        CodegenArrayTensorProduct, CodegenArrayElementwiseAdd,
        CodegenArrayPermuteDims, CodegenArrayDiagonal, _CodegenArrayAbstract)
from sympy.utilities.lambdify import lambdify
from sympy.core.relational import Eq, Ne, Ge, Gt, Le, Lt
from sympy.functions import \
    Abs, ceiling, exp, floor, sign, sin, asin, cos, \
    acos, tan, atan, cosh, acosh, sinh, asinh, tanh, atanh, \
    re, im, arg, erf, loggamma, sqrt
from sympy.testing.pytest import skip
from sympy.external import import_module
from sympy.printing.torch import TorchPrinter

torch = import_module("torch")

M = MatrixSymbol("M", 3, 3)
N = MatrixSymbol("N", 3, 3)
P = MatrixSymbol("P", 3, 3)
Q = MatrixSymbol("Q", 3, 3)


x, y, z, t = symbols("x y z t")

if torch is not None:
    llo = [[j for j in range(i, i+3)] for i in range(0, 9, 3)]
    m3x3 = torch.tensor(llo)
    m3x3sympy = Matrix(llo)


def _compare_torch_matrix(variables, expr):
    f = lambdify(variables, expr, 'torch')

    random_matrices = [Matrix([[random.randint(0, 10) for k in
                       range(i.shape[1])] for j in range(i.shape[0])]) for i in variables]

    random_variables = [eval(torch_code(i)) for i in random_matrices]

    r = f(*random_variables)
    e = expr.subs({k: v for k, v in zip(variables, random_matrices)}).doit()
    if isinstance(e, _CodegenArrayAbstract):
        e = e.doit()
    if e.is_Matrix or isinstance(e, NDimArray):
        e = torch.FloatTensor(e.tolist())
    assert (r == e).all()


def _compare_torch_scalar(variables, expr, rng):
    f = lambdify(variables, expr, 'torch')
    rvs = [rng() for v in variables]
    t_rvs = [eval(torch_code(i)) for i in rvs]
    r = f(*torch.tensor(t_rvs))
    e = expr.subs({k: v for k, v in zip(variables, rvs)}).evalf().doit()
    assert abs(r - e) < 10 ** -6


def _compare_torch_relational(variables, expr, rng=lambda: random.randint(0, 10)):
    f = lambdify(variables, expr, 'torch')
    rvs = [rng() for v in variables]
    t_rvs = [eval(torch_code(i)) for i in rvs]
    r = f(*torch.tensor(t_rvs))
    e = expr.subs({k: v for k, v in zip(variables, rvs)}).doit()
    assert r.item() == e


def test_torch_math():
    if not torch:
        skip("Torch not installed")

    ma = torch.tensor([[1, 2, -3, -4]])

    expr = Abs(x)
    assert torch_code(expr) == "torch.abs(x)"
    f = lambdify(x, expr, 'torch')
    y = f(ma)
    c = torch.abs(ma)
    assert (y == c).all()

    expr = sign(x)
    assert torch_code(expr) == "torch.sign(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.randint(0, 10))

    expr = ceiling(x)
    assert torch_code(expr) == "torch.ceil(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = floor(x)
    assert torch_code(expr) == "torch.floor(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = exp(x)
    assert torch_code(expr) == "torch.exp(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = sqrt(x)
    assert torch_code(expr) == "torch.sqrt(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = x ** 4
    assert torch_code(expr) == "torch.pow(x, 4)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = cos(x)
    assert torch_code(expr) == "torch.cos(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = acos(x)
    assert torch_code(expr) == "torch.acos(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.uniform(0, 0.95))

    expr = sin(x)
    assert torch_code(expr) == "torch.sin(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = asin(x)
    assert torch_code(expr) == "torch.asin(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = tan(x)
    assert torch_code(expr) == "torch.tan(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = atan(x)
    assert torch_code(expr) == "torch.atan(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    # ToDo: Implement torch Known functions handler for multiple inputs
    # expr = atan2(y, x)
    # assert torch_code(expr) == "torch.atan2(y, x)"
    # _compare_torch_scalar((y, x), expr, rng=lambda: random.random())

    expr = cosh(x)
    assert torch_code(expr) == "torch.cosh(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = acosh(x)
    assert torch_code(expr) == "torch.acosh(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.uniform(1, 2))

    expr = sinh(x)
    assert torch_code(expr) == "torch.sinh(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.uniform(1, 2))

    expr = asinh(x)
    assert torch_code(expr) == "torch.asinh(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.uniform(1, 2))

    expr = tanh(x)
    assert torch_code(expr) == "torch.tanh(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.uniform(1, 2))

    expr = atanh(x)
    assert torch_code(expr) == "torch.atanh(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.uniform(-.5, .5))

    expr = erf(x)
    assert torch_code(expr) == "torch.erf(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())

    expr = loggamma(x)
    assert torch_code(expr) == "torch.lgamma(x)"
    _compare_torch_scalar((x,), expr, rng=lambda: random.random())


def test_torch_complexes():
    assert torch_code(re(x)) == "torch.real(x)"
    assert torch_code(im(x)) == "torch.imag(x)"
    assert torch_code(arg(x)) == "torch.angle(x)"


def test_torch_relational():
    if not torch:
        skip("Torch not installed")

    expr = Eq(x, y)
    assert torch_code(expr) == "torch.eq(x, y)"
    _compare_torch_relational((x, y), expr)

    expr = Ne(x, y)
    assert torch_code(expr) == "torch.ne(x, y)"
    _compare_torch_relational((x, y), expr)

    expr = Ge(x, y)
    assert torch_code(expr) == "torch.ge(x, y)"
    _compare_torch_relational((x, y), expr)

    expr = Gt(x, y)
    assert torch_code(expr) == "torch.gt(x, y)"
    _compare_torch_relational((x, y), expr)

    expr = Le(x, y)
    assert torch_code(expr) == "torch.le(x, y)"
    _compare_torch_relational((x, y), expr)

    expr = Lt(x, y)
    assert torch_code(expr) == "torch.lt(x, y)"
    _compare_torch_relational((x, y), expr)


def test_torch_matrix():
    if torch is None:
        skip("Torch not installed")

    expr = M
    assert torch_code(expr) == "M"
    f = lambdify((M,), expr, "torch")
    assert f(eye(3)) == eye(3)

    expr = M*N
    assert torch_code(expr) == "torch.matmul(M, N)"
    _compare_torch_matrix((M, N), expr)

    expr = M**3
    assert torch_code(expr) == "torch.mm(torch.mm(M, M), M)"
    _compare_torch_matrix((M,), expr)

    expr = M*N*P*Q
    assert torch_code(expr) == "torch.matmul(torch.matmul(torch.matmul(M, N), P), Q)"
    _compare_torch_matrix((M, N, P, Q), expr)


def test_codegen_extra():
    if not torch:
        skip("Torch not installed")

    M = MatrixSymbol("M", 2, 2)
    N = MatrixSymbol("N", 2, 2)
    P = MatrixSymbol("P", 2, 2)
    Q = MatrixSymbol("Q", 2, 2)
    ma = torch.tensor([[1, 2], [3, 4]])
    mb = torch.tensor([[1, -2], [-1, 3]])
    mc = torch.tensor([[2, 0], [1, 2]])
    md = torch.tensor([[1, -1], [4, 7]])

    cg = CodegenArrayTensorProduct(M, N)
    assert torch_code(cg) == 'torch.einsum("ab,cd", [M, N])'
    f = lambdify((M, N), cg, 'torch')
    y = f(ma, mb)
    c = torch.einsum("ij,kl", ma, mb)
    assert (y == c).all()

    cg = CodegenArrayElementwiseAdd(M, N)
    assert torch_code(cg) == 'torch.add(M, N)'
    f = lambdify((M, N), cg, 'torch')
    y = f(ma, mb)
    c = ma + mb
    assert (y == c).all()

    cg = CodegenArrayElementwiseAdd(M, N, P)
    assert torch_code(cg) == 'torch.add(torch.add(M, N), P)'
    f = lambdify((M, N, P), cg, 'torch')
    y = f(ma, mb, mc)
    c = ma + mb + mc
    assert (y == c).all()

    cg = CodegenArrayElementwiseAdd(M, N, P, Q)
    assert torch_code(cg) == 'torch.add(torch.add(torch.add(M, N), P), Q)'
    f = lambdify((M, N, P, Q), cg, 'torch')
    y = f(ma, mb, mc, md)
    c = ma + mb + mc + md
    assert (y == c).all()

    cg = CodegenArrayPermuteDims(M, [1, 0])
    assert torch_code(cg) == 'M.permute(1, 0)'
    f = lambdify((M,), cg, 'torch')
    y = f(ma)
    c = ma.T
    assert (y == c).all()

    cg = CodegenArrayPermuteDims(CodegenArrayTensorProduct(M, N), [1, 2, 3, 0])
    assert torch_code(cg) == 'torch.einsum("ab,cd", [M, N]).permute(1, 2, 3, 0)'
    f = lambdify((M, N), cg, 'torch')
    y = f(ma, mb)
    c = torch.einsum("ab,cd", ma, mb).permute(1, 2, 3, 0)
    assert (y == c).all()

    cg = CodegenArrayDiagonal(CodegenArrayTensorProduct(M, N), (1, 2))
    assert torch_code(cg) == 'torch.einsum("ab,bc->acb", [M, N])'
    f = lambdify((M, N), cg, 'torch')
    y = f(ma, mb)
    c = torch.einsum("ab,bc->acb", ma, mb)
    assert (y == c).all()


def test_torch_derivative():
    expr = Derivative(sin(x), x)
    assert torch_code(expr) == 'torch.autograd.grad(torch.sin(x), x)[0]'


def test_requires_grad():
    M = MatrixSymbol("M", 2, 2)
    N = MatrixSymbol("N", 2, 2)
    P = MatrixSymbol("P", 2, 2)

    ma = torch.tensor([[1.0, 2.0], [3.0, 4.0]], requires_grad=True)
    mb = torch.tensor([[1.0, -2.0], [-1.0, 3.0]], requires_grad=True)
    mc = torch.tensor([[2.0, 0.0], [1.0, 2.0]], requires_grad=True)

    cg = CodegenArrayElementwiseAdd(M, N, P)
    assert torch_code(cg) == 'torch.add(torch.add(M, N), P)'
    f = lambdify((M, N, P), cg, 'torch')
    y = f(ma, mb, mc)
    assert y.grad_fn is not None


def test_matrix_requires_grad():
    x, y, a, b, c, d = symbols('x y a b c d')
    X = x, y
    t = symbols('t')
    args = a, b, c, d

    lotka_volterra = Matrix([[a * x - b * x * y], [-c * y + d * x * y]])
    expr = lotka_volterra

    assert expr.is_Matrix

    printer = TorchPrinter(settings={'requires_grad': True})
    f = lambdify([t, X, args], expr, printer=printer, modules='torch')

    k_ = torch.tensor([1.5, 1.0, 3.0, 1.0], requires_grad=True).float()
    y_ = torch.tensor([10.0, 5.0], requires_grad=True).float()
    t_ = torch.tensor([0.0], requires_grad=True).float()

    r = f(t_, y_, k_)

    assert r.requires_grad is True
