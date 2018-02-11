from sympy.multipledispatch import dispatch, Dispatcher
from sympy.core import Basic, Expr, Function, Add, Mul, Pow, Dummy, Integer
from sympy import Min, Max, Set, sympify, Lambda, symbols, exp, log
from sympy.sets import imageset, Interval, FiniteSet, Union, ImageSet, ProductSet
from sympy.core.function import FunctionClass


_x, _y = symbols("x y")


@dispatch(Set, Set)
def add_sets(x, y):
    return ImageSet(Lambda((_x, _y), (_x+_y)), ProductSet(x, y))


@dispatch(Expr, Expr)
def add_sets(x, y):
    return x+y


@dispatch(Interval, Interval)
def add_sets(x, y):
    """
    Additions in interval arithmetic
    https://en.wikipedia.org/wiki/Interval_arithmetic
    """
    return Interval(x.start + y.start, x.end + y.end,
        x.left_open or y.left_open, x.right_open or y.right_open)


@dispatch(Expr, Expr)
def sub_sets(x, y):
    return x-y


@dispatch(Set, Set)
def sub_sets(x, y):
    return ImageSet(Lambda((_x, _y), (_x - _y)), ProductSet(x, y))


@dispatch(Interval, Interval)
def sub_sets(x, y):
    """
    Subtractions in interval arithmetic
    https://en.wikipedia.org/wiki/Interval_arithmetic
    """
    return Interval(x.start - y.end, x.end - y.start,
        x.left_open or y.right_open, x.right_open or y.left_open)


@dispatch(Set, Set)
def mul_sets(x, y):
    return ImageSet(Lambda((_x, _y), (_x * _y)), ProductSet(x, y))


@dispatch(Expr, Expr)
def mul_sets(x, y):
    return x*y


@dispatch(Interval, Interval)
def mul_sets(x, y):
    """
    Multiplications in interval arithmetic
    https://en.wikipedia.org/wiki/Interval_arithmetic
    """
    comvals = (
        (x.start * y.start, bool(x.left_open or y.left_open)),
        (x.start * y.end, bool(x.left_open or y.right_open)),
        (x.end * y.start, bool(x.right_open or y.left_open)),
        (x.end * y.end, bool(x.right_open or y.right_open)),
    )
    # TODO: handle symbolic intervals
    minval, minopen = min(comvals)
    maxval, maxopen = max(comvals)
    return Interval(
        minval,
        maxval,
        minopen,
        maxopen
    )
    return SetExpr(Interval(start, end))


@dispatch(Expr, Expr)
def div_sets(x, y):
    return x/y


@dispatch(Set, Set)
def div_sets(x, y):
    return ImageSet(Lambda((_x, _y), (_x / _y)), ProductSet(x, y))


@dispatch(Interval, Interval)
def div_sets(x, y):
    """
    Divisions in interval arithmetic
    https://en.wikipedia.org/wiki/Interval_arithmetic
    """
    if (y.start*y.end).is_negative:
        from sympy import oo
        return Interval(-oo, oo)
    return mul_sets(x, Interval(1/y.end, 1/y.start, y.right_open, y.left_open))


@dispatch(Set, Set)
def pow_sets(x, y):
    return ImageSet(Lambda((_x, _y), (_x ** _y)), ProductSet(x, y))


@dispatch(Expr, Expr)
def pow_sets(x, y):
    return x**y


@dispatch(Interval, Integer)
def pow_sets(x, y):
    """
    Powers in interval arithmetic
    https://en.wikipedia.org/wiki/Interval_arithmetic
    """
    exponent = sympify(exponent)
    if exponent.is_odd:
        return Interval(x.start**exponent, x.end**exponent, x.left_open, x.right_open)
    if exponent.is_even:
        if (x.start*x.end).is_negative:
            if -x.start > x.end:
                left_limit = x.start
                left_open = x.right_open
            else:
                left_limit = x.end
                left_open = x.left_open
            return Interval(S.Zero, left_limit ** exponent, S.Zero not in x, left_open)
        elif x.start.is_negative and x.end.is_negative:
            return Interval(x.end**exponent, x.start**exponent, x.right_open, x.left_open)
        else:
            return Interval(x.start**exponent, x.end**exponent, x.left_open, x.right_open)


@dispatch(FunctionClass, FiniteSet)
def function_sets(f, x):
    return FiniteSet(*[f(i) for i in x])

@dispatch(FunctionClass, Interval)
def function_sets(f, x):
    if f == exp:
        return Interval(exp(x.start), exp(x.end), x.left_open, x.right_open)
    elif f == log:
        return Interval(log(x.start), log(x.end), x.left_open, x.right_open)
    return ImageSet(Lambda(_x, f(_x)), x)

@dispatch(FunctionClass, Set)
def function_sets(f, x):
    return ImageSet(Lambda(_x, f(_x)), x)
