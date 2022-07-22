#
# sympy.polys.matrices.linsolve module
#
# This module defines the _linsolve function which is the internal workhorse
# used by linsolve. This computes the solution of a system of linear equations
# using the SDM sparse matrix implementation in sympy.polys.matrices.sdm. This
# is a replacement for solve_lin_sys in sympy.polys.solvers which is
# inefficient for large sparse systems due to the use of a PolyRing with many
# generators:
#
#     https://github.com/sympy/sympy/issues/20857
#
# The implementation of _linsolve here handles:
#
# - Extracting the coefficients from the Expr/Eq input equations.
# - Constructing a domain and converting the coefficients to
#   that domain.
# - Using the SDM.rref, SDM.nullspace etc methods to generate the full
#   solution working with arithmetic only in the domain of the coefficients.
#
# The routines here are particularly designed to be efficient for large sparse
# systems of linear equations although as well as dense systems. It is
# possible that for some small dense systems solve_lin_sys which uses the
# dense matrix implementation DDM will be more efficient. With smaller systems
# though the bulk of the time is spent just preprocessing the inputs and the
# relative time spent in rref is too small to be noticeable.
#

from collections import defaultdict

from sympy.core.add import Add
from sympy.core.mul import Mul
from sympy.core.power import Pow
from sympy.core.singleton import S

from sympy.polys.constructor import construct_domain
from sympy.polys.solvers import PolyNonlinearError

from .sdm import (
    SDM,
    sdm_irref,
    sdm_particular_from_rref,
    sdm_nullspace_from_rref
)

from sympy.utilities.misc import filldedent


def _linsolve(eqs, syms, strict=True):

    """Solve a linear system of equations.

    Examples
    ========

    Solve a linear system with a unique solution:

    >>> from sympy import symbols, Eq
    >>> from sympy.polys.matrices.linsolve import _linsolve
    >>> x, y = symbols('x, y')
    >>> eqs = [Eq(x + y, 1), Eq(x - y, 2)]
    >>> _linsolve(eqs, [x, y])
    {x: 3/2, y: -1/2}

    In the case of underdetermined systems the solution will be expressed in
    terms of the unknown symbols that are unconstrained:

    >>> _linsolve([Eq(x + y, 0)], [x, y])
    {x: -y, y: y}

    """
    # Number of unknowns (columns in the non-augmented matrix)
    nsyms = len(syms)

    # Convert to sparse augmented matrix (len(eqs) x (nsyms+1))
    eqsdict, rhs = _linear_eq_to_dict(eqs, syms, strict)
    Aaug = sympy_dict_to_dm(eqsdict, rhs, syms)
    K = Aaug.domain

    # sdm_irref has issues with float matrices. This uses the ddm_rref()
    # function. When sdm_rref() can handle float matrices reasonably this
    # should be removed...
    if K.is_RealField or K.is_ComplexField:
        Aaug = Aaug.to_ddm().rref()[0].to_sdm()

    # Compute reduced-row echelon form (RREF)
    Arref, pivots, nzcols = sdm_irref(Aaug)

    # No solution:
    if pivots and pivots[-1] == nsyms:
        return None

    # Particular solution for non-homogeneous system:
    P = sdm_particular_from_rref(Arref, nsyms+1, pivots)

    # Nullspace - general solution to homogeneous system
    # Note: using nsyms not nsyms+1 to ignore last column
    V, nonpivots = sdm_nullspace_from_rref(Arref, K.one, nsyms, pivots, nzcols)

    # Collect together terms from particular and nullspace:
    sol = defaultdict(list)
    for i, v in P.items():
        sol[syms[i]].append(K.to_sympy(v))
    for npi, Vi in zip(nonpivots, V):
        sym = syms[npi]
        for i, v in Vi.items():
            sol[syms[i]].append(sym * K.to_sympy(v))

    # Use a single call to Add for each term:
    sol = {s: Add(*terms) for s, terms in sol.items()}

    # Fill in the zeros:
    zero = S.Zero
    for s in set(syms) - set(sol):
        sol[s] = zero

    # All done!
    return sol


def sympy_dict_to_dm(eqs_coeffs, eqs_rhs, syms):
    """Convert a system of dict equations to a sparse augmented matrix"""
    elems = set(eqs_rhs).union(*(e.values() for e in eqs_coeffs))
    K, elems_K = construct_domain(elems, field=True, extension=True)
    elem_map = dict(zip(elems, elems_K))
    neqs = len(eqs_coeffs)
    nsyms = len(syms)
    sym2index = dict(zip(syms, range(nsyms)))
    eqsdict = []
    for eq, rhs in zip(eqs_coeffs, eqs_rhs):
        eqdict = {sym2index[s]: elem_map[c] for s, c in eq.items()}
        if rhs:
            eqdict[nsyms] = - elem_map[rhs]
        if eqdict:
            eqsdict.append(eqdict)
    sdm_aug = SDM(enumerate(eqsdict), (neqs, nsyms+1), K)
    return sdm_aug


def _linear_eq_to_dict(eqs, syms, strict):
    """Convert a system Expr/Eq equations into dict form, returning
    the coefficient dictionaries and a list of syms-independent terms
    from each expression in ``eqs```. When ``strict`` is False terms
    that may share symbols in syms but are not generators in syms
    will be treated as constants, otherwise the PolyNonlinearError
    will still be raised.

    Examples
    ========

    >>> from sympy.polys.matrices.linsolve import _linear_eq_to_dict as F
    >>> from sympy.abc import x
    >>> F([2*x + 3], {x}, True)
    ([{x: 2}], [3])
    """
    coeffs = []
    ind = []
    symset = set(syms)
    for i, e in enumerate(eqs):
        c, d = _lin_eq2dict(e, symset, strict)
        coeffs.append(d)
        ind.append(c)
    return coeffs, ind


def _has_gen(self, s):
    # return True if self has a generator in set s
    if self in s:
        return True
    if self.is_Add or self.is_Mul:
        return any(_has_gen(a, s) for a in self.args)
    b, ce = self.as_base_exp()
    if ce != 1:
        if ce.is_Integer:
            return _has_gen(b, s)
        c, e = ce.as_coeff_Mul()
        if c != 1:  # if c == 1 we already tested at top
            return Pow(b, e/c.q, evaluate=False) in s
    return False


def _lin_eq2dict(a, symset, strict=True):
    """return (c, d) where c is the sym-independent part of ``a`` and
    ``d`` is an efficiently calculated dictionary mapping symbols to
    their coefficients. A PolyNonlinearError is raised if non-linearity
    is detected. Nonlinearity involving function, derivative or other
    non-Add, Mul, Pow object which depend on symbols given, can be
    ignored by using ``strict=False``

    The values in the dictionary will be non-zero.

    Examples
    ========

    >>> from sympy.polys.matrices.linsolve import _lin_eq2dict
    >>> from sympy.abc import x, y
    >>> _lin_eq2dict(x + 2*y + 3, {x, y})
    (3, {x: 1, y: 2})

    The following does not raise an error because ``x**2`` does not appear in
    ``x``:

    >>> strict = True
    >>> _lin_eq2dict(x*(x**2 + 1), {x**2}, strict)
    (x, {x**2: x})

    But the reverse raises an error because ``x`` is a generator
    of ``x**2``. The error can be supressed only if the nonlinear
    term does not contain a generator in the provided symbols. This
    allows functions or derivatives involving the generators to
    be considered as independent:

    >>> from sympy import cos
    >>> _lin_eq2dict(x*(cos(x) + 1), {x}, not strict)
    (0, {x: cos(x) + 1})


    See Also
    ========
    sympy.solvers.solveset.linear_coeffs
    """
    if a in symset:
        return S.Zero, {a: S.One}
    if a.is_Add:
        terms_list = defaultdict(list)
        coeff_list = []
        for ai in a.args:
            ci, ti = _lin_eq2dict(ai, symset, strict)
            coeff_list.append(ci)
            for mij, cij in ti.items():
                terms_list[mij].append(cij)
        coeff = Add(*coeff_list)
        terms = {sym: Add(*coeffs) for sym, coeffs in terms_list.items()}
        return coeff, terms
    if a.is_Mul:
        terms = terms_coeff = None
        coeff_list = []
        for ai in a.args:
            ci, ti = _lin_eq2dict(ai, symset, strict)
            if not ti:
                coeff_list.append(ci)
            elif terms is None:
                terms = ti
                terms_coeff = ci
            else:
                # since ti is not null and we already have
                # a term, this is a cross term
                raise PolyNonlinearError(filldedent('''
                    symbol-dependent cross-term'''))
        coeff = Mul._from_args(coeff_list)
        if terms is None:
            return coeff, {}
        else:
            terms = {sym: coeff * c for sym, c in terms.items()}
            return  coeff * terms_coeff, terms
    if a.is_Equality:
        (coeff, terms), (cR, tR) = [_lin_eq2dict(ai, symset, strict)
            for ai in a.args]
        # there were no nonlinear errors so now
        # cancellation is allowed
        coeff -= cR
        for k, v in tR.items():
            if k in terms:
                terms[k] -= v
            else:
                terms[k] = v
        # don't store coefficients of 0, however
        terms = {k: v for k, v in terms.items() if v}
        return coeff, terms
    if not a.has_xfree(symset):
        return a, {}
    if not _has_gen(a, symset):
        # no generator in symset
        if not strict:
            # treat it as constant
            return a, {}
        raise PolyNonlinearError(filldedent('''
                nonlinearity can be ignored
                using `strict=False`'''))
    # strict setting won't help in this case
    raise PolyNonlinearError('nonlinear in given symbols')
