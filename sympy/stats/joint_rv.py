"""
Joint Random Variables Module

See Also
========
sympy.stats.rv
sympy.stats.frv
sympy.stats.crv
sympy.stats.drv
"""

from __future__ import print_function, division

# __all__ = ['marginal_distribution']

from sympy import Basic, Lambda, sympify, Indexed, Symbol
from sympy.concrete.summations import Sum, summation
from sympy.integrals.integrals import Integral, integrate
from sympy.stats.rv import (ProductPSpace, NamedArgsMixin,
     ProductDomain, RandomSymbol)
from sympy.matrices import ImmutableMatrix

class JointPSpace(ProductPSpace):
    """
    Represents a joint probability space. Represented using symbols for
    each component and a distribution.
    """
    def __new__(cls, sym, dist):
        sym = sympify(sym)
        return Basic.__new__(cls, sym, dist)

    @property
    def set(self):
        return self.distribution.set

    @property
    def symbol(self):
        return self.args[0]

    @property
    def distribution(self):
        return self.args[1]

    @property
    def value(self):
        return JointRandomSymbol(self.symbol, self)

    @property
    def component_count(self):
        return len(self.distribution.set.args)

    def pdf(self, *args):
        return self.distribution(*args)

    def marginal_distribution(self, *indices):
        count = self.component_count
        orig = [Indexed(self.symbol, i) for i in range(count)]
        all_syms = [Symbol(str(i)) for i in orig]
        replace_dict = dict(zip(all_syms, orig))
        sym = [Symbol(str(Indexed(self.symbol, i))) for i in indices]
        limits = list([i,] for i in all_syms if i not in sym)
        index = 0
        for i in range(count):
            if i not in indices:
                limits[index].append(self.distribution.set.args[i])
                limits[index] = tuple(limits[index])
                index += 1
        limits = tuple(limits)
        if self.distribution.is_Continuous:
            f = Lambda(sym, integrate(self.distribution(*all_syms), limits))
        elif self.distribution.is_Discrete:
            f = Lambda(sym, summation(self.distribution(all_syms), limits))
        return f.xreplace(replace_dict)

    def where(self, condition):
        raise NotImplementedError()

    def compute_density(self, expr):
        raise NotImplementedError()

    def sample(self):
        raise NotImplementedError()

    def probability(self, condition):
        raise NotImplementedError()

class JointDistribution(Basic, NamedArgsMixin):
    """
    Represented by the random variables part of the joint distribution.
    Contains methods for PDF, CDF, sampling, marginal densities, etc.
    """

    _argnames = ('pdf', )

    def __new__(cls, *args):
        args = list(map(sympify, args))
        for i in range(len(args)):
            if isinstance(args[i], list):
                args[i] = ImmutableMatrix(args[i])
        return Basic.__new__(cls, *args)

    @property
    def domain(self):
        return ProductDomain(self.symbols)

    @property
    def pdf(self, *args):
        return self.density.args[1]

    def cdf(self, other):
        assert isinstance(other, dict)
        rvs = other.keys()
        _set = self.domain.set
        expr = self.pdf(tuple(i.args[0] for i in self.symbols))
        for i in len(other):
            if rvs[i].is_Continuous:
                density = Integral(expr, (rvs[i], _set[i].inf,
                    other[rvs[i]]))
            elif rvs[i].is_Discrete:
                density = Sum(expr, (rvs[i], _set[i].inf,
                    other[rvs[i]]))
        return density

    def __call__(self, *args):
        return self.pdf(*args)

class JointRandomSymbol(RandomSymbol):
    """
    Representation of random symbols with joint probability distributions
    to allow indexing."
    """
    def __getitem__(self, key):
        from sympy.stats.joint_rv import JointPSpace
        if isinstance(self.pspace, JointPSpace):
            return Indexed(self, key)

class JointDistributionHandmade(JointDistribution, NamedArgsMixin):

    _argnames = ('pdf',)
    is_Continuous = True

    @property
    def set(self):
        return self.args[1]

def marginal_distribution(rv, *indices):
    """
    Marginal distribution function of a joint random variable.

    Parameters:
    ==========

    rv: A random variable with a joint probability distribution.
    indices: component indices or the indexed random symbol
        for whom the joint distribution is to be calculated

    Returns:
    =======
    A Lambda expression n `sym`.

    Example:
    =======
    >>> from sympy.stats.crv_types import Normal
    >>> from sympy.stats.joint_rv import marginal_distribution
    >>> m = Normal('X', [1, 2], [[2, 1], [1, 2]])
    >>> marginal_distribution(m, m[0])(1)
    1/(2*sqrt(pi))

    """
    indices = list(indices)
    for i in range(len(indices)):
        if isinstance(indices[i], Indexed):
            indices[i] = indices[i].args[1]
    prob_space = rv.pspace
    if indices == ():
        raise ValueError(
            "At least one component for marginal density is needed.")
    if hasattr(prob_space.distribution, 'marginal_distribution'):
        return prob_space.distribution.marginal_distribution(indices, rv.symbol)
    return prob_space.marginal_distribution(*indices)
