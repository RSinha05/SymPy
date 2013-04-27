"""Implementation of :class:`ComplexField` class. """

from sympy.core.numbers import Float, I

from sympy.polys.domains.field import Field
from sympy.polys.domains.simpledomain import SimpleDomain
from sympy.polys.domains.characteristiczero import CharacteristicZero
from sympy.polys.domains.mpelements import MPContext

from sympy.polys.polyerrors import DomainError, CoercionFailed

import math

class ComplexField(Field, CharacteristicZero, SimpleDomain):
    """Complex numbers up to the given precision. """

    rep = 'CC'

    is_Exact = False
    is_Numerical = True

    has_assoc_Ring = False
    has_assoc_Field = True

    _default_precision = 53

    @property
    def has_default_precision(self):
        return self.precision == self._default_precision

    @property
    def precision(self):
        return self._context.prec

    @property
    def dps(self):
        return self._context.dps

    @property
    def tolerance(self):
        return self._context.tolerance

    def __init__(self, prec=_default_precision, dps=None, tol=None):
        context = MPContext(prec, dps, tol)
        context._parent = self
        self._context = context

        self.dtype = context.mpc
        self.zero = self.dtype(0)
        self.one = self.dtype(1)

    def to_sympy(self, element):
        """Convert ``element`` to SymPy number. """
        return Float(element.real, self.dps) + I*Float(element.imag, self.dps)

    def from_sympy(self, expr):
        """Convert SymPy's number to ``dtype``. """
        number = expr.evalf(n=self.dps)
        real, imag = number.as_real_imag()

        if real.is_Number and imag.is_Number:
            return self.dtype(real, imag)
        else:
            raise CoercionFailed("expected complex number, got %s" % expr)

    def from_ZZ_python(self, element, base):
        return self.dtype(element)

    def from_QQ_python(self, element, base):
        return self.dtype(element.numerator) / element.denominator

    def from_ZZ_gmpy(self, element, base):
        return self.dtype(int(element))

    def from_QQ_gmpy(self, element, base):
        return self.dtype(int(element.numerator)) / int(element.denominator)

    def from_RealField(self, element, base):
        return self.dtype(element)

    def from_ComplexField(self, element, base):
        if self == base:
            return element
        else:
            return self.dtype(element)

    def get_ring(self):
        """Returns a ring associated with ``self``. """
        raise DomainError("there is no ring associated with %s" % self)

    def get_exact(self):
        """Returns an exact domain associated with ``self``. """
        raise DomainError("there is no exact domain associated with %s" % self)

    def gcd(self, a, b):
        """Returns GCD of ``a`` and ``b``. """
        return self.one

    def lcm(self, a, b):
        """Returns LCM of ``a`` and ``b``. """
        return a*b
