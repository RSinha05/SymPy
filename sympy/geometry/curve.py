"""Curves in 2-dimensional Euclidean space.

Contains
--------
Curve

"""

from sympy.core import sympify, C
from sympy.geometry.exceptions import GeometryError
from sympy.geometry.point import Point
from entity import GeometryEntity

class Curve(GeometryEntity):
    """A curve in space.

    A curve is defined by parametric functions for the coordinates, a
    parameter and the lower and upper bounds for the parameter value.

    Parameters
    ----------
    function : list of functions
    limits : 3-tuple
        Function parameter and lower and upper bounds.

    Attributes
    ----------
    functions
    parameter
    limits

    Raises
    ------
    GeometryError
        When `functions`
    ValueError
        When `limits` are specified incorrectly.


    Examples
    --------
    >>> from sympy import sin, cos, Symbol
    >>> from sympy.abc import t
    >>> from sympy.geometry import Curve
    >>> C = Curve([sin(t), cos(t)], (t, 0, 2))
    >>> C.functions
    [sin(t), cos(t)]
    >>> C.limits
    (t, 0, 2)
    >>> C.parameter
    t

    """

    def __new__(cls, function, limits):
        fun = sympify(function)
        if not fun:
            raise GeometryError("%s.__new__ don't know how to handle" % cls.__name__)
        if not isinstance(limits, (list, tuple)) or len(limits) != 3:
            raise ValueError("Limits argument has wrong syntax")
        return GeometryEntity.__new__(cls, fun, limits)

    @property
    def functions(self):
        """The functions specifying the curve.

        Returns
        -------
        functions : list of parameterized coordinate functions.

        Examples
        --------
        >>> from sympy.abc import t
        >>> from sympy.geometry import Curve
        >>> C = Curve([t, t**2], (t, 0, 2))
        >>> C.functions
        [t, t**2]

        """
        return self.__getitem__(0)

    def arbitrary_point(self, parameter_name='t'):
        """
        A parameterized point on the curve.

        Parameters
        ----------
        parameter_name : str, optional
            Default value is 't'.

        Returns
        -------
        arbitrary_point : Point

        See Also
        --------
        Point

        Examples
        --------
        >>> from sympy.abc import s
        >>> from sympy.geometry import Curve
        >>> C = Curve([s, s**2], (s, 0, 2))
        >>> C.arbitrary_point()
        Point(2*t, 4*t**2)
        >>> C.arbitrary_point('s')
        Point(2*s, 4*s**2)

        """
        if isinstance(parameter_name, C.Symbol):
            tnew = parameter_name
        elif parameter_name != self.parameter.name:
            tnew = C.Symbol(parameter_name, real=True)
        else:
            tnew = self.parameter
        t = self.parameter
        start, finish = self.limits[1:]
        tnew *= finish - start
        return Point(*[w.subs(t, tnew) for w in  self.functions])

    def plot_interval(self, parameter_name='t'):
        """The plot interval for the default geometric plot of the curve.

        Parameters
        ----------
        parameter_name : str, optional
            Default value is 't'.

        Returns
        -------
        plot_interval : list (plot interval)
            [parameter, lower_bound, upper_bound]

        Examples
        --------
        >>> from sympy import Curve, sin
        >>> from sympy.abc import x
        >>> Curve((x, sin(x)), (x, 1, 2)).plot_interval()
        [t, 1, 2]

        """
        t = C.Symbol(parameter_name, real=True)
        return [t] + list(self.limits[1:])

    @property
    def parameter(self):
        """The curve function variable.

        Returns
        -------
        parameter : sympy symbol

        Examples
        --------
        >>> from sympy.abc import t
        >>> from sympy.geometry import Curve
        >>> C = Curve([t, t**2], (t, 0, 2))
        >>> C.parameter
        t

        """
        return self.__getitem__(1)[0]

    @property
    def limits(self):
        """The limits for the curve.

        Returns
        -------
        limits : tuple
            Contains parameter and lower and upper limits.

        Examples
        --------
        >>> from sympy.abc import t
        >>> from sympy.geometry import Curve
        >>> C = Curve([t, t**3], (t, -2, 2))
        >>> C.limits
        (t, -2, 2)

        """
        return self.__getitem__(1)
