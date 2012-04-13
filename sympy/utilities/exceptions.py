"""
General SymPy exceptions and warnings.
"""

class SymPyDeprecationWarning(DeprecationWarning):
    r"""A warning for deprecated features of SymPy.

    This class is expected to be used with the warnings.warn function
    (note that one has to explicitly turn on deprecation warnings):

    >>> import warnings
    >>> from sympy.utilities.exceptions import SymPyDeprecationWarning
    >>> warnings.simplefilter("always", SymPyDeprecationWarning)
    >>> warnings.warn("Don't do this, it's deprecated", SymPyDeprecationWarning) #doctest:+SKIP
    __main__:1: SymPyDeprecationWarning: "Don't do this, it's deprecated"

    The recommended way to use this class is, however, by directly passing
    instances of it to warnings.warn:

    >>> warnings.warn(SymPyDeprecationWarning("Don't do this, it's deprecated")) #doctest:+SKIP
    __main__:1: SymPyDeprecationWarning: "Don't do this, it's deprecated"

    To provide additional information, create an instance of this
    class in this way:

    >>> SymPyDeprecationWarning(feature="such and such",
    ...     last_supported_version="1.2.3",
    ...     useinstead="this other feature")
    SymPyDeprecationWarning("The feature such and such is deprecated.
    It will be last supported in SymPy version 1.2.3.  Use this other
    feature instead.")

    Either (or both) of the arguments last_supported_version and
    useinstead can be omitted.  In this case the corresponding
    sentence will not be shown:

    >>> SymPyDeprecationWarning(feature="such and such",
    ...     useinstead="this other feature")
    SymPyDeprecationWarning("The feature such and such is deprecated.
    Use this other feature instead.")

    You can still provide the argument value.  If it is a string, it
    will be appended to the end of the message:

    >>> SymPyDeprecationWarning(feature="such and such",
    ...     useinstead="this other feature",
    ...     value="Contact the developers for further information.")
    SymPyDeprecationWarning("The feature such and such is deprecated.
    Use this other feature instead.  Contact the developers for
    further information.")

    If, however, the argument value does not hold a string, a string
    representation of the object will be appended to the message:

    >>> SymPyDeprecationWarning(feature="such and such",
    ...     useinstead="this other feature",
    ...     value=[1,2,3])
    SymPyDeprecationWarning("The feature such and such is deprecated.
    Use this other feature instead.  ([1, 2, 3])")

    To mark a function as deprecated, you can use the decorator
    @deprecated.

    See Also
    ========
    sympy.core.decorators.deprecated
    """

    def __init__(self, value=None, feature=None, last_supported_version=None,
                 useinstead=None):
        self.fullMessage=""

        if feature:
            self.fullMessage = "The feature %s is deprecated." % feature

            if last_supported_version:
                self.fullMessage += "  It will be last supported in SymPy version %s." % last_supported_version
            if useinstead:
                self.fullMessage += "  Use %s instead." % useinstead

        if value:
            if not isinstance(value, str):
                value = "(%s)" % repr(value)
        else:
            value = ""

        if self.fullMessage:
            if value:
                self.fullMessage += "  " + value
        else:
            self.fullMessage = value

    def __str__(self):
        return "SymPyDeprecationWarning(\"%s\")" % self.fullMessage
