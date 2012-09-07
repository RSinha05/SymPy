import random

from sympy.core import Basic, S, FiniteSet, Tuple
from sympy.core.compatibility import is_sequence
from sympy.utilities.iterables import (flatten, has_variety, minlex,
    has_dups, runs)
from sympy.polys.polytools import lcm
from sympy.matrices import zeros
from sympy.mpmath.libmp.libintmath import ifac

def _af_mul(*a, **kwargs):
    """
    Product of two or more permutations in array form, following the
    L to R convention: given args [A, B, C], B is applied to A and
    then C is applied to the result.

    Examples
    ========

    >>> from sympy.combinatorics.permutations import _af_mul, Permutation
    >>> Permutation.print_cyclic = False
    >>> a, b = [1, 0, 2], [0, 2, 1]
    >>> _af_mul(a, b)
    [1, 2, 0]
    >>> list(Permutation(a)*Permutation(b))
    [2, 0, 1]

    """
    m = len(a)
    if m == 2:
        a, b = a
        return [a[i] for i in b]
    rv = range(len(a[0]))
    for ai in a:
        rv = [rv[j] for j in ai]
    return rv

def _af_parity(pi):
    """
    Computes the parity of a permutation in array form.

    The parity of a permutation reflects the parity of the
    number of inversions in the permutation, i.e., the
    number of pairs of x and y such that x > y but p[x] < p[y].

    Examples
    ========

    >>> from sympy.combinatorics.permutations import _af_parity
    >>> _af_parity([0,1,2,3])
    0
    >>> _af_parity([3,2,0,1])
    1

    See Also
    ========

    Permutation
    """
    n = len(pi)
    a = [0] * n
    c = 0
    for j in range(n):
        if a[j] == 0:
            c += 1
            a[j] = 1
            i = j
            while pi[i] != j:
                i = pi[i]
                a[i] = 1
    return (n - c) % 2

def _af_invert(a):
    """
    Finds the inverse, ~A, of a permutation, A, given in array form.

    Examples
    ========

    >>> from sympy.combinatorics.permutations import _af_invert, _af_mul
    >>> A = [1, 2, 0, 3]
    >>> _af_invert(A)
    [2, 0, 1, 3]
    >>> _af_mul(_, A)
    [0, 1, 2, 3]

    See Also
    ========

    Permutation, __invert__
    """
    inv_form = [0] * len(a)
    for i, ai in enumerate(a):
        inv_form[ai] = i
    return inv_form

def _af_commutes_with(a, b):
    """
    Checks if the two permutations with array forms
    given by ``a`` and ``b`` commute.

    Examples
    ========

    >>> from sympy.combinatorics.permutations import _af_commutes_with
    >>> _af_commutes_with([1,2,0], [0,2,1])
    False

    See Also
    ========

    Permutation, commutes_with
    """
    return not any(a[b[i]] != b[a[i]] for i in range(len(a) - 1))

class Cycle(dict):
    """
    Wrapper around dict which provides the functionality of a disjoint cycle.

    A cycle shows the rule to use to move elements in a set to obtain
    a permutation. The Cycle class is more flexible that Permutation in
    that all elements need not be present in order to investigate how
    multiple cycles act in sequence:

    >>> from sympy.combinatorics.permutations import Perm, Cycle
    >>> a = Cycle(1, 2)

    A Cycle will automatically parse a cycle given as a tuple on the rhs:

    >>> print Cycle(1, 2)(2, 3)
    [(1, 3, 2)]

    The identity cycle, Cycle(), can be used to start a product:

    >>> print Cycle()(1, 2)(2,3)
    [(1, 3, 2)]

    The array form of a Cycle can be obtained with the as_list
    property. With no argument, the list will show elements 0 through
    the maximum encountered. If a larger (or smaller) range is desired
    it can be provided (but the Cycle cannot be truncated to a size
    smaller than the largest element that is out of place):

    >>> a.as_list()
    [0, 2, 1]
    >>> a.as_list(4)
    [0, 2, 1, 3]
    >>> a.as_list(2) # can't truncate
    [0, 2, 1]

    The array form can be used to instantiate a Permutation so other
    properties of the permutation can be investigated:

    >>> Perm(a.as_list(4)).transpositions()
    {(1, 2)}

    See Also
    ========

    Permutation
    """
    def __missing__(self, arg):
        """Return enter arg into dictionary and return arg."""
        self[arg] = arg
        return arg

    def __call__(self, *other):
        """Return product of cycles processed from R to L.

        Examples
        ========
        >>> from sympy.combinatorics.permutations import Cycle as C
        >>> from sympy.combinatorics.permutations import Permutation as Perm
        >>> print C(1, 2)(2, 3)
        [(1, 3, 2)]

        An instance of a Cycle will automatically parse list-like
        objects and Permutations that are on the right. It is more
        flexible than the Permutation in that all elements need not
        be present:

        >>> a = C(1, 2)
        >>> print a(2, 3)
        [(1, 3, 2)]
        >>> print a(2, 3)(4, 5)
        [(1, 3, 2), (4, 5)]

        """
        rv = Cycle(*other)
        for k, v in zip(self, [rv[self[k]] for k in self]):
            rv[k] = v
        return rv

    def as_list(self, size=None):
        """Return the cycles as an explicit list starting from 0 up
        to the greatr of the largest value in the cycles and size.

        Examples
        ========
        >>> from sympy.combinatorics.permutations import Cycle as C
        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = C(2, 3)(4, 5)
        >>> p.as_list()
        [0, 1, 3, 2, 5, 4]
        >>> p.as_list(10)
        [0, 1, 3, 2, 5, 4, 6, 7, 8, 9]
        >>> p.as_list(2) # can't truncate
        [0, 1, 3, 2, 5, 4]
        """
        if not self and not size:
            raise ValueError('must give size for empty Cycle')
        if size:
            big = max([i for i in self if self[i] != i])
            size = max(size, big + 1)
        else:
            size = max(size, max(self) + 1)
        return [self[i] for i in range(size)]

    def __repr__(self):
        reduced = Perm(self.as_list()).cyclic_form
        return str([tuple(c) for c in reduced])

    def __init__(self, *args):
        """Load up a Cycle instance with the values for the cycle.

        Examples
        ========
        >>> from sympy.combinatorics.permutations import Cycle as C
        >>> C(1, 2, 6)
        {1: 2, 2: 6, 6: 1}
        >>> print _
        [(1, 2, 6)]
        """

        if not args:
            return
        if len(args) == 1:
            if isinstance(args[0], Permutation):
                for c in args[0].cyclic_form:
                    self.update(self(*c))
                return
            elif isinstance(args[0], Cycle):
                for k, v in args[0].iteritems():
                    self[k] = v
                return
        args = [int(a) for a in args]
        if has_dups(args):
            raise ValueError('All elements must be unique in a cycle.')
        for i in range(-len(args), 0):
            self[args[i]] = args[i + 1]

    def copy(self):
        return Cycle(self)

class Permutation(Basic):
    """
    A permutation, alternatively known as an 'arrangement number' or 'ordering'
    is an arrangement of the elements of an ordered list into a one-to-one
    mapping with itself.

    >>> from sympy.combinatorics import Permutation
    >>> Permutation.print_cyclic = False
    >>> p = Permutation([0, 2, 1])
    >>> p(1)
    2
    >>> dict([(i, p(i)) for i in range(p.size)])
    {0: 0, 1: 2, 2: 1}

    A representation of a permutation as a product of permutation cycles is
    unique (up to the ordering of the cycles). An example of a cyclic
    decomposition is the permutation [3, 1, 0, 2] of the set [0, 1, 2, 3].
    This is denoted as [[1], [0, 3, 2]], corresponding to the disjoint
    permutation cycles [1] and [0, 3, 2]. We can choose the cyclic form as we
    want since the cycles are disjoint and can therefore be specified in any
    order and a rotation of a given cycle specifies the same cycle [1]_.
    Therefore, (320)(1), (203)(1), (032)(1), (1)(320), (1)(203), and (1)(032)
    all describe the same permutation.

    >>> Permutation([[1], [0, 3, 2]])
    Permutation([3, 1, 0, 2])
    >>> Permutation([[1], [3, 2, 0]])
    Permutation([3, 1, 0, 2])
    >>> Permutation([[2, 0, 3]])
    Permutation([3, 1, 0, 2])

    As the last example shows, cycles can be entered without the singletons.
    The abbreviated cyclic form for any permutation can be seen with the
    cyclic_form method:

    >>> p = _
    >>> p.cyclic_form
    [[0, 3, 2]]
    >>> p.full_cyclic_form
    [[0, 3, 2], [1]]

    Another notation that explicitly identifies the positions occupied by
    elements before and after application of a permutation on n elements uses a
    2xn matrix, where the first row is the identity permutation and the second
    row is the new arrangement [2]_. The Permutation class stores only the
    second row of the matrix.

    Any permutation is also a product of transpositions.

    >>> p.transpositions()
    {(0, 2), (0, 3)}

    Permutations are commonly denoted in lexicographic or transposition order.

    >>> p.array_form
    [3, 1, 0, 2]

    The number of permutations on a set of n elements is given by n! and is
    called the cardinality.

    >>> p.size
    4
    >>> p.cardinality
    24

    A given permutation has a rank among all the possible permutations of the
    same elements, but what that rank is depends on how the permutations are
    enumerated. (There are a number of different methods of doing so.) The
    lexicographic rank is given by the rank method:

    >>> p.rank()
    20
    >>> p.next_lex()
    Permutation([3, 1, 2, 0])
    >>> _.rank()
    21
    >>> p.unrank_lex(p.size, 0)
    Permutation([])

    The product of two permutations a and q is defined as their composition as
    functions, (p*q)(i) = p(q(i)) [6]_.

    >>> q = Permutation([0, 1, 3, 2])
    >>> list(p*q)
    [2, 1, 0, 3]

    The permutation can be 'applied' to any list-like object, not only
    Permutations:

    >>> p(['zero', 'one', 'three', 'two'])
    ['two', 'one', 'zero', 'three']
    >>> p('zo32')
    ['2', 'o', 'z', '3']

    See Also
    ========

    Cycle

    References
    ==========

    .. [1] Skiena, S. 'Permutations.' 1.1 in Implementing Discrete Mathematics
           Combinatorics and Graph Theory with Mathematica.  Reading, MA:
           Addison-Wesley, pp. 3-16, 1990.

    .. [2] Knuth, D. E. The Art of Computer Programming, Vol. 4: Combinatorial
           Algorithms, 1st ed. Reading, MA: Addison-Wesley, 2011.

    .. [3] Wendy Myrvold and Frank Ruskey. 2001. Ranking and unranking
           permutations in linear time. Inf. Process. Lett. 79, 6 (September 2001),
           281-284. DOI=10.1016/S0020-0190(01)00141-7

    .. [4] D. L. Kreher, D. R. Stinson 'Combinatorial Algorithms'
           CRC Press, 1999

    .. [5] Graham, R. L.; Knuth, D. E.; and Patashnik, O.
           Concrete Mathematics: A Foundation for Computer Science, 2nd ed.
           Reading, MA: Addison-Wesley, 1994.

    .. [6] http://en.wikipedia.org/wiki/Permutation#Product_and_inverse

    .. [7] http://en.wikipedia.org/wiki/Lehmer_code

    """

    is_Permutation = True

    _array_form = None
    _cyclic_form = None
    _size = None

    def __new__(cls, args, size=None):
        """
        Constructor for the Permutation object from a list or a
        list of lists in which all elements of the permutation may
        appear only once.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False

        Permutations entered in array-form are left unaltered:

        >>> Permutation([0, 2, 1])
        Permutation([0, 2, 1])

        Permutations entered in cyclic form are converted to array form;
        singletons need not be entered:

        >>> Permutation([[4, 5, 6], [0, 1]])
        Permutation([1, 0, 2, 3, 5, 6, 4])

        When printing, the unchanged tail of the permutation is not shown:

        >>> Permutation([1, 0, 2, 3, 4, 5])
        Permutation([1, 0])
        >>> _.array_form
        [1, 0, 2, 3, 4, 5]

        All manipulation of permutations assumes that the smallest element
        is 0; if a permutation is not entered with a 0, one will be added:

        >>> Permutation([1, 2])
        Permutation([])

        If a permutation is entered in cyclic form, it can be entered without
        singletons and the ``size`` specified so those values can be filled
        in, otherwise the array form will only extend to the maximum value
        in the cycles:

        >>> Permutation([[1, 4], [3, 5, 2]], size=10)
        Permutation([0, 4, 3, 5, 1, 2])
        >>> _.array_form
        [0, 4, 3, 5, 1, 2, 6, 7, 8, 9]
        """
        if size is not None:
            size = int(size)

        if isinstance(args, Perm):
            p = args
            if size is None or size == p.size:
                return args.copy()
            return Perm(p.array_form, size=size)

        if isinstance(args, Cycle):
            return Perm._af_new(args.as_list(size))

        if any(is_sequence(a) and not a for a in args) or \
           has_variety(is_sequence(a) for a in args):
            raise ValueError("Permutation argument must be a list of ints, "
                             "a list of lists, Permutation or Cycle.")


        # safe to assume args are valid
        args = list(args)

        is_cycle = args and is_sequence(args[0])

        # if there are n elements present, 0, 1, ..., n-1 should be present
        # unless a cycle notation has been provided. A 0 will be added
        # for convenience in case one wants to enter permutations where
        # counting starts from 1.

        temp = [int(i) for i in flatten(args)]
        if has_dups(temp):
            if is_cycle:
                raise ValueError('there were repeated elements; to resolve '
                'cycles use Cycle%s.' % ''.join([str(tuple(c)) for c in args]))
            else:
                raise ValueError('there were repeated elements.')
        temp = set(temp)

        if args and 0 not in temp:
            if is_cycle:
                if size is None:
                    args.append([0])
            else:
                args.insert(0, 0)
            temp.add(0)


        if not is_cycle and not size and \
            any(i not in temp for i in range(len(temp))):
            raise ValueError("Integers 0 through %s must be present "
                             "or size must be given." %
                             max(temp))

        if is_cycle:
            # it's not necessarily canonical so we won't store
            # it -- use the array form instead
            c = Cycle()
            for ci in args:
                c = c(*ci)
            aform = c.as_list()
        else:
            aform = list(args)
        if size and size > len(aform):
            # don't allow for truncation of permutation which
            # might split a cycle and lead to an invalid aform
            # but do allow the permutation size to be increased
            aform.extend(range(len(aform), size))
        size = len(aform)
        obj = Basic.__new__(cls, aform)
        obj._array_form = aform
        obj._size = size
        return obj

    @staticmethod
    def _af_new(perm):
        """A method to produce a Permutation object from a list;
        the list is bound to the _array_form attribute, so it must
        not be modified; this method is meant for internal use only;
        the list ``a`` is supposed to be generated as a temporary value
        in a method, so p = Perm._af_new(a) is the only object
        to hold a reference to ``a``::

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Perm
        >>> Perm.print_cyclic = False
        >>> a = [2,1,3,0]
        >>> p = Perm._af_new(a)
        >>> p
        Permutation([2, 1, 3, 0])

        """
        p = Basic.__new__(Perm, perm)
        p._array_form = perm
        p._size = len(perm)
        return p

    @property
    def array_form(self):
        """
        This is used to convert from cyclic notation to the
        canonical notation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([[2,0], [3,1]])
        >>> p.array_form
        [2, 3, 0, 1]
        >>> Permutation([[2,0,3,1]]).array_form
        [3, 2, 0, 1]
        >>> Permutation([2,0,3,1]).array_form
        [2, 0, 3, 1]
        >>> Permutation([[1, 2], [4, 5]]).array_form
        [0, 2, 1, 3, 5, 4]

        See Also
        ========

        cyclic_form
        """
        # watch that given list doesn't shadow the argument:
        # store a copy; return a copy
        if self._array_form is not None:
            return list(self._array_form)
        cycles = self.args[0]
        perm = range(self.size)
        for c in cycles:
            for i in range(len(c) - 1):
                perm[c[i]] = c[i + 1]
            perm[c[-1]] = c[0]
        self._array_form = perm[:]
        return self.array_form

    @property
    def cyclic_form(self):
        """
        This is used to convert to the cyclic notation
        from the canonical notation. Singletons are omitted.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([0, 3, 1, 2])
        >>> p.cyclic_form
        [[1, 3, 2]]
        >>> Permutation([1, 0, 2, 4, 3, 5]).cyclic_form
        [[0, 1], [3, 4]]

        See Also
        ========

        array_form, full_cyclic_form
        """
        if self._cyclic_form is not None:
            return list(self._cyclic_form)
        array_form = self.array_form
        unchecked = [True] * len(array_form)
        cyclic_form = []
        for i in range(len(array_form)):
            if unchecked[i]:
                cycle = []
                cycle.append(i)
                unchecked[i] = False
                j = i
                while unchecked[array_form[j]]:
                    j = array_form[j]
                    cycle.append(j)
                    unchecked[j] = False
                if len(cycle) > 1:
                    cyclic_form.append(cycle)
                    assert cycle == list(minlex(cycle))
        cyclic_form.sort()
        self._cyclic_form = cyclic_form[:]
        return cyclic_form

    @property
    def full_cyclic_form(self):
        """Return permutation in cyclic form including singletons.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation([0, 2, 1]).full_cyclic_form
        [[0], [1, 2]]
        """
        need = set(range(self.size)) - set(flatten(self.cyclic_form))
        rv = self.cyclic_form
        rv.extend([[i] for i in need])
        rv.sort()
        return rv

    @property
    def size(self):
        """
        Returns the number of elements in the permutation.

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation([[3, 2], [0, 1]]).size
        4

        See Also
        ========

        cardinality, length, order, rank
        """
        return self._size

    def support(self):
        """Return the elements in permutation, P, for which P[i] != i.

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> p = Permutation([[3, 2], [0, 1], [4]])
        >>> p.array_form
        [1, 0, 3, 2, 4]
        >>> p.support()
        [0, 1, 2, 3]
        """
        a = self.array_form
        return [i for i, e in enumerate(self.array_form) if a[i] != i]

    def __eq__(self, other):
        if not isinstance(other, self.func):
            return False
        return self.cyclic_form == other.cyclic_form

    def __add__(self, other):
        """
        Routine for addition of permutations by their inversion vectors.

        This is defined in terms of the Lehmer code of a
        permutation. The Lehmer code is nothing but the
        inversion vector of a permutation. In this scheme
        the identity permutation is like a zero element.
        See [1].

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> I = Permutation([0, 1, 2, 3])
        >>> a = Permutation([2, 1, 3, 0])
        >>> I + a == a
        True
        >>> a = Permutation([0, 3, 1, 2])
        >>> b = Permutation([2, 1, 0, 3])
        >>> a + b
        Permutation([2, 0, 1])

        See Also
        ========

        __sub__, inversion_vector

        References
        ==========

        1. http://en.wikipedia.org/wiki/Lehmer_code
        """
        n = self.size
        if n != other.size:
            raise ValueError("The permutations must be of equal size.")
        a = self.inversion_vector()
        b = other.inversion_vector()
        result_inv = [(a[i] + b[i]) % (n - i) for i in range(n - 1)]
        return Perm.from_inversion_vector(result_inv)

    def __sub__(self, other):
        """
        Routine for subtraction of permutations by their inversion vectors.

        The idea behind this is the same as in ``__add__``

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0, 1, 2, 3])
        >>> q = Permutation([2, 1, 3, 0])
        >>> q - p == q
        True

        See Also
        ========

        __add__, inversion_vector

        References
        ==========

        1. http://en.wikipedia.org/wiki/Lehmer_code
        """
        n = self.size
        if n != other.size:
            raise ValueError("The permutations must be of equal size.")
        a = self.inversion_vector()
        b = other.inversion_vector()
        result_inv = [(a[i] - b[i]) % (n - i) for i in range(n - 1)]
        return Perm.from_inversion_vector(result_inv)

    @staticmethod
    def lmul(*args):
        """
        Return product of permutations following L to R order.

        Examples
        ========

        >>> from sympy.combinatorics import Permutation as Perm
        >>> Perm.print_cyclic = False
        >>> a = Perm([1, 0, 2])
        >>> b = Perm([0, 2, 1])
        >>> a.cyclic_form, b.cyclic_form
        ([[0, 1]], [[1, 2]])
        >>> a*b
        Permutation([2, 0, 1])
        >>> Perm.lmul(a, b)
        Permutation([1, 2, 0])

        Notes
        =====

        All items in the sequence will be parsed by Permutation as
        necessary as long as the first item is a Permutation. For speed
        reasons this routine does not do any initial coersion since in
        the codebase, places where is used already had a Permutation
        on the left and handle the rhs operand as necessary, i.e.
        P*foo was changed to lmul(P, foo):

        >>> Perm.lmul(a, [0, 2, 1]) == Perm.lmul(a, b)
        True

        The reverse order of arguments will raise a TypeError.

        """
        rv = args[0]
        for i in range(1, len(args)):
            rv = args[i]*rv
        return rv

    def __rmul__(self, other):
        """This is needed to coerse other to Permutation in lmul."""
        return Perm(other)*self

    def __mul__(self, other):
        """
        Routine for multiplication of permutations following R to L order.

        Note
        ====

        a*b*c applies permutations in order c, b, a which is consistent with
        function notation abc(x) meaning a(b(c(x))).

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([1, 2, 3, 0])
        >>> q = Permutation([3, 2, 0, 1])
        >>> q*p
        Permutation([0, 3, 1, 2])
        >>> q*p == p*q
        False

        If one of the permutations is in a cyclic form then it is first
        converted to an array form and then multiplied:

        >>> p = Permutation(p.cyclic_form)
        >>> q*p
        Permutation([0, 3, 1, 2])

        It is acceptable for the arrays to have different lengths; the shorter
        one will be padded to match the longer one:

        >>> p*Permutation([[1, 0]])
        Permutation([0, 2, 3, 1])
        >>> Permutation([[1, 0]])*p
        Permutation([2, 1, 3, 0])

        It is also acceptable to allow coercion to handle conversion of a
        single list to the left of a Permutation:

        >>> [0, 1]*p # no change: 2-element identity
        Permutation([1, 2, 3, 0])
        >>> [[0, 1]]*p # exchange first two elements
        Permutation([2, 1, 3, 0])

        You cannot use more than 1 cycle notation in a product of cycles
        since coercion can only handle one argument to the left. To handle
        multiple cycles it is convenient to use Cycle instead of Permutation:

        >>> [[1, 2]]*[[2, 3]]*Permutation([]) # doctest: +SKIP
        >>> from sympy.combinatorics.permutations import Cycle
        >>> print Cycle(1, 2)(2, 3)
        [(1, 3, 2)]

        """
        a = self.array_form
        # __rmul__ makes sure the other is a Permutation
        b = other.array_form
        if not b:
            perm = a
        else:
            b.extend(range(len(b), len(a)))
            perm = [b[i] for i in a] + b[len(a):]
        return Perm._af_new(perm)

    def commutes_with(self, other):
        """
        Checks if the elements are commuting.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> a = Permutation([1,4,3,0,2,5])
        >>> b = Permutation([0,1,2,3,4,5])
        >>> a.commutes_with(b)
        True
        >>> b = Permutation([2,3,5,4,1,0])
        >>> a.commutes_with(b)
        False
        """
        a = self.array_form
        b = other.array_form
        return _af_commutes_with(a, b)

    def __pow__(self, n):
        """
        Routine for finding powers of a permutation.

        Power notation is also used for conjugation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([2,0,3,1])
        >>> q = Permutation([1,0,3,2])
        >>> r = Permutation([0,2,3,1])
        >>> p**4
        Permutation([])
        >>> p**q == p.conjugate(q)
        True
        >>> q**p == q.conjugate(p)
        True
        >>> (p**q)**r == p**(r*q)
        True
        """
        if type(n) == Perm:
            return self.conjugate(n)
        n = int(n)
        if n == 0:
            return Perm._af_new(range(self.size))
        if n < 0:
            return pow(~self, -n)
        a = self.array_form
        if n == 2:
            b = [a[i] for i in a]
        elif n == 3:
            b = [a[a[i]] for i in a]
        elif n == 4:
            b = [a[a[a[i]]] for i in a]
        else:
            # use binary multiplication
            b = range(len(a))
            while 1:
                if n & 1:
                    b = [b[i] for i in a]
                    n -= 1
                    if not n:
                        break
                if n % 4 == 0:
                    a = [a[a[a[i]]] for i in a]
                    n = n // 4
                elif n % 2 == 0:
                    a = [a[i] for i in a]
                    n = n // 2
        return Perm._af_new(b)

    def transpositions(self):
        """
        Return the permutation as a product of transpositions.

        It is always possible to express a permutation as the product of
        transpositions, see [1]

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([[1,2,3],[0,4,5,6,7]])
        >>> p.transpositions()
        {(0, 4), (0, 5), (0, 6), (0, 7), (1, 2), (1, 3)}

        References
        ==========

        1. http://en.wikipedia.org/wiki/Transposition_%28mathematics%29#Properties

        """
        a = self.cyclic_form
        res = []
        for x in a:
            nx = len(x)
            if nx == 2:
                res.append(tuple(x))
            elif nx > 2:
                first = x[0]
                for y in x[nx-1:0:-1]:
                    res.append((first,y))
        return FiniteSet(res)

    def __invert__(self):
        """
        Return the inverse of the permutation.

        A permutation multiplied by its inverse is the identity permutation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([[2,0], [3,1]])
        >>> ~p
        Permutation([2, 3, 0, 1])
        >>> p*~p == ~p*p == Permutation([0, 1, 2, 3])
        True
        """
        return Perm._af_new(_af_invert(self.array_form))

    def __iter__(self):
        """Yield elements from array form.

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> list(Permutation(range(3)))
        [0, 1, 2]
        """
        for i in self.array_form:
            yield i

    def __call__(self, i):
        """
        Allows applying a permutation instance as a bijective function.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([[2,0], [3,1]])
        >>> p.array_form
        [2, 3, 0, 1]
        >>> [p(i) for i in range(4)]
        [2, 3, 0, 1]

        If an array is given then the permutation selects the items
        from the array (i.e. the permutation is applied to the array):

        >>> from sympy.abc import x
        >>> p([x, 1, 0, x**2])
        [0, x**2, x, 1]
        """
        # list indices can be Integer or int; leave this
        # as it is (don't test or convert it) because this
        # gets called a lot and should be fast
        try:
            return self.array_form[i]
        except TypeError:
            return [i[j] for j in self.array_form]

    def atoms(self):
        """
        Returns all the elements of a permutation

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation([0, 1, 2, 3, 4, 5]).atoms()
        set([0, 1, 2, 3, 4, 5])
        >>> Permutation([[0, 1], [2, 3], [4, 5]]).atoms()
        set([0, 1, 2, 3, 4, 5])
        """
        return set(self.array_form)

    def next_lex(self):
        """
        Returns the next permutation in lexicographical order.
        If self is the last permutation in lexicographical order
        it returns None.
        See [4] section 2.4.


        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([2, 3, 1, 0])
        >>> p = Permutation([2, 3, 1, 0]); p.rank()
        17
        >>> p = p.next_lex(); p.rank()
        18

        See Also
        ========

        rank, unrank_lex
        """
        perm = self.array_form[:]
        n = len(perm)
        i = n - 2
        while perm[i+1] < perm[i]:
            i -= 1
        if i == -1:
            return None
        else:
            j = n - 1
            while perm[j] < perm[i]:
                j -= 1
            perm[j], perm[i] = perm[i], perm[j]
            i += 1
            j = n - 1
            while i < j:
                perm[j], perm[i] = perm[i], perm[j]
                i += 1
                j -= 1
        return Perm._af_new(perm)

    @classmethod
    def unrank_nonlex(self, n, r):
        """
        This is a linear time unranking algorithm that does not
        respect lexicographic order [3].

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> Permutation.unrank_nonlex(4, 5)
        Permutation([2, 0, 3, 1])
        >>> Permutation.unrank_nonlex(4, -1)
        Permutation([])

        See Also
        ========

        next_nonlex, rank_nonlex
        """
        def _unrank1(n, r, a):
            if n > 0:
                a[n - 1], a[r % n] = a[r % n], a[n - 1]
                _unrank1(n - 1, r//n, a)

        id_perm = range(n)
        n = int(n)
        r = r % ifac(n)
        _unrank1(n, r, id_perm)
        return Perm._af_new(id_perm)

    def rank_nonlex(self, inv_perm = None):
        """
        This is a linear time ranking algorithm that does not
        enforce lexicographic order [3].


        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.rank_nonlex()
        23

        See Also
        ========

        next_nonlex, unrank_nonlex
        """
        def _rank1(n, perm, inv_perm):
            if n == 1:
                return 0
            s = perm[n - 1]
            t = inv_perm[n - 1]
            perm[n - 1], perm[t] = perm[t], s
            inv_perm[n - 1], inv_perm[s] = inv_perm[s], t
            return s + n*_rank1(n - 1, perm, inv_perm)

        if inv_perm is None:
            inv_perm = (~self).array_form
        if not inv_perm:
            return 0
        perm = self.array_form[:]
        r = _rank1(len(perm), perm, inv_perm)
        return r

    def next_nonlex(self):
        """
        Returns the next permutation in nonlex order [3].
        If self is the last permutation in this order it returns None.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([2, 0, 3, 1]); p.rank_nonlex()
        5
        >>> p = p.next_nonlex(); p
        Permutation([3, 0, 1, 2])
        >>> p.rank_nonlex()
        6

        See Also
        ========

        rank_nonlex, unrank_nonlex
        """
        r = self.rank_nonlex()
        if r == ifac(self.size) - 1:
            return None
        return Perm.unrank_nonlex(self.size, r+1)

    def rank(self):
        """
        Returns the lexicographic rank of the permutation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.rank()
        0
        >>> p = Permutation([3,2,1,0])
        >>> p.rank()
        23

        See Also
        ========

        next_lex, unrank_lex, cardinality, length, order, size
        """
        rank = 0
        rho = self.array_form[:]
        n = self.size - 1
        size = n + 1
        psize = int(ifac(n))
        for j in range(size - 1):
            rank += rho[j]*psize
            for i in range(j + 1, size):
                if rho[i] > rho[j]:
                    rho[i] -= 1
            psize //= n
            n -= 1
        return rank

    @property
    def cardinality(self):
        """
        Returns the number of all possible permutations.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.cardinality
        24

        See Also
        ========

        length, order, rank, size
        """
        return int(ifac(self.size))

    def parity(self):
        """
        Computes the parity of a permutation.

        The parity of a permutation reflects the parity of the
        number of inversions in the permutation, i.e., the
        number of pairs of x and y such that ``x > y`` but ``p[x] < p[y]``.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.parity()
        0
        >>> p = Permutation([3,2,0,1])
        >>> p.parity()
        1

        See Also
        ========

        _af_parity
        """
        if self._cyclic_form is not None:
            return (self.size - self.cycles) % 2

        return _af_parity(self.array_form)

    @property
    def is_even(self):
        """
        Checks if a permutation is even.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.is_even
        True
        >>> p = Permutation([3,2,1,0])
        >>> p.is_even
        True

        See Also
        ========

        is_odd
        """
        return not self.is_odd

    @property
    def is_odd(self):
        """
        Checks if a permutation is odd.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.is_odd
        False
        >>> p = Permutation([3,2,0,1])
        >>> p.is_odd
        True

        See Also
        ========

        is_even
        """
        return bool(self.parity() % 2)

    @property
    def is_Singleton(self):
        """
        Checks to see if the permutation contains only one number and is
        thus the only possible permutation of this set of numbers

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation([0]).is_Singleton
        True
        >>> Permutation([0, 1]).is_Singleton
        False

        See Also
        ========

        is_Empty
        """
        return self.size == 1

    @property
    def is_Empty(self):
        """
        Checks to see if the permutation is a set with zero elements

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation([]).is_Empty
        True
        >>> Permutation([0]).is_Empty
        False

        See Also
        ========

        is_Singleton
        """
        return self.size == 0

    @property
    def is_Identity(self):
        """
        Returns True if the Permutation is an identity permutation.

        Examples
        ========
        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([])
        >>> p.is_Identity
        True
        >>> p = Permutation([[0], [1], [2]])
        >>> p.is_Identity
        True
        >>> p = Permutation([0, 1, 2])
        >>> p.is_Identity
        True
        >>> p = Permutation([0, 2, 1])
        >>> p.is_Identity
        False

        See Also
        ========

        order
        """
        return self.array_form == range(self.size)

    def ascents(self):
        """
        Returns the positions of ascents in a permutation, ie, the location
        where p[i] < p[i+1]

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([4,0,1,3,2])
        >>> p.ascents()
        [1, 2]

        See Also
        ========

        descents, inversions, min, max
        """
        a = self.array_form
        pos = [i for i in range(len(a)-1) if a[i] < a[i+1]]
        return pos

    def descents(self):
        """
        Returns the positions of descents in a permutation, ie, the location
        where p[i] > p[i+1]

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([4,0,1,3,2])
        >>> p.descents()
        [0, 3]

        See Also
        ========

        ascents, inversions, min, max
        """
        a = self.array_form
        pos = [i for i in range(len(a)-1) if a[i] > a[i+1]]
        return pos

    def max(self):
        """
        The maximum element moved by the permutation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([1,0,2,3,4])
        >>> p.max()
        1

        See Also
        ========

        min, descents, ascents, inversions
        """
        max = 0
        a = self.array_form
        for i in range(len(a)):
            if a[i] != i and a[i] > max:
                max = a[i]
        return max

    def min(self):
        """
        The minimum element moved by the permutation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,4,3,2])
        >>> p.min()
        2

        See Also
        ========

        max, descents, ascents, inversions
        """
        a = self.array_form
        min = len(a)
        for i in range(len(a)):
            if a[i] != i and a[i] < min:
                min = a[i]
        return min

    def inversions(self):
        """
        Computes the number of inversions of a permutation.

        An inversion is where i > j but p[i] < p[j].

        For small length of p, it iterates over all i and j
        values and calculates the number of inversions.
        For large length of p, it uses a variation of merge
        sort to calculate the number of inversions.

        References
        ==========

        [1] http://www.cp.eng.chula.ac.th/~piak/teaching/algo/algo2008/count-inv.htm

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3,4,5])
        >>> p.inversions()
        0
        >>> Permutation([3,2,1,0]).inversions()
        6

        See Also
        ========

        descents, ascents, min, max
        """
        inversions = 0
        a = self.array_form
        n = len(a)
        if n < 130:
            for i in range(n - 1):
                b = a[i]
                for c in a[i + 1:]:
                    if b > c:
                        inversions += 1
        else:
            k = 1
            right = 0
            arr = a[:]
            temp = a[:]
            while k < n:
                i = 0
                while i + k < n:
                    right = i + k * 2 - 1
                    if right >= n:
                        right = n -1
                    inversions += _merge(arr, temp, i, i+k, right)
                    i = i + k * 2;
                k = k * 2
        return inversions

    def conjugate(self, x):
        """
        Computes the conjugate Permutation ``x*p*~x``

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> a = Permutation([0,2,1,3])
        >>> b = Permutation([0,2,3,1])
        >>> a.conjugate(b)
        Permutation([0, 3, 2, 1])
        >>> b*a*~b
        Permutation([0, 3, 2, 1])
        >>> a**b
        Permutation([0, 3, 2, 1])
        """

        a = self.array_form
        b = x.array_form
        n = len(a)
        if len(b) != n:
            raise ValueError("The number of elements in the permutations "
                             "do not match.")
        invb = [None]*n
        for i in range(n):
            invb[b[i]] = i
        return Perm._af_new([invb[a[i]] for i in b])

    def commutator(self, x):
        """
        Computes the commutator Permutation ``~p*~x*p*x``

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> a = Permutation([0,2,1,3])
        >>> b = Permutation([0,2,3,1])
        >>> a.commutator(b)
        Permutation([0, 3, 1, 2])
        >>> ~a*~b*a*b
        Permutation([0, 3, 1, 2])
        """

        a = self.array_form
        b = x.array_form
        n = len(a)
        if len(b) != n:
            raise ValueError("The number of elements in the permutations "
                             "do not match.")
        inva = [None]*n
        for i in range(n):
            inva[a[i]] = i
        invb = [None]*n
        for i in range(n):
            invb[b[i]] = i
        return Perm._af_new([inva[invb[a[i]]] for i in b])

    def signature(self):
        """
        Gives the signature of the permutation needed to place the
        elements of the permutation in canonical order.

        The signature is calculated as (-1)^<# no. of inversions>

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2])
        >>> p.signature()
        1
        >>> q = Permutation([0,2,1])
        >>> q.signature()
        -1

        See Also
        ========

        inversions
        """
        if self.is_even:
            return 1
        return -1

    def order(self):
        """
        Computes the order of a permutation.

        When the permutation is raised to the power of its
        order it equals the identity permutation.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([3,1,5,2,4,0])
        >>> p.order()
        4
        >>> (p**(p.order()))
        Permutation([])

        See Also
        ========

        identity, cardinality, length, rank, size
        """

        return reduce(lcm,[1]+[len(cycle) for cycle in self.cyclic_form])

    def length(self):
        """
        Returns the number of integers moved by a permutation.

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation([0, 3, 2, 1]).length()
        2
        >>> Permutation([[0, 1], [2, 3]]).length()
        4

        See Also
        ========

        min, max, suppport, cardinality, order, rank, size
        """

        return len(self.support())


    @property
    def cycles(self):
        """
        Returns the number of cycles that the permutation
        has been decomposed into (including singletons).

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation([0, 1, 2]).cycles
        3
        >>> Permutation([[0, 1], [2, 3]]).cycles
        2
        """
        return len(self.full_cyclic_form)

    def index(self):
        """
        Returns the index of a permutation.

        The index of a permutation is the sum of all
        subscripts j such that p[j] is greater than
        p[j+1].

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([3, 0, 2, 1, 4])
        >>> p.index()
        2
        """
        a = self.array_form

        return sum([j for j in range(len(a) - 1) if a[j] > a[j+1]])

    def runs(self):
        """
        Returns the runs of a permutation.

        An ascending sequence in a permutation is called a run [5]


        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([2,5,7,3,6,0,1,4,8])
        >>> p.runs()
        [[2, 5, 7], [3, 6], [0, 1, 4, 8]]
        >>> q = Permutation([1,3,2,0])
        >>> q.runs()
        [[1, 3], [2], [0]]
        """
        return runs(self.array_form)

    def inversion_vector(self):
        """
        Gets the inversion vector of the permutation.

        The inversion vector consists of elements whose value
        indicates the number of elements in the permutation
        that are lesser than it and lie on its right hand side.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([4,8,0,7,1,5,3,6,2])
        >>> p.inversion_vector()
        [4, 7, 0, 5, 0, 2, 1, 1]
        >>> p = Permutation([3,2,1,0])
        >>> p.inversion_vector()
        [3, 2, 1]
        """
        self_array_form = self.array_form
        n = len(self_array_form)
        inversion_vector = [0] * (n - 1)

        for i in range(n - 1):
            val = 0
            for j in range(i+1, n):
                if self_array_form[j] < self_array_form[i]:
                    val += 1
            inversion_vector[i] = val
        return inversion_vector

    def rank_trotterjohnson(self):
        """
        Returns the Trotter Johnson rank, which we get from the minimal
        change algorithm. See [4] section 2.4.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0,1,2,3])
        >>> p.rank_trotterjohnson()
        0
        >>> p = Permutation([0,2,1,3])
        >>> p.rank_trotterjohnson()
        7

        See Also
        ========

        unrank_trotterjohnson, next_trotterjohnson
        """
        if self.array_form == [] or self.is_Identity:
            return 0
        if self.array_form == [1, 0]:
            return 1
        perm = self.array_form
        n = self.size
        rank = 0
        for j in range(1, n):
            k = 1
            i = 0
            while perm[i] != j:
                if perm[i] < j:
                    k += 1
                i += 1
            j1 = j + 1
            if rank % 2 == 0:
                rank = j1*rank + j1 - k
            else:
                rank = j1*rank + k - 1
        return rank

    @classmethod
    def unrank_trotterjohnson(self, size, rank):
        """
        Trotter Johnson permutation unranking. See [4] section 2.4.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.unrank_trotterjohnson(5, 10)
        Permutation([0, 3, 1, 2])

        See Also
        ========

        rank_trotterjohnson, next_trotterjohnson
        """
        perm = [0]*size
        r2 = 0
        n = ifac(size)
        pj = 1
        for j in range(2, size+1):
            pj *= j
            r1 = (rank * pj) // n
            k = r1 - j*r2
            if r2 % 2 == 0:
                for i in range(j-1, j-k-1, -1):
                    perm[i] = perm[i-1]
                perm[j-k-1] = j-1
            else:
                for i in range(j-1, k, -1):
                    perm[i] = perm[i-1]
                perm[k] = j-1
            r2 = r1
        return Perm._af_new(perm)

    def next_trotterjohnson(self):
        """
        Returns the next permutation in Trotter-Johnson order.
        If self is the last permutation it returns None.
        See [4] section 2.4.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> p = Permutation([3, 0, 2, 1])
        >>> p.rank_trotterjohnson()
        4
        >>> p = p.next_trotterjohnson(); p
        Permutation([0, 3, 2, 1])
        >>> p.rank_trotterjohnson()
        5

        See Also
        ========

        rank_trotterjohnson, unrank_trotterjohnson
        """
        pi = self.array_form[:]
        n = len(pi)
        st = 0
        rho = pi[:]
        done = False
        m = n-1
        while m > 0 and not done:
            d = rho.index(m)
            for i in range(d, m):
                rho[i] = rho[i+1]
            par = _af_parity(rho[:m])
            if par == 1:
                if d == m:
                    m -= 1
                else:
                    pi[st+d], pi[st+d+1] = pi[st+d+1], pi[st+d]
                    done = True
            else:
                if d == 0:
                    m -= 1
                    st += 1
                else:
                    pi[st+d], pi[st+d-1] = pi[st+d-1], pi[st+d]
                    done = True
        if m == 0:
            return None
        return Perm._af_new(pi)

    def get_precedence_matrix(self):
        """
        Gets the precedence matrix. This is used for computing the
        distance between two permutations.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation.josephus(3,6,1)
        >>> p
        Permutation([2, 5, 3, 1, 4, 0])
        >>> p.get_precedence_matrix()
        [0, 0, 0, 0, 0, 0]
        [1, 0, 0, 0, 1, 0]
        [1, 1, 0, 1, 1, 1]
        [1, 1, 0, 0, 1, 0]
        [1, 0, 0, 0, 0, 0]
        [1, 1, 0, 1, 1, 0]

        See Also
        ========

        get_precedence_distance, get_adjacency_matrix, get_adjacency_distance
        """
        m = zeros(self.size)
        perm = self.array_form
        for i in range(m.rows):
            for j in range(i + 1, m.cols):
                m[perm[i], perm[j]] = 1
        return m

    def get_precedence_distance(self, other):
        """
        Computes the precedence distance between two permutations.

        Suppose p and p' represent n jobs. The precedence metric
        counts the number of times a job j is prededed by job i
        in both p and p'. This metric is commutative.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([2, 0, 4, 3, 1])
        >>> q = Permutation([3, 1, 2, 4, 0])
        >>> p.get_precedence_distance(q)
        7
        >>> q.get_precedence_distance(p)
        7

        See Also
        ========

        get_precedence_matrix, get_adjacency_matrix, get_adjacency_distance
        """
        if self.size != other.size:
            raise ValueError("The permutations must be of the same size.")
        self_prec_mat = self.get_precedence_matrix()
        other_prec_mat = other.get_precedence_matrix()
        n_prec = 0
        for i in range(self.size):
            for j in range(self.size):
                if i == j:
                    continue
                if self_prec_mat[i, j] * other_prec_mat[i, j] == 1:
                    n_prec += 1
        d = self.size * (self.size - 1)//2 - n_prec
        return d

    def get_adjacency_matrix(self):
        """
        Computes the adjacency matrix of a permutation.

        If job i is adjacent to job j in a permutation p
        then we set m[i, j] = 1 where m is the adjacency
        matrix of p.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation.josephus(3,6,1)
        >>> p.get_adjacency_matrix()
        [0, 0, 0, 0, 0, 0]
        [0, 0, 0, 0, 1, 0]
        [0, 0, 0, 0, 0, 1]
        [0, 1, 0, 0, 0, 0]
        [1, 0, 0, 0, 0, 0]
        [0, 0, 0, 1, 0, 0]

        >>> q = Permutation([0, 1, 2, 3])
        >>> q.get_adjacency_matrix()
        [0, 1, 0, 0]
        [0, 0, 1, 0]
        [0, 0, 0, 1]
        [0, 0, 0, 0]

        See Also
        ========

        get_precedence_matrix, get_precedence_distance, get_adjacency_distance
        """
        m = zeros(self.size)
        perm = self.array_form
        for i in range(self.size - 1):
            m[perm[i], perm[i + 1]] = 1
        return m

    def get_adjacency_distance(self, other):
        """
        Computes the adjacency distance between two permutations.

        This metric counts the number of times a pair i,j of jobs is
        adjacent in both p and p'. If n_adj is this quantity then
        the adjacency distance is n - n_adj - 1 [1]

        [1] Reeves, Colin R. Landscapes, Operators and Heuristic search, Annals
        of Operational Research, 86, pp 473-490. (1999)


        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0, 3, 1, 2, 4])
        >>> q = Permutation.josephus(4, 5, 2)
        >>> p.get_adjacency_distance(q)
        3
        >>> r = Permutation([0, 2, 1, 4, 3])
        >>> p.get_adjacency_distance(r)
        4

        See Also
        ========

        get_precedence_matrix, get_precedence_distance, get_adjacency_matrix
        """
        if self.size != other.size:
            raise ValueError("The permutations must be of the same size.")
        self_adj_mat = self.get_adjacency_matrix()
        other_adj_mat = other.get_adjacency_matrix()
        n_adj = 0
        for i in range(self.size):
            for j in range(self.size):
                if i == j:
                    continue
                if self_adj_mat[i, j] * other_adj_mat[i, j] == 1:
                    n_adj += 1
        d = self.size - n_adj - 1
        return d

    def get_positional_distance(self, other):
        """
        Computes the positional distance between two permutations.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> p = Permutation([0, 3, 1, 2, 4])
        >>> q = Permutation.josephus(4, 5, 2)
        >>> r = Permutation([3, 1, 4, 0, 2])
        >>> p.get_positional_distance(q)
        12
        >>> p.get_positional_distance(r)
        12

        See Also
        ========

        get_precedence_distance, get_adjacency_distance
        """
        a = self.array_form
        b = other.array_form
        if len(a) != len(b):
            raise ValueError("The permutations must be of the same size.")
        return sum([abs(a[i] - b[i]) for i in range(len(a))])

    @classmethod
    def josephus(self, m, n, s = 1):
        """Return as a permutation the shuffling of range(n) using the Josephus
        scheme in which every m-th item is selected until all have been chosen.
        The returned permutation has elements listed by the order in which they
        were selected.

        The parameter ``s`` stops the selection process when there are ``s``
        items remaining and these are selected by countinuing the selection,
        counting by 1 rather than by ``m``.

        Consider selecting every 3rd item from 6 until only 2 remain::

            choices    chosen
            ========   ======
              012345
              01 345   2
              01 34    25
              01  4    253
              0   4    2531
              0        25314
                      253140

        Examples
        ========

        >>> from sympy.combinatorics import Permutation
        >>> Permutation.josephus(3, 6, 2).array_form
        [2, 5, 3, 1, 4, 0]

        References
        ==========

        1. http://en.wikipedia.org/wiki/Flavius_Josephus
        2. http://en.wikipedia.org/wiki/Josephus_problem
        3. http://www.wou.edu/~burtonl/josephus.html

        """
        from collections import deque
        m -= 1
        Q = deque(range(n))
        perm = []
        while len(Q) > max(s, 1):
            for dp in range(m):
                Q.append(Q.popleft())
            perm.append(Q.popleft())
        perm.extend(list(Q))
        return Perm(perm)

    @classmethod
    def from_inversion_vector(self, inversion):
        """
        Calculates the permutation from the inversion
        vector.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> Permutation.from_inversion_vector([3,2,1,0,0])
        Permutation([3, 2, 1, 0])

        """
        size = len(inversion)
        N = range(size + 1)
        perm = []
        try:
            for k in range(size):
                val = N[inversion[k]]
                perm.append(val)
                N.remove(val)
        except IndexError:
            raise ValueError("The inversion vector is not valid.")
        perm.extend(N)
        return Perm._af_new(perm)

    @classmethod
    def random(self, n):
        """
        Generates a random permutation of length ``n``.

        Uses the underlying Python psuedo-random number generator.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.random(2) in (Permutation([1, 0]), Permutation([0, 1]))
        True

        """
        perm_array = range(n)
        random.shuffle(perm_array)
        return Perm._af_new(perm_array)

    @classmethod
    def unrank_lex(self, size, rank):
        """
        Lexicographic permutation unranking.

        Examples
        ========

        >>> from sympy.combinatorics.permutations import Permutation
        >>> Permutation.print_cyclic = False
        >>> a = Permutation.unrank_lex(5,10)
        >>> a.rank()
        10
        >>> a
        Permutation([0, 2, 4, 1, 3])

        See Also
        ========

        rank, next_lex
        """
        perm_array = [0] * size
        psize = 1
        for i in range(size):
            new_psize = psize*(i + 1)
            d = (rank % new_psize) // psize
            rank -= d*psize
            perm_array[size - i - 1] = d
            for j in range(size - i, size):
                if perm_array[j] > d-1:
                    perm_array[j] += 1
            psize = new_psize
        return Perm._af_new(perm_array)

    # global flag to control how permutations are printed
    # when True, Permutation([0, 2, 1, 3]) -> Cycle(1, 2)
    # when False, Permutation([0, 2, 1, 3]) -> Permutation([0, 2, 1])
    print_cyclic = True

def _merge(arr, temp, left, mid, right):
    """
    Merges two sorted arrays and calculates the inversion count.

    Helper function for calculating inversions. This method is
    for internal use only.
    """
    i = k = left
    j = mid
    inv_count = 0
    while i < mid and j <= right:
        if arr[i] < arr[j]:
            temp[k] = arr[i]
            k += 1
            i += 1
        else:
            temp[k] = arr[j]
            k += 1
            j += 1
            inv_count += (mid -i)
    while i < mid:
        temp[k] = arr[i]
        k += 1
        i += 1
    if j <= right:
        k += right - j + 1
        j += right - j + 1
        arr[left:k + 1] = temp[left:k + 1]
    else:
        arr[left:right + 1] = temp[left:right + 1]
    return inv_count

Perm = Permutation
