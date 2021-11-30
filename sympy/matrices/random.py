from random import Random

from ..core import I as _i
from ..core.mul import Mul as _multiply
from ..functions import sqrt as _sqrt, re as _re, im as _im, \
    transpose as _t, adjoint as _c
from ..tensor import shape as _shape
from .dense import eye as _eye
from .inverse import _inv

__all__ = 'projection', 'jordan', 'transposition', \
          'permutation', 'elementary', 'rotation', 'reflection', \
          'diagonal_normal', 'jordan_normal', 'isometry_normal', \
          'triangular', 'invertible', 'singular', \
          'idempotent', 'nilpotent', 'diagonalizable', 'trigonalizable', \
          'orthogonal', 'unitary', 'normal', \
          'symmetric', 'hermite', 'square', \
          'regular_to_singular', 'complex_to_real', 'rand'


_ALT = False
_EPS = 1e-15


# === random number generator functions ===


rand = Random()
r""" default random number generator

    Explanation
    ===========

    the module default random generator
    which is a instance of ``random.Random()``.
    Hence, the state can be set by invoking ``rand.seed()``.


    Examples
    ========

    >>> from sympy.matrices.random import rand, orthogonal

    >>> rand.seed(1)
    >>> orthogonal(3)
    Matrix([
    [-1,          0,          0],
    [ 0,  sqrt(2)/2, -sqrt(2)/2],
    [ 0, -sqrt(2)/2, -sqrt(2)/2]])

    See Also
    ========
    projection

"""


def _rand(seed=None):
    if hasattr(seed, 'sample'):
        return seed
    return rand if seed is None else Random(seed)


def _sample(scalars, k=None, rng=None):
    _k = k or 1
    if hasattr(scalars, 'sample'):
        smpl = scalars.sample(_k)
    elif hasattr(rng, 'sample'):
        smpl = rng.sample(scalars, _k)
    else:
        smpl = rand.sample(scalars, _k)
    return smpl if k else smpl[0]


# === helper ===


def _set_value(o, i, j, v):
    o[i, j] = v


def _cs(scalar, real=True, rng=None):
    if isinstance(scalar, (tuple, list)) and len(scalar) == 2:
        c, s = scalar
    elif _is_complex(scalar):
        c, s = _re(scalar), _im(scalar)
    else:
        rng = _rand(rng)
        c, s = scalar, _sample((-1, 1), rng=rng) * _sqrt(1 - scalar ** 2)
    abs_cs = abs(c ** 2 + s ** 2 - 1)
    if isinstance(scalar, (int, float, complex)):
        if abs(scalar) > 1 or abs_cs > _EPS:
            msg = "isometry scalar argument must have norm equal to 1"
            msg += " or - if real - norm less than 1"
            msg += " not abs%s=%s" % (str(scalar), str(abs_cs))
            raise ValueError(msg)
    return (c, s) if real else (c + s * _i,)


def _is_complex(z):
    return isinstance(z, complex) or \
           (getattr(z, 'is_complex', False) is True and
            getattr(z, 'is_real', True) is False)


def _is_abs_one(x):
    return isinstance(abs(x), (int, float)) and abs(x) == 1


# === default sets ===


_elementary_scalars = -1, 1
_elementary_units = _elementary_scalars
_rotation_scalars = (_sqrt(2) / 2, _sqrt(2) / 2), (0, -1),
_unitary_scalars = tuple(c * 1 + s * _i for c, s in _rotation_scalars)

# === fundamental constructor ===


def super_elementary_matrix(dim,
                            index=None,
                            value=None,
                            *scalars):
    r"""super elementary matrix n x n, i.e. identiy with a 2 x 2 block

    Explanation
    ===========
    The super elementary matrix $A$ is a generalization of elementary matrices
    as well as of a rotation matrices that rotate only a single plane.

    In two dimensions any invertible matrix

    .. math::

       A = \left[\begin{array}{cc}a & b \\ -c & d\end{array}\right]

    is super elementary, i.e. $ad+bc \neq 0$. In higher dimensions,
    any matrix $A$ looking like itdentiy matrix but with entries

    .. math::

        A[i,i] = a, \quad A[i,j] = b, \quad A[j,i] = -c, \quad A[j,j] = d

    and $ad + bc \neq 0$.

    This includes elementary matrices of matrix operation
    for Gauss elimination. So multiplication with $A$ gives for

    * $a=0=d$ and $b=1=-c$ a *transposition*, i.e. swapping rows $i$ and $j$
    * $a=1$, $d=\lambda$ and $b=0=-c$ this matrix describes scaling the
      row $j$ by $\lambda$
    * $a=1=d$, $b=\lambda$, $-c=0$ adding the $\lambda$ multiple
      of the row $i$ to row $j$.

    Moreover, a simple rotation by $\phi$ in $(i,j)$ plane is super elementary,
    given by $a=\cos \phi = d$ and $b = \sin \phi = c$. Hence,

    .. math::

       \left[\begin{array}{cc}a & b \\ -c & d\end{array}\right]
       =
       \left[\begin{array}{cc}
        \cos \phi & \sin \phi \\ -\sin \phi & \cos \phi \end{array}\right]

    In this module the super elementary matrix serves as the base class to
    create futher matrices of given type.

    Examples
    ========

    >>> from sympy.matrices.random import super_elementary_matrix

    >>> super_elementary_matrix(3, (0,1))
    Matrix([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1]])

    >>> super_elementary_matrix(3, (0,2), value=5)
    Matrix([
    [1, 0, 5],
    [0, 1, 0],
    [0, 0, 1]])

    >>> super_elementary_matrix(3, (2,2), value=5)
    Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 5]])

    >>> super_elementary_matrix(3, (1,2), 1,2,3,4)
    Matrix([
    [1, 0, 0],
    [0, 1, 2],
    [0, -3, 4]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    index : tuple(int, int)
        coordinates (i,j) of super elementary square
    value : symbol (optional)
        value as elementary entry
    *scalars : symbols (optional)
        up to three additional values as additional super elementry entries

    See Also
    ========
    diagonal
    elementary
    transposition
    rotation

    """
    obj = _eye(dim)
    if index is None:
        return obj

    if not isinstance(index, (list, tuple, set)) or not len(index) == 2:
        raise ValueError(
            "index argument must be tuple of two matrix index integer.")
    row, col = index
    if not isinstance(row, int) or not isinstance(col, int):
        raise ValueError(
            "index argument must be tuple of two matrix index integer.")

    if row == col:
        # identity or _multiply by scalar
        obj[row, col] = value or 1
        if scalars:
            raise ValueError(
                "if row==col no further args than value may be supplied.")
    elif not scalars:
        # elementary
        if value is None:
            # transposition
            obj[row, row] = 0
            obj[row, col] = 1
            obj[col, row] = 1
            obj[col, col] = 0
        else:
            # add scalar multiple to another
            obj[row, col] = value
    elif len(scalars) == 3:
        obj[row, row] = value
        y, v, u = scalars
        obj[row, col] = y
        obj[col, row] = -v
        obj[col, col] = u
    else:
        msg = "either no, one or three additional scalars arguments " \
              "may be supplied, but not %i"
        raise ValueError(msg % len(scalars))
    return obj


# === fundamental matrix functions ===


def complex_to_real(mat=None):
    r""" returns real version 2n x 2n of a complex matrix n x n

    Explanation
    ===========
    Any complex vector space $V$ with basis $(b_1, \dots, b_n)$
    can be seen as a real vector space $V^{\prime}$ with basis
    $(b_1, b_1 \ i, \dots, b_n, b_n \ i)$
    of dimension $2 \ n$.

    Since multiplication with complex number is linear on $V$
    resp. $V^{\prime}$,
    any complex homomorphism
    (i.e. complex matrix $\mathbf{A}$) on $V$
    becomes a real homomorphism
    (i.e. matrix $\mathbf{A}^{\prime}$) on $V^{\prime}$.

    This transforms a complex matrix $\mathbf{A}$
    to the corresponding real matrix $\mathbf{A}^{\prime}$.

    Examples
    ========
    >>> from sympy import Matrix, I
    >>> from sympy.abc import a, b
    >>> from sympy.matrices.random import complex_to_real

    >>> z = a + b * I
    >>> A = Matrix([[z]])
    >>> A
    Matrix([[a + I*b]])

    >>> complex_to_real(A)
    Matrix([
    [ re(a) - im(b), re(b) + im(a)],
    [-re(b) - im(a), re(a) - im(b)]])

    >>> A = Matrix([[z, 0],[1, I]])
    >>> A
    Matrix([
    [a + I*b, 0],
    [      1, I]])
    >>> complex_to_real(A)
    Matrix([
    [ re(a) - im(b), re(b) + im(a),  0, 0],
    [-re(b) - im(a), re(a) - im(b),  0, 0],
    [             1,             0,  0, 1],
    [             0,             1, -1, 0]])

    Parameters
    ==========
    mat : Matrix
        a complex matrix

    """
    dim = _shape(mat)[0]
    mat = mat or _eye(dim)
    obj = super_elementary_matrix(2 * dim)
    for i in range(dim):
        for j in range(dim):
            z_value = mat[i, j]
            a, b = _re(z_value), _im(z_value)
            obj[2 * i, 2 * j] = a
            obj[2 * i, 2 * j + 1] = b
            obj[2 * i + 1, 2 * j] = -b
            obj[2 * i + 1, 2 * j + 1] = a
    return obj


def regular_to_singular(mat, rank=None, rng=None):
    r""" build matrix with linear combination of matrix columns to meet rank

    Explanation
    ===========
    Let $\mathbf{I}$ be the $n \times n$ identity matrix
    and $\mathbf{D}$ an diagonal matrix with only zero and one entries
    of rank $r$ (i.e. all entries but $n-r$ diagonal entries are zero).

    If a full rank matrix $\mathbf{A}$ is given, then

    .. math::

        \mathbf{B}
        = \mathbf{D} \cdot
        (\mathbf{I} + \mathbf{A} \cdot (\mathbf{I}-\mathbf{D}))
        = \mathbf{D} + \mathbf{D} \cdot \mathbf{A} \cdot(\mathbf{I}-\mathbf{D})

    is of rank $r$, too.  Some for $\mathbf{A}\cdot\mathbf{B}$
    which contains $r$ columns with $\mathbf{A}$.

    If $\mathbf{A}$ is an upper triangular matrix $\mathbf{B}$
    as well $\mathbf{A} \cdot \mathbf{B}$ is one, too.

    Here, $\mathbf{A} \cdot \mathbf{B}$ is returned.
    Note, if $n=r$ the matrix $\mathbf{B}$ is the identity.

    Examples
    ========
    >>> from sympy import Matrix
    >>> from sympy.matrices.random import regular_to_singular
    >>> m = Matrix([[1, 2, 3],[4, 5, 6], [7, 8 ,0]])
    >>> m.rank()
    3
    >>> n = regular_to_singular(m, 2)
    >>> n
    Matrix([
    [1, 2, 15],
    [4, 5, 42],
    [7, 8, 69]])
    >>> n.rank()
    2

    Parameters
    ==========
    mat : Matrix
        a complex matrix
    rank : integer
        rank of matrix
    rng : random number generator (optional)
        provides randomness of sigulatries

    """
    if rank is None:
        return mat
    dim = _shape(mat)[0]
    if rank == dim:
        return mat
    i = _eye(dim)
    p = permutation(dim, seed=_rand(rng))
    d = projection(dim, (0, rank))
    d = _multiply(p.inv(), d, p)
    md = _multiply(mat, d)
    return md + _multiply(md, mat, i - d)


# === base matrices ===


def projection(dim,
               index=None,
               seed=None):
    r"""A randomly generated n x n projection matrix

    Explanation
    ===========

    A projection is a identity like matrix
    but with zero diagonal entiries off *index*.

    Examples
    ========

    >>> from sympy.matrices.random import rand, projection
    >>> rand.seed(1)

    >>> projection(3)
    Matrix([
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 0]])

    >>> projection(3, index=(1,3))  # no more random
    Matrix([
    [0, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])

    Parameters
    ==========

    dim : integer
        dimension n of matrix
    index : tuple(integer, integer) (optional)
        coordinates ``(i,j)`` for start (included) and end (excluded)
        to project onto
    seed : random seed (optional)

        * either seed of a random number generator, i.e.
          ``NoneType``, ``int``, ``float``, ``str``, ``bytes``
          or ``bytearray``,
        * or a random number generator itself,
          i.e. it should be an instance of ``random.Random``,
        * or at least have ``sample(population, k)``
          instance method implemented
          with same signature using the first two argumnents
          as ``random.Random``,
          i.e. drawing (*without putting back*) a list of **k** random items
          of **population**.

    See Also
    ========
    jordan
    elementary
    diagonal_normal

    """
    rng = _rand(seed)
    index = index or _sample(range(dim + 1), 2, rng=rng)
    obj = _eye(dim)
    start, end = sorted(index)
    for i in range(dim):
        v = 1 if start <= i < end else 0
        obj[i, i] = v
    return obj


def jordan(dim,
           index=None,
           scalar=None,
           seed=None):
    r"""n x n matrix with a single Jordan block of given eigenvalue

    Explanation
    ===========
    A matrix with a single Jordan block matrix
    with only non-zero blocks on the diagonal
    which have the form of an Jordan block $\mathbf{J}$ which is

    .. math::

        \mathbf{J}
        = \left[\begin{array}{cccccc}
            \lambda  & 1       & 0         & \dots     &         & 0     \\
             0       & \lambda & 1         & 0         & \dots   & 0     \\
             0       & \ddots  & \ddots    & \ddots    & \ddots  & 0     \\
             0       & \dots   & 0         & 0         & \lambda & 1     \\
             0       & \dots   &           &           & 0       & \lambda
             \end{array}\right]

    Finally, each $\lambda$ of each Jordan block will be an eigenvalue.

    >>> from sympy.matrices.random import rand, jordan
    >>> rand.seed(1)

    >>> jordan(3)
    Matrix([
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1]])

    >>> jordan(3, scalar=2)
    Matrix([
    [2, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])

    >>> jordan(4, index=(1,4), scalar=2)  # no more random
    Matrix([
    [1, 0, 0, 0],
    [0, 2, 1, 0],
    [0, 0, 2, 1],
    [0, 0, 0, 2]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    index : tuple(integer, integer) (optional)
        coordinates ``(i,j)`` for start (included) and end (excluded)
        of the Jordan square
    scalar : symbol (optional)
        eigenvalue used for the Jordan block
        defaults to values -1 or 1
    seed : random seed (optional)
        see :func:`projection`

    See Also
    ========
    jordan_normal

    """
    rng = _rand(seed)
    index = index or _sample(range(dim), 2, rng=rng)
    scalar = _sample(_elementary_scalars, rng=rng) \
        if scalar is None else scalar
    obj = _eye(dim)
    start, end = sorted(index)
    obj[start, start] = scalar
    for i in range(start + 1, end):
        obj[i - 1, i] = 1
        obj[i, i] = scalar
    return obj


def transposition(dim,
                  index=None,
                  seed=None):
    r"""n x n transposition matrix

    Explanation
    ===========

    A transposition matrix is an identity matrix where two columns (or rows)
    are swapped. It is a special permutation matrix.
    Moreover, any permutaion matrix is a product of transposition matrices.

    In two dimensions it is

    .. math::

       \mathbf{T} = \left[\begin{array}{cc}0 & 1 \\ 1 & 0\end{array}\right] \\

    If index is given, it sets which column or row will be swapped.

    Examples
    ========

    >>> from sympy.matrices.random import rand, transposition
    >>> rand.seed(1)

    >>> transposition(4)
    Matrix([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]])

    >>> transposition(3, (0,1))  # no more random
    Matrix([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1]])

    Parameters
    ==========
    dim : integer
        dimension n of matrix
    index : tuple(integer, integer) (optional)
        coordinates (i,j) of transposition
    seed : random seed (optional)
        see :func:`projection`

    See Also
    ========
    elementary
    permutation

    """
    rng = _rand(seed)
    index = index or _sample(range(dim), 2, rng=rng)
    return super_elementary_matrix(dim, index)


def permutation(dim,
                perm=None,
                seed=None):
    r"""permutation matrix n x n

    Explamation
    ===========

    A permutation matrix is a matrix with only 0 or 1 entries
    and each column and each row consists of only one non-zero entry.

    Such a matrix can be obtained by shuffling the rows
    of an identiy matrix .i.e ``permutation(dim, perm)`` is eqivalent to
    ``eye(dim).permute(perm)``.

    Examples
    ========
    >>> from sympy.matrices.random import rand, permutation
    >>> rand.seed(1)

    >>> permutation(3)
    Matrix([
    [1, 0, 0],
    [0, 0, 1],
    [0, 1, 0]])

    >>> permutation(3, (2,0,1))  # no more random
    Matrix([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]])

    Parameters
    ==========
    dim : integer
        dimension n of matrix
    perm : tuple or list of integer (optional)
        permutation as list of n integers, e.g. ``[2,1,0,3]`` as a
        permutation of ``[0,1,2,3]``
    seed : random seed (optional)
        see :func:`transposition`

    See Also
    ========
    transposition

    """
    rng = _rand(seed)
    perm = perm or _sample(range(dim), dim, rng=rng)
    obj = _eye(dim)
    for i, j in enumerate(perm):
        obj[i, i] = 0  # aka zeros(dim)
        obj[i, j] = 1
    return obj  # aka _eye(dim).permute(perm)


def elementary(dim,
               index=None,
               scalar=None,
               seed=None):
    r"""an elementary matrix n x n for Gauss elimination

    Explanantion
    ============
    Elementary matrices are matrices for Gauss elimination operation.

    In two dimensions any matrix of the following types are elemenarty

    .. math::

       \mathbf{T}
       = \left[\begin{array}{cc}0 & 1 \\ 1 & 0\end{array}\right] \\

       \mathbf{M}
       = \left[\begin{array}{cc}\lambda & 0 \\ 0 & 1\end{array}\right] \\

       \mathbf{A}
       = \left[\begin{array}{cc}1 & \mu \\ 0 & 1\end{array}\right] \\

    In higher dimensions, any matrix $\mathbf{A}$
    looking like identity matrix but with entries

    .. math::

        A_{ii} = a, \quad A_{ij} = b, \quad A_{ji} = -c, \quad A_{jj} = d

    and $ad - bc \neq 0$.
    So multiplication with $\mathbf{A}$ gives for

    * $a=0=d$ and $b=1=-c$ a :func:`transposition`,
      i.e. swapping rows $i$ and $j$
    * $a=1$, $d=\lambda$ and $b=0=-c$ this matrix describes scaling the
      row $j$ by $\lambda$
    * $a=1=d$, $b=\mu$, $-c=0$ adding the $\mu$ multiple
      of the row $i$ to row $j$.

    Examples
    ========
    >>> from sympy.matrices.random import rand, elementary
    >>> rand.seed(1)

    >>> elementary(3)
    Matrix([
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0]])

    >>> elementary(3, (0,2), scalar=5)  # no more random
    Matrix([
    [1, 0, 5],
    [0, 1, 0],
    [0, 0, 1]])

    >>> elementary(3, (2,2), scalar=5)  # no more random
    Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 5]])

    >>> elementary(3, (2,1), 4)  # no more random
    Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 4, 1]])

    >>> elementary(3, (0,1), None)  # no more random
    Matrix([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    index : tuple(integer, integer) (optional)
        coordinates ``(i,j)`` of matrix operation
        if ``i`` equals ``j`` the *elementary matrix*
        will be of type $\mathbf{M}$,
        else of type $\mathbf{A}$ or $\mathbf{T}$
    scalar : symbol (optional)
        value of elementary entry,
        defaults to values -1 or 1
        if **scalar** is **None** the *elementary matrix*
        will be a transposition $\mathbf{T}$ or identity
    seed : random seed (optional)
        see :func:`projection`

    See Also
    ========
    diagonal
    transposition

    """
    rng = _rand(seed)
    index = index or _sample(range(dim), 2, rng=rng)
    # scalar = scalar or _sample(_elementary_scalars + (None,), 1)
    return super_elementary_matrix(dim, index, scalar)


def rotation(dim,
             index=None,
             scalar=None,
             seed=None):
    r"""a square matrix n x n of a plane rotation

    Explanation
    ===========
    The matrix decribes in n dimensional Euclidian space
    a rotation of $(i,j)$ plane given by a single rotation square matrix
    $\mathbf{R}=\mathbf{R(c,s)}$ with entries like the identiy matrix but

    .. math::

        R_{ii} = c, \quad R_{ij} = s, \quad R_{ji} = -s, \quad R_{jj} = c

    with $|c|^2+|s|^2=1$.
    Usually $s, c$ are $\sin\phi$ and $\cos\phi$ for some angle $\phi$.

    In two dimensions this leads to

    .. math::

        \mathbf{R(c,s)} =
        \left[\begin{array}{cc} c & s \\ -s & c \end{array}\right].


    Examples
    ========
    >>> from sympy import sqrt, cos, symbols, eye, I, re, im, expand, simplify
    >>> from sympy.matrices.random import rand, rotation
    >>> rand.seed(1)

    >>> rotation(3)
    Matrix([
    [0, 0, -1],
    [0, 1,  0],
    [1, 0,  0]])

    >>> cos_a = sin_a = sqrt(2)/2
    >>> rotation(3, scalar=cos_a)
    Matrix([
    [ sqrt(2)/2, sqrt(2)/2, 0],
    [-sqrt(2)/2, sqrt(2)/2, 0],
    [         0,         0, 1]])

    >>> rotation(3, (1, 2), scalar=(cos_a, sin_a))  # no more random
    Matrix([
    [1,          0,         0],
    [0,  sqrt(2)/2, sqrt(2)/2],
    [0, -sqrt(2)/2, sqrt(2)/2]])

    >>> z = cos_a + sin_a * I
    >>> rotation(3, (1, 2), scalar=z)  # no more random
    Matrix([
    [1,          0,         0],
    [0,  sqrt(2)/2, sqrt(2)/2],
    [0, -sqrt(2)/2, sqrt(2)/2]])

    >>> r = rotation(3, scalar=z)
    >>> r.T * r == eye(3)
    True

    works with symbols too

    >>> cos_phi = cos(symbols('phi'))
    >>> r = rotation(3, scalar=cos_phi)
    >>> r
    Matrix([
    [              cos(phi), 0, sqrt(1 - cos(phi)**2)],
    [                     0, 1,                     0],
    [-sqrt(1 - cos(phi)**2), 0,              cos(phi)]])

    >>> r.det()
    1

    >>> r.T * r == eye(3)
    True

    as well to give complex rotations,
    i.e. a special unitary matrix,

    >>> w, u = z * z, z * I
    >>> cos_x, sin_x = simplify(z * re(u)), simplify(w * im(u))
    >>> cos_x, sin_x
    (-1/2 - I/2, sqrt(2)*I/2)

    >>> r = rotation(3, scalar=(cos_x, sin_x))
    >>> r
    Matrix([
    [ -1/2 - I/2, sqrt(2)*I/2, 0],
    [sqrt(2)*I/2,  -1/2 + I/2, 0],
    [          0,           0, 1]])

    >>> r.det()
    1

    >>> expand(r.H * r)
    Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])


    Parameters
    ==========
    dim : integer
        dimension of matrix
    index : tuple(integer, integer) (optional)
        coordinates ``(i,j)`` of rotation plane
    scalar : tuple(symbol, symbol) or symbol (optional)
        * either a tuple of cosine value $c$ and sine value $s$
          of rotation square
        * or just cosine value $c$ of rotation square (then, a corresponding
          sin value $s$ will be drawn randomly)
        * or a complex unit $z$ (complex number with $|z|=1$) so
          $c=re(z)$ and $s=im(z)$.

        If given, the **scalar** $c$ is be between -1 and 1.

        The resulting rotation square

        .. math::

            \left[\begin{array}{cc} c & \pm s \\ \mp s & c \end{array}\right]

        takes **scalar** to be $c$ and $\pm \sqrt{1-c^2}$ for $s$.
    seed : random seed (optional)
        see :func:`projection`

    See Also
    ========
    reflection
    isometry_normal
    orthogonal

    """
    rng = _rand(seed)
    index = index or _sample(range(dim), 2, rng=rng)
    scalar = scalar or _sample(_rotation_scalars, rng=rng)
    c, s = _cs(scalar, rng=rng)
    if _is_complex(c) or _is_complex(s):
        return super_elementary_matrix(dim, index, c, s, _c(s), _c(c))
    else:
        return super_elementary_matrix(dim, index, c, s, s, c)


def reflection(dim,
               index=None,
               scalar=None,
               seed=None):
    r"""a square matrix n x n of a hyperplane reflection

    Explanation
    ===========
    The matrix describes a reflaction in n dimensional Euclidian space.
    Constructed as ``reflection =  rotation * transposition``, i.e.
    there will be the reflection square

    .. math::

        \left[\begin{array}{cc} s & c \\ c & -s \end{array}\right]

    Examples
    ========
    >>> from sympy import sqrt, eye, I
    >>> from sympy.matrices.random import rand, reflection
    >>> rand.seed(1)

    >>> reflection(3)
    Matrix([
    [-1, 0, 0],
    [ 0, 1, 0],
    [ 0, 0, 1]])

    >>> cos_a = sin_a = sqrt(2)/2
    >>> reflection(3, scalar=cos_a)
    Matrix([
    [sqrt(2)/2,  sqrt(2)/2, 0],
    [sqrt(2)/2, -sqrt(2)/2, 0],
    [        0,          0, 1]])

    >>> r = reflection(3, scalar=(cos_a, sin_a))
    >>> r
    Matrix([
    [1,         0,          0],
    [0, sqrt(2)/2,  sqrt(2)/2],
    [0, sqrt(2)/2, -sqrt(2)/2]])

    >>> r.det()
    -1

    >>> r.T * r == eye(3)
    True

    >>> z = cos_a + sin_a * I
    >>> reflection(3, (1, 2), scalar=z)  # no more random
    Matrix([
    [1,         0,          0],
    [0, sqrt(2)/2,  sqrt(2)/2],
    [0, sqrt(2)/2, -sqrt(2)/2]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    index : tuple(integer, integer) (optional)
        coordinates ``(i,j)`` of transposition and rotation
    scalar : tuple(symbol, symbol) or symbol (optional)

        * either a tuple of cosine value $c$ and sine value $s$
          of the underlying rotation square
        * or just cosine value $c$ of rotation square (then, a corresponding
          sin value $s$ will be drawn randomly)
        * or a complex unit $z$ (complex number with $|z|=1$) so
          $c=re(z)$ and $s=im(z)$.

    seed : random seed (optional)
        see :func:`rotation`

    See Also
    ========
    rotation

    """
    rng = _rand(seed)
    index = index or _sample(range(dim), 2, rng=rng)
    item = transposition(dim, index, seed=seed)
    return rotation(dim, index, scalar, seed=seed) * item


# === normal form matrices, i.e. defined by eigenvalues ===


def diagonal_normal(dim,
                    spec=None,
                    seed=None):
    r"""n x n matrix with random values placed on the diagonal.

    .. _matrices-random-diagonal:

    Explanation
    ===========
    Creates a square diagonal matrix, i.e. matrix with only zero non-diagonal
    entries.
    The  diagonal values will be choose from **spec**,
    the *spectrum* argument which is the set of eigenvalues.

    Examples
    ========

    >>> from sympy.matrices.random import rand, diagonal_normal
    >>> rand.seed(1)

    >>> diagonal_normal(3)
    Matrix([
    [-1,  0, 0],
    [ 0, -1, 0],
    [ 0,  0, 1]])

    >>> diagonal_normal(3, (4,-3,2))  # no more random
    Matrix([
    [4,  0, 0],
    [0, -3, 0],
    [0,  0, 2]])

    >>> diagonal_normal(3, (-2,2))
    Matrix([
    [-2, 0, 0],
    [ 0, 2, 0],
    [ 0, 0, 2]])

    Parameters
    ==========
    dim : integer
        dimension n of matrix
    spec : tuple or list of symbols (optional)
        set of values of which scalars (diagonal entries) are choosen.

        * If **dim** meets the length of **spec**,
          spec will be the eigenvalues (diagonal entries) as it is.
        * If **dim** and the length of **spec** differs,
          the diagonal entries will be choosen randomly from **spec**.
        * If not given **spec** defaults to $\{ -1, 1 \}$.

    seed : random seed (optional)
        see :func:`elementary`

    See Also
    ========
    sympy.matrices.dense.eye
    sympy.matrices.dense.diag

    """
    rng = _rand(seed)
    spec = spec or _elementary_scalars

    # choose spec randomly if dim and len(spec) does not meet
    if not dim == len(spec):
        spec = tuple(_sample(spec, rng=rng) for _ in range(dim))

    # set diagonal entries
    if _ALT:
        # multiplicative matrix construction (only for testing)
        items = list()
        for start, scalar in enumerate(spec):
            items.append(elementary(dim, index=(start, start), scalar=scalar))
        return _multiply(*items)

    else:
        obj = _eye(dim)
        for start, scalar in enumerate(spec):
            obj[start, start] = scalar
        return obj  # eq. to gauss_jordan(dim, rank, eigenvalue_set, dim)


def jordan_normal(dim,
                  spec=None,
                  seed=None):
    r"""n x n matrix in Jordan normal form

    Explanation
    ===========
    A matrix in Jordan normal form is a block matrix
    with only non-zero blocks on the diagonal
    which have the form of a Jordan block $\mathbf{J}$ which is

    .. math::

        \mathbf{J}
        = \left[\begin{array}{cccccc}
            \lambda  & 1       & 0         & \dots     &         & 0     \\
             0       & \lambda & 1         & 0         & \dots   & 0     \\
             0       & \ddots  & \ddots    & \ddots    & \ddots  & 0     \\
             0       & \dots   & 0         & 0         & \lambda & 1     \\
             0       & \dots   &           &           & 0       & \lambda
             \end{array}\right]

    Finally, each $\lambda$ of each Jordan block will be an eigenvalue.

    To set the eigenvalues and block sizes use **spec**.
    Simply provide a sequence of pairs *(eigenvalue, block size)*,
    i.e. tuple of length 2.

    For example **spec** = [(1,2),(2,1),(2,1)]  would lead to

    .. math::

        \mathbf{J}
        = \left[\begin{array}{cccc}
            1 & 1 & 0 & 0 \\
            0 & 1 & 0 & 0 \\
            0 & 0 & 2 & 0 \\
            0 & 0 & 0 & 2
        \end{array}\right]

    Alternatively (sometimes more handy),
    a simple list of eigenvalues would work, too.

    As long as an eigenvalue is repeated in **spec**
    it mandates the same *Jordan* block until
    the values changes or the sequence of eigenvalues
    is interrupted by a **None** entry.

    So the above example is equivalent to **spec** = (1,1,2, None, 2)

    .. math::

    Note, if the length of **spec** (**None** entries excluded)
    does not meet **dim**, blocks will be choosen randomly.

    To meet **dim** the final block might be truncated.

    Examples
    ========

    >>> from sympy.matrices.random import rand, jordan_normal
    >>> rand.seed(1)

    >>> jordan_normal(3, spec=(1,3))
    Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 3]])

    >>> jordan_normal(3, spec=(2,2,2))  # no more random
    Matrix([
    [2, 1, 0],
    [0, 2, 1],
    [0, 0, 2]])

    >>> jordan_normal(6, spec=(2,None,2,2,2,2,0))  # no more random
    Matrix([
    [2, 0, 0, 0, 0, 0],
    [0, 2, 1, 0, 0, 0],
    [0, 0, 2, 1, 0, 0],
    [0, 0, 0, 2, 1, 0],
    [0, 0, 0, 0, 2, 0],
    [0, 0, 0, 0, 0, 0]])

    >>> # equivalent to
    >>> jordan_normal(6, spec=((2,1),(2,4),(0,1)))  # no more random
    Matrix([
    [2, 0, 0, 0, 0, 0],
    [0, 2, 1, 0, 0, 0],
    [0, 0, 2, 1, 0, 0],
    [0, 0, 0, 2, 1, 0],
    [0, 0, 0, 0, 2, 0],
    [0, 0, 0, 0, 0, 0]])

    Parameters
    ==========
    dim : integer
        dimension n of matrix
    spec : tuple or list of symbols (optional)
        set of values of which scalars (diagonal entries) are choosen.

        * If **dim** meets the length of **spec**,
          spec will be the eigenvalues (diagonal entries) as it is.
        * If **dim** and the length of **spec** differs,
          the diagonal entries will be choosen randomly from **spec**.
        * If not given **spec** defaults to $\{ -1, 1 \}$.

    seed : random seed (optional)
        see :func:`jordan`

    See Also
    ========
    diagonal_normal

    """
    rng = _rand(seed)
    spec = spec or _elementary_scalars

    # make spec list of (scalar, size) tuples
    if spec and spec[0] is None:
        raise ValueError("spec argument must not start with None")

    block_list = list()
    val, cnt = None, 0
    for s in spec:
        if s == val:
            cnt += 1
            continue
        elif s is None:
            block_list.append((val, cnt))
            val, cnt = None, 0
            continue
        elif cnt:
            block_list.append((val, cnt))
        if isinstance(s, (tuple, list)) and len(s) == 2:
            block_list.append(s)
            val, cnt = None, 0
            continue
        val, cnt = s, 1
    if cnt:
        block_list.append((val, cnt))
    spec = block_list
    del block_list

    # choose block randomly if dim and len(spec) does not meet
    if not dim == sum(cnt for val, cnt in spec):
        spec = tuple(_sample(spec, rng=rng) for _ in range(dim))

    # set entries of jordan blocks
    if _ALT:
        # multiplicative matrix construction (only for testing)
        items = list()
        start = 0
        for val, cnt in spec:
            end = min(dim, start + cnt)
            items.append(jordan(dim, index=(start, end), scalar=val))
            if end == dim:
                break
            start = end
        return _multiply(*items)

    else:
        obj = _eye(dim)
        start = 0
        for scalar, size in spec:
            end = min(dim, start + size)
            obj[start, start] = scalar
            for i in range(start + 1, end):
                obj[i - 1, i] = 1
                obj[i, i] = scalar
            if end == dim:
                break
            start = end
        return obj


def isometry_normal(dim,
                    spec=None,
                    real=True,
                    seed=None):
    r""" isometry matrix n x n in normal form

    Explanation
    ===========

    Isometries preserve the geometric structure of vectorspaces.
    Let $<-,->$ be either the standard scalar product (over the reals)
    or the standard *Herminte* form (for complex vectorspaces).

    For vectors $x,y$ and a *isometry matrix* $\mathbf{A}$ we have

    .. math::

        x^Ty
        = <x,y>
        = <\mathbf{A}x,\mathbf{A}y>
        = (\mathbf{A}x)^T\mathbf{A}y
        = x^T \mathbf{A}^T\mathbf{A} y

    Hence, $\mathbf{A}^T\mathbf{A} = \mathbf{I}$ for real $\mathbf{A}$
    which is called *orthogonal*
    and $\mathbf{A}^H\mathbf{A} = \mathbf{I}$ for complex $\mathbf{A}$
    (called *unitary*).

    In normal from complex isometries have only diagonal entries of
    norm 1, i.e. for a diagonal element $z$ the following holds.

    .. math::

        |z| = z * \bar{z} = 1

    As - in normal form - real isometry (othogonal) matrices
    may have non diagonal entries but $2 \times 2$ rotation blocks

    .. math::

        \left[\begin{array}{cc} c & s \\ -s & c \end{array}\right]

    with $c^2 + s^2 = 1$.

    .. math::

    So, **spec** is assumed to be a list of entries which are either
    scalars $z$ with $|z| = z * \bar{z} = 1$
    or
    pairs $(c,s)$ with $c^2 + s^2 = 1$
    to form the above rotaion square.

    If a **spec** entry $c$ is neither such a pair nor $|c| = 1$
    a corresponding $s = \pm \sqrt{1-c^2}$ is choosen randomly.

    Examples
    ========

    >>> from sympy.matrices.random import rand, isometry_normal
    >>> rand.seed(1)

    >>> isometry_normal(3)
    Matrix([
    [ sqrt(2)/2, sqrt(2)/2, 0],
    [-sqrt(2)/2, sqrt(2)/2, 0],
    [         0,         0, 1]])
    >>> isometry_normal(3, spec=(0.5, -0.5))
    Matrix([
    [              -0.5, 0.866025403784439, 0],
    [-0.866025403784439,              -0.5, 0],
    [                 0,                 0, 1]])
    >>> isometry_normal(3, spec=(-1, 1j, 1))  # no more random
    Matrix([
    [-1,     0, 0],
    [ 0, 1.0*I, 0],
    [ 0,     0, 1]])
    >>> z = 1+1j
    >>> isometry_normal(3, spec=((-1,), (z/abs(z),), (1,)))  # no more random
    Matrix([
    [-1,                                       0, 0],
    [ 0, 0.707106781186547 + 0.707106781186547*I, 0],
    [ 0,                                       0, 1]])

    Parameters
    ==========
    dim : integer
        dimension n of matrix
    spec : tuple or list of symbols (optional)
        set of values of which scalars (diagonal entries) are choosen.

        Each list item has to be

        * either 1 or -1
        * or any complex unit $z$ (complex number with $|z|=1$)
        * or a tuple of cosine value $c$ and sine value $s$
          of a rotation square
        * or just cosine value $c$ of rotation square (then, a corresponding
          sin value $s$ will be drawn randomly).

        * If **dim** meets the length of **spec**,
          spec will be the eigenvalues (diagonal entries) as it is.
        * If **dim** and the length of **spec** differs,
          the diagonal entries will be choosen randomly from **spec**.
        * If not given **spec** defaults to
          $\{ (\frac{\sqrt(2)}{2}, \frac{\sqrt(2)}{2}), (0, -1) \}$.

    real : bool (optional default **True**)
        flag to force real valued isometry,
        i.e. if **True** all complex units will give a rotation square,
        if **False** even tuple of cosine value $c$ and sine value $s$
        will be treated as complex unit $c + s\ i$.
    seed : random seed (optional)
        see :func:`rotation`

    See Also
    ========
    rotation
    diagonal_normal
    orthogonal
    unitary

    """
    rng = _rand(seed)
    spec = spec or _rotation_scalars

    # make spec list of (scalar,) or (scalar, +/-sqrt(1-scalar**2)) tuples
    block_list = list()
    for c in spec:
        if isinstance(c, (tuple, list)):
            block_list.append(c)
        elif _is_abs_one(c):
            block_list.append((c,))
        else:
            block_list.append(_cs(c, real, rng=rng))
    spec = block_list
    del block_list

    # choose spec randomly if dim and len(spec) does not meet
    if not dim == sum(len(s) for s in spec):
        spec = tuple(_sample(spec, rng=rng) for _ in range(dim))

    # set entries of rotation blocks

    if _ALT:
        # multiplicative matrix construction (only for testing)
        items = list()
        start = 0
        for s in spec:
            end = start + len(s)
            if dim < end:
                # leave the last diagonal entry one
                break
            if len(s) == 1:
                scalar, = s
                items.append(
                    elementary(dim, index=(start, start), scalar=scalar))
            else:
                c, s = s
                items.append(rotation(dim, (start, start + 1), (c, s)))
            start = end
        return _multiply(*items)

    else:
        obj = _eye(dim)
        start = 0
        for s in spec:
            end = start + len(s)
            if dim < end:
                # leave the last diagonal entry one
                break
            if len(s) == 1:
                scalar, = s
                obj[start, start] = scalar
            else:
                c, s = s
                obj[start, start] = c
                obj[start, start + 1] = s
                obj[start + 1, start] = -1 * s
                obj[start + 1, start + 1] = c
            start = end
        return obj


# === compound matrices, i.e. product of base matrices ===


def triangular(dim,
               rank=None,
               scalars=_elementary_scalars,
               units=_elementary_units,
               length=None,
               seed=None):
    r"""n x n upper triangular matrix with random entries.

    Explanation
    ===========
    Matrix with values placed above and on the diagonal. Constructed as
    product of random upper *triangular* :func:`elementary` matrices.

    It is constructed from an invertible triangular matrix $\mathbf{S}$
    of which $r$ basis columns are randomly choosen
    and the $n-r$ columns are build as linear combinations
    of the basis colums.

    Examples
    ========
    >>> from sympy.matrices.random import rand, triangular
    >>> rand.seed(1)

    >>> triangular(3, scalars=(2, -2, 0))
    Matrix([
    [-1, -2,  2],
    [ 0,  1, -2],
    [ 0,  0, -1]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    rank : integer (optional with default dim)
        rank of matrix
    scalars : tuple or list of symbols (optional)
        default values for random choosen scalars of
        elementary matrices to build the invertible matrix
    units : tuple or list of symbols (optional)
        default values for random choosen scalar of
        elementary matrices to build the invertible matrix
        and non-zero diagonal entries
        which are assumed to have a multiplicative inverse
    length : integer (optional with default 2 * **dim**)
        number of invertible matrices to build the resulting matrix.
    seed : random seed (optional)
        see :func:`projection`

    See Also
    ========
    elementary
    square

    """
    rng = _rand(seed)
    if length == 0:
        return regular_to_singular(_eye(dim), rank, rng=rng)
    scalars = scalars or (0,)
    units = units or (1,)
    length = length or 2 * dim
    row_indicies = [_sample(range(dim), rng=rng) for _ in range(length)]
    indicies = [(i, _sample(range(i, dim), rng=rng)) for i in row_indicies]
    scalars = [_sample(units, rng=rng) if i == j
               else _sample(scalars, rng=rng) for i, j in indicies]
    items = [elementary(dim, ix, s) for ix, s in zip(indicies, scalars)]
    return regular_to_singular(_multiply(*items), rank, rng=rng)


def square(dim,
           rank=None,
           scalars=_elementary_scalars,
           units=_elementary_units,
           length=None,
           seed=None):
    r"""a square matrix n x n with a given rank

    Explanation
    ===========
    A full rank square matrix has **rank=dim**.
    Such a matrix is constructed as
    product of :func:`elementary` matrices.

    If **rank** is less than **dim**,
    it is constructed from a full rank matrix $\mathbf{S}$
    by choosing randomly $r$ basis columns
    and build $n-r$ columes as a linear combinations
    of the basis colums.

    Such a square matrix $\mathbf{S}$ represents an endomorphism
    $$f_S:V \rightarrow V, v \mapsto \mathbf{S}\cdot v$$
    of a $n$ dimensional vector space $V$.

    Parameters
    ==========
    dim : integer
        dimension of matrix
    rank : integer (optional with default **dim**)
        rank of matrix
    scalars : tuple or list of symbols (optional)
        default values for random choosen scalar of
        elementary matrices to build the matrix
    units : tuple or list of symbols (optional)
        default values for random choosen scalar of
        elementary matrices to build the matrix
        which are assumed to have a multiplicative inverse
    length : integer (optional with default 2 * **dim**)
        number of invertible matrices
        to build the resulting matrix.
    seed : random seed (optional)
        see :func:`triangular`

    Examples
    ========
    >>> from sympy.matrices.random import rand, square
    >>> rand.seed(1)

    >>> square(3)
    Matrix([
    [-1, -1, -1],
    [-2, -1, -1],
    [ 0,  0,  1]])

    See Also
    ========
    triangular
    invertible
    singular

    """
    rng = _rand(seed)
    if length == 0:
        return regular_to_singular(_eye(dim), rank, rng=rng)

    length = length or 2 * dim
    lwr = triangular(dim, None, scalars, units,
                     int(length / 2), seed=rng)
    upr = triangular(dim, rank, scalars, units,
                     length - int(length / 2), seed=rng)
    return _multiply(_t(lwr), upr)


def invertible(dim,
               scalars=_elementary_scalars,
               units=_elementary_units,
               length=None,
               seed=None):
    r"""an invertible matrix n x n

    Explanation
    ===========

    Same as :func:`square` but with full rank, i.e. **rank=dim**.

    Parameters
    ==========
    dim : integer
        dimension of matrix
    scalars : tuple or list of symbols (optional)
        default values for random choosen scalar of
        elementary matrices to build the matrix
    units : tuple or list of symbols (optional)
        default values for random choosen scalar of
        elementary matrices to build the matrix
        which are assumed to have a multiplicative inverse
    length : integer (optional with default 2 * **dim**)
        number of invertible matrices
        to build the resulting matrix.
    seed : random seed (optional)
        see :func:`triangular`

    Examples
    ========
    >>> from sympy import symbols
    >>> from sympy.matrices.random import rand, invertible
    >>> rand.seed(1)

    >>> invertible(3)
    Matrix([
    [-1, -1, -1],
    [-2, -1, -1],
    [ 0,  0,  1]])

    >>> phi = symbols('phi')
    >>> m = invertible(3, scalars=(1, -1, phi, -phi))
    >>> m
    Matrix([
    [1, 0,     -phi],
    [1, 1, -phi - 1],
    [0, 0,        1]])
    >>> m.det()
    1
    >>> m.inv()
    Matrix([
    [ 1, 0, phi],
    [-1, 1,   1],
    [ 0, 0,   1]])

    """
    return square(dim, None, scalars, units, length, seed=seed)


def singular(dim,
             rank=None,
             scalars=_elementary_scalars,
             units=_elementary_units,
             length=None,
             seed=None):
    r"""a singular square matrix n x n with a given rank.

    Explanation
    ===========

    Same as :func:`square` but with rank defaulting to $dim-1$.

    Examples
    ========
    >>> from sympy.matrices.random import rand, singular
    >>> rand.seed(1)

    >>> m = singular(3, 2)
    >>> m
    Matrix([
    [-1, 1, -1],
    [-2, 2, -1],
    [ 0, 0,  1]])
    >>> m.rank()
    2

    Parameters
    ==========
    dim : integer
        dimension of matrix
    rank : integer (optional with default **dim**-1)
        rank of matrix
    scalars : tuple or list of symbols (optional)
        default values for random choosen scalars of
        elementary matrices to build the matrix
    units : tuple or list of symbols (optional)
        default values for random choosen scalar of
        elementary matrices to build the matrix
        which are assumed to have a multiplicative inverse
    length : integer (optional with default 2 * **dim**)
        number of invertible matrices to build the resulting matrix.
    seed : random seed (optional)
        see :func:`triangular`

    See Also
    ========
    invertible
    triangular

    """
    rank = dim - 1 if rank is None else rank
    return square(dim, rank, scalars, units, length, seed=seed)


# === conjugate matrices, i.e. defined by similarity to a normal from ==


def idempotent(dim,
               rank=None,
               scalars=_elementary_scalars,
               units=_elementary_units,
               length=None,
               seed=None):
    r"""an idempotent square matrix of a given rank s.th. $A*A=A$

    Explanation
    ===========
    An idempotent matrix is a matrix $\mathbf{A}$
    such that $\mathbf{A}^2 = \mathbf{A}$.
    It is constructed as product

    .. math::

        \mathbf{S} \cdot \mathbf{P} \cdot \mathbf{S}^{-1}

    of an :func:`invertible` matrices $\mathbf{S}$
    and a :func:`projection` matrix $\mathbf{P}$ of given rank .

    Examples
    ========
    >>> from sympy.matrices.random import rand, idempotent
    >>> rand.seed(1)

    >>> A = idempotent(3, 2)
    >>> A
    Matrix([
    [1, 0, 0],
    [0, 1, 1],
    [0, 0, 0]])
    >>> A.rank()
    2
    >>> A*A == A
    True

    Parameters
    ==========
    dim : integer
        dimension of matrix
    rank : integer (optional with default **dim**)
        see :func:`projection`
    scalars : tuple or list of symbols (optional)
        see :func:`invertible`
    units : tuple or list of symbols (optional)
        see :func:`invertible`
    length : integer (optional with default 2 * **dim**)
        see :func:`invertible`
    seed : random seed (optional)
        see :func:`invertible`

    See Also
    ========
    elementary
    projection
    invertible
    nilpotent

    References
    ==========
    .. [1] https://en.wikipedia.org/wiki/Idempotence#Idempotent_functions

    """
    rank = dim - 1 if rank is None else rank
    normal_form = projection(dim, (0, rank))
    s = invertible(dim, scalars, units, length, seed=seed)
    return _multiply(_inv(s), normal_form, s)


def nilpotent(dim,
              rank=None,
              scalars=_elementary_scalars,
              units=_elementary_units,
              length=None,
              seed=None):
    r"""a nilpotent matrix of dinemsion n x n.

    Explanation
    ===========
    A *nilpotent* matrix is a matrix $\mathbf{A}$ such that there is
    an integer $n$ with $\mathbf{A}^n = 0$.
    It is build as the product

    .. math::

        \mathbf{S} \cdot \mathbf{T} \cdot \mathbf{S}^{-1}

    where $\mathbf{S}$ is an :func:`invertible`,
    $\mathbf{T}$ a :func:`jordan_normal` matrix
    with zero diagonal entries.

    Examples
    ========
    >>> from sympy import zeros
    >>> from sympy.matrices.random import rand, nilpotent
    >>> rand.seed(1)

    >>> A = nilpotent(3, 2)
    >>> A
    Matrix([
    [-2, -1, -2],
    [ 4,  2,  3],
    [ 0,  0,  0]])
    >>> A.rank()
    2
    >>> A*A*A == zeros(3)
    True

    Parameters
    ==========
    dim : integer
        dimension of matrix
    rank : integer (optional with default **dim**
        rank of the matrix
    scalars : tuple or list of symbols (optional)
        see :func:`invertible`
    units : tuple or list of symbols (optional)
        see :func:`invertible`
    length : integer (optional with default 2 * **dim**)
        see :func:`invertible`
    seed : random seed (optional)
        see :func:`invertible`

    See Also
    ========
    elementary
    jordan_normal
    invertible
    idempotent

    References
    ==========
    .. [1] Lang, S., Algebra, DOI 10.1007/978-1-4613-0041-0, (2002)
    .. [2] https://en.wikipedia.org/wiki/nilpotent

    """
    def numbers_with_sum(n, k):
        """n numbers with sum dim"""
        if n == 1:
            return [k]
        num = _sample(range(k - n)) + 1
        return [num] + numbers_with_sum(n - 1, k - num)

    rng = _rand(seed)
    rank = dim - 1 if rank is None else rank
    index = numbers_with_sum(dim - rank, dim)
    spec = tuple((0, i) for i in index)
    normal_form = jordan_normal(dim, spec=spec, seed=rng)
    s = invertible(dim, scalars, units, length, seed=rng)
    return _multiply(_inv(s), normal_form, s)


def diagonalizable(dim,
                   spec=_elementary_scalars,
                   scalars=_elementary_scalars,
                   units=_elementary_units,
                   length=None,
                   seed=None):
    r"""a square matrix n x n of a given rank

    Explanation
    ===========
    Creates a square matrix of a given rank
    which is diagonalizable.

    The matrix will be a product

    .. math::

        \mathbf{S} \cdot \mathbf{D} \cdot \mathbf{S}^{-1}

    of an :func:`invertible` matrix $\mathbf{S}$ and
    a :func:`diagonal_normal` matrix of given rank
    and eigenvalues (diagonal entries) from **spec**.

    To obtain a matrix with *complex* *eigenvalues*
    but non-complex entries,
    e.g. $\lambda_{1/2}=a \pm b \ i$,
    you better may use a product of an invertible matrix $\mathbf{S}$
    with non-complex entries (**scalars**)
    and where $\mathbf{D}$ is a non-complex block diagonal matrix
    with a block $\mathbf{D}_z$

    .. math::

        \mathbf{D}_z = \left[\begin{array}{cc}a & b \\ -b & a\end{array}\right]

    Examples
    ========
    >>> from sympy.matrices.random import rand, diagonalizable
    >>> rand.seed(1)

    >>> diagonalizable(3)
    Matrix([
    [-1,  0, 0],
    [-2, -3, 4],
    [-2, -2, 3]])

    >>> m = diagonalizable(3, spec=(1,2,2), scalars=(1,2,3), length=10)
    >>> m
    Matrix([
    [ 4,  2, -6],
    [-6, -4, 18],
    [-1, -1,  5]])
    >>> m.eigenvals(multiple=True)
    [1, 2, 2]
    >>> m.jordan_form(calc_transform=False)
    Matrix([
    [1, 0, 0],
    [0, 2, 0],
    [0, 0, 2]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    spec : tuple or list of symbols (optional)
        set of values of which scalars (diagonal entries) are choosen
        see :func:`diagonal_normal`
    scalars : tuple or list of symbols (optional)
        see :func:`invertible`
    units : tuple or list of symbols (optional)
        see :func:`invertible`
    length : integer (optional with default 2 * **dim**)
        see :func:`invertible`
    seed : random seed (optional)
        see :func:`invertible`

    """
    rng = _rand(seed)
    normal_form = diagonal_normal(dim, spec, seed=rng)
    s = invertible(dim, scalars, units, length, seed=rng)
    return _multiply(_inv(s), normal_form, s)


def trigonalizable(dim,
                   spec=_elementary_scalars,
                   scalars=_elementary_scalars,
                   units=_elementary_units,
                   length=None,
                   seed=None):
    r"""Creates a square matrix n x n of a given rank

    Explanation
    ===========
    Creates a square matrix n x n of a given rank
    which is trigonalizable (has echelon form)
    and has eigenvalues from a given set.

    The matrix will be a product

    .. math::

        \mathbf{S} \cdot \mathbf{T} \cdot \mathbf{S}^{-1}

    of an invertible matrix $\mathbf{S}$ and a *Jordan* matrix $\mathbf{T}$
    and eigenvalues (diagonal entries) from **spec**.

    Examples
    ========
    >>> from sympy import sqrt
    >>> from sympy.matrices.random import rand, trigonalizable
    >>> rand.seed(1)

    >>> trigonalizable(3)
    Matrix([
    [-1,  0, 0],
    [-2, -3, 4],
    [-2, -2, 3]])

    >>> m = trigonalizable(3, spec=(1, 1, 3), scalars=(1, sqrt(2), 2))
    >>> m
    Matrix([
    [            1,           1, 0],
    [            0,           1, 0],
    [2*sqrt(2) + 4, sqrt(2) + 2, 3]])
    >>> m.eigenvals(multiple=True)
    [1, 1, 3]
    >>> m.jordan_form(calc_transform=False)
    Matrix([
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 3]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    spec : tuple or list of symbols (optional)
        set of values of which scalars (diagonal entries) are choosen
        see :func:`jordan_normal`
    scalars : tuple or list of symbols (optional)
        see :func:`invertible`
    units : tuple or list of symbols (optional)
        see :func:`invertible`
    length : integer (optional with default 2 * **dim**)
        see :func:`invertible`
    seed : random seed (optional)
        see :func:`invertible`

    See Also
    ========
    jordan_normal
    invertible

    """
    rng = _rand(seed)
    normal_form = jordan_normal(dim, spec, seed=rng)
    s = invertible(dim, scalars, units, length, seed=rng)
    return _multiply(_inv(s), normal_form, s)


# === matrices conjugate by isometries ==


def orthogonal(dim,
               spec=None,
               scalars=_rotation_scalars,
               length=None,
               seed=None):
    r"""an orthogonal matrix n x n

    Explanation
    ===========
    An orthogonal matrix $\mathbf{O}$ is an isometry
    in n dimensional Euclidian space. Constructed as
    product of random :func:`rotation`
    and :func:`reflection` matrices.

    The values for random choosen scalar of rotations
    are from **scalars**.

    If **spec** is given, they will be used to set
    the normal form of $\mathbf{O}$ by an
    :func:`isometry_normal` matrix $\mathbf{D}$.

    Then the resulting orthogonal matrix will be the
    conjugate roduct of $\mathbf{D}$ by some orthogonal $\mathbf{Q}$

    .. math::

        \mathbf{O} = \mathbf{Q} \cdot \mathbf{D} \cdot \mathbf{Q}^T

    Examples
    ========
    >>> from sympy import sqrt, eye, expand
    >>> from sympy.matrices.random import rand, orthogonal
    >>> rand.seed(1)

    >>> orthogonal(3, scalars=(-1,1))
    Matrix([
    [-1,  0, 0],
    [ 0, -1, 0],
    [ 0,  0, 1]])

    >>> s = sqrt(2)/2
    >>> orthogonal(3, scalars=(s,-s), length=2)
    Matrix([
    [       1/2, -sqrt(2)/2,       1/2],
    [       1/2,  sqrt(2)/2,       1/2],
    [-sqrt(2)/2,          0, sqrt(2)/2]])

    >>> o = orthogonal(3, spec=(-1, (s, s)), scalars=(s,), length=2)
    >>> expand(o * o.T) == eye(3)
    True

    Parameters
    ==========
    dim : integer
        dimension of matrix
    spec : tuple or list of symbols (optional),
        set of values of which scalars (diagonal entries or 2 x 2 blocks)
        of :func:`isometry_normal` matrix.

        Each list item has to be

        * either 1 or -1
        * or a tuple of cosine value $c$ and sine value $s$
          of a rotation square
        * or just cosine value $c$ of rotation square (then, a corresponding
          sin value $s$ will be drawn randomly)
        * or a complex unit $z$ (complex number with $|z|=1$) so
          $c=re(z)$ and $s=im(z)$.

    scalars : tuple or list of symbols (optional),
        default values for random choosen scalar of
        rotations to build the orthogonal matrix
        :func:`rotation`
    length : integer (optional with default 2 * **dim**)
        number rotations to build the matrix.
    seed : random seed (optional)
        see :func:`rotation`

    See Also
    ========
    rotation
    reflection
    isometry_normal
    unitary

    """
    rng = _rand(seed)
    if spec is None:
        if length == 0:
            return _eye(dim)
        length = length or 2 * dim
        scalars = [_sample(scalars, rng=rng) for _ in range(length)]
        items = [rotation(dim, scalar=s, seed=rng) for s in scalars]
        return _multiply(*items)
    normal_form = isometry_normal(dim, spec, seed=rng)
    s = orthogonal(dim, scalars=scalars, length=length, seed=rng)
    return _multiply(_t(s), normal_form, s)


def unitary(dim,
            spec=None,
            scalars=_unitary_scalars,
            length=None,
            seed=None):
    r"""an unitary matrix n x n

    Explanation
    ===========
    An unitary matrix is an isometry
    in n dimensional unitary space
    (complex vectorspace with Hermite form).

    Constructed as product
    of random complex :func:`rotation` matrices $\mathbf{A(a,b)}$,
    i.e. where the $s,c$ pair of \mathbf{R(c,s)} is
    actural a pair of complex numbers $a, b$ such that $|a|^2+|b|^2=1$.
    So,

    .. math::

        \left[\begin{array}{cc}a & b \\ -\bar{b} & \bar{a} \end{array}\right]
        \in SU(2).

    Those pairs $b$ and $b$ are constructed from three complexs units
    $z, w, u$ by $a=z*\operatorname{Re}(u)$ and $b=w*\operatorname{Im}(u)$.

    But if **spec** is given,
    the resulting *unitary* matrix $\mathbf{V}$ is constructed as

    .. math::

        \mathbf{V} = \mathbf{U} \cdot \mathbf{D} \cdot \mathbf{U}^H


    where a diagonal unitary matrix $\mathbf{D}$ is
    build with entries from **spec**
    and $U$ is a unitary matrix given as a product of matrices
    $\mathbf{A(a,b)}$ as described above.

    Note, to give an *unitary* matrix,
    **spec** must consist of complex roots of units only,
    i.e. complex munbers $z$, s.th. $|z| = z * \bar{z} = 1$.

    Examples
    ========
    >>> from sympy import sqrt, I, simplify, expand
    >>> from sympy.matrices.random import rand, unitary
    >>> rand.seed(1)
    >>> u = unitary(3)
    >>> expand(u)
    Matrix([
    [                   -I/2,               -1/4 - I/4,              1/4 - 3*I/4],
    [sqrt(2)/4 + sqrt(2)*I/4, -sqrt(2)/4 - sqrt(2)*I/2,               -sqrt(2)/4],
    [              sqrt(2)/2,  sqrt(2)/4 + sqrt(2)*I/4, -sqrt(2)/4 - sqrt(2)*I/4]])

    >>> s = sqrt(2)/2
    >>> z = simplify(s + s * I)
    >>> spec_u = [-1, z, -z]
    >>> unitary(3, spec=spec_u, length=0)  # no more random
    Matrix([
    [-1,                       0,                        0],
    [ 0, sqrt(2)/2 + sqrt(2)*I/2,                        0],
    [ 0,                       0, -sqrt(2)/2 - sqrt(2)*I/2]])

    >>> u = simplify(unitary(3, spec=spec_u))
    >>> u
    Matrix([
    [(1 - I)*(-1 - sqrt(2)*I - I)/4,         1/4 - I/4 + sqrt(2)*I/4,        1/4 - sqrt(2)*I/4 + I/4],
    [      -1/4 - sqrt(2)*I/4 + I/4,  (1 - I)*(-1 - I + sqrt(2)*I)/8, (1 - I)*(-3 - sqrt(2) - 3*I)/8],
    [        -sqrt(2)/4 + 1/4 - I/4, (1 - I)*(3 - 3*I - sqrt(2)*I)/8, (1 - I)*(-1 - I + sqrt(2)*I)/8]])
    >>> expand(u * u.H)
    Matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])

    >>> simplify(u.det())
    I

    Parameters
    ==========
    dim : integer
        dimension of matrix
    spec : tuple or list of symbols (optional),
        set of values of which scalars (diagonal entries)
        of :func:`isometry_normal` matrix.

        Each list item has to be

        * either 1 or -1
        * or any complex unit $z$ (complex number with $|z|=1$)
        * or a tuple of cosine value $c$ and sine value $s$
          of a rotation square (the $z = c + s * I$)
        * or just cosine value $c$ (then, a corresponding
          sin value $s$ will be drawn randomly).

    scalars : tuple or list of symbols with norm 1 (optional),
        default values for random choosen scalar of
        rotations to build the orthogonal matrix
        see :func:`orthogonal`
    length : integer (optional with default 2 * **dim**)
        number rotations to build the matrix.
        see :func:`orthogonal`
    seed : random seed (optional)
        see :func:`orthogonal`

    See Also
    ========
    isometry_normal
    orthogonal

    """
    rng = _rand(seed)
    if spec is None:
        if length == 0:
            return _eye(dim)
        length = length or 2 * dim
        # draw triples of complex units $(z,w,u)$ such that by
        # $c=z*\re(u)$ and $s=w*\im(u)$ each rotation
        # yields a complex rotation matrix
        if len(scalars) < 3:
            scalars = scalars + scalars + scalars
        csc = (lambda z, w, u: (z * _re(u), w * _im(u)))
        scalars = [csc(*_sample(scalars, k=3, rng=rng)) for _ in range(length)]
        items = [rotation(dim, scalar=s, seed=rng) for s in scalars]
        return _multiply(*items)
    normal_form = isometry_normal(dim, spec, real=False, seed=rng)
    s = unitary(dim, scalars=scalars, length=length, seed=rng)
    return _multiply(_c(s), normal_form, s)


def normal(dim,
           spec=_elementary_scalars,
           scalars=None,
           length=None,
           seed=None):
    r""" normal n x n matrix

    Explanation
    ===========

    By definition for a *normal* matrix $\mathbf{A}$

    .. math::

        \mathbf{A}^H \cdot \mathbf{A} = \mathbf{A} \cdot \mathbf{A}^H

    holds. Note, this extends the notion of
    an *unitary* matrix $\mathbf{U}$
    since

    .. math::

        \mathbf{U}^H \cdot \mathbf{U}
        = \mathbf{I}
        = \mathbf{U} \cdot \mathbf{U}^H.

    A matrix $\mathbf{A}$ is *normal* if and only if it is diagonalizable
    by an *unitary* matrix $\mathbf{U}$, i.e.
    $\mathbf{U}^H \cdot \mathbf{A} \cdot \mathbf{U} = \mathbf{D}$
    with diagonal matrix $\mathbf{D}$.
    Since $\mathbf{U}^H = \mathbf{U}^{-1}$
    a *normal* matrix is constructed as

    .. math::

        \mathbf{A} = \mathbf{U} \cdot \mathbf{D} \cdot \mathbf{U}^H.

    The entries of $\mathbf{D}$, which are actually eigenvalues,
    are taken from **spec**,
    while the entries to build $\mathbf{U}$ are taken from **scalars**,
    see :func:`unitary` for details.

    Since *orthogonal* matrices are *unitary*,
    $\mathbf{U}$ will be *orthogonal*
    if **scalars** consists of real entries only.
    So the final matrix $\mathbf{A}$ will be real, too.

    Moreover, $\mathbf{A}$ will be symmetric
    but in contrast to :func:`symmetric`
    with a given spectrum of eigenvalues from **spec**.

    Examples
    ========

    >>> from sympy import I, zeros, expand, simplify, sqrt
    >>> from sympy.matrices.random import rand, normal
    >>> rand.seed(44)

    >>> normal(3)
    Matrix([
    [         0, -sqrt(2)/2, -sqrt(2)/2],
    [-sqrt(2)/2,       -1/2,        1/2],
    [-sqrt(2)/2,        1/2,       -1/2]])

    >>> n = simplify(normal(3, spec=(1 + I, 2 * I, 3)))
    >>> n
    Matrix([
    [1 + I,                           0,                           0],
    [    0,                     3/2 + I, sqrt(2)*(1 - I)*(2 + 3*I)/4],
    [    0, sqrt(2)*(1 - I)*(3 - 2*I)/4,                     3/2 + I]])

    >>> expand(n.T * n - n * n.T) == zeros(3)  # different sample may give True
    False
    >>> expand(n.H * n - n * n.H) == zeros(3)
    True

    >>> cos_a = sin_a = sqrt(2) / 2
    >>> spec_n = 1, 2, 3
    >>> n = simplify(normal(3, spec=spec_n, scalars=(cos_a, sin_a, -1)))
    >>> n
    Matrix([
    [1,    0,    0],
    [0,  5/2, -1/2],
    [0, -1/2,  5/2]])

    >>> ev = n.eigenvals()
    >>> sorted(ev)
    [1, 2, 3]
    >>> all(v in ev for v in spec_n)
    True
    >>> expand(n.T * n - n * n.T) == zeros(3)
    True
    >>> expand(n.H * n - n * n.H) == zeros(3)
    True

    Parameters
    ==========
    dim : integer
        dimension of matrix
    spec : tuple or list of symbols (optional)
        set of values of which scalars (diagonal entries)
        see :func:`isometry_normal` for details
    scalars : tuple or list of symbols (optional)
        see :func:`orthogonal` and :func:`unitary`
    length : integer (optional with default 2 * **dim**)
        see :func:`orthogonal` and :func:`unitary`
    seed : random seed (optional)
        see :func:`orthogonal` and :func:`unitary`

    See Also
    ========
    isometry_normal
    orthogonal
    unitary

    """
    rng = _rand(seed)
    normal_form = diagonal_normal(dim, spec, seed=rng)
    if any(_is_complex(c) for c in spec):
        scalars = scalars or _unitary_scalars
        s = unitary(dim, None, scalars, length, seed=rng)
        return _multiply(_c(s), normal_form, s)
    else:
        scalars = scalars or _rotation_scalars
        s = orthogonal(dim, None, scalars, length, seed=rng)
        return _multiply(_t(s), normal_form, s)


# === symmetric or complex adjoined matrices ===


def symmetric(dim,
              scalars=_elementary_scalars,
              units=_elementary_units,
              length=None,
              seed=None):
    r"""Creates a symmetric square matrix n x n.

    Explanation
    ===========
    A *symmetric* matrix is a matrix such that $\mathbf{A}^T = \mathbf{A}$
    The matrix is simply constructed as

    .. math::

        \mathbf{S} \cdot \mathbf{D} \cdot \mathbf{S}^T

    by an ``invertible`` matrix $\mathbf{S}$
    and a ``diagonal`` matrix $\mathbf{D}$.

    To obtain a matrix with given *eigenvalues*,
    you better may use :func:`normal` matrix, which is a product

    .. math::

        \mathbf{O} \cdot \mathbf{D} \cdot \mathbf{O}^T

    by an *orthogonal* matrix $\mathbf{O}$ and
    a *diagonal* matrix $\mathbf{D}$ with given eigenvalues (diagonal entries).

    Examples
    ========
    >>> from sympy.matrices.random import rand, symmetric
    >>> rand.seed(1)

    >>> symmetric(3)
    Matrix([
    [5, 3, 3],
    [3, 2, 2],
    [3, 2, 3]])

    For a symmetric matrix with given eigenvalues
    :func:`normal` works

    >>> from sympy.matrices.random import rand, normal
    >>> rand.seed(1)

    >>> n = normal(3, spec=(1,2,3))
    >>> sorted(n.eigenvals(multiple=True))
    [1, 2, 3]

    Parameters
    ==========
    dim : integer
        dimension of matrix
    scalars : tuple or list of symbols (optional)
        see :func:`invertible`
    units : tuple or list of symbols (optional)
        see :func:`invertible`
    length : integer (optional with default 2 * **dim**)
        see :func:`invertible`
    seed : random seed (optional)
        see :func:`invertible`

    See Also
    ========
    invertible
    normal
    hermite

    """
    rng = _rand(seed)
    s = invertible(dim, scalars, units, length, seed=rng)
    return _multiply(_t(s), s)


def hermite(dim,
            scalars=_elementary_scalars,
            units=_elementary_units,
            length=None,
            seed=None):
    r"""Creates a Hermintian square matrix n x n.

    Explanation
    ===========
    A *Hermintian* matrix is a matrix such that $\mathbf{A}^H = \mathbf{A}$,
    where $\mathbf{A}^H$ means complex adjoint matrix,
    i.e. the transposed matrix with complex conjugate entries.

    .. math::

        \text{If } (\mathbf{A})_{ij} = A_{ij}
        \text{ then } (\mathbf{A}^H)_{ij} = \bar{A}_{ji}.

    The matrix is simply constructed as

    .. math::

        \mathbf{S} \cdot \mathbf{D} \cdot \mathbf{S}^H

    by an ``invertible`` matrix $\mathbf{S}$
    and a ``diagonal`` matrix $\mathbf{D}$ of given rank.

    To obtain a matrix with given *eigenvalues*,
    you better may use :func:`normal` matrix, which is a product

    .. math::

        \mathbf{U} \cdot \mathbf{D} \cdot \mathbf{U}^H

    by an *unitary* matrix $\mathbf{U}$ and
    a *diagonal* matrix $\mathbf{D}$ with given eigenvalues (diagonal entries).

    Examples
    ========
    >>> from sympy.matrices.random import rand, hermite
    >>> rand.seed(1)

    >>> z = complex(1,-1)
    >>> hermite(3, (z, ), length=1)
    Matrix([
    [          1, 0,                     1.0 - 1.0*I],
    [          0, 1,                               0],
    [1.0 + 1.0*I, 0, 1 + (1.0 - 1.0*I)*(1.0 + 1.0*I)]])

    Parameters
    ==========
    dim : integer
        dimension of matrix
    scalars : tuple or list of symbols (optional)
        see :func:`invertible`
    units : tuple or list of symbols (optional)
        see :func:`invertible`
    length : integer (optional with default 2 * **dim**)
        see :func:`invertible`
    seed : random seed (optional)
        see :func:`invertible`

    See Also
    ========
    invertible
    normal
    hermite

    """
    rng = _rand(seed)
    s = invertible(dim, scalars, units, length, seed=rng)
    return _multiply(_c(s), s)
