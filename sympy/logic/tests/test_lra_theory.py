from sympy.concrete.summations import Sum
from sympy.core.function import Function
from sympy.core.numbers import (I, Rational, oo, pi)
from sympy.core.relational import (Eq, Ge, Gt, Le, Lt, Ne)
from sympy.core.singleton import S
from sympy.core.symbol import (Dummy, Symbol, symbols)
from sympy.matrices.dense import Matrix
from sympy.assumptions.ask import Q
from sympy.functions.elementary.complexes import Abs
from sympy.functions.elementary.exponential import (exp, log)
from sympy.functions.elementary.miscellaneous import (root, sqrt)
from sympy.functions.elementary.piecewise import Piecewise
from sympy.functions.elementary.trigonometric import (cos, sin, tan)
from sympy.integrals.integrals import Integral
from sympy.logic.boolalg import (And, Or)
from sympy.polys.polytools import (Poly, PurePoly)
from sympy.sets.sets import (FiniteSet, Interval, Union)
from sympy.polys.rootoftools import rootof
from sympy.solvers.solvers import solve
from sympy.solvers.solveset import solveset, linear_eq_to_matrix
from sympy.abc import x, y, z, a
from sympy.assumptions.cnf import CNF, EncodedCNF
from sympy.core.sympify import sympify

from sympy.logic.algorithms.lra_theory import LRASolver, Boundry, standardize_binrel


def make_random_problem(num_variables=2, num_constraints=2, sparsity=.1, rational=True,
                        disable_strict = False, disable_nonstrict=False, disable_equality=False):
    from sympy.core.random import random, choice, randint
    from sympy.core.sympify import sympify
    from sympy.ntheory.generate import randprime
    def rand(sparsity=sparsity):
        if random() < sparsity:
            print("sparity")
            return sympify(0)
        if rational:
            int1, int2 = [randprime(0, 50) for _ in range(2)]
            return Rational(int1, int2) * choice([-1, 1])
        else:
            return randint(1, 10) * choice([-1, 1])

    variables = symbols('x1:%s' % (num_variables + 1))
    constraints = []
    for _ in range(num_constraints):
        lhs, rhs = sum(rand() * x for x in variables), rand(sparsity=0) # sparsity=0  bc of bug with smtlib_code
        r = random()
        options = []
        if not disable_equality:
            options += [Eq(lhs, rhs)]
        if not disable_nonstrict:
            options += [lhs <= rhs, lhs >= rhs]
        if not disable_strict:
            options += [lhs < rhs, lhs > rhs]

        constraints.append(choice(options))

    return constraints

def check_if_satisfiable_with_z3(constraints):
    from sympy.external.importtools import import_module
    from sympy.printing.smtlib import smtlib_code
    from sympy.logic.boolalg import And
    boolean_formula = And(*constraints)
    z3 = import_module("z3")
    if z3:
        smtlib_string = smtlib_code(boolean_formula)
        s = z3.Solver()
        s.from_string(smtlib_string)
        res = str(s.check())
        if res == 'sat':
            return True
        elif res == 'unsat':
            return False
        else:
            raise ValueError(f"z3 was not able to check the satisfiability of {boolean_formula}")

def find_rational_assignment(constr, assignment, iter=20):
    eps = sympify(1)

    for _ in range(iter):
        assign = {key: val[0] + val[1]*eps for key, val in assignment.items()}
        try:
            for cons in constr:
                assert cons.subs(assign) == True
            return assign
        except AssertionError:
            eps = eps/2

    return None



def test_from_encoded_cnf():
    # phi = (x >= 0) & ((x + y <= 2) | (x + 2 * y - z >= 6)) & (Eq(x + y, 2) | (x + 2 * y - z >= 4))
    # cnf = CNF.from_prop(phi)
    # enc = EncodedCNF()
    # enc.from_cnf(cnf)
    # lra, _, _ = LRASolver.from_encoded_cnf(enc)
    # bound_consts = set(bound.bound for bound in lra.boundry_enc.values())
    # assert bound_consts == {0, 2, -6, 2, -4}
    #
    # m = Matrix([[1, 0,  1, -2, -1],
    #             [0, 1, -1,  1,  1]])
    # assert lra.A.rref() == m.rref()


    from sympy.core.relational import StrictLessThan, StrictGreaterThan
    import itertools

    special_cases = []; x1, x2, x3 = symbols("x1 x2 x3")
    #special_cases.append([x1 - 3 * x2 <= -5, 6 * x1 + 4 * x2 <= 0, -7 * x1 + 3 * x2 <= 3]) bug with smtlib_code
    special_cases.append([-3 * x1 >= 3, Eq(4 * x1, -1)])
    special_cases.append([-4 * x1 < 4, 6 * x1 <= -6])
    special_cases.append([-3 * x2 >= 7, 6 * x1 <= -5, -3 * x2 <= -4])
    special_cases.append([-2 * x1 - 2 * x2 >= 7, -9 * x1 >= 7, -6 * x1 >= 5])
    special_cases.append([2 * x1 > -3, -9 * x1 < -6, 9 * x1 <= 6])

    feasible_count = 0
    for i in range(300):
        if i % 8 == 0:
            constraints = make_random_problem(num_variables=1, num_constraints=2, rational=False)
        elif i % 8 == 1:
            constraints = make_random_problem(num_variables=2, num_constraints=4, rational=False, disable_equality=True,
                                              disable_nonstrict=True)
        elif i % 8 == 2:
            constraints = make_random_problem(num_variables=2, num_constraints=4, rational=False, disable_strict=True)
        elif i % 8 == 3:
            constraints = make_random_problem(num_variables=3, num_constraints=12, rational=False)
        else:
            constraints = make_random_problem(num_variables=3, num_constraints=6, rational=False)

        if i < len(special_cases):
            constraints = special_cases[i-1]

        # constraints = make_random_problem(num_variables=2, num_constraints=4, rational=False, disable_strict=False,
        #                                  disable_nonstrict=False, disable_equality=False)
        #constraints = [-2*x1 < -4, 9*x1 > -9]
        #constraints = [-6*x1 >= -1, -8*x1 + x2 >= 5, -8*x1 + 7*x2 < 4, x1 > 7]
        #constraints = [Eq(x1, 2), Eq(5*x1, -2), Eq(-7*x2, -6), Eq(9*x1 + 10*x2, 9)]
        #constraints = [Eq(3*x1, 6), Eq(x1 - 8*x2, -9), Eq(-7*x1 + 5*x2, 3), Eq(3*x2, 7)]
        #constraints = [-4*x1 < 4, 6*x1 <= -6]
        #constraints = [-3*x1 + 8*x2 >= -8, -10*x2 > 9, 8*x1 - 4*x2 < 8, 10*x1 - 9*x2 >= -9]
        #constraints= [x1 + 5*x2 >= -6, 9*x1 - 3*x2 >= -9, 6*x1 + 6*x2 < -10, -3*x1 + 3*x2 < -7]
        #constraints = [-9*x1 < 7, -5*x1 - 7*x2 < -1, 3*x1 + 7*x2 > 1, -6*x1 - 6*x2 > 9]
        #constraints = [9*x1 - 6*x2 >= -7, 9*x1 + 4*x2 < -8, -7*x2 <= 1, 10*x2 <= -7]

        if False in constraints or True in constraints:
            continue

        print(constraints)
        phi = And(*constraints)
        if phi == False:
            continue
        cnf = CNF.from_prop(phi); enc = EncodedCNF()
        enc.from_cnf(cnf)
        assert all(0 not in clause for clause in enc.data)
        lra, x_subs, s_subs = LRASolver.from_encoded_cnf(enc)
        print(lra.A)
        print(lra.boundry_enc)
        lra.run_checks = True
        s_subs_rev = {value: key for key, value in s_subs.items()}
        lits = set(lit for clause in enc.data for lit in clause)

        bounds = [(lra.boundry_enc[l], l) for l in lits if l in lra.boundry_enc]
        bounds = sorted(bounds, key=lambda x: (str(x[0].var), x[0].bound, str(x[0].upper))) # to remove nondeterminism

        for b, l in bounds:
            if lra.result and lra.result[0] == "UNSAT":
                print("Constraints are unsatisfiable")
                break
            print("var:", b.var, "bound:", b.bound, "upper:", b.upper, "strict:", b.strict)
            lra.assert_enc_boundry(l)
            print(lra.assign, lra.lower, lra.upper)

        feasible = lra.check()
        print(feasible)
        if feasible[0] == "SAT":
            feasible_count += 1
            assert check_if_satisfiable_with_z3(constraints) is True
            cons_funcs = [cons.func for cons in constraints]
            assignment = feasible[1]
            constraints = [constr.subs(x_subs) for constr in constraints]
            if not (StrictLessThan in cons_funcs or StrictGreaterThan in cons_funcs):
                assignment = {key: value[0] for key, value in assignment.items()}
                for cons in constraints:
                    assert cons.subs(assignment) == True

            else:
                rat_assignment = find_rational_assignment(constraints, assignment)
                assert rat_assignment is not None
        else:
            assert check_if_satisfiable_with_z3(constraints) is False

            conflict = feasible[1]
            conflict = {lra.boundry_enc[-l].get_inequality() for l in conflict}
            conflict = {clause.subs(s_subs_rev) for clause in conflict}
            assert check_if_satisfiable_with_z3(conflict) is False

            # check that conflict clause is probably minimal
            for subset in itertools.combinations(conflict, len(conflict)-1):
                assert check_if_satisfiable_with_z3(subset) is True

    print("\nnumber of feasible problems: ", feasible_count)


# TODO: test pivot method

def test_preprocessing():
    s1, s2 = symbols("s1 s2")

    # Test preprocessing
    # Example is from section 3 of paper.
    phi = (x >= 0) & ((x + y <= 2) | (x + 2 * y - z >= 6)) & (Eq(x + y, 2) | (x + 2 * y - z > 4))
    phi = Q.real(a) | phi
    phi_prime = Q.real(a) | (x >= 0) & (Eq(s1, 2) | (s2 > 4)) & ((s2 >= 6) | (s1 <= 2))
    eqs = [Eq(s1, x + y), Eq(s2, x + 2 * y - z)]
    m, _ = linear_eq_to_matrix(eqs, [x, y, z, s1, s2])
    m = -m  # identity matrix should be negative
    preprocessed, lra, _ = LRASolver.preprocess(phi, [x, y, z])
    assert preprocessed != phi_prime  # should be identical save for s1 and s2 are dummy variables
    assert phi.args[0] == Q.real(a)  # must handle non-relational predicate objects
    assert lra.A == m
    assert lra.slack != [s1, s2]
    assert len(lra.slack) == 2
    assert lra.nonslack == [x, y, z]

    # Test how preprocessing handles functions
    g = Function('g')(x)
    f = Function('f')()
    phi = (g + x + f >= 2)
    phi_prime = (s1 >= 2)
    eqs = [Eq(s1, g + x + f)]
    m, _ = linear_eq_to_matrix(eqs, [x, f, g, s1])
    m = -m  # identity matrix should be negative
    preprocessed, lra, _ = LRASolver.preprocess(phi, [g, f, x])
    assert preprocessed != phi_prime
    assert lra.A == m
    assert lra.slack != [s1]
    assert len(lra.slack) == 1
    assert lra.nonslack == [g, f, x]


def test_LRA_solver():
    s1, s2 = symbols("s1 s2")
    # Empty matrix should be handled.
    # If the preprocessing step doesn't do anything, then the matrix is empty.
    m = Matrix()
    lra = LRASolver(m, [], [x, y])
    assert lra.assert_con(Q.ge(x, 0)) == ("OK", None)
    assert lra.assert_con(Q.ge(x, -1)) == ('SAT', {x: 0, y: 0})
    assert lra.assert_con(Q.le(x, -1)) == ('UNSAT', {x <= -1, x >= 0})

    m = Matrix()
    lra = LRASolver(m, [], [x, y])
    assert lra.assert_con(Q.le(x, -1)) == ("OK", None)
    assert lra.assert_con(Q.ge(x, 0)) == ('UNSAT', {x <= -1, x >= 0})

    m = Matrix([[-1, -1, 1, 0], [-2, 1, 0, 1]])
    #assert LRASolver._pivot(m, 0, 0) == Matrix([[1, 1, -1, 0], [0, 3, -2, 1]])

    # Example from page 89–90 of
    # "A Fast Linear-Arithmetic Solver for DPLL(T)"
    equations = [Eq(s1, -x + y), Eq(s2, x + y)]
    A, _ = linear_eq_to_matrix(equations, [x, y, s1, s2])
    A = -A # the identity matrix should be negative
    lra = LRASolver(A, [s1, s2], [x, y])
    assert lra.check() == ('SAT', {x: 0, y: 0, s1: 0, s2: 0})
    assert {v: lra.assign[v] for v in lra.all_var} == {x:0, y:0, s1:0, s2:0}

    assert lra.assert_con(x <= -4) == ("OK", None)
    assert lra.check() == ('SAT', {x: -4, y: 0, s1: 4, s2: -4})
    assert {v: lra.assign[v] for v in lra.all_var} == {x:-4, y:0, s1: 4, s2:-4}

    assert lra.assert_con(x >= -8) == ("OK", None)
    assert lra.check() == ('SAT', {x: -4, y: 0, s1: 4, s2: -4})
    assert {v: lra.assign[v] for v in lra.all_var} == {x:-4, y:0, s1:4, s2:-4}

    # note that this is the first time a pivot is used
    assert lra.assert_con(s1 <= 1) == ("OK", None)
    assert lra.check() == ('SAT', {x: -4, y: -3, s1: 1, s2: -7})
    assert {v: lra.assign[v] for v in lra.all_var} == {x:-4, y:-3, s1:1, s2:-7}

    assert lra.assert_con(s2 >= -3) == ("OK", None)
    assert lra.check() == ('UNSAT', {s1 <= 1, x <= -4, s2 >= -3})


    r1 = x <= 0
    r2 = y <= 0
    r3 = s1 >= 2
    equations = [Eq(s1, x+y)]
    A, _ = linear_eq_to_matrix(equations, [x, y, s1])
    A = -A  # the identity matrix should be negative
    lra = LRASolver(A, [s1], [x, y])
    lra.assert_con(x <= 0)
    lra.assert_con(y <= 0)
    lra.assert_con(s1 >= 2)
    assert lra.check() == ('UNSAT', {s1 >= 2, x <= 0, y <= 0})


    # test potential edge case
    r1 = x <= 1
    r2 = -x <= -5
    lra = LRASolver(Matrix(), [], [x])
    lra.assert_con(x <= 1)