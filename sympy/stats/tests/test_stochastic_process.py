from sympy import (S, symbols, FiniteSet, Eq, Matrix, MatrixSymbol, Float, And)
from sympy.stats import DiscreteMarkovChain, P, TransitionMatrixOf, E
from sympy.stats.rv import RandomIndexedSymbol
from sympy.stats.symbolic_probability import Probability, Expectation
from sympy.stats.joint_rv import JointDistribution
from sympy.utilities.pytest import raises

def test_DiscreteMarkovChain():

    # pass only the name
    X = DiscreteMarkovChain("X")
    assert X.state_space == S.Reals
    assert X.index_set == S.Naturals0
    assert X.transition_probabilities == None
    t = symbols('t', positive=True, integer=True)
    assert isinstance(X[t], RandomIndexedSymbol)
    assert E(X[0]) == Expectation(X[0])
    raises(TypeError, lambda: DiscreteMarkovChain(1))
    raises(NotImplementedError, lambda: X(t))

    # pass name and state_space
    Y = DiscreteMarkovChain("Y", [1, 2, 3])
    assert Y.transition_probabilities == None
    assert Y.state_space == FiniteSet(1, 2, 3)
    assert P(Eq(Y[2], 1), Eq(Y[0], 2)) == Probability(Eq(Y[2], 1), Eq(Y[0], 2))
    assert E(X[0]) == Expectation(X[0])
    raises(TypeError, lambda: DiscreteMarkovChain("Y", dict((1, 1))))

    # pass name, state_space and transition_probabilities
    T = Matrix([[0.5, 0.2, 0.3],[0.2, 0.5, 0.3],[0.2, 0.3, 0.5]])
    TS = MatrixSymbol('T', 3, 3)
    Y = DiscreteMarkovChain("Y", [0, 1, 2], T)
    YS = DiscreteMarkovChain("Y", [0, 1, 2], TS)
    assert Y.joint_distribution(1, Y[2], 3) == JointDistribution(Y[1], Y[2], Y[3])
    raises(ValueError, lambda: Y.joint_distribution(Y[1].symbol, Y[2].symbol))
    assert P(Eq(Y[3], 2), Eq(Y[1], 1)).round(2) == Float(0.36, 2)
    assert str(P(Eq(YS[3], 2), Eq(YS[1], 1))) == \
        "T[0, 2]*T[1, 0] + T[1, 1]*T[1, 2] + T[1, 2]*T[2, 2]"
    TO = Matrix([[0.25, 0.75, 0],[0, 0.25, 0.75],[0.75, 0, 0.25]])
    assert P(Eq(Y[3], 2), Eq(Y[1], 1) & TransitionMatrixOf(Y, TO)).round(3) == Float(0.375, 3)
    assert E(Y[3], evaluate=False) == Expectation(Y[3])
    assert E(Y[3], Eq(Y[2], 1)).round(2) == Float(1.1, 3)
    TSO = MatrixSymbol('T', 4, 4)
    raises(ValueError, lambda: str(P(Eq(YS[3], 2), Eq(YS[1], 1) & TransitionMatrixOf(YS, TSO))))
    raises(TypeError, lambda: DiscreteMarkovChain("Z", [0, 1, 2], symbols('M')))
    raises(ValueError, lambda: DiscreteMarkovChain("Z", [0, 1, 2], MatrixSymbol('T', 3, 4)))
    raises(IndexError, lambda: str(P(Eq(YS[3], 3), Eq(YS[1], 1))))
    raises(ValueError, lambda: str(P(Eq(YS[1], 1), Eq(YS[2], 2))))
    raises(ValueError, lambda: E(Y[3], Eq(Y[2], 6)))
    raises(ValueError, lambda: E(Y[2], Eq(Y[3], 1)))


    # extended tests for probability queries
    TO1 = Matrix([[S(1)/4, S(3)/4, 0],[S(1)/3, S(1)/3, S(1)/3],[0, S(1)/4, S(3)/4]])
    assert P(And(Eq(Y[2], 1), Eq(Y[1], 1), Eq(Y[0], 0)),
            Eq(Probability(Eq(Y[0], 0)), S(1)/4) & TransitionMatrixOf(Y, TO1)) == S(1)/16
    assert P(And(Eq(Y[2], 1), Eq(Y[1], 1), Eq(Y[0], 0)), TransitionMatrixOf(Y, TO1)) == \
            Probability(Eq(Y[0], 0))/4
    raises (ValueError, lambda: str(P(And(Eq(Y[2], 1), Eq(Y[1], 1), Eq(Y[0], 0)), Eq(Y[1], 1))))
