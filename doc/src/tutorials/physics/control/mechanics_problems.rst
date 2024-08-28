.. _mechanics_problems-physics:

=============================================
Mechanics Problems using StateSpace
=============================================

Below are some Mechanics problems that can be solved using
StateSpace.

Example 1
---------

        .. image:: Mechanics_Problems_Q1.svg
           :align: center

A spring-mass-damping system can be modeled using a mass (m), a spring with a constant (k), and a damper with a damping coefficient (b). The spring force is proportional to the displacement of the mass, and the damping force is proportional to the velocity of the mass.
Find the frequency response of the system.
The free-body diagram for this system is shown below:

        .. image:: Mechanics_Problems_Q1_FBD.svg
           :align: center

The equation of motion for the mass-spring-damper system is given by:

.. math::

   m\ddot{x} + b\dot{x} + kx = F(t)

where:

* :math:`x` is the displacement of the mass,
* :math:`\dot{x}` is the velocity of the mass,
* :math:`\ddot{x}` is the acceleration of the mass,
* :math:`F(t)` is the external force applied to the system.

To determine the state-space representation of the mass-spring-damper system, we reduce the second-order differential equation to a set of two first-order differential equations. We choose the position and velocity as our state variables:

.. math::

   x_1 = x \quad \text{and} \quad x_2 = \dot{x}

The state equations become:

.. math::

   \dot{x}_1 = x_2 \\

   \dot{x}_2 = -\frac{k}{m}x_1 - \frac{b}{m}x_2 + \frac{1}{m}F(t)\\

The state-space can be represented by:

.. math::

   \mathbf{A} = \begin{bmatrix} 0 & 1 \\ -\frac{k}{m} & -\frac{b}{m} \end{bmatrix}, \quad
   \mathbf{B} = \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}, \quad
   \mathbf{C} = \begin{bmatrix} 1 & 0 \end{bmatrix}

The state equation can be written as

.. math::

   \dot{x} =
   \begin{bmatrix}
   \dot{x} \\
   \ddot{x}
   \end{bmatrix}
   =
   \begin{bmatrix}
   0 & 1 \\
   -\frac{k}{m} & -\frac{b}{m}
   \end{bmatrix}
   \begin{bmatrix}
   x \\
   \dot{x}
   \end{bmatrix}
   +
   \begin{bmatrix}
   0 \\
   \frac{1}{m}
   \end{bmatrix}
   F(t)

Using SymPy's Control Systems Toolbox (CST), we can define the state-space representation and convert it to the transfer function.

Solution
^^^^^^^^

The following code demonstrates how to define the state-space representation of the spring-mass-damper system and convert it to a transfer function using SymPy:

    >>> from sympy import symbols, Matrix
    >>> from sympy.physics.control import *

    Define the variables

    >>> m, k, b = symbols('m k b')

    Define the state-space matrices

    >>> A = Matrix([[0, 1], [-k/m, -b/m]])
    >>> B = Matrix([[0], [1/m]])
    >>> C = Matrix([[1, 0]])
    >>> D = Matrix([[0]])

    Create the StateSpace model

    >>> ss = StateSpace(A, B, C, D)
    >>> ss
    StateSpace(Matrix([
    [      0,       1],
    [-k/m, -b/m]]), Matrix([
    [    0],
    [1/m]]), Matrix([
    [1, 0]]), Matrix([[0]]))

    Converting StateSpace to TransferFunction by rewrite method.

    >>> tf = sp.rewrite(TransferFunction)[0][0]
    >>> tf
    TransferFunction(1, b*s + k + m*s**2, s)

References
^^^^^^^^^^
1. `ctms.engin.umich.edu <https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=SystemModeling>`_
