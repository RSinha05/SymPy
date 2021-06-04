from sympy.physics.mechanics.body import Body
from sympy.physics.vector import Vector, dynamicsymbols
from abc import ABC, abstractmethod

__all__ = ['Joint', 'PinJoint', 'SlidingJoint']


class Joint(ABC):
    """Abstract Base class for all specific joints.

    Explanation
    ===========

    A Joint connects two bodies (a parent and child) by adding different degrees
    of freedom to child with respect to parent.
    This is the base class for all specific joints and holds all common methods
    acting as an interface for all joints. Custom joint can be created by
    inheriting Joint class and defining all abstract functions.

    Parameters
    ==========

    name : string
        The Joint's name which makes it unique.
    parent : Body
        The parent body of joint.
    child : Body
        The child body of joint.
    dist : Vector, optional
        The vector distance between parent's masscenter and child's masscenter.
    coordinates: List of dynamicsymbols/dynamicsymbol, optional
        Coordinates of joint.
    speeds : List of dynamicsymbols/dynamicsymbol, optional
        Speeds of joint.
    parent_joint_pos : Vector, optional
        Defines the joint's point where the parent will be connected to child.
        Default value is masscenter of Parent Body.
    child_joint_pos : Vector, optional
        Defines the joint's point where the child will be connected to parent.
        Default value is masscenter of Child Body.
    parent_axis : Vector, optional
        Axis of parent frame which would be be aligned with child's
        axis. Default is x axis in parent's frame.

    """

    def __init__(self, name, parent, child, dist=None, coordinates=None, speeds=None, parent_joint_pos=None,
        child_joint_pos=None, parent_axis=None):

        if not isinstance(name, str):
            raise TypeError('Supply a valid name.')
        self._name = name

        if not isinstance(parent, Body):
            raise TypeError('Parent must be an instance of Body.')
        self._parent = parent

        if not isinstance(child, Body):
            raise TypeError('Child must be an instance of Body.')
        self._child = child

        if dist is not None:
            if not isinstance(dist, Vector):
                raise TypeError('Distance between masscenters should be a Vector.')
            self._set_masscenter_pos(dist)

        self._coordinates = self._generate_coordinates(coordinates)
        self._speeds = self._generate_speeds(speeds)
        self._kdes = self._generate_kdes()

        self._parent_axis = self._axis(parent, parent_axis)

        self._parent_joint = self._locate_joint_pos(parent,parent_joint_pos)
        self._child_joint = self._locate_joint_pos(child, child_joint_pos)

        self._orient_frames()
        self._set_angular_velocity()
        self._set_linear_velocity()

    def __str__(self):
        return self._name

    def __repr__(self):
        return self.__str__()

    def parent(self):
        """ Parent body of Joint."""
        return self._parent

    def child(self):
        """ Child body of Joint."""
        return self._child

    def coordinates(self):
        """ List of coordinates of Joint."""
        return self._coordinates

    def speeds(self):
        """ List of speeds of Joint."""
        return self._speeds

    def kdes(self):
        """ KDE of Joint."""
        return self._kdes

    @abstractmethod
    def _generate_coordinates(self, coordinates):
        """ Generate list of coordinates."""
        pass

    @abstractmethod
    def _generate_speeds(self, speeds):
        """ Generate list of speeds. """
        pass

    @abstractmethod
    def _orient_frames(self):
        """Orient frames as per the joint"""
        pass

    @abstractmethod
    def _set_angular_velocity(self):
        pass

    @abstractmethod
    def _set_linear_velocity(self):
        pass

    def _generate_kdes(self):
        kdes = []
        t = dynamicsymbols._t
        for i in range(len(self._coordinates)):
            kdes.append(-self._coordinates[i].diff(t) + self._speeds[i])
        return kdes

    def _axis(self, body, ax):
        if ax is None:
            ax = body.frame.x
            return ax
        if not isinstance(ax, Vector):
            raise TypeError("Axis must be of type Vector. Example-> A.x wehere 'A' is ReferenceFrame.")
        return ax

    def _locate_joint_pos(self, body, joint_pos):
        if joint_pos is None:
            joint_pos = Vector(0)
        if not isinstance(joint_pos, Vector):
            raise ValueError('Joint Position must be supplied as Vector.')
        return body.masscenter.locatenew(self._name + '_' + body.name + '_joint', joint_pos)

    def _set_masscenter_pos(self, dist):
        self._child.masscenter.set_pos(self._parent.masscenter, dist)


class PinJoint(Joint):
    """Pin (Revolute) Joint.

    Explanation
    ===========

    It is defined such that the child body rotates with respect to the parent body
    about the body fixed parent axis through the angle theta. The point of joint
    (pin joint) is defined by two points in each body which coincides together.
    In addition, the child's reference frame can be arbitrarily rotated a constant
    amount with respect to the parent axis by passing an axis in child body which
    must align with parent axis after the rotation.

    Parameters
    ==========

    name : string
        The Joint's name which makes it unique.
    parent : Body
        The parent body of joint.
    child : Body
        The child body of joint.
    dist : Vector
        The vector distance between parent's masscenter and child's masscenter.
    coordinates: dynamicsymbol, optional
        Coordinates of joint.
    speeds : dynamicsymbol, optional
        Speeds of joint.
    parent_joint_pos : Vector, optional
        Defines the joint's point where the parent will be connected to child.
        Default value is masscenter of Parent Body.
    child_joint_pos : Vector, optional
        Defines the joint's point where the child will be connected to parent.
        Default value is masscenter of Child Body.
    parent_axis : Vector, optional
        Axis of parent frame which would be be aligned with child's
        axis. Default is x axis in parent's frame.

    Examples
    =========

    >>> from sympy.physics.mechanics import Body, PinJoint
    >>> parent = Body('parent')
    >>> child = Body('child')
    >>> P = PinJoint('P', parent, child, child.frame.x)
    >>> P.coordinates()
    [P_theta(t)]
    >>> P.speeds()
    [P_omega(t)]

    """

    def __init__(self, name, parent, child, dist, coordinates=None, speeds=None, parent_joint_pos=None,
        child_joint_pos=None, parent_axis=None):

        super().__init__(name, parent, child, dist, coordinates, speeds, parent_joint_pos, child_joint_pos,
            parent_axis)

    def _generate_coordinates(self, coordinate):
        coordinates = []
        if coordinate is None:
            theta = dynamicsymbols(self._name + '_theta')
            coordinate = theta
        coordinates.append(coordinate)
        return coordinates

    def _generate_speeds(self, speed):
        speeds = []
        if speed is None:
            omega = dynamicsymbols(self._name + '_omega')
            speed = omega
        speeds.append(speed)
        return speeds

    def _orient_frames(self):
        self._child.frame.orient_axis(self._parent.frame, self._parent_axis, self._coordinates[0])

    def _set_angular_velocity(self):
        self._child.frame.set_ang_vel(self._parent.frame, self._speeds[0] * self._parent_axis)

    def _set_linear_velocity(self):
        self._parent_joint.set_vel(self._parent.frame, 0)
        self._child_joint.set_vel(self._parent.frame, 0)
        self._child_joint.set_pos(self._parent_joint, 0)
        self._child.masscenter.v2pt_theory(self._parent.masscenter, self._parent.frame, self._child.frame)


class SlidingJoint(Joint):
    """Sliding (Prismatic) Joint.

    Explanation
    ===========

    It is defined such that the child body translates with respect to the parent
    body along the body fixed parent axis. The point of joint (sliding joint) is defined 
    by two points in each body which coincides initially.
    In addition, the child's reference frame can be arbitrarily rotated a
    constant amount with respect to the parent axis by passing an axis in child
    body which must align with parent axis after the rotation.

    Parameters
    ==========

    name : string
        The Joint's name which makes it unique.
    parent : Body
        The parent body of joint.
    child : Body
        The child body of joint.
    dist : Vector
        The vector distance between parent's masscenter and child's masscenter.
    coordinates: dynamicsymbol, optional
        Coordinates of joint.
    speeds : dynamicsymbol, optional
        Speeds of joint.
    parent_joint_pos : Vector, optional
        Defines the joint's point where the parent will be connected to child.
        Default value is masscenter of Parent Body.
    child_joint_pos : Vector, optional
        Defines the joint's point where the child will be connected to parent.
        Default value is masscenter of Child Body.
    parent_axis : Vector, optional
        Axis of parent frame which would be be aligned with child's
        axis. Default is x axis in parent's frame.

    Examples
    =========

    >>> from sympy.physics.mechanics import Body, SlidingJoint
    >>> parent = Body('parent')
    >>> child = Body('child')
    >>> P = SlidingJoint('P', parent, child)
    >>> P.coordinates()
    [P_x(t)]
    >>> P.speeds()
    [P_v(t)]

    """

    def __init__(self, name, parent, child, dist=None, coordinates=None, speeds=None, parent_joint_pos=None,
        child_joint_pos=None, parent_axis=None):

        super().__init__(name, parent, child, dist, coordinates, speeds, parent_joint_pos, child_joint_pos,
            parent_axis)

    def _generate_coordinates(self, coordinate):
        coordinates = []
        if coordinate is None:
            x = dynamicsymbols(self._name + '_x')
            coordinate = x
        coordinates.append(coordinate)
        return coordinates

    def _generate_speeds(self, speed):
        speeds = []
        if speed is None:
            y = dynamicsymbols(self._name + '_v')
            speed = y
        speeds.append(speed)
        return speeds

    def _orient_frames(self):
        self._child.frame.orient_axis(self._parent.frame, self._parent_axis, 0)

    def _set_angular_velocity(self):
        return

    def _set_linear_velocity(self):
        self._parent_joint.set_vel(self._parent.frame, 0)
        self._child_joint.set_vel(self._child.frame, 0)
        self._child_joint.set_pos(self._parent_joint, self._coordinates[0] * self._parent_axis)
        self._child_joint.set_vel(self._parent.frame, self._coordinates[0] * self._parent_axis)
        self._child.masscenter.set_vel(self._parent.frame, self._coordinates[0] * self._parent_axis)
