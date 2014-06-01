"""A module to manipulate symbolic objects with indices including tensors

"""
from .indexed import IndexedBase, Idx, Indexed
from .index_methods import get_contraction_structure, get_indices
from arraypy import arraypy, tensor, matrix2arraypy, matrix2tensor, list2arraypy, list2tensor 
from tensor_fields import df, grad, rot, div, LieXY, dw
from Riemannian_Geometry import scal_prod, Christoffel_1, Christoffel_2, Covar_der, Riemann, Ricci, Scal_curv
from tensor_methods import *
