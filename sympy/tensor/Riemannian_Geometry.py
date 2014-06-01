# -*- coding: utf-8 -*-
""" 
Riemannian_Geometry module contains functions for working with the Riemann tensor fields of geometry: 
    the scalar product, the Christoffel symbols of the first and second kind, 
    the covariant derivative, the curvature tensor, the Ricci tensor, scalar and sectional curvature. 
Functions work with multidimensional arrays arraypy and tensors. 
Classes and methods tensor and arraypy contained in the module arraypy.
""" 

from sympy.matrices import *
from sympy.tensor.arraypy import *
from sympy import Add,diff,symbols,simplify 


# ---------------- scal_prod g(X,Y)--------------------------------

def scal_prod(X, Y, g):
    """ Returns scalar product of vectors 
        g(X,Y)=sum_{i,j}([g[i,j]*X[i]*Y[j])
        Example:
	========
	>>> x1, x2 = symbols('x1, x2')
	>>> X = [1,2]
	>>> Y = [4,5]
	>>> A = arraypy((2,2))
	>>> g = tensor(A,(-1,-1))
	>>> g[0,0] = cos(x2)**2
	>>> g[0,1] = 0
	>>> g[1,0] = 0
	>>> g[1,1] = 1
	>>> print g
	>>> skal = scal_prod(X, Y, g) 
    """
    
# Handling of a input argument - metric tensor g
    if not isinstance(g,(Matrix,tensor,arraypy)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    g = g.To_matrix() 
    if not g.is_symmetric():
	raise ValueError('The metric tensor must be symmetric.')
    
# Handling of a input arguments - vector or vector fields X
    if not isinstance(X,(list, arraypy, tensor)):
	raise TypeError('The type of vector must be list, arraypy or tensor')
    if isinstance(X, (tensor,arraypy)):
	if len(X.shape) != 1:
	    raise ValueError("The dimension of X must be 1!")
	if type(X) == tensor:
	    if not X.type_pq == (1,0):
		raise ValueError('The valence or ind_char of X must be (+1)')
    if isinstance(X, (tensor,arraypy)):
	X = X.To_list()		
    
# Handling of a input arguments - vector or vector fields Y  
    if not isinstance(Y,(list, arraypy, tensor)):
	raise TypeError('The type of vector must be list, arraypy or tensor')
    if isinstance(Y, (tensor,arraypy)):
	if len(Y.shape) != 1:
	    raise ValueError("The dimension of Y must be 1!")
	if type(Y) == tensor:
	    if not Y.type_pq == (1,0):
		raise ValueError('The valence or ind_char of Y must be (+1)')
    if isinstance(Y, (tensor,arraypy)):
	Y = Y.To_list()		
  
    if not len(X) == len(Y):
	raise ValueError('The vectors must be identical length')
    elif len(X) != g.rows:
	raise ValueError('The vector fields and dimension of metric tensor must be identical length')
    
# Calculation
    indices = range(len(X))  
    scal = sum([g[i,j]*X[i]*Y[j] for i in indices
            for j in indices])
# Output  
    return scal


# ---------------- Christoffel_1 --------------------------------

def Christoffel_1(g, var, type_output='t'):
    """Return the Christoffel symbols of tesor type (-1,-1,-1) for the given metric.
       This returns the Christoffel symbol of first kind that represents the
       Levi-Civita connection for the given metric.
       christoffel_1[i,j,k] = (diff(g[j,k],x[i])+diff(g[i,k],x[j])-diff(g[i,j],x[k]))/2.
       Example:
       ========
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> A = arraypy((2,2))
       >>> g = tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> christoffel1=Christoffel_1(g, var, 't')

    """
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()
# Definition of number of variables
    n=len(var)             

# Handling of a input argument - metric tensor g
    if not isinstance(g,(Matrix, arraypy, tensor)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    if not (g.To_matrix()).is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    if not (g.start_index[0] == g.start_index[1]):
		raise ValueError('The starting indices of metric tensor must be identical')
	    idx_start = g.start_index[0]
	elif type(g) == Matrix:
	    if not g.is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    idx_start = 0    
    
# The definition of diapason changes in an index
    [n1, n2] = g.shape
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not coincide with the number of variables.')
           
    indices = range(idx_start, idx_start + n)
    
# Creating of output array with new indices
    Ch = arraypy([3, n, idx_start])
    
# Calculation
    for i in indices:
	for j in indices:
	    for k in indices:
		Ch[i,j,k] = (diff(g[j,k],var[i-idx_start]) + diff(g[i,k],var[j-idx_start]) - diff(g[i,j],var[k-idx_start]))/2 
    
# Handling of an output array
    if type_output == str('t') or type_output == Symbol('t'):
	Christoffel_1 = Ch.To_tensor((-1, -1, -1))
    elif type_output == str('a') or type_output == Symbol('a'):
	Christoffel_1 = Ch
    else: raise ValueError("The parameter of type output result must 'a' - arraypy or 't' and None - tensor.") 
    
# Output  
    return Christoffel_1


# ---------------- Christoffel_2 --------------------------------

def Christoffel_2(g,var, type_output='t'):
    """Return the Christoffel symbols of tesor type (-1,-1,+1) for the given metric.
       This returns the Christoffel symbol of second kind that represents the
       Levi-Civita connection for the given metric.
       christoffel_2[i,j,k] = 
       = Sum_{l}(g^{-1}[k,l]/2*(diff(g[j, l],x[i])+diff(g[i,l],x[j])-diff(g[i,j],x[l]))/2	   
       Example:
       ========
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> A = arraypy((2,2))
       >>> g = Tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> Christoffel_2(g,var,'a')

    """	
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()
# Definition of number of variables
    n=len(var)                 
    
# Handling of a input argument - metric tensor g
    if not isinstance(g,(Matrix, arraypy, tensor)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    if not (g.To_matrix()).is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    if not (g.start_index[0] == g.start_index[1]):
		raise ValueError('The starting indices of metric tensor must be identical')
	    idx_start = g.start_index[0]
	    g_inv = (g.To_matrix()).inv()
	elif type(g) == Matrix:
	    if not g.is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    idx_start = 0
	    g_inv = g.inv()
    
# The definition of diapason changes in an index
    [n1, n2] = g.shape
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not coincide with the number of variables.')
         
    indices = range(idx_start, idx_start + n)
	
# Creating of output array with new indices    
    Ch = arraypy([3, n, idx_start])
    	
# Calculation
    for i in indices:
	for j in indices:
	    for k in indices:
		Ch[i,j,k] = Add(*[g_inv[k-idx_start, l-idx_start]*(diff(g[j, l], var[i-idx_start]) + diff(g[i, l], var[j-idx_start]) - diff(g[i, j], var[l-idx_start]))/2 for l in indices])
		

# Other variant calculation
    """Christ_1 = Christoffel_1(g,arg)
    for i in indices:
	for j in indices:
	    for k in indices:
		Ch[i,j,k] = Add(*[g_inv[k, l] *Christ_1[i,j,l] for l in indices])"""

# Handling of an output array
    if type_output == str('t') or type_output == Symbol('t'):
	Christoffel_2 = Ch.To_tensor((-1, -1, -1))
    elif type_output == str('a') or type_output == Symbol('a'):
	Christoffel_2 = Ch
    else: raise ValueError("The parameter of type output result must 'a' - arraypy or 't' and None - tensor.") 
    
# Output  
    return Christoffel_2  



# ---------------- Covar_der --------------------------------
    
def Covar_der(X, g, var, type_output='t'):
    """Return the covariant derivative the vector field.
       nabla X[i,j] = diff(X[j],x[i])+Sum_{k}(Gamma2[k,i,j]*X[k])
       Example:
       ========
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> A = arraypy((2,2))
       >>> g = tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> X = [x1*x2**3,x1-cos(x2)]
       >>> Covar_der(X, g, var, 't')
    """
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()    
    
# Definition of number of variables
    n=len(var)
    
# Handling of a input argument - metric tensor g		   
    if not isinstance(g,(Matrix, arraypy, tensor)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    if not (g.To_matrix()).is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    if not (g.start_index[0] == g.start_index[1]):
		raise ValueError('The starting indices of metric tensor must be identical')
	    idx_g = g.start_index[0]
	elif type(g) == Matrix:
	    if not g.is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    idx_g = 0   
	
# Handling of a input argument - vector field X		   
    if not isinstance(X,(list, arraypy, tensor)):
	raise TypeError('The type of vector field must be list, tensor or arraypy')
    else:
	if isinstance(X, (arraypy, tensor)):
	    if len(X.shape) != 1:
			raise ValueError("The dimension of X must be 1!")	    
	    if type(X) == tensor:
		if not X.type_pq == (1,0):
		    raise ValueError('The valence or ind_char of vector field must be (+1)')
	    idx_X = X.start_index[0]
	elif type(X) == list:
	    idx_X = 0 
    
# The definition of diapason changes in an index
    [n1, n2] = g.shape
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not coincide with the number of variables.')
    
    if (idx_g != idx_X):
	raise ValueError('The start index of the metric tensor and vector field must be equal')
    else: idx_start = idx_g
    
    indices = range(idx_start, idx_start + n)
    
# Creating of output array with new indices    
    Cov = arraypy([2, n, idx_start])	
    ch_2 = Christoffel_2(g, var)
# Calculation
    for i in indices:
	for j in indices:
	    Cov[i,j] = diff(X[j], var[i-idx_start]) + Add(*[ch_2[k,i,j]*X[k] for k in indices])
	
# Handling of an output array
    if type_output == str('t') or type_output == Symbol('t'):
	Cov_der = Cov.To_tensor((-1, 1))
    elif type_output == str('a') or type_output == Symbol('a'):
	Cov_der = Cov
    else: raise ValueError("The parameter of type output result must 'a' - arraypy or 't' and None - tensor.") 
    
# Output  
    return Cov_der


# ---------------- Covar_der_XY --------------------------------
   
def Covar_der_XY(X, Y, g, var, type_output='t'):
    """Return the covariant derivative the vector field along another field.
       nabla_Y(X)[j] = Sum_{i}(nabla X[i,j]*Y[i])
       Example:
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> A = arraypy((2,2))
       >>> g = Tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> X = [x1*x2**3, x1-cos(x2)]
       >>> Y = [1, 2, 3]
       >>> Covar_der_XY(X, Y, g, var, 't')
    """
    # Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()    
    
    #Definition of number of variables
    n=len(var)
    
    # Handling of a input argument - metric tensor g		   
    if not isinstance(g,(Matrix, arraypy, tensor)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    if not (g.To_matrix()).is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    if not (g.start_index[0] == g.start_index[1]):
		raise ValueError('The starting indices of metric tensor must be identical')
	    idx_g = g.start_index[0]
	elif type(g) == Matrix:
	    if not g.is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    idx_g = 0   
	
    # Handling of a input argument - vector field X		   
    if not isinstance(X,(list, arraypy, tensor)):
	raise TypeError('The type of vector field must be list, tensor or arraypy')
    else:
	if isinstance(X, (arraypy, tensor)):
	    if len(X.shape) != 1:
			raise ValueError("The dimension of X must be 1!")	    
	    if type(X) == tensor:
		if not X.type_pq == (1,0):
		    raise ValueError('The valence or ind_char of vector field must be (+1)')
	    idx_X = X.start_index[0]
	elif type(X) == list:
	    idx_X = 0
    
    # Handling of a input argument - vector field Y		   
    if not isinstance(Y,(list, arraypy, tensor)):
	raise TypeError('The type of vector field must be list, tensor or arraypy')
    else:
	if isinstance(Y, (arraypy, tensor)):
	    if len(Y.shape) != 1:
			raise ValueError("The dimension of Y must be 1!")	    
	    if type(Y) == tensor:
		if not Y.type_pq == (1,0):
		    raise ValueError('The valence or ind_char of vector field must be (+1)')
	    idx_Y = Y.start_index[0]
	elif type(Y) == list:
	    idx_Y = 0    

    [n1, n2] = g.shape    
    if not len(X) == len(Y):
	raise ValueError('The vectors must be identical length')
    elif not idx_X==idx_Y:
	raise ValueError('The start index of vector fields must be equal')
    elif not(idx_g == idx_X):
	raise ValueError('The start index of the metric tensor and vector field must be equal') 
    else: idx_start = idx_g
    if len(X) != n1:
	raise ValueError('The vector fields and dimension of metric tensor must be identical length')    
           
    
# The definition of diapason changes in an index
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not concide with the number of variables.')
    indices = range(idx_start, idx_start + n)
    
# Creating of output array with new indices    
    nabla_XY = arraypy([1, n, idx_start])	
    nabla_X = Covar_der(X, g, var)
    
# Calculation
    for j in indices:
	nabla_XY[j] = sum([nabla_X[i,j]*Y[i] for i in indices])
	
# Handling of an output array
    if type_output == str('t') or type_output == Symbol('t'):
	Cov_der_XY = nabla_XY.To_tensor((1))
    elif type_output == str('a') or type_output == Symbol('a'):
	Cov_der_XY = nabla_XY
    else: raise ValueError("The parameter of type output result must 'a' - arraypy or 't' and None - tensor.") 
    
    # Output  
    return Cov_der_XY


# ---------------- Riemann --------------------------------

def Riemann(g, var, type_output='t'):
    """Return the Riemann curvature tensor of type (-1,-1,-1,+1)  for the given metric tensor.
       Riemann[i,j,k,l] = diff(Gamma_2[j,k,l],x[i])-diff(Gamma_2[i,k,l],x[j]) + 
       + Sum_{p}( Gamma_2[i,p,l]*Gamma_2[j,k,p] -Gamma_2[j,p,l]*Gamma_2[i,k,p]    
       Example:
       ========
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> A = arraypy((2,2))
       >>> g = tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> Riemann(g, var, 'a')
    """    
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()     
	
# Definition of number of variables
    n=len(var)
       
# Handling of a input argument - metric tensor g		   
    if not isinstance(g,(Matrix, arraypy, tensor)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    if not (g.To_matrix()).is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    if not (g.start_index[0] == g.start_index[1]):
		raise ValueError('The starting indices of metric tensor must be identical')
	    idx_start = g.start_index[0]
	elif type(g) == Matrix:
	    if not g.is_symmetric():
		raise ValueError('The metric tensor must be symmetric.')
	    idx_start = 0   
    
# The definition of diapason changes in an index
    [n1, n2] = g.shape
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not coincide with the number of variables.')

    indices = range(idx_start, idx_start + n)
    
# Creating of output array with new indices
    R = arraypy([4, n, idx_start])
    ch_2 = Christoffel_2(g, var)
    
    # Calculation
    for i in indices:
	for j in indices:
	    for k in indices:
		for l in indices:
		    R[i,j,k,l] =diff(ch_2[j,k,l], var[i-idx_start]) - diff(ch_2[i,k,l], var[j-idx_start])+ sum([ch_2[i,p,l]*ch_2[j,k,p] - ch_2[j,p,l]*ch_2[i,k,p] for p in indices])
    
# Handling of an output array
    if type_output == str('t') or type_output == Symbol('t'):
	Riemann = R.To_tensor((-1, -1, -1, 1))
    elif type_output == str('a') or type_output == Symbol('a'):
	Riemann = R
    else: raise ValueError("The parameter of type output result must 'a' - arraypy or 't' and None - tensor.") 
    
# Output  
    return Riemann


# ---------------- Ricci --------------------------------

def Ricci(riemann, var, type_output='t'):
    """Return the tensor Ricci of type (-1,-1) for given Riemann curvature tensor.
       Ricci[j,k] = Sum_{i}(Riemann[i,j,k,i])
       Example:
       ========
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> A = arraypy((2,2))
       >>> g = Tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> cur = Riemann(g, var, 't')
       >>> Ric = Ricci(cur, var, 't')
    """
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()     
	
# Definition of number of variables
    n=len(var)    
    
# Handling of a input argument Riemann curvature tensor - riemann		   
    if not isinstance(riemann,(arraypy, tensor)):
	raise TypeError('The type of Riemann curvature tensor must be arraypy or tensor')
    if type(riemann) == tensor:
	if not riemann.type_pq == (1,3):
	    raise ValueError('The valence or ind_char of Riemann curvature tensor must be (-1,-1,-1,+1)')
    idx_start = riemann.start_index[0]
    
# The definition of diapason changes in an index
    [n1, n2, n3, n4] = riemann.shape	
    if not n == n1:
	raise ValueError('The rank of the Riemann curvature tensor does not coincide with the number of variables.')
    
    indices = range(idx_start, idx_start + n)
    
# Creating of output array with new indices
    Ri = arraypy([2, n, idx_start])    
    
# Calculation
    for j in indices:
	for k in indices:
	    Ri[j, k] = sum([riemann[i,j,k,i] for i in indices])
	    
# Handling of an output array
    if type_output == str('t') or type_output == Symbol('t'):
	Ricci = Ri.To_tensor((-1, -1))
    elif type_output == str('a') or type_output == Symbol('a'):
	Ricci = Ri
    else: raise ValueError("The parameter of type output result must 'a' - arraypy or 't' and None - tensor.") 
    
    # Output  	
    return Ricci


# ---------------- Scal_curv --------------------------------

def Scal_curv(g, ricci, var):
    """ The scalar curvature (or the Ricci scalar) 
        is the simplest curvature invariant of a Riemannian manifold.
	S=sum_{j,k} Ricci[j,k]*g_inv[j,k]
	Example:
	========
	>>> x1, x2 = symbols('x1, x2')
	>>> arg = [x1, x2]
	>>> A = arraypy((2,2))
	>>> g = Tensor(A,(-1,-1))
	>>> g[0,0] = cos(x2)**2
	>>> g[0,1] = 0
	>>> g[1,0] = 0
	>>> g[1,1] = 1
	>>> print g
	>>> Scal_curv(g, r, arg) 
    """
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()
    
# Definition of number of variables
    n=len(var)       
    
# Handling of a input argument - metric tensor g
    if not isinstance(g,(Matrix, arraypy, tensor)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    g = g.To_matrix()
    if not g.is_symmetric():
	raise ValueError('The metric tensor must be symmetric.')
# The definition of inverse matrix of the metric tensor 
    g_inv = g.inv()
   
# Handling of a input argument tensor Ricci - ricci
    if not isinstance(ricci,(Matrix, arraypy, tensor)):
	raise TypeError('The type of tensor Ricci must be Matrix, tensor or arraypy')
    else:
	if isinstance(ricci, (arraypy, tensor)):
	    if type(ricci) == tensor:
		if not ricci.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of tensor Ricci must be (-1,-1)')
	    ricci = ricci.To_matrix()
    if not ricci.is_symmetric():
	raise ValueError('The Ricci tensor must be symmetric.') 
    
    if not (g.shape == ricci.shape):
	raise ValueError('The rank of the metric tensor does not coincide with the rank of tensor Ricci.')
	    
# The definition of diapason changes in an index
    [n1, n2] = g.shape
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not coincide with the number of variables.')
    
# Calculation
    indices = range(n)
    for i in indices:
	for j in indices:
	    Scal_curv = g_inv[i,j] * ricci[i,j]
# Output
    return Scal_curv  


#-----------------K_sigma----------------------------

def K_sigma(X, Y, R, g, var):
    """Return Sectional curvature of the Riemannian manifold 
       in the direction two-dimensional area formed by 
       vectors X, Y  for the given metric tensor R.
       K_sigma = Sum_{i,j,k,r,s}( g[r,s]*Riemann[i,j,k,r] *X[i]*Y[j]*Y[k]X[s])/ (scal_prod(X,X,g)*scal_prod(Y,Y,g) - scal_prod(X,Y,g)^2   
       Example:
       ========
       >>> x1, x2 = symbols('x1, x2')
       >>> var = [x1, x2]
       >>> X = [1,2]
       >>> Y = [3,4]
       >>> A = arraypy((2,2))
       >>> g = tensor(A,(-1,-1))
       >>> g[0,0] = cos(x2)**2
       >>> g[0,1] = 0
       >>> g[1,0] = 0
       >>> g[1,1] = 1
       >>> print g
       >>> R = Riemann(g, var)
       >>> K_sig = K_sigma(X, Y, R, g, var) 
    """    
# Handling of input vector arguments var
    if not isinstance(var,(list, arraypy, tensor)):
	raise TypeError('The type of vector arguments(var) must be a list, arraypy or tensor')
    if isinstance(var, (tensor,arraypy)):
	if len(var.shape) != 1:
	    raise ValueError("The dimension of variables must be 1!")
	if type(var) == tensor:
	    if not var.type_pq == (1,0):
		raise ValueError('The valence or ind_char of vector variables must be (+1)')
    if isinstance(var, (tensor,arraypy)):
	var = var.To_list()     
	
# Definition of number of variables
    n=len(var)
       
# Handling of a input argument - metric tensor g		   
    if not isinstance(g,(Matrix,tensor,arraypy)):
	raise TypeError('The type of metric tensor must be Matrix, tensor or arraypy')
    else:
	if isinstance(g, (arraypy, tensor)):
	    if type(g) == tensor:
		if not g.type_pq == (0,2):
		    raise ValueError('The valence or ind_char of metric tensor must be (-1,-1)')
	    g = g.To_matrix() 
    if not g.is_symmetric():
	raise ValueError('The metric tensor must be symmetric.')
	    
# Handling of a input arguments - vector or vector fields X
    if not isinstance(X,(list, arraypy, tensor)):
	raise TypeError('The type of vector must be list, arraypy or tensor')
    if isinstance(X, (tensor,arraypy)):
	if len(X.shape) != 1:
	    raise ValueError("The dimension of X must be 1!")
	if type(X) == tensor:
	    if not X.type_pq == (1,0):
		raise ValueError('The valence or ind_char of X must be (+1)')
    if isinstance(X, (tensor,arraypy)):
	X = X.To_list()		
    
# Handling of a input arguments - vector or vector fields Y  
    if not isinstance(Y,(list, arraypy, tensor)):
	raise TypeError('The type of vector must be list, arraypy or tensor')
    if isinstance(Y, (tensor,arraypy)):
	if len(Y.shape) != 1:
	    raise ValueError("The dimension of Y must be 1!")
	if type(Y) == tensor:
	    if not Y.type_pq == (1,0):
		raise ValueError('The valence or ind_char of Y must be (+1)')
    if isinstance(Y, (tensor,arraypy)):
	Y = Y.To_list()		
  
    if not len(X) == len(Y):
	raise ValueError('The vectors must be identical length')
    elif len(X) != g.rows:
	raise ValueError('The vector fields and dimension of metric tensor must be identical length')    
    
# Handling of a input argument Riemann curvature tensor - R		   
    if not isinstance(R,(Matrix, arraypy, tensor)):
	raise TypeError('The type of Riemann curvature tensor must be Matrix, arraypy or tensor')
    else:
	if isinstance(R, (arraypy, tensor)):
	    if type(R) == tensor:
		if not R.type_pq == (1,3):
		    raise ValueError('The valence or ind_char of Riemann curvature tensor must be (-1,-1,-1,+1)')
		if not (R.start_index[0] == R.start_index[1]):
				raise ValueError('The starting indices of Riemann curtivate tensor must be identical')		
	    idx_R = R.start_index[0]
	    
	
# The definition of diapason changes in an index
    [n1, n2] = g.shape
    if not n == n1:
	raise ValueError('The rank of the metric tensor does not coincide with the number of variables.')
    [n1, n2, n3, n4] = R.shape	
    if not n == n1:
	raise ValueError('The rank of the Riemann curvature tensor does not concide with the number of variables.')
    
    indices = range(len(X))
        
# Calculation
    Sc_pr = scal_prod(X, X, g) * scal_prod(Y, Y, g) - scal_prod(X, Y, g)**2
    if (Sc_pr == 0):
	raise ValueError('The two-dimensional area is a degenerate!') 
    
    numerator = sum([g[r,s] * R[i+idx_R,j+idx_R,k+idx_R,r+idx_R]*X[i]*Y[j]*Y[k]*X[s] for i in indices
                     for j in indices	
                     for k in indices
                     for r in indices
                     for s in indices])
		    
    K_sigma = simplify(numerator/Sc_pr)

# Output  
    return K_sigma