#-*- coding: utf-8 -*-
"""
Module tensor_fields contains functions for working with the tensor fields: 
calculation of the differential and the gradient of the function, 
curl and divergence of a vector field, 
the calculation of the Li derivative and the external differentiation of differential forms. 
Functions are work with the multidimensional arrays arraypy and tensors, 
classes and methods which are contained in the module arraypy.

"""

from sympy.matrices import *
from sympy.tensor.arraypy import *
from sympy import Add,diff,symbols,simplify
from sympy import sqrt

# ---------------- df --------------------------------

def df(f,args,output_type='l'):
    """ Returns an the 1-form df, differential of function f(x).
	    Examples
	    ========
	    >>> from sympy import *
	    >>> x1, x2, x3, a= symbols('x1 x2 x3 a')
	    >>> f=x1**2*x2+sin(x2*x3-x2)
	    >>> args=[x1, x2, x3]
	    >>> df(f, args, a)   
	    ========
	    >>> from sympy import *
	    >>> x1, x2, x3= symbols('x1 x2 x3')
	    >>> f=x1**2*x2+sin(x2*x3-x2)
	    >>> args_t=arraypy([1,3,1]).To_tensor(1)
	    >>> args_t[1]=x1
	    >>> args_t[2]=x2
	    >>> args_t[3]=x3
	    >>> df(f, args_t, 't') 
    """
# Handling of a vector of arguments
    if not isinstance(args, (list,tensor,arraypy)):
	    raise TypeError('The type of vector of arguments must be list, Tensor or arraypy')
    if isinstance(args, (tensor,arraypy)):
	    if len(args.shape)!=1:
		    raise ValueError("The dimension of argument must be 1")
	    if isinstance(args,tensor):
		    if args.type_pq != (1,0):
			    raise ValueError('The valency(ind_char) of tensor must be (+1)') 		
	    idx=args.start_index[0]    
    if isinstance(args, list):
	    idx=0
	
# Creating the output array in accordance with start indexes
    n=len(args) 
    array=arraypy([1,n,idx])
    indices = range(idx,idx+n)
    
# Calculation
    for k in indices:
	    array[k]=diff(f,args[k])
       
# Handling of an output array
    if output_type=='t' or output_type==Symbol('t'):
	    differential=arraypy.To_tensor(array,-1)
    elif output_type=='a' or output_type==Symbol('a'):
	    differential=array
    elif output_type=='l' or output_type==Symbol('l'):
	    differential=arraypy.To_list(array)
    else:
	    raise TypeError("The third arguments must be 't'-tensor,'a'-massiv arraypy,'l'-list")
# Output  
    return differential    

# ---------------- grad --------------------------------

def grad(f,args,g=None,output_type=None):
    """ Returns the vector field Gradient(f(x)) of a function f(x).               
		Examples
		========
		>>> from sympy import *
		>>> x1, x2, x3 = symbols('x1 x2 x3')
		>>> f=x1**2*x2+sin(x2*x3-x2)
		>>> args_t=arraypy([1,3,1]).To_tensor(1)
		>>> args_t[1]=x1
		>>> args_t[2]=x2
		>>> args_t[3]=x3
		>>> grad(f,args_t,output_type='t')
		
		========     
		>>> from sympy import *
		>>> x1, x2, x3 = symbols('x1 x2 x3')
		>>> f=x1**2*x2+sin(x2*x3-x2)
		>>> var=[x1,x2,x3]
		>>> g=arraypy([2,3,1])
		>>> g_t=g.To_tensor((-1,-1))
		>>> g_t[1,1]=2
		>>> g_t[1,2]=1
		>>> g_t[1,3]=0
		>>> g_t[2,1]=1
		>>> g_t[2,2]=3
		>>> g_t[2,3]=0
		>>> g_t[3,1]=0
		>>> g_t[3,2]=0
		>>> g_t[3,3]=1
		>>> m=grad(f,var,g_t,'a')
    """
# Handling of a vector of arguments
    if not isinstance(args, (list,tensor,arraypy)):
	    raise TypeError('The type of vector of arguments must be list, tensor or arraypy')
    if isinstance(args, (tensor,arraypy)):
	    if len(args.shape)!=1:
		    raise ValueError("The dimension of argument must be 1")
	    if isinstance(args,tensor):
		    if args.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)')	    
	    idx_args=args.start_index[0]
    if isinstance(args, list):
	    idx_args=0
	    
# Handling of the metric tensor
  # 1. if g is not NULL
    if g is not None:
	    if output_type is None:
		    output_type='t'
	    if not isinstance(g,(tensor,Matrix,arraypy)):
		    raise ValueError('Type must be Matrix or tensor or arraypy')
	    if isinstance(g,tensor):
		    if g.type_pq != (0,2):
			    raise ValueError('The indices of tensor must be (-1,-1)')
			
   #The definition of the start index
	    if isinstance(g,Matrix):
		    idx_st=0
	    else:
		    idx_st=g.start_index[0] 
	    if type(g)==type(args) and idx_st!=idx_args:
		    raise ValueError('The start index of the metric tensor and vector of arguments must be equal')
	    
	    if isinstance(g,(tensor,arraypy)):	    
		    g = g.To_matrix()
	    if not g.is_symmetric():
		    raise ValueError('The metric is not symmetric')		    
  # 2.if g is NULL
    else:
      # g - the identity matrix
	    g=eye(len(args))
	    idx_st=0
	    
# Creating the output array in accordance with start indexes
    n=len(args) 
    array=arraypy([1,n,idx_st])
    indices = range(idx_st,idx_st+n)
         
# Calculating
    g_inv=g.inv()
    if isinstance(args,(tensor,arraypy)):
	    args=args.To_list()    
    for i in indices:
	    for j in indices:		
		    array[i]+=(g_inv[i-idx_st,j-idx_st]*diff(f,args[j-idx_st]))
		    
# Handling of an output array
    if output_type=='t' or output_type==Symbol('t'):
	    gradient=arraypy.To_tensor(array,1)
    elif output_type=='a' or output_type==Symbol('a'):
	    gradient=array
    elif output_type=='l' or output_type==Symbol('l') or output_type is None:
	    gradient=arraypy.To_list(array)
    else:
	    raise TypeError("The third arguments must be 't'-tensor,'a'-massiv arraypy,'l'-list")
# Output  
    return gradient  

# ---------------- rot --------------------------------
     
def rot(X,args,output_type=None):
    """Returns the vorticity vector field rot(X) of a vector field X in R^3 (curl, rotation, rotor, vorticity)
	A rotor can be calculated for only in three-dimensional Euclidean space. 
    
	Examples
	========
	>>> from sympy import *
	>>> x1, x2, x3 = symbols('x1 x2 x3')
	>>> X=arraypy(3)
	>>> X_t=tensor(X,(1))
	>>> X_t[0]=x1*x2**3
	>>> X_t[1]=x2-cos(x3)
	>>> X_t[2]=x3**3-x1
	>>> arg=[x1,x2,x3]
	>>> r=rot(X_t,arg,'t')

    """
# Handling of a vector of arguments
    if not isinstance(args, (list,tensor,arraypy)):
	    raise ValueError('The type of arguments vector must be list, tensor or arraypy')
    if len(args)!=3:
	    raise ValueError('ERROW:three variables are required')
    if isinstance(args, (tensor,arraypy)):
	    if len(args.shape)!=1:
		    raise ValueError("The lenght of argument must be 1")	    
	    if isinstance(args,tensor):
		    if args.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    idx_args=args.start_index[0]
    if isinstance(args, list):
	    idx_args=0
	       
# Handling of a vector field
    if not isinstance(X, (list,tensor,arraypy)):
	    raise ValueError('The type of vector fields must be list, tensor or arraypy')	
    if len(X)!=3:
	    raise ValueError('ERROW:a three-dimensional vector is necessary')
	
    if isinstance(X, (tensor,arraypy)):
	    if len(X.shape)!=1:
		    raise ValueError("The dim of argument must be 1")
	    if isinstance(X,tensor):
		    out_t='t'
		    if X.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    idx_X=X.start_index[0]
    elif isinstance(X, list):
	    idx_X=0
	    out_t='l'
	    
    if output_type is None:
	    if out_t is not None:
		    output_type=out_t
	    else:
		    output_type='a'
		    	    
# The definition of the start index
    if type(X)==type(args) and (idx_X!=idx_args):
	    raise ValueError('The start index of vector field and vector of arguments must be equal')    
    idx_st=idx_X
    
# Creating the output array in accordance with start indexes
    array=arraypy([1,3,idx_st])

# Calculation
    if isinstance(X, (tensor,arraypy)):
	    X=X.To_list()
    if isinstance(args, (tensor,arraypy)):
	    args=args.To_list()
	    
    array[idx_st]=(diff(X[2],args[1])-diff(X[1],args[2]))
    array[idx_st+1]=diff(X[0],args[2])-diff(X[2],args[0])
    array[idx_st+2]=diff(X[1],args[0])-diff(X[0],args[1])

       
# Handling of an output array
    if output_type=='t' or output_type==Symbol('t'):
	    rotor=arraypy.To_tensor(array,1)
    elif output_type=='a' or output_type==Symbol('a'):
	    rotor=array
    elif output_type=='l' or output_type==Symbol('l'):
	    rotor=arraypy.To_list(array)
    else:
	    raise TypeError("The third arguments must be 't'-tensor,'a'-massiv arraypy,'l'-list")    
# Output  
    return rotor
# ---------------- div --------------------------------

def div(X,args,g=None):
    """ Returns the divergence of a vector field X.
	Compute divergence of vector field consisting of N elements.
	
	Examples
	========
	>>> from sympy import *
	>>> x1, x2, x3 = symbols('x1 x2 x3')
	>>> X=[x1*x2**3,x2-cos(x3),x3**3-x1]
	>>> g=Matrix([[2,1,0],[1,3,0],[0,0,1]])
	>>> arg=[x1, x2, x3]
	>>> div(X,arg,g)
    """
# Handling of a vector of arguments
    if not isinstance(args, (list,tensor,arraypy)):
	    raise ValueError('The type of arguments vector must be list, tensor or arraypy')
    if isinstance(args, (tensor,arraypy)):
	    if len(args.shape)!=1:
		    raise ValueError("The lenght of argument must be 1")	    
	    if isinstance(args,tensor):
		    if args.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)')
	    args=args.To_list()
		   
# Handling of a vector field
    if not isinstance(X, (list,tensor,arraypy)):
	    raise ValueError('The type of vector fields must be list, tensor or arraypy')	   
    if isinstance(X, (tensor,arraypy)):
	    if len(X.shape)!=1:
		    raise ValueError("The dim of argument must be 1")
	    if isinstance(X,tensor):
		    if X.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    X=X.To_list()
	    
# Handling of the metric tensor   
    if g is not None:
	    if not isinstance(g,(tensor,Matrix,arraypy)):
		    raise ValueError('Type must be Matrix or tensor or arraypy')
	    else:
		    if isinstance(g,(tensor,arraypy)):
			    if isinstance(g,tensor):
				    if g.type_pq != (0,2):
					    raise ValueError('The indices of tensor must be (-1,-1)')
			    g = g.To_matrix()
		    if not g.is_symmetric():
			    raise ValueError('The metric is not symmetric')
    else:
	    g=eye(len(args)) 
		    
#Calculation
    sq=sqrt(abs(Matrix.det(g)))
    divergenc=0
    for k in range(len(args)):
	    divergenc += simplify(1/sq*sum([diff(X[k]*sq,args[k])]))	
# Output  
    return divergenc 


#------------------LieXY-------------------------------

def LieXY(X,Y,args,output_type=None):
    """ Returns the vector field [X,Y], Lie bracket (commutator) of a vector fields X and Y
	
	Examples
	========
	>>> from sympy import *
	>>> x1, x2, x3 = symbols('x1 x2 x3')
	>>> X=[-x**3,9*y**2,5*z, 8*h+x]
	>>> Y=[x1**3*x2**3,x2*x3-sin(x1*x3),x3**3-x1**2]
	>>> arg=[x1, x2, x3]
	>>> LieXY(X,Y,arg,'a')
    """
# Handling of a vector of arguments
    if not isinstance(args, (list,tensor,arraypy)):
	    raise ValueError('The type of arguments vector must be list, tensor or arraypy')
    if isinstance(args, (tensor,arraypy)):
	    if len(args.shape)!=1:
		    raise ValueError("The lenght of argument must be 1")	    
	    if isinstance(args,tensor):
		    if args.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    idx_args=args.start_index[0]
    if isinstance(args, list):
	    idx_args=0   
	    
# Handling of the first vector field
    if not isinstance(X, (list,tensor,arraypy)):
	    raise ValueError('The type of vector fields must be list, tensor or arraypy')	   
    if isinstance(X, (tensor,arraypy)):
	    if len(X.shape)!=1:
		    raise ValueError("The dim of argument must be 1")
	    if isinstance(X,tensor):
		    out_t='t'
		    if X.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    idx_X=X.start_index[0]
    if isinstance(X, list):
	    idx_X=0 
	    out_t='l'
	
# Handling of the second vector field
    if not isinstance(Y, (list,tensor,arraypy)):
	    raise ValueError('The type of vector fields must be list, tensor or arraypy')	   
    if isinstance(Y, (tensor,arraypy)):
	    if len(Y.shape)!=1:
		    raise ValueError("The dim of argument must be 1")
	    if isinstance(Y,tensor):
		    if Y.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    idx_Y=Y.start_index[0]
    if isinstance(Y, list):
	    idx_Y=0	
	        
    if len(Y)!=len(X):
	    raise ValueError('The different number of arguments of the vector fields')
    elif len(args)!=len(X) or len(args)!=len(Y):
	    raise ValueError('The different number of components at the vector field and vector of variables') 
    
# Define the start index in the output tensor
    if type(Y)==type(X)==type(args):
	    if idx_Y!=idx_X or idx_Y!=idx_args or idx_X!=idx_args:
		    raise ValueError('The start index of vector fields and vetcor of argements must be equal')
    if idx_Y!=idx_X:
		    raise ValueError('The start index of tensor and the vector field must be equal')
    idx_st=idx_Y   
    
    if output_type is None:
	    if out_t is not None:
		    output_type=out_t
	    else:
		    output_type='a'
    
# Creating the output array in accordance with start indexes
    Li=arraypy([1,len(X),idx_st])
    
# Calculating
    if isinstance(Y, (tensor,arraypy)):
	    Y=Y.To_list()
    if isinstance(X, (tensor,arraypy)):
	    X=X.To_list()	
    if isinstance(args, (tensor,arraypy)):
	    args=args.To_list()    
    
    if X==Y:
	    return 0
    else:
	    indices = range(len(args))
	    for i in indices:
		    for k in indices:
			    Li[i+idx_st]+=Add(diff(Y[i],args[k])*X[k]-diff(X[i],args[k])*Y[k])	 
			    
# Handling of an output array
    if output_type=='t' or output_type==Symbol('t'):
	    Lie=arraypy.To_tensor(Li,1)
    elif output_type=='a' or output_type==Symbol('a'):
	    Lie=Li
    elif output_type=='l' or output_type==Symbol('l'):
	    Lie=arraypy.To_list(Li)
    else:
	    raise TypeError("The third arguments must be 't'-tensor,'a'-massiv arraypy,'l'-list")    
	             
# Output  
    return Lie


# ---------------- NotNeedElement --------------------------------

def NotNeedElement (_list, index):
	"""The function returns a tuple containing the remainder of the input list 
	after you remove the element at the specified index.
	"""
	res = []
	for i in range(len(_list)):
		if i != index:
			res.append((_list[i]))
	return (tuple(res))
    

# ---------------- dw --------------------------------
def dw(omega, args):
    """ Returns the exterior differential of a differential form
	    
	    Examples
	    ========
	    >>> from sympy import *
	    >>> x1, x2, x3 = symbols('x1 x2 x3')
	    >>> omega=arraypy([2,3,1]).To_tensor((-1,-1))
	    >>> omega[1,2]=x3
	    >>> omega[1,3]=-x2
	    >>> omega[2,1]=-x3
	    >>> omega[2,3]=x1
	    >>> omega[3,1]=x2 
	    >>> omega[3,2]=-x1
	    >>> domega=dw(omega, [x1,x2,x3]) 
    """
# Handling of a vector of arguments
    if not isinstance(args, (list,tensor,arraypy)):
	    raise ValueError('The type of arguments vector must be list, tensor or arraypy')
    if isinstance(args, (tensor,arraypy)):
	    if len(args.shape)!=1:
		    raise ValueError("The lenght of argument must be 1")	    
	    if isinstance(args,tensor):
		    if args.type_pq != (1,0):
			    raise ValueError('The valency of tensor must be (+1)') 		
	    idx_args=args.start_index[0]
    if isinstance(args, list):
	    idx_args=0

# Handling of a differential form
    if not isinstance(omega, (tensor,arraypy)):
	    raise ValueError("Type must be Tensor or arraypy") 
    idx_omega = omega.start_index[0]
    
# Define the start index in the output tensor
    if type(omega)==type(args) and idx_omega!=idx_args:
		    raise ValueError("Raznie indeksi!!!")		    
    idx_st=idx_omega

# Creating the output array in accordance with start indexes
    n=omega.shape[0] #the dimensionality of the input array
    p=len(omega.shape) #the rank of the input array
    a=arraypy([p+1,n,idx_st])
    valence_ind=[(-1) for k in range(p+1)]
    d_omega=a.To_tensor(valence_ind)
    
# Calculation
    idx=d_omega.start_index 
    if isinstance(args, (tensor,arraypy)):
	        args=args.To_list()
		
    for i in range(len(d_omega)):
	    #list of tuple. example:[(0, 1), (0, 1), (0, 0)]
	    tuple_list_indx = [NotNeedElement(idx, f) for f in range(len(idx))]	    
	    for k in range(p+1):
		    d_omega[idx]+=Add(((-1)**k)*diff(omega[tuple_list_indx[k]],args[idx[k]-idx_st]))
	    idx = d_omega.Next_index(idx)
	    
# Output  
    return d_omega

# ---------------- NeedElementK --------------------------------

def NeedElementK (_list, index,k):
    """The function replaces the item "index" on the element "k". 
    The result is a tuple.
    """
    output = []
    for i in range(len(_list)):
	    if i != index:
		    output.append((_list[i]))                
	    else:
		    output.append(k)       
    return (tuple(output))


# ---------------- Lie_omega --------------------------------

def Lie_w(omega, X, args):
	"""Returns the Lie derivative of a differential form 
		
		Examples
		========
		>>> from sympy import *
		>>> x1, x2, x3 = symbols('x1 x2 x3')
		>>> omega=arraypy([2,3,1]).To_tensor((-1,-1))
		>>> omega[1,2]=x3
		>>> omega[1,3]=-x2
		>>> omega[2,1]=-x3
		>>> omega[2,3]=x1
		>>> omega[3,1]=x2
		>>> omega[3,2]=-x1
		>>> arg=[x1, x2, x3]
		>>> X=[x1*x2**3,x2-cos(x3),x3**3-x1]
		>>> Lie_w(omega,X,arg)
	"""
# Handling of a vector of arguments
	if not isinstance(args, (list,tensor,arraypy)):
		raise ValueError('The type of arguments vector must be list, tensor or arraypy')
	if isinstance(args, (tensor,arraypy)):
		if len(args.shape)!=1:
			raise ValueError("The lenght of argument must be 1")	    
		if isinstance(args,tensor):
			if args.type_pq != (1,0):
				raise ValueError('The valency of tensor must be (+1)') 		
		idx_args=args.start_index[0]
	if isinstance(args, list):
		idx_args=0

# Handling of a vector field
	if not isinstance(X, (list,tensor,arraypy)):
		raise ValueError('The type of vector fields must be list, tensor or arraypy')	   
	if isinstance(X, (tensor,arraypy)):
		if len(X.shape)!=1:
			raise ValueError("The dim of argument must be 1")
		if isinstance(X,tensor):
			if X.type_pq != (1,0):
				raise ValueError('The valency of tensor must be (+1)') 		
		idx_X=X.start_index[0]
	if isinstance(X, list):
		idx_X=0
		
# Handling of a differential form
	if not isinstance(omega, (tensor,arraypy)):
		raise ValueError("Type must be Tensor or arraypy") 
	idx_omega = omega.start_index[0]	


#Define the start index in the output tensor
	if type(omega)==type(X)==type(args):
		if idx_omega!=idx_X or idx_omega!=idx_args or idx_X!=idx_args:
			raise ValueError('The start index of tensor,vector field and vetcor of argements must be equal')
	if type(omega)==type(X) and idx_omega!=idx_X:
			raise ValueError('The start index of tensor and vector field must be equal')
	idx_st=idx_omega

#Creating the output array in accordance with start indexes
	n=omega.shape[0]    # the dimensionality of the input array
	r=len(omega.shape)  # the rank of the input array
	a=arraypy([r,n,idx_st])
	valence_list=[(-1) for k in range(r)]
	diff_Lie=a.To_tensor(valence_list)
	  
# Calculation
	idx = diff_Lie.start_index
	if isinstance(args,(tensor,arraypy)):
		args=args.To_list()
	if isinstance(X,(tensor,arraypy)):
		X=X.To_list()
		
	for p in range(len(diff_Lie)):
	    for k in range(len(idx)+1):			    
		    tuple_list_indx = [NeedElementK(idx, f, k+idx_st) for f in range(len(idx))]
		    diff_omega=diff(omega[idx],args[k])*X[k]
		    for j in range(len(idx)):
			    diff_Lie[idx]+=diff(X[k],args[idx[j]-idx_st])*omega[tuple_list_indx[j]]
		    diff_Lie[idx]=diff_Lie[idx]+diff_omega		    
	    idx = diff_Lie.Next_index(idx)
# Output
	return diff_Lie
