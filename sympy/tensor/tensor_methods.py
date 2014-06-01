# -*- coding: utf-8 -*-
"""
Tensor Methods module contains functions for working with tensors: 
     Creates the symmetric form of input tensor.
     Creates the asymmetric form of input tensor.
Classes and methods tensor and arraypy contained in the module arraypy.

     """

from itertools import *
from sympy.tensor import arraypy, tensor
from random import randint

def Symmetric(In_Arr):
     """
     Creates the symmetric form of input tensor.
     Input: arraypy or tensor with equal axes (array shapes).
     Output: symmetric array. Output type - arraypy or tensor, depends of input
     
     Examples:
     >>> a = list2arraypy(range(9), (3,3))
     >>> b = Symmetric(a)
     >>> print (b)
     0.0 2.0 4.0 
     2.0 4.0 6.0 
     4.0 6.0 8.0 
     """
     if not isinstance(In_Arr, arraypy):
	  raise TypeError('Input must be arraypy or Tensor type')
     
     flag=0
     for j in range(0, In_Arr.rank):
	  if (In_Arr.shape[0]!=In_Arr.shape[j]):
	       raise ValueError ('Different size of arrays axes')
     
     # forming list of tuples for arraypy constructor of type a = arraypy( [(a, b), (c, d), ... , (y, z)] )	  
     arg = [(In_Arr.start_index[i], In_Arr.end_index[i]) for i in range (In_Arr.rank)]
     
     # if In_Arr tensor, then ResArr will be tensor, else it will be arraypy	  
     if isinstance(In_Arr, tensor):
	  ResArr = tensor ( arraypy(arg), In_Arr.ind_char)
     else:
	  ResArr = arraypy(arg)
    
     index = [In_Arr.start_index[i] for i in range(In_Arr.rank)]
	
     for i in range(len(In_Arr)):													    
	  perm = list(permutations(index))
	  for temp_index in perm:
	       ResArr[tuple(index)]+=In_Arr[tuple(temp_index)]
	  if isinstance(ResArr[tuple(index)], int):     
	       ResArr[tuple(index)] = float(ResArr[tuple(index)]) 	  
	  ResArr[tuple(index)] /= fac(In_Arr.rank) 
	 
	  index = In_Arr.Next_index(index)
	      
     return ResArr

def Asymmetric(In_Arr):
     """
     Creates the asymmetric form of input tensor.
     Input: arraypy or tensor with equal axes (array shapes).
     Output: asymmetric array. Output type - arraypy or tensor, depends of input
     
     Examples:
     
     >>> a = list2arraypy(range(9), (3,3))
     >>> b = Asymmetric(a)
     >>> print (b)
     0.0 -1.0 -2.0 
     1.0 0.0 -1.0 
     2.0 1.0 0.0 
     """
     if not isinstance(In_Arr, arraypy):
	  raise TypeError('Input must be arraypy or Tensor type')
     
     flag=0
     for j in range(In_Arr.rank):
	  if (In_Arr.shape[0]!=In_Arr.shape[j]):
	       raise ValueError ('Different size of arrays axes')
	  
     # forming list of tuples for arraypy constructor of type a = arraypy( [(a, b), (c, d), ... , (y, z)] )	  
     arg = [(In_Arr.start_index[i], In_Arr.end_index[i]) for i in range (In_Arr.rank)]
	  
     # if In_Arr tensor, then ResArr will be tensor, else it will be arraypy	  
     if isinstance(In_Arr, tensor):
	  ResArr = tensor ( arraypy(arg), In_Arr.ind_char)
     else:
	  ResArr = arraypy(arg)
     
     signs = [0 for i in range(fac(In_Arr.rank))]
     temp_i=0
     for p in permutations(range(In_Arr.rank)):
	  signs[temp_i]=PermParity(list(p))
	  temp_i+=1     
    
     index = [In_Arr.start_index[i] for i in range(In_Arr.rank)]
	
     for i in range(len(In_Arr)):													    
	  perm = list(permutations(index))
	  perm_number = 0
	  for temp_index in perm:
	       ResArr[tuple(index)]+=signs[perm_number]*In_Arr[tuple(temp_index)]
	       perm_number+=1
	  if isinstance(ResArr[tuple(index)], int):     
	       ResArr[tuple(index)] = float(ResArr[tuple(index)]) 
	  ResArr[tuple(index)] /= fac(In_Arr.rank) 
	 
	  index = In_Arr.Next_index(index)
	      
     return ResArr

def PermParity(lst):
     '''\
     THANKS TO Paddy McCarthy FROM http://code.activestate.com/ FOR THIS FUNCTION!
     Given a permutation of the digits 0..N in order as a list, 
     returns its parity (or sign): +1 for even parity; -1 for odd.
     
     Example:
     >>> signs=zeros(6)
     >>> temp_i=0
     >>> for p in permutations(range(3)):
             signs[temp_i]=PermParity(list(p))
             print(signs[temp_i], p)
             temp_i+=1

    (1.0, (0, 1, 2))
    (-1.0, (0, 2, 1))
    (-1.0, (1, 0, 2))
    (1.0, (1, 2, 0))
    (1.0, (2, 0, 1))
    (-1.0, (2, 1, 0))
     '''
     parity = 1
     for i in range(0,len(lst)-1):
          if lst[i] != i:
               parity *= -1
               mn = min(range(i,len(lst)), key=lst.__getitem__)
               lst[i],lst[mn] = lst[mn],lst[i]
     return parity

def fac(n):
     """
     Finds factorial of n
     
     Examples:
     >>> fac(1)
     1
     >>> fac(3)
     6
     >>> fac(12)
     479001600
     """
     if n == 0:
	  return 1
     return fac(n-1) * n

#====================================================
#a = arraypy((2,2), 'Py')
#b1 = Symmetric(a)

#a = arraypy((2,2), 'Py')
#b = Asymmetric(a)
#print (a)
#print (b)
#print (b1)

##from sympy.tensor1 import *
##a = list2arraypy(range(9), (3,3))
#a = arraypy ( [2, 3, 1] )
#b = Symmetric(a)
#print (b)