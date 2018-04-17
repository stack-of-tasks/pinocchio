from pinocchio.utils import zero, eye
import numpy as np
import numpy.linalg as npl

'''
This file implements a sparse linear problem (quadric cost, linear constraints -- LCQP)
where the decision variables are denoted by x=(x1 ... xn), n being the number of factors.
The problem can be written:
  min      Sum_i=1^p  || A_i x - b_i ||^2
x1...xn

so that    forall j=1:q   C_j x = d_i

Matrices A_i and C_j are block sparse, i.e. they are acting only on some (few) of the variables
x1 .. xn.

The file implements the main class FactorGraph, which stores the LCQP problem and solve it.
It also provides a secondary class Factor, used to set up FactorGraph
'''


class Factor(object):
    '''
    A factor is a part of a linear constraint corresponding either a cost ||A x - b|| or
    a constraint Cx = d.
    In both cases, we have Ax = sum A_i x_i, where some A_i are null. One object of class
    Factor stores one of the A_i, along with the correspond <i> index. It is simply a pair
    (index, matrix).

    This class is used as a arguments of some of the setup functions of FactorGraph.
    '''
    def __init__(self, index, matrix):
        self.index = index
        self.matrix = matrix


class FactorGraph(object):
    '''
    The class FactorGraph stores a block-sparse linear-constrained quadratic program (LCQP)
    of variable x=(x1...xn). The size of the problem is set up at construction of the object.
    Methods add_factor() and add_factor_constraint() are used to set up the problem.
    Method solve() is used to compute the solution to the problem.
    '''
    def __init__(self, variableSize, nbVariables):
        '''
        Initialize a QP sparse problem as min || A x - b || so that C x = d
        where  x = (x1, .., xn), and dim(xi) = variableSize and n = nbVariables
        After construction, A, b, C and d are allocated and set to 0.
        '''
        self.nx = variableSize
        self.N = nbVariables
        self.A = zero([0, self.N * self.nx])
        self.b = zero(0)
        self.C = zero([0, self.N * self.nx])
        self.d = zero(0)

    def matrix_form_factor(self, factors):
        '''
        Internal function: not designed to be called by the user.
        Create a factor matrix [ A1 0 A2 0 A3 ... ] where the Ai's are placed at
        the indexes of the factors.
        '''
        assert(len(factors) > 0)
        nr = factors[0].matrix.shape[0] # nb rows of the factor
        nc = self.nx * self.N           # nb cols

        # Define and fill the new rows to be added
        A = zero([nr, nc])               # new factor to be added to self.A
        for factor in factors:
            assert(factor.matrix.shape == (nr, self.nx))
            A[:, self.nx * factor.index:self.nx * (factor.index + 1)] = factor.matrix
        return A

    def add_factor(self, factors, reference):
        '''
        Add a factor || sum_{i} factor[i].matrix * x_{factor[i].index} - reference ||
        to the cost.
        '''
        # Add the new rows to the cost matrix.
        self.A = np.vstack([self.A, self.matrix_form_factor(factors)])
        self.b = np.vstack([self.b, reference])

    def add_factor_constraint(self, factors, reference):
        '''
        Add a factor sum_{i} factor[i].matrix * x_{factor[i].index} =  reference
        to the constraints.
        '''
        # Add the new rows to the cost matrix.
        self.C = np.vstack([self.C, self.matrix_form_factor(factors)])
        self.d = np.vstack([self.d, reference])

    def solve(self, eps=1e-8):
        '''
        Implement a LCQP solver, with numerical threshold eps.
        '''
        Cp = npl.pinv(self.C, eps)
        xopt = Cp * self.d
        P = eye(self.nx * self.N) - Cp * self.C
        xopt += npl.pinv(self.A * P, eps) * (self.b - self.A * xopt)
        return xopt
