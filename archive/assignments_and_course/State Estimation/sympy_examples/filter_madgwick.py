import sympy as sp
import numpy as np
from quaternion_helper import *

from sympy import init_printing

if __name__ == "__main__":

    init_printing()

    q = quat_make("q")
    a = vec_make("a") # assume this is normalized
    f = quat_rotate_vector(quat_conjugate(q), sp.Matrix([0,0,1]))  - a

    # Note that the Madgwick solution in z is different
    # if one uses the known SO(3) property qw^2+qx^2+qy^2+qz^2=1
    # we have 
    #    -az + qw^2 - qx^2 - qy^2 + qz^2
    # =  (qw^2+qx^2+qy^2+qz^2)-2qx^2-2qy^2 -az
    # =  1-2qx^2-2qy^2 -az (Madgwick version)
    sp.pprint(f)

    # # compute the Jacobian
    # J = f.jacobian(q)
    # sp.pprint(J)

    # # compute the step
    # sp.pprint(sp.simplify(J.T * f))