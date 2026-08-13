import sympy as sp
import numpy as np
from quaternion_helper import *

from sympy import init_printing

if __name__ == "__main__":

    init_printing()

    # state
    p = vec_make("p") # world frame
    b = vec_make("b") # body frame
    d = vec_make("d") # error angles

    x = sp.Matrix([p,b,d])

    sp.pprint(x)

    # action
    a = vec_make("a")
    w = vec_make("w")
    u = sp.Matrix([w, a])

    # dynamics
    dt = sp.Symbol("dt")
    qref = quat_make("qref")

    d_norm = sp.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])
    d_norm_squared = d[0]*d[0]+d[1]*d[1]+d[2]*d[2]
    # 18a in Markley
    # q_from_delta = sp.Matrix([
    #     sp.cos(d_norm/2),
    #     d / d_norm * sp.sin(d_norm/2)
    # ])
    # 18c in Markley
    # q_from_delta = 1/(16+d_norm_squared)*sp.Matrix([8*d, 16-d_norm_squared])
    # sp.pprint(q_from_delta)

    # 19 in Markley
    q_from_delta = sp.Matrix([1-d_norm_squared/8, d/2])
    sp.pprint(q_from_delta)

    # g = sp.Matrix([
    #     p + quat_rotate_vector(quat_mult(qref, q_from_delta), v) * dt,
    #     v + (quat_rotate_vector(quat_conjugate(quat_mult(qref, q_from_delta)), sp.Matrix([0,0,-9.81])) + a)*dt,
    #     d + w * dt,
    #     ])
    
    # simplified version
    g = sp.Matrix([
        p + quat_rotate_vector(qref, v) * dt,
        v + (quat_rotate_vector(quat_conjugate(qref), sp.Matrix([0,0,-9.81])) + a)*dt,
        d + w * dt,
    ])

    # compute G
    G = g.jacobian(x)
    sp.pprint(G)

