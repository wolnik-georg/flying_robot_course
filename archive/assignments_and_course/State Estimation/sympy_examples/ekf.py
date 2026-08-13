import sympy as sp
import numpy as np
from quaternion_helper import *

from sympy import init_printing

if __name__ == "__main__":

    init_printing()

    px = sp.Symbol("x")
    px_dot = sp.Symbol("x_dot")
    pz = sp.Symbol("z")
    pz_dot = sp.Symbol("z_dot")
    theta = sp.Symbol("theta")
    theta_dot = sp.Symbol("theta_dot")

    x = sp.Matrix([px, px_dot, pz, pz_dot, theta, theta_dot])
    sp.pprint(x)

    f1 = sp.Symbol("f1")
    f2 = sp.Symbol("f2")

    m = sp.Symbol("m")
    grav = sp.Symbol("g")
    l = sp.Symbol("l")
    J = sp.Symbol("J")

    # dynamics
    f = sp.Matrix([
        px_dot,
        -(f1+f2)*sp.sin(theta)/m,
        pz_dot,
        (f1+f2)*sp.cos(theta)/m - grav,
        theta_dot,
        (f2-f1)*l/J,
        ])
    
    # step function (Euler)
    dt = sp.Symbol("dt")
    g = x + f * dt
    sp.pprint(g)

    # compute G
    G = g.jacobian(x)
    sp.pprint(G)
