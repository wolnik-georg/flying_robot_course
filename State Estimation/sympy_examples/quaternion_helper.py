import sympy as sp
import numpy as np

from sympy import init_printing

def vec_make(name):
    x = sp.Symbol(name + "x")
    y = sp.Symbol(name + "y")
    z = sp.Symbol(name + "z")
    return sp.Matrix([x, y, z])

def quat_make(name):
    w = sp.Symbol(name + "w")
    x = sp.Symbol(name + "x")
    y = sp.Symbol(name + "y")
    z = sp.Symbol(name + "z")
    return sp.Matrix([w, x, y, z])

def quat_conjugate(q):
    return sp.Matrix([q[0], -q[1], -q[2], -q[3]])

def quat_mult(q, p):
    q_w = q[0]
    q_x = q[1]
    q_y = q[2]
    q_z = q[3]
    p_w = p[0]
    p_x = p[1]
    p_y = p[2]
    p_z = p[3]
    return sp.Matrix([
    [q_w*p_w - q_x*p_x - q_y*p_y - q_z*p_z],
    [q_x*p_w + q_w*p_x - q_z*p_y + q_y*p_z],
    [q_y*p_w + q_z*p_x + q_w*p_y - q_x*p_z],
    [q_z*p_w - q_y*p_x + q_x*p_y + q_w*p_z],
    ])

def quat_im(q):
    return sp.Matrix([q[1], q[2], q[3]])

def quat_promote(v):
    return sp.Matrix([0, v[0], v[1], v[2]])

def quat_rotate_vector(q, v):
    return quat_im(quat_mult(quat_mult(q, quat_promote(v)), quat_conjugate(q)))

if __name__ == "__main__":
    init_printing()

    q = quat_make("q")
    p = vec_make("p")
    v = sp.Matrix([0, 0, sp.Symbol("vz")])
    sp.pprint(quat_rotate_vector(q, v))