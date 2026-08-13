import sympy as sp
from quaternion_helper import *

if __name__ == "__main__":

    sp.init_printing()

    # state
    p = vec_make("p")
    b = vec_make("b")
    d = vec_make("d")

    x = sp.Matrix([p,b,d])

    # Measurement flow
    dt = sp.Symbol("dt")
    N = sp.Symbol("N")
    theta = sp.Symbol("theta")

    h = dt * N / (p[2] * theta) * sp.Matrix([b[0], b[1]])
    sp.pprint(h)

    H = h.jacobian(x)
    sp.pprint(H)

    # # kalman gain

    # sigma = sp.Matrix([[sp.Symbol("sigma{}{}".format(row, column)) for column in range(9)] for row in range(9)])
    # # sp.pprint(H * sigma * H.T)

    # Q = sp.Matrix([sp.Symbol("q00")])

    # K = sigma * H.T * (H * sigma * H.T + Q).inv()
    # sp.pprint(K)
    # #
    # sp.pprint((sp.eye(9) - K * H) )