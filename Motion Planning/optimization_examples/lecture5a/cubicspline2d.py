import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# Evaluate a cubic spline with coefficients a[0] ... a[3] at t in [0,1]
def eval(a, t):
    return a[:,0] + a[:,1] * t + a[:,2] * t * t + a[:,3] * t * t * t

# Evaluate first derivative of cubic spline with coefficients a[0] ... a[3] at t in [0,1]
def eval_p(a, t):
    return a[:,1] + 2 * a[:,2] * t + 3 * a[:,3] * t * t

# Evaluate second derivative of cubic spline with coefficients a[0] ... a[3] at t in [0,1]
def eval_pp(a, t):
    return 2 * a[:,2] + 6 * a[:,3] * t

# Compute acceleration cost of cubic spline with coefficients a[0] ... a[3]
def cost_acc(a):
    return 2 * a[:,2] + 3 * a[:,3]


def main():
    points = np.array([
        [0, 0],
        [0.2, 0.8],
        [1, 1],
    ])

    # Construct the problem.
    a = cp.Variable((2,4))
    b = cp.Variable((2,4))
    objective = cp.Minimize(cp.sum(cost_acc(a) + cost_acc(b)))
    constraints = [
        eval(a, 0) == points[0],
        eval(a, 1) == points[1],
        eval(b, 0) == points[1],
        eval(b, 1) == points[2],
        eval_p(a, 1) == eval_p(b, 0),
        eval_pp(a, 1) == eval_pp(b, 0),
        eval_p(a, 0) == 0,
        eval_p(b, 1) == 0,
    ]
    prob = cp.Problem(objective, constraints)

    # The optimal objective value is returned by `prob.solve()`.
    result = prob.solve()#solver=cp.OSQP, verbose=True)
    # The optimal value for x is stored in `x.value`.

    print(a.value)
    print(b.value)

    ts = np.linspace(0, 1, 100)
    p_a = np.array([eval(a.value, t) for t in ts])
    p_b = np.array([eval(b.value, t) for t in ts])

    plt.plot(p_a[:,0], p_a[:,1])
    plt.plot(p_b[:,0], p_b[:,1])

    plt.scatter(points[:,0], points[:,1])
    plt.show()



if __name__ == "__main__":
  main()