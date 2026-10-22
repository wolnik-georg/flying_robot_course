"""Our KNODE training for the learned-FBL residual (Task 1, step 2): the training loop of Chee's thesis (App. B: one-step prediction, RK4, Adam, mean squared
error, weight decay, keep the best loss) around the downwash model of the authors' bundle (`ODEFunc`, imported read-only, never copied or edited).

Data layout (one row per time step, as the thesis and the bundle): ego state x (13: p, v, q_xyzw, body rates) | input u (4: thrust N, torque N m) | neighbour state (13),
sampled at dt. One-step prediction: x_hat(i+1) = RK4 over dt of ODEFunc starting at the logged row i (input and neighbour held over the step); loss = mean of
(x_hat - x(i+1))^2 weighted by W_x = diag(1 x10, 0.1 x3)  (paper Eq. 23: last three = body rates).

  from knode_train import train, make_pairs
No torchdiffeq: a differentiable RK4 step is written here (the thesis uses torchdiffeq.odeint(method='rk4', step_size=dt), the same scheme)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import bundle as B  # noqa: E402

WX = torch.tensor([1.0] * 10 + [0.1] * 3)


def make_pairs(X, U, XT, dt, min_r=2e-3):
    """X (N,13), U (N,4), XT (N,13) -> y0 (N-1, 1, 30), target (N-1, 13).
    Rows whose horizontal distance to the neighbour is below min_r (2 mm) are dropped: the flow frame of the model is degenerate at exact vertical alignment (the
    authors' runtime has the same singularity) and its backward pass gives NaN there (found 2026-10-22: a static top directly above the start point)."""
    y0 = np.concatenate([X[:-1], U[:-1], XT[:-1]], axis=1)
    keep = np.hypot(X[:-1, 0] - XT[:-1, 0], X[:-1, 1] - XT[:-1, 1]) >= min_r
    return torch.tensor(y0[keep], dtype=torch.float32).unsqueeze(1), torch.tensor(X[1:][keep], dtype=torch.float32)


def rk4_step(f, y, dt):
    """One RK4 step of the augmented ODE (derivative of inputs and neighbour is zero inside f, as in the bundle's training wrapper)."""
    t = torch.tensor(0.0)
    k1 = f(t, y); k2 = f(t, y + 0.5 * dt * k1); k3 = f(t, y + 0.5 * dt * k2); k4 = f(t, y + dt * k3)
    return y + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def predict(ode, y0, dt):
    return rk4_step(ode, y0, dt)[:, 0, :13]


def setup_ode(c_d_rom: float, mass: float | None = None, train_torque_net: bool = True):
    """ODEFunc with the ROM drag coefficient c_d_rom (module global of the bundle, in memory only); optional vehicle mass override of the nominal model.
    The torque net can be frozen at zero output (force-only training, as the hardware recipe compensates forces only)."""
    b = B.load(); b.nominal.C_d = float(c_d_rom)
    ode = b.nnm.ODEFunc()
    if mass is not None:
        ode.nominal_model.mass = float(mass)
    if not train_torque_net:
        last = ode.net_t[2]; torch.nn.init.zeros_(last.weight); torch.nn.init.zeros_(last.bias)
        for p in ode.net_t.parameters(): p.requires_grad_(False)
    return b, ode


def train(ode, y0, target, dt, iters=2000, lr=2e-2, weight_decay=2e-9, seed=123, wx=WX, log_every=250, verbose=True, y0_val=None, target_val=None):
    """Full-batch Adam on the one-step loss, keeps the parameters with the lowest loss (thesis B.3). Returns (best_state_dict, history)."""
    torch.manual_seed(seed); np.random.seed(seed)
    opt = torch.optim.Adam([p for p in ode.parameters() if p.requires_grad], lr=lr, weight_decay=weight_decay)
    best, best_state, hist = float("inf"), None, []
    for it in range(1, iters + 1):
        pred = predict(ode, y0, dt)
        loss = (((pred - target) ** 2) * wx).mean()
        opt.zero_grad(); loss.backward(); opt.step()
        l = float(loss.detach()); hist.append(l)
        if l < best:
            best = l; best_state = {k: v.detach().clone() for k, v in ode.state_dict().items()}
        if verbose and (it % log_every == 0 or it == 1):
            msg = f"iter {it:5d}  loss {l:.4e}  best {best:.4e}"
            if y0_val is not None:
                with torch.no_grad(): msg += f"  val {float((((predict(ode, y0_val, dt) - target_val) ** 2) * wx).mean()):.4e}"
            print(msg, flush=True)
    ode.load_state_dict(best_state)
    return best_state, hist


def one_step_loss(ode, y0, target, dt, wx=WX):
    with torch.no_grad():
        return float((((predict(ode, y0, dt) - target) ** 2) * wx).mean())
