#!/usr/bin/env python3
"""Compare the three phases of the residual dry run and write a short report.

Three questions, in the order they can actually be answered:

1. **Does the model predict?** Phase 3 has the weights loaded with `rnn.en=0`, so the prediction
   and the measurement are independent. Correlation and RMSE between `rnn_pred_*` and `a_res_*`
   are the honest measure of the model, and this is the only phase where they mean anything.

2. **Does compensation help?** Phase 4 against phase 1: does the formation hold its commanded
   geometry better with the feedforward on. Measured on the *realised* relative geometry, since
   that is what a downwash-rejection method is for.

3. **Did anything go wrong quietly?** The output clamp engaging, the residual collapsing to
   exactly zero, or the drone diverging all produce plausible-looking summary numbers.

Phase 4's own `a_res` is deliberately NOT used as the headline: with the network in the loop the
residual it sees is the one that survives its own compensation, so a small value there is
consistent with a good model and with a stable-but-wrong feedback loop alike.
"""

import argparse
import sys

import numpy as np


def load(path):
    with open(path) as f:
        header = f.readline().rstrip("\n").split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if a.size == 0:
        raise SystemExit(f"{path}: no rows")
    return {n: a[:, i] for i, n in enumerate(header)}


def drones(cols):
    return sorted({k.split(".", 1)[0] for k in cols if k.endswith(".x")})


def steady(cols, names, frac=0.15):
    """Mask off the first fraction: takeoff and the approach to formation are not the test."""
    t = cols["t"]
    return t >= t[0] + frac * (t[-1] - t[0])


def vec(cols, name, prefix):
    return np.stack([cols[f"{name}.{prefix}{a}"] for a in "xyz"], axis=1)


def report_prediction(cols, names, out):
    out.append("## 1. Does the model predict? (phase 3, `rnn.en=0`)\n")
    out.append("Prediction and measurement are independent here — the network is not touching "
               "the vehicle — so these numbers mean what they say.\n")
    out.append("| Drone | RMSE(a_res) | RMSE(pred − a_res) | reduction | corr | clamped |")
    out.append("|---|---|---|---|---|---|")
    m = steady(cols, names)
    worst = None
    for n in names:
        a = vec(cols, n, "a_res_")[m]
        p = vec(cols, n, "rnn_pred_")[m]
        base = float(np.sqrt(np.mean(a ** 2)))
        err = float(np.sqrt(np.mean((p - a) ** 2)))
        red = 100.0 * (1.0 - err / base) if base > 1e-9 else float("nan")
        # Correlation on the z axis, where the downwash lives.
        if a[:, 2].std() > 1e-9 and p[:, 2].std() > 1e-9:
            corr = float(np.corrcoef(a[:, 2], p[:, 2])[0, 1])
        else:
            corr = float("nan")
        clamp = float(np.mean(np.linalg.norm(p, axis=1) > 7.99) * 100)
        out.append(f"| {n} | {base:.4f} | {err:.4f} | {red:+.1f}% | {corr:.3f} | {clamp:.1f}% |")
        if worst is None or red < worst:
            worst = red
    out.append("")
    out.append("All values m/s². *Reduction* is how much of the measured residual the "
               "prediction accounts for; a model that cannot beat predicting zero shows ≤ 0%.\n")
    return worst


def report_geometry(a, b, names, out, label_a, label_b):
    out.append(f"\n## 2. Does compensation help? ({label_b} vs {label_a})\n")
    if len(names) < 2:
        out.append("Only one drone — no relative geometry to compare.\n")
        return None
    if f"{names[0]}.cmd_x" not in a:
        out.append("The logs carry no commanded position, so realised separation cannot be "
                   "separated from commanded motion. Re-fly with a current simulator.\n")
        return None

    out.append("Error in the *separation between the vehicles*: realised minus commanded. "
               "Downwash shows up as a **bias** — the lower vehicle sags, so the held gap "
               "closes — which is why the mean is the headline and the spread is secondary.\n")
    best = [None]
    out.append("| Pair | Axis | quantity | " + label_a + " | " + label_b + " | change |")
    out.append("|---|---|---|---|---|---|")
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            rows = []
            for label, cols in ((label_a, a), (label_b, b)):
                m = steady(cols, names)
                real = vec(cols, names[j], "")[m] - vec(cols, names[i], "")[m]
                cmd = vec(cols, names[j], "cmd_")[m] - vec(cols, names[i], "cmd_")[m]
                rows.append(real - cmd)
            ea, eb = rows
            for k, ax in enumerate("xyz"):
                for q, fn in (("mean error", np.mean), ("spread (std)", np.std)):
                    va, vb = float(fn(ea[:, k])), float(fn(eb[:, k]))
                    if q.startswith("mean"):
                        va, vb = abs(va), abs(vb)
                    if va < 0.001:
                        chg = "baseline < 1 mm"
                    else:
                        chg = f"{100 * (vb - va) / va:+.1f}%"
                    out.append(f"| {names[i]}–{names[j]} | {ax} | {q} | {va * 1000:.2f} mm | "
                               f"{vb * 1000:.2f} mm | {chg} |")
                    # The bias on the axis the wash acts along is the headline. Keep the
                    # largest one seen, so a pair that got worse cannot be hidden by a pair
                    # that got better.
                    if q.startswith("mean") and va >= 0.001:
                        if best[0] is None or va - vb < best[0][0] - best[0][1]:
                            best[0] = (va, vb, f"{names[i]}–{names[j]} {ax}")
    out.append("")
    return best[0]


def report_sanity(phases, names, out):
    out.append("\n## 3. Did anything go wrong quietly?\n")
    out.append("| Phase | rows | a_res ≡ 0 | max |a_res| | max altitude excursion |")
    out.append("|---|---|---|---|---|")
    ok = True
    for label, cols in phases:
        m = steady(cols, names)
        zero = all(np.allclose(vec(cols, n, "a_res_")[m], 0.0) for n in names)
        mx = max(float(np.abs(vec(cols, n, "a_res_")[m]).max()) for n in names)
        zr = max(float(np.ptp(cols[f"{n}.z"][m])) for n in names)
        out.append(f"| {label} | {len(cols['t'])} | {'YES' if zero else 'no'} | "
                   f"{mx:.3f} m/s² | {zr:.3f} m |")
        if zero:
            ok = False
    out.append("")
    if not ok:
        out.append("> **`a_res` is identically zero in at least one phase.** That is the "
                   "signature of no RPM source reaching the controller, not of an absence of "
                   "interaction. Nothing below that line is a measurement.\n")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--collect", required=True)
    ap.add_argument("--predict", required=True)
    ap.add_argument("--compensate", required=True)
    ap.add_argument("--out", default="-")
    args = ap.parse_args()

    c, p, k = load(args.collect), load(args.predict), load(args.compensate)
    names = drones(c)

    out = ["# Residual-learning dry run — simulation", ""]
    out.append("Collect → train → predict → compensate, entirely in simulation, through the "
               "same code paths the lab will use. **These are simulation numbers.** They say "
               "the pipeline works end to end; they say nothing about how well the method works "
               "on a real drone, where the residual is measured rather than generated by the "
               "same kind of model being fitted.\n")
    out.append(f"Drones: {', '.join(names)}\n")

    sane = report_sanity([("1 collect", c), ("3 predict", p), ("4 compensate", k)], names, out)
    red = report_prediction(p, names, out)
    geo = report_geometry(c, k, names, out, "1 collect", "4 compensate")

    out.append("\n## Verdict\n")
    if not sane:
        out.append("**INVALID** — see section 3. A phase produced no residual measurement.")
        verdict = 1
    elif red is None or red <= 0:
        out.append("**Pipeline works, model does not.** Every stage ran and the weights loaded, "
                   "but the prediction does not beat predicting zero. That is a training or "
                   "data-coverage problem, not a plumbing problem.")
        verdict = 0
    else:
        line = (f"**Pipeline works end to end.** The model accounts for {red:.0f}% of the "
                f"measured residual on the worst drone with the loop open, and the "
                f"compensation phase ran with `rnn.en=1` without destabilising the vehicle.")
        if geo is not None:
            va, vb, where = geo
            line += (f" Turning it on took the steady separation error on {where} from "
                     f"{va * 1000:.2f} mm to {vb * 1000:.2f} mm.")
        out.append(line)
        verdict = 0

    text = "\n".join(out) + "\n"
    if args.out == "-":
        print(text)
    else:
        with open(args.out, "w") as f:
            f.write(text)
        print(text)
        print(f"wrote {args.out}")
    return verdict


if __name__ == "__main__":
    sys.exit(main())
