"""Turn flight logs into training tensors for the residual model.

Input is a merged multi-drone CSV from `tools/merge_usd_logs.py` (uSD logs, 500 Hz, time-aligned).
Output is `(rel, mask, y)`: relative neighbour states, a presence mask, and the measured residual
acceleration `indi.a_res_*` = f_res/m that the network has to predict.

Three things here are easy to get wrong and expensive to discover later:

**Sign convention.** The firmware feeds `peer - own`. `merge_usd_logs.py` writes its `rel.i_j`
columns as `i - j`, i.e. *own minus peer* when `i` is the ego drone -- the opposite. This module
therefore computes relative states from the absolute columns itself and never reads `rel.*`, so
there is one convention and it is the firmware's. A sign flip here trains a mirrored model that
would push the drone *into* the disturbance.

**The firmware's input guards.** Neighbours beyond `MAX_DIST` are dropped and separations below
`MIN_DIST` are clamped, onboard, before the network sees anything. The same guards are applied
here. Without that the model is trained on a distribution the firmware never presents.

**`a_res` reading exactly zero.** That is what happens with no RPM source, and it is not a
measurement of "no interaction" -- it is the absence of a measurement. Those samples are dropped
and counted, loudly, because a dataset of zeros trains a network that predicts nothing and looks
like it converged beautifully.
"""

import sys

import numpy as np

from model import MAX_NEIGHBOURS, MAX_DIST, MIN_DIST, PHI_IN


def load_merged(path):
    """Read a merged CSV into {column: array}."""
    with open(path) as f:
        header = f.readline().rstrip("\n").split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {n: a[:, i] for i, n in enumerate(header)}


def apply_input_guards(dp):
    """The firmware's distance guards, applied to an (N, 3) block of relative positions.

    Onboard, a neighbour past MAX_DIST is skipped and a separation below MIN_DIST is clamped
    radially without turning the direction. Training data must go through the same funnel: a
    model fitted to inputs the firmware never presents is fitted to the wrong distribution, and
    the disagreement is invisible until predicted and measured residuals are compared in flight.

    Returns (dp_guarded, near, n_clamped).
    """
    d = np.linalg.norm(dp, axis=1)
    near = d <= MAX_DIST
    tiny = (d < MIN_DIST) & (d > 1e-6)
    scale = np.where(tiny, MIN_DIST / np.maximum(d, 1e-6), 1.0)[:, None]
    return dp * scale, near, int((tiny & near).sum())


def drone_names(cols):
    """Drone names, from the `<name>.x` columns. Sorted, so ordering is reproducible."""
    return sorted({k.split(".", 1)[0] for k in cols
                   if k.endswith(".x") and not k.startswith("rel.")})


def build(sources, z_floor=0.0, drop_zero_a_res=True, verbose=True):
    """Build training arrays from one or more merged CSVs (paths or already-loaded dicts).

    Every drone in every file contributes samples as the ego vehicle in turn: with two drones a
    100 s flight yields two 100 s trajectories of training data, not one. The interaction is not
    symmetric -- the lower drone is in the wash and the upper one is barely affected -- so both
    roles carry information.

    Returns (rel, mask, y, stats).
      rel  (N, MAX_NEIGHBOURS, 6) float32 -- peer minus own, position then velocity, world frame
      mask (N, MAX_NEIGHBOURS)    float32 -- 1 where a neighbour is present and within range
      y    (N, 3)                 float32 -- measured a_res [m/s^2]
    """
    rels, masks, ys = [], [], []
    stats = {"files": 0, "rows_in": 0, "dropped_zero_a_res": 0, "dropped_z_floor": 0,
             "dropped_no_neighbour": 0, "clamped_min_dist": 0, "dropped_far": 0,
             "per_source": []}

    for src in sources:
        cols = load_merged(src) if isinstance(src, str) else src
        name = src if isinstance(src, str) else "<dict>"
        names = drone_names(cols)
        if len(names) < 2:
            print(f"WARNING: {name}: {len(names)} drone(s) -- no interaction to learn, skipped.",
                  file=sys.stderr)
            continue
        stats["files"] += 1
        n_rows = len(cols["t"])
        kept_here = 0

        for ego in names:
            need = [f"{ego}.a_res_{a}" for a in "xyz"]
            if any(k not in cols for k in need):
                print(f"WARNING: {name}: {ego} has no a_res_* -- skipped. On hardware this "
                      f"means no RPM source was present, which is a flight to redo, not a "
                      f"dataset to trim.", file=sys.stderr)
                continue

            y = np.stack([cols[k] for k in need], axis=1)
            p_e = np.stack([cols[f"{ego}.{a}"] for a in "xyz"], axis=1)
            v_e = np.stack([cols[f"{ego}.v{a}"] for a in "xyz"], axis=1)

            keep = np.ones(n_rows, bool)
            if drop_zero_a_res:
                zero = np.all(y == 0.0, axis=1)
                stats["dropped_zero_a_res"] += int(zero.sum())
                keep &= ~zero
            if z_floor > 0.0:
                low = p_e[:, 2] < z_floor
                stats["dropped_z_floor"] += int(low.sum())
                keep &= ~low

            peers = [n for n in names if n != ego][:MAX_NEIGHBOURS]
            rel = np.zeros((n_rows, MAX_NEIGHBOURS, PHI_IN), np.float32)
            mask = np.zeros((n_rows, MAX_NEIGHBOURS), np.float32)

            for k, peer in enumerate(peers):
                p_p = np.stack([cols[f"{peer}.{a}"] for a in "xyz"], axis=1)
                v_p = np.stack([cols[f"{peer}.v{a}"] for a in "xyz"], axis=1)
                dp = p_p - p_e                      # firmware convention: peer - own
                dv = v_p - v_e

                dp, near, n_clamped = apply_input_guards(dp)
                stats["dropped_far"] += int((~near).sum())
                stats["clamped_min_dist"] += n_clamped

                rel[:, k, :3] = dp
                rel[:, k, 3:] = dv
                mask[:, k] = near.astype(np.float32)

            no_nb = mask.sum(axis=1) == 0
            stats["dropped_no_neighbour"] += int((no_nb & keep).sum())
            keep &= ~no_nb

            rels.append(rel[keep])
            masks.append(mask[keep])
            ys.append(y[keep].astype(np.float32))
            kept_here += int(keep.sum())

        stats["rows_in"] += n_rows * len(names)
        stats["per_source"].append((name, len(names), n_rows, kept_here))

    if not rels:
        raise SystemExit("No usable samples. Check that the logs carry a_res_* and that it is "
                         "not identically zero -- see docs/13_Residual_Learning.md.")

    rel = np.concatenate(rels).astype(np.float32)
    mask = np.concatenate(masks).astype(np.float32)
    y = np.concatenate(ys).astype(np.float32)

    if verbose:
        print(f"dataset: {len(y)} samples from {stats['files']} file(s)")
        for nm, nd, nr, kept in stats["per_source"]:
            print(f"  {nm}: {nd} drones x {nr} rows -> {kept} kept")
        for k in ("dropped_zero_a_res", "dropped_z_floor", "dropped_no_neighbour",
                  "dropped_far", "clamped_min_dist"):
            if stats[k]:
                print(f"  {k}: {stats[k]}")
        if stats["dropped_zero_a_res"] > 0.5 * max(stats["rows_in"], 1):
            print("  WARNING: most samples had a_res identically zero. That is the signature of "
                  "a missing RPM source, not of an absence of interaction.", file=sys.stderr)
    return rel, mask, y, stats


def normalisation(rel, mask):
    """Per-input mean and std over present neighbours only.

    Padding rows are exact zeros; folding them into the statistics would pull the mean toward
    zero by an amount that depends on how many drones happened to be flying, which would make
    a 2-drone model and a 3-drone model normalise differently for no physical reason.
    """
    flat = rel.reshape(-1, PHI_IN)
    sel = mask.reshape(-1) > 0
    if sel.sum() < 2:
        raise SystemExit("Fewer than two neighbour observations -- nothing to normalise.")
    mu = flat[sel].mean(axis=0).astype(np.float64)
    sigma = flat[sel].std(axis=0).astype(np.float64)
    # A channel that never moved cannot be scaled; leave it at unit gain rather than dividing
    # by noise. Reported by train.py so it is visible rather than silently absorbed.
    sigma = np.where(sigma < 1e-6, 1.0, sigma)
    return mu, sigma


def split(n, val_frac=0.2, seed=0):
    """Contiguous-block split, not per-sample shuffling.

    Consecutive 500 Hz samples are nearly identical, so a random per-sample split leaks almost
    every validation point into training and reports a validation error that is meaningless.
    Blocks keep the two sets genuinely separate while still sampling the whole flight.
    """
    rng = np.random.default_rng(seed)
    n_blocks = max(10, int(n / 500))                 # ~1 s blocks at 500 Hz
    edges = np.linspace(0, n, n_blocks + 1).astype(int)
    order = rng.permutation(n_blocks)
    n_val = max(1, int(round(val_frac * n_blocks)))
    val = np.zeros(n, bool)
    for b in order[:n_val]:
        val[edges[b]:edges[b + 1]] = True
    return ~val, val


# ── Synthetic data ──────────────────────────────────────────────────────────

def synthetic(n=20000, n_neighbours=1, seed=0, noise=0.02):
    """A downwash-shaped function, for exercising the pipeline with no flight data.

    This is **not** a substitute for measurement and no result may be quoted from it. It exists
    so that training, export, upload and the firmware round-trip can be tested end to end before
    a drone has flown -- every one of those steps can fail, and finding out in the lab costs
    flight time.

    Shape: force downward on a vehicle *below* another, falling off as a Gaussian in horizontal
    offset and decaying with vertical separation. That is the qualitative structure Neural-Swarm2
    reports and the structure the simulator applies; the constants here are invented.
    """
    rng = np.random.default_rng(seed)
    rel = np.zeros((n, MAX_NEIGHBOURS, PHI_IN), np.float32)
    mask = np.zeros((n, MAX_NEIGHBOURS), np.float32)
    y = np.zeros((n, 3), np.float32)

    for k in range(n_neighbours):
        dp = np.stack([rng.uniform(-0.5, 0.5, n), rng.uniform(-0.5, 0.5, n),
                       rng.uniform(-1.0, 1.0, n)], axis=1)
        dv = rng.normal(0.0, 0.3, (n, 3))
        # Through the same guards as real data, and the target is computed from the GUARDED
        # separation -- the firmware genuinely cannot see below MIN_DIST, so a target that
        # depended on the unguarded value would ask the model to recover information it is
        # never given.
        dp, near, _ = apply_input_guards(dp)
        rel[:, k, :3] = dp
        rel[:, k, 3:] = dv
        mask[:, k] = near.astype(np.float32)

        r = np.linalg.norm(dp[:, :2], axis=1)
        dz = dp[:, 2]
        # Only a neighbour ABOVE (dz > 0) pushes the ego vehicle down.
        above = np.clip(dz, 0.0, None)
        a_z = -3.0 * np.exp(-(r / 0.15) ** 2) * np.exp(-above / 0.4) * (dz > 0.02)
        y[:, 2] += a_z.astype(np.float32)
        y[:, 0] += (0.15 * a_z * dp[:, 0] / np.maximum(r, 1e-3)).astype(np.float32)
        y[:, 1] += (0.15 * a_z * dp[:, 1] / np.maximum(r, 1e-3)).astype(np.float32)

    y += rng.normal(0.0, noise, y.shape).astype(np.float32)
    return rel, mask, y
