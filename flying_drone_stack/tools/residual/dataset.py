"""Turn flight logs into training tensors for the residual model.

Input is a merged multi-drone CSV from `tools/merge_usd_logs.py` (uSD logs, 500 Hz, time-aligned).
Output is `(rel, mask, ground, y, stats)`: relative neighbour states, a presence+gate mask, each
sample's own ground-effect term, and the measured residual the network has to predict.

Three things here are easy to get wrong and expensive to discover later:

**Sign convention.** The firmware feeds `peer - own`. `merge_usd_logs.py` writes its `rel.i_j`
columns as `i - j`, i.e. *own minus peer* when `i` is the ego drone -- the opposite. This module
therefore computes relative states from the absolute columns itself and never reads `rel.*`, so
there is one convention and it is the firmware's. A sign flip here trains a mirrored model that
would push the drone *into* the disturbance.

**The gate, not a distance guard.** The OLD architecture clamped/dropped neighbours by distance
before the network ever saw them (`MAX_DIST`/`MIN_DIST`). Neural-Swarm2 has no such guard --
`model.build_mask` (presence AND the reference's `|dx|<0.2, |dy|<0.2, |dvx|<1.5` gate) is the only
filter, applied unconditionally every tick, onboard and here alike. A neighbour that fails the
gate is not dropped from the dataset; the ROW is still a valid sample (phi_G's ground term is
evaluated unconditionally, gate or no gate) -- only that neighbour's contribution is masked out,
exactly as the firmware would.

**`a_res` reading exactly zero.** That is what happens with no RPM source, and it is not a
measurement of "no interaction" -- it is the absence of a measurement. Those samples are dropped
and counted, loudly, because a dataset of zeros trains a network that predicts nothing and looks
like it converged beautifully. Checked on all three components together, since a real interaction
event essentially never leaves all three exactly zero.

2026-09-15 rewrite for the Neural-Swarm2 architecture, resolving the three items the previous
(deliberately import-blocked) version of this file left open:

1. **The target is a scalar**, and this is a property of the already-verified network
   (`model.NeuralSwarm2` only ever outputs a Z force), not a choice this loader makes. `y` is
   `a_res_z` alone, still in **acceleration** units [m/s^2] -- the conversion to the reference's
   grams unit (`model.accel_to_grams`) is left to the training step, which is where a per-flight
   mass would actually be known, rather than guessing one here. The x/y components of the
   measured residual are still read (for the zero-check) but have no predictor in this
   architecture; that is a real scoping fact about Strategy 2, not a limitation of the data --
   `a_res_x`/`a_res_y` remain fully logged and available for any other analysis.
2. **`ground` is a new, required output** -- `[0 - own_z, -own_vx, -own_vy, -own_vz]` per sample,
   evaluated for every row regardless of neighbours.
3. **`z_floor` defaults to 0.0 (off).** Neural-Swarm2 models ground effect explicitly via
   `phi_G`, so dropping low-altitude samples would starve exactly the input it needs. Left as a
   parameter, not removed, so a *different* analysis that genuinely wants ground-effect samples
   excluded still can.

2026-09-17: `train.py` rewritten to match this file's contract -- `NeuralSwarm2` (not the removed
`DeepSets`), the two separate `normalisation_rel`/`normalisation_ground` calls
`model.fold_normalisation` needs, the 5-tuple `build()` return, and the network's native
grams output converted to/from this project's acceleration convention at the loss boundary
(`model.accel_to_grams`/`grams_to_accel`). The pipeline is exercised end-to-end via
`--synthetic` (`fold check: max |trained - exported| = 8.45e-08 m/s^2`, 19297 weights) --
what remains untested is `dataset.build()` against a real merged CSV, only reachable with a
real flight log.
"""

import sys

import numpy as np

import model


def load_merged(path):
    """Read a merged CSV into {column: array}.

    Skips leading `# meta:...` comment lines (the convention `merge_usd_logs.py --meta` writes,
    e.g. `t_zero=scenario_start`) before reading the real header -- without this, the comment
    line is mistaken for the header and the real header row is fed to `np.loadtxt` as data,
    which fails immediately (`could not convert string 't' to float64`) rather than silently
    producing wrong columns. This loader does not need `t_zero` itself (it never reconstructs a
    commanded trajectory), so the value is skipped, not parsed.
    """
    with open(path) as f:
        lines = f.readlines()
    header_idx = next((i for i, l in enumerate(lines) if l.strip() and not l.startswith("#")),
                      None)
    if header_idx is None:
        raise ValueError(f"{path}: no header line found")
    header = lines[header_idx].rstrip("\n").split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=header_idx + 1, ndmin=2)
    return {n: a[:, i] for i, n in enumerate(header)}


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

    Returns (rel, mask, ground, y, stats).
      rel    (N, MAX_NEIGHBOURS, 6) float32 -- peer minus own, position then velocity, world frame
      mask   (N, MAX_NEIGHBOURS)    float32 -- present AND passes the reference's proximity gate
      ground (N, 4)                 float32 -- [0 - own_z, -own_vx, -own_vy, -own_vz]
      y      (N,)                   float32 -- measured a_res_z [m/s^2] (NOT yet grams)
    """
    rels, masks, grounds, ys = [], [], [], []
    stats = {"files": 0, "rows_in": 0, "dropped_zero_a_res": 0, "dropped_z_floor": 0,
             "no_neighbour_influence": 0, "per_source": []}

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

            a_res = np.stack([cols[k] for k in need], axis=1)     # (N, 3), for the zero-check
            p_e = np.stack([cols[f"{ego}.{a}"] for a in "xyz"], axis=1)
            v_e = np.stack([cols[f"{ego}.v{a}"] for a in "xyz"], axis=1)

            keep = np.ones(n_rows, bool)
            if drop_zero_a_res:
                zero = np.all(a_res == 0.0, axis=1)
                stats["dropped_zero_a_res"] += int(zero.sum())
                keep &= ~zero
            if z_floor > 0.0:
                low = p_e[:, 2] < z_floor
                stats["dropped_z_floor"] += int(low.sum())
                keep &= ~low

            # Ground term: unconditional, every row, regardless of neighbours or `keep` -- it is
            # evaluated by the firmware every tick whether or not any peer is nearby.
            ground = np.stack([0.0 - p_e[:, 2], -v_e[:, 0], -v_e[:, 1], -v_e[:, 2]],
                              axis=1).astype(np.float32)

            peers = [n for n in names if n != ego][:model.MAX_NEIGHBOURS]
            rel = np.zeros((n_rows, model.MAX_NEIGHBOURS, model.PHI_IN_SL), np.float32)
            present = np.zeros((n_rows, model.MAX_NEIGHBOURS), np.float32)

            for k, peer in enumerate(peers):
                p_p = np.stack([cols[f"{peer}.{a}"] for a in "xyz"], axis=1)
                v_p = np.stack([cols[f"{peer}.v{a}"] for a in "xyz"], axis=1)
                rel[:, k, :3] = p_p - p_e            # firmware convention: peer - own
                rel[:, k, 3:] = v_p - v_e
                present[:, k] = 1.0

            mask = model.build_mask(rel, present)
            stats["no_neighbour_influence"] += int(((mask.sum(axis=1) == 0) & keep).sum())
            # Deliberately NOT dropped: a row with no neighbour inside the gate is a legitimate
            # sample -- phi_G's ground term still applies, and the network needs "no interaction"
            # examples as much as it needs interaction ones.

            rels.append(rel[keep])
            masks.append(mask[keep])
            grounds.append(ground[keep])
            ys.append(a_res[keep, 2].astype(np.float32))   # scalar target: a_res_z only
            kept_here += int(keep.sum())

        stats["rows_in"] += n_rows * len(names)
        stats["per_source"].append((name, len(names), n_rows, kept_here))

    if not rels:
        raise SystemExit("No usable samples. Check that the logs carry a_res_* and that it is "
                         "not identically zero -- see docs/13_Residual_Learning.md.")

    rel = np.concatenate(rels).astype(np.float32)
    mask = np.concatenate(masks).astype(np.float32)
    ground = np.concatenate(grounds).astype(np.float32)
    y = np.concatenate(ys).astype(np.float32)

    if verbose:
        print(f"dataset: {len(y)} samples from {stats['files']} file(s)")
        for nm, nd, nr, kept in stats["per_source"]:
            print(f"  {nm}: {nd} drones x {nr} rows -> {kept} kept")
        for k in ("dropped_zero_a_res", "dropped_z_floor", "no_neighbour_influence"):
            if stats[k]:
                note = ""
                if k == "dropped_zero_a_res":
                    note = " (row-ticks summed over all ego drones; partner may read 0 a_res)"
                print(f"  {k}: {stats[k]}{note}")
        if stats["dropped_zero_a_res"] > 0.5 * max(stats["rows_in"], 1):
            print("  WARNING: most samples had a_res identically zero. That is the signature of "
                  "a missing RPM source, not of an absence of interaction.", file=sys.stderr)
    return rel, mask, ground, y, stats


def normalisation_rel(rel, mask):
    """Per-input mean and std of the neighbour term, over present-and-gated neighbours only.

    Padding rows are exact zeros; folding them into the statistics would pull the mean toward
    zero by an amount that depends on how many drones happened to be flying, which would make
    a 2-drone model and a 3-drone model normalise differently for no physical reason. A row
    masked out by the GATE (a real neighbour, just outside the proximity window) is excluded for
    the same reason -- it never reaches phi_S with a non-zero weight either.
    """
    flat = rel.reshape(-1, model.PHI_IN_SL)
    sel = mask.reshape(-1) > 0
    if sel.sum() < 2:
        raise SystemExit("Fewer than two gated neighbour observations -- nothing to normalise.")
    mu = flat[sel].mean(axis=0).astype(np.float64)
    sigma = flat[sel].std(axis=0).astype(np.float64)
    sigma = np.where(sigma < 1e-6, 1.0, sigma)
    return mu, sigma


def normalisation_ground(ground):
    """Per-input mean and std of the ground term. No mask: `ground` applies to every row."""
    if len(ground) < 2:
        raise SystemExit("Fewer than two samples -- nothing to normalise.")
    mu = ground.astype(np.float64).mean(axis=0)
    sigma = ground.astype(np.float64).std(axis=0)
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
    reports and the structure the simulator applies; the constants here are invented. Ground
    effect is a small, separate downward term at low own_z, so the ground path has something
    non-trivial to fit too.
    """
    rng = np.random.default_rng(seed)
    rel = np.zeros((n, model.MAX_NEIGHBOURS, model.PHI_IN_SL), np.float32)
    present = np.zeros((n, model.MAX_NEIGHBOURS), np.float32)
    y = np.zeros(n, np.float32)

    own_z = rng.uniform(0.05, 1.5, n).astype(np.float32)
    own_vel = rng.normal(0.0, 0.3, (n, 3)).astype(np.float32)
    ground = np.stack([0.0 - own_z, -own_vel[:, 0], -own_vel[:, 1], -own_vel[:, 2]],
                      axis=1).astype(np.float32)
    y += (-0.4 * np.exp(-own_z / 0.15)).astype(np.float32)          # ground effect, near-field only

    for k in range(n_neighbours):
        dp = np.stack([rng.uniform(-0.5, 0.5, n), rng.uniform(-0.5, 0.5, n),
                       rng.uniform(-1.0, 1.0, n)], axis=1).astype(np.float32)
        dv = rng.normal(0.0, 0.3, (n, 3)).astype(np.float32)
        rel[:, k, :3] = dp
        rel[:, k, 3:] = dv
        present[:, k] = 1.0

        r = np.linalg.norm(dp[:, :2], axis=1)
        dz = dp[:, 2]
        # Only a neighbour ABOVE (dz > 0) pushes the ego vehicle down. Same shape as before the
        # rewrite, but the target the gate would actually hide is no longer subtracted out --
        # samples the real gate masks still get this contribution in `y`, same as a real flight
        # would show a real (ungated) force that the network simply isn't shown the cause of.
        above = np.clip(dz, 0.0, None)
        a_z = -3.0 * np.exp(-(r / 0.15) ** 2) * np.exp(-above / 0.4) * (dz > 0.02)
        y += a_z.astype(np.float32)

    mask = model.build_mask(rel, present)
    y += rng.normal(0.0, noise, y.shape).astype(np.float32)
    return rel, mask, ground, y
