#!/usr/bin/env python3
"""RPM filter evidence: (1) raw DShot spikes vs optical deck, the real firmware filter (rpm_filter.h compiled and applied to the logged raw stream),
(2) on-drone check: does a spike reach the residual a_res (computed from the filtered RPM) before / after each firmware step?
System python3. Output: out/rpm_filter_evidence/ (figures + numbers.json)."""
import ctypes, glob, json, os, re, subprocess, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent; REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
import decode_usd_log as dec
OUT = HERE / "out" / "rpm_filter_evidence"; OUT.mkdir(parents=True, exist_ok=True)
SRC = OUT / "rpm_filter_wrap.c"
SRC.write_text('#include "%s"\nuint16_t step(uint16_t r, uint8_t s, uint16_t *p){return rpm_filter_step(r,s,p);}\n' % (REPO / "flying_drone_stack/firmware_app/rpm_filter.h"))


def lib(hold):
    so = OUT / f"rpm_filter_hold{hold}.so"
    subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", f"-DRPM_FILTER_HOLD_SENTINEL={hold}", "-o", str(so), str(SRC)])
    L = ctypes.CDLL(str(so)); L.step.argtypes = [ctypes.c_uint16, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint16)]; L.step.restype = ctypes.c_uint16
    return L


def run_filter(L, raw):
    prev = ctypes.c_uint16(0); out = np.empty(len(raw))
    for k, v in enumerate(raw): out[k] = L.step(int(v), 1, ctypes.byref(prev))
    return out


def flights():
    out = []
    for f in sorted(glob.glob(str(REPO / "experiments/logs/usd_raw/**/cf5_*.bin"), recursive=True)):
        m = re.search(r"(20\d\d-\d\d-\d\d)_(\d\d-\d\d-\d\d)", os.path.basename(f))
        if not m or os.path.getsize(f) < 50000: continue
        try: r = dec.load(f)
        except Exception: continue
        if "motor_m1_rpm" not in r or "a_res_z" not in r or (r["z"] > 0.3).sum() < 1000 or np.abs(r["a_res_z"]).max() == 0: continue
        out.append((m.group(1) + " " + m.group(2), f, r))
    return out


def period(stamp):   # firmware steps: spike guard committed 2026-09-29 21:21; sentinel hold flown from 2026-10-09
    d = stamp
    return "A: before the spike guard" if d < "2026-09-29 21:21" else ("B: spike guard" if d < "2026-10-09" else "C: spike guard + sentinel hold")


def spike_ticks(r, i):
    raw = r[f"motor_m{i}_rpm"].astype(float); deck = r[f"rpm_m{i}"].astype(float); up = r["z"] > 0.3
    big = up & (raw > 28000) & (raw < 65000)
    dip = up & (raw < 28000) & (np.abs(raw - deck) > 10000) & (deck > 5000) & (raw > 0)
    sen = up & (raw >= 65000)
    if sen.sum() > 0.01 * up.sum(): sen = np.zeros_like(sen)   # a motor that reads 0xFFFF all the time is not an occasional invalid reading
    return big, dip, sen


def main():
    F = flights(); print(len(F), "flights with a_res and DShot raw")
    # ---- (2) a_res response at spike ticks per period and spike type
    stats = {}
    for stamp, f, r in F:
        P = period(stamp); ar = r["a_res_z"].astype(float); d = np.abs(np.diff(ar, prepend=ar[0])); up = r["z"] > 0.3
        for i in range(1, 5):
            for name, m in zip(("huge (>28000)", "dip (>10000 below deck)", "invalid 0xFFFF"), spike_ticks(r, i)):
                ks = np.where(m)[0]; ks = ks[(ks > 5) & (ks < len(ar) - 5)]
                st = stats.setdefault((P, name), dict(spike=[], base=[]))
                st["spike"] += [float(d[k:k + 2].max()) for k in ks]
        st = stats.setdefault((P, "all"), dict(spike=[], base=[]))
        idx = np.where(up)[0]; idx = idx[(idx > 5) & (idx < len(ar) - 5)]
        st["base"] += list(d[idx][::7])
    res = {}
    for (P, name), v in sorted(stats.items()):
        if name == "all": continue
        base = stats[(P, "all")]["base"]
        res[f"{P} | {name}"] = dict(n=len(v["spike"]), median_da=float(np.median(v["spike"])) if v["spike"] else None, p95_da=float(np.percentile(v["spike"], 95)) if v["spike"] else None,
                                     base_median=float(np.median(base)), base_p95=float(np.percentile(base, 95)))
    for k, v in res.items(): print(k, v)
    # ---- (1) example flight: the one with the largest raw spike
    best = None
    for stamp, f, r in F:
        for i in range(1, 5):
            raw = r[f"motor_m{i}_rpm"].astype(float); m = (r["z"] > 0.3) & (raw < 65000)
            mx = raw[m].max() if m.any() else 0
            if best is None or mx > best[0]: best = (mx, stamp, f, r, i)
    mx, stamp, f, r, i = best; print("largest raw spike", mx, stamp, os.path.basename(f), "motor", i)
    L1 = lib(1); raw = r[f"motor_m{i}_rpm"].astype(float); deck = r[f"rpm_m{i}"].astype(float); filt = run_filter(L1, raw); t = r["t"] - r["t"][0]
    up = r["z"] > 0.3
    n_all = {}
    for stamp2, f2, r2 in F:
        for j in range(1, 5):
            rw = r2[f"motor_m{j}_rpm"].astype(float); fl = run_filter(L1, rw); u = r2["z"] > 0.3
            n_all.setdefault("ticks", 0); n_all["ticks"] += int(u.sum())
            n_all.setdefault("raw_over_28000_valid", 0); n_all["raw_over_28000_valid"] += int((u & (rw > 28000) & (rw < 65000)).sum())
            n_all.setdefault("raw_invalid", 0); n_all["raw_invalid"] += int((u & (rw >= 65000)).sum())
            n_all.setdefault("filtered_over_28000", 0); n_all["filtered_over_28000"] += int((u & (fl > 28000)).sum())
            n_all.setdefault("changed_by_filter", 0); n_all["changed_by_filter"] += int((u & (fl != np.where(rw >= 65000, fl, rw))).sum())
            n_all.setdefault("max_raw_valid", 0.0); n_all["max_raw_valid"] = max(n_all["max_raw_valid"], float(rw[u & (rw < 65000)].max()))
            n_all.setdefault("max_filtered", 0.0); n_all["max_filtered"] = max(n_all["max_filtered"], float(fl[u].max()))
    print(n_all)
    k0 = int(np.argmax(np.where(up & (raw < 65000), raw, 0)))
    fig, ax = plt.subplots(2, 2, figsize=(13, 7), gridspec_kw=dict(width_ratios=[1.6, 1]))
    rp = np.where(raw >= 65000, np.nan, raw)
    for a, y, ttl, c in ((ax[0, 0], rp, "raw DShot reading (0xFFFF = invalid not drawn)", "#0072B2"), (ax[1, 0], filt, "after the firmware filter (rpm_filter.h, the code that runs on the drone)", "#009E73")):
        a.plot(t, deck, color="#888888", lw=0.8, label="optical deck RPM (reference)"); a.plot(t, y, color=c, lw=0.9, label=ttl)
        a.set_ylim(0, 62000); a.set_ylabel(f"motor {i} RPM"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    ax[1, 0].set_xlabel("time (s)  —  full flight, same y-axis"); ax[0, 0].set_title(f"cf5 {stamp[:10]}, motor {i}: whole flight, raw vs filtered")
    w = slice(max(0, k0 - 30), k0 + 31); tz = (t[w] - t[k0]) * 1e3
    for a, y, c, ttl in ((ax[0, 1], raw, "#0072B2", "raw"), (ax[1, 1], filt, "#009E73", "filtered")):
        a.plot(tz, deck[w], color="#888888", lw=1.4, label="deck"); a.plot(tz, y[w], color=c, lw=1.4, marker=".", label=ttl)
        a.set_ylim(0, 62000); a.set_xlabel("ms around the largest spike"); a.legend(fontsize=8); a.grid(alpha=0.3)
    ax[0, 1].set_title(f"zoom: raw peak {mx:,.0f} RPM"); ax[1, 1].set_title(f"zoom: filtered peak {filt[w].max():,.0f} RPM")
    fig.tight_layout(); fig.savefig(OUT / "rpm_spikes_raw_vs_filtered.png", dpi=150); plt.close(fig)
    # ---- on-drone effect plot: grouped bars, spike type x firmware step
    types = [("huge (>28000)", "huge spike (> 28,000 RPM)"), ("invalid 0xFFFF", "invalid reading (0xFFFF)")]
    pers = [("A: before the spike guard", "#CC0000", "before the filter"), ("B: spike guard", "#E6A100", "spike filter (from 09-29)"), ("C: spike guard + sentinel hold", "#009E73", "spike filter + invalid-reading fix (from 10-09)")]
    fig, ax = plt.subplots(figsize=(10, 4.6)); wdt = 0.26
    for j, (P, c, lab) in enumerate(pers):
        vals = [res.get(f"{P} | {t}", {}).get("median_da") or 0 for t, _ in types]; ns = [res.get(f"{P} | {t}", {}).get("n", 0) for t, _ in types]
        xs = np.arange(len(types)) + (j - 1) * wdt
        ax.bar(xs, vals, wdt, color=c, label=lab)
        for xi, v, n in zip(xs, vals, ns): ax.text(xi, v + 0.02, f"{v:.2f}\nn={n}", ha="center", fontsize=8)
    ax.axhline(np.median([res[k]["base_median"] for k in res]), color="k", ls="--", lw=1, label="ordinary tick")
    ax.set_xticks(range(len(types))); ax.set_xticklabels([l for _, l in types]); ax.set_ylabel("change of the residual a_res_z at the spike tick (m/s², median)")
    ax.set_ylim(0, 1.9); ax.legend(fontsize=8); ax.grid(alpha=0.3, axis="y"); ax.set_title("Does a bad RPM reading reach the control residual? (cf5 flights, uSD, RPM from DShot)")
    fig.tight_layout(); fig.savefig(OUT / "rpm_spike_effect_on_ares.png", dpi=150); plt.close(fig)
    (OUT / "numbers.json").write_text(json.dumps(dict(ares_response=res, filter_over_all_flights=n_all, example=dict(flight=os.path.basename(f), motor=i, raw_peak=float(mx), filtered_peak=float(filt[w].max()))), indent=1))


if __name__ == "__main__":
    main()
