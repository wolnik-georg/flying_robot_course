"""Shared helper for the retrain banks: keep a_res_* only for the BOTTOM vehicle (cf5).

v1 was trained on bottom-ego samples only: the top vehicle's a_res was zero in the old logs, so dataset.build dropped it
(`dropped_zero_a_res`). The Omar label reconstruction now produces a label for both drones; the top vehicle (neighbour BELOW, not
in the wash) would add rows the network was never meant to learn from and that make a v1-vs-new comparison unfair. The top vehicle's
a_res_x/y/z columns are therefore set to exactly zero (same mechanism as before). The merged file notes it in a comment line."""
MARK = "# top_a_res_zeroed"

def zero_top_labels(path, top_prefix="cf_second"):
    lines = open(path).read().split("\n")
    if any(l.startswith(MARK) for l in lines[:30]):
        return False
    hi = next(i for i, l in enumerate(lines) if l and not l.startswith("#"))
    hdr = lines[hi].split(",")
    idx = [i for i, h in enumerate(hdr) if h.startswith(top_prefix) and h.rsplit(".", 1)[-1] in ("a_res_x", "a_res_y", "a_res_z")]
    if not idx:
        return False
    out = lines[:hi] + [MARK + " (top vehicle labels set to 0; v1 convention, bottom-ego training only)"] + [lines[hi]]
    for l in lines[hi + 1:]:
        if not l:
            out.append(l); continue
        c = l.split(",")
        for i in idx:
            c[i] = "0"
        out.append(",".join(c))
    open(path, "w").write("\n".join(out))
    return True
