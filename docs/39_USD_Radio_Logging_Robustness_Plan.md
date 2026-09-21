# 39 — uSD/radio logging robustness: root cause + engineering plan (for Cursor agent)

**Status:** planning only, written 2026-09-21 evening. No code changed by this doc. Hand the
section "Prompt for Cursor agent" below to Cursor verbatim; it is self-contained.

## Why 2026-09-21 pairing was hard (root cause, verified in source)

The whole day's post-hoc pairing problem — same-index guesses failing, RMS grid search,
ambiguous dz matches — traces to **one missing capability**: `usddeck.c`
(`src/deck/drivers/src/usddeck.c` in `crazyflie-firmware`) picks the next free `thesisNN`
filename internally (`usdWriteTask`, scans for the first non-existent name) and **never
reports that name, or anything else identifying the session, back to the host** — confirmed by
grep: there is no getter, no log variable, nothing. The host broadcasts `usd.logging=1` and
has zero visibility into what file was just created on either card.

Everything downstream is a workaround for that gap:
- `copy_usd_log.py` can only find "the largest non-empty `thesisNN`" — it cannot know which
  flight a file belongs to.
- The lab-session doc's "write down thesisNN by hand" mitigation depends on a human catching
  every session in real time and never mis-transcribing — it drops the instant a card is
  swapped between drones without updating the note, or a session aborts without a matching
  note on both cards.
- The desk-side fix (`merge_usd_logs.py --meta --roles`, grid-searched by
  `experiments/analysis/merge_c1_2026_09_21.py`) is inherently a **content-based guess**: fit
  each candidate file's trajectory against the commanded one and accept the first one under
  15 cm RMS. This works, but:
  1. It is expensive (one full merge subprocess per candidate pair).
  2. It is **fundamentally ambiguous for any trajectory whose bottom (cf5) role doesn't depend
     on the scenario parameter that actually varies** — A1's static hold varies only in `dz`,
     which is a *top-drone* offset, so the bottom-role fit is identical across all `dz` values
     and cannot disambiguate by itself (root cause of the `thesis15` false-positive across two
     different `dz` stamps on 2026-09-21).
  3. A **genuine tracking failure** (cf5 z-error 15–21 cm, a real control problem) is
     indistinguishable, from the search's point of view, from **"wrong file"** — both just look
     like "RMS too high." 2026-09-21 spent real effort re-confirming by exhaustive search that
     3 A1 stamps had no better-fitting file, i.e. proving a negative that a positive
     identifier would have made moot.

None of this is a tooling bug to patch harder — it's a missing **primary key**. The fix is to
put one there.

## The fix: a session tag, logged by the firmware itself

Add a new param + log variable to the existing `usd` group in `usddeck.c` (confirmed both a
`PARAM_GROUP_START(usd)` and a `LOG_GROUP_START(usd)` already exist in that file — this is an
addition to groups that are already there, not a new subsystem):

```c
// near the other static usd-deck state
static uint32_t runTag;

// in PARAM_GROUP_START(usd) ... PARAM_GROUP_STOP(usd)
PARAM_ADD_CORE(PARAM_UINT32, runTag, &runTag)

// in LOG_GROUP_START(usd) ... LOG_GROUP_STOP(usd)
LOG_ADD_CORE(LOG_UINT32, runTag, &runTag)
```

Then add `usd.runTag` as a logged channel in `usd_thesis_config.txt`. **This fits exactly** —
`README_usd_thesis_logging.md` already states the config uses 47 of the 48-variable cap
(`MAX_USD_LOG_VARIABLES_PER_EVENT` in local `usddeck.c`), so this is the very last free slot;
if any other variable is added in the future something else must be dropped first.

Host side (`crazyswarm2/crazyflie_examples/crazyflie_examples/{formation_flight,run_formation,
flight,simple_flight}.py`), right before the existing `allcfs.setParam("usd.logging", 1)`
broadcast (all four scripts have this exact call, each already commented as "ONE BROADCAST, not
a per-drone loop" — see line ~677 in `formation_flight.py` and ~658 in `run_formation.py`):

```python
run_tag = int(time.time())          # unix seconds -- unique per flight start, human-decodable
allcfs.setParam("usd.runTag", run_tag)
th.sleep(0.05)                      # let the broadcast land before logging opens the file
allcfs.setParam("usd.logging", 1)
```

...and add `"usd_run_tag": run_tag` to the `meta.json` sidecar dict each script already writes
(e.g. `run_formation.py`'s `json.dump({...})` around line 683).

Why unix seconds and not `usec.reset`-style zero-basing or a persisted counter file: it needs
no shared state between processes or days, it can never collide across two *different* flight
starts (they are always more than a second apart in this workflow), and it is trivially
human-checkable against the wall-clock stamp already in every filename — no new bookkeeping
file to keep in sync or lose.

**This makes matching an O(1) lookup, not a search.** Post-flight: decode a candidate uSD log,
read the (constant) `usd_run_tag` column, compare to the `meta.json`'s `usd_run_tag`. Equal ->
that is the file, full stop — no RMS search needed to *find* the pair. RMS from
`merge_usd_logs.py --meta --roles` is then a **pure quality gate** on a known-correct pair
(catch a real tracking failure), not a disambiguation mechanism — which also finally makes the
"wrong file vs. real tracking problem" distinction unambiguous: if the tag matches and RMS is
still bad, it is a real control/estimation problem, worth its own investigation, not a
pairing artifact.

Card-position tracking (which physical THESIS1/THESIS2 card sat in which drone) becomes a
**nice-to-have cross-check, no longer load-bearing** — the tag identifies the flight
regardless of which card the file came off.

## Immediate, zero-firmware-change mitigations (adopt now, independent of the above)

These reduce the blast radius of the *current* firmware while the tag change goes through
its own build/flash/verify cycle:

1. **Archive both cards after every scenario block (every 4-6 flights), not once at end of
   day.** 2026-09-21's ambiguity came from a single end-of-day dump spanning 18 sessions per
   card; archiving in small batches shrinks the index-search window to the size of one block,
   which is exactly what made the existing ±5 search occasionally return a false positive
   (A1's `thesis15` collision) — a smaller pool has fewer chances to collide.
2. **Log the terminal echo, not just intent.** Both scripts already print
   `[formation] uSD logging started (broadcast) at t=...s` right after the broadcast — copy
   that whole line (or a screenshot of the terminal) into the lab session doc per flight, not
   just the scenario name and time. Cheap, already happening, just not being captured.
3. **Never fly two reps of the same scenario with the same varying parameter back-to-back**
   when avoidable (e.g. reorder so consecutive A1 reps differ in `dz`) — this is exactly what
   defeats the desk-side ambiguity guard once the tag fix lands as a transition aid for any
   pre-tag archive still being reprocessed.

## Tooling changes (after the firmware/host change above ships)

All in this repo (`flying_robot_course`) unless noted:

1. **`decode_usd_log.py`**: add `"usd.runTag": "run_tag"` to `RENAME` so the new channel
   surfaces with the project's naming convention.
2. **New tool `flying_drone_stack/tools/index_usd_archive.py`**: scan an archive directory
   (e.g. `usd_raw/<date>_THESIS{1,2}/`), decode each `.bin`'s `run_tag` column (just needs the
   first valid sample, not the whole file), and emit a JSON index
   `{run_tag: {path, card, n_rows, duration_s, first_ts}}`. Flag and report (not silently
   skip) any file with **no `run_tag` column at all** (pre-upgrade firmware) or **all-zero /
   inconsistent `run_tag` values within one file** (would indicate the param write raced the
   file-open — should not happen given the 50 ms settle above, but check for it rather than
   assume). This index is the thing that turns matching into a lookup.
3. **`merge_usd_logs.py`**: add a `--run-tag <tag>` mode that, given the two archive
   directories and a tag, finds the matching bottom/top files via the index from (2) directly
   — no candidate search. Keep the existing `--meta --roles` path unchanged as the fallback for
   pre-tag archives and as the quality gate described above (run it automatically after a
   tag-based pair is found, report RMS, but do not use RMS to *accept or reject the pairing
   itself* when the tag already identified it — only to flag a suspiciously bad fit for
   separate investigation).
4. **`experiments/analysis/merge_c1_2026_09_21.py`-style batch script, going forward**: for
   any *future* dated session, the batch script should read each `<SCENARIO>_<date>_<stamp>
   .meta.json`'s `usd_run_tag`, look it up in the two per-card indexes from (2), and merge
   directly. The ±N grid search this file currently implements should be kept only as the
   documented fallback path for logs recorded before the firmware upgrade — gate it behind
   "meta has no `usd_run_tag` field" so new data never pays the old cost or ambiguity.
5. **`copy_usd_log.py`**: right after a successful copy, decode just enough of the new file to
   print its `run_tag` and row count/duration to the terminal, and warn loudly (not just
   silently archive) if the row count implies a recording shorter than a plausible flight
   (e.g. < 5 s) — this turns 2026-09-21's `A3 13-12-16` (~49 top radio rows) and `A7 12-37-31`
   style failures into an immediate in-lab signal instead of a desk-side discovery days later.
6. **`docs/28_USD_Radio_Matching_and_Session_Analysis.md`**: update the "Matching rules"
   table to lead with "match by `usd_run_tag`, RMS is a quality gate not a search" once the
   firmware change is flashed and verified; keep the current RMS-search recipe as the
   documented fallback for older archives (2026-09-21 and earlier will never have tags).

## Rollout / validation (do in this order, do not skip steps)

1. Add the `usddeck.c` param/log lines above on a feature branch of the firmware fork used by
   this project (see `firmware_app/host/LOCAL_MODIFICATIONS.md` for where local firmware
   changes are tracked — this is a pure instrumentation addition, no control-loop code touched,
   so it does not need to go through the attitude/control-law caution in that file, but it
   still needs the normal build+flash+bench-verify cycle any firmware change gets).
2. Add `usd.runTag` to `usd_thesis_config.txt` (48th and last free slot — confirm total stays
   ≤ `MAX_USD_LOG_VARIABLES_PER_EVENT` in the local `usddeck.c`, currently 48).
3. Rebuild, flash **both** study drones (cf5 and cf_second both need the new firmware — a tag
   emitted by only one side is useless).
4. **Bench test before any flight**: on the ground, `setParam('usd.runTag', 12345)`, then
   `setParam('usd.logging', 1)`, wait 2 s, `setParam('usd.logging', 0)`, copy the file, decode
   it, and confirm every row's `run_tag` column reads exactly `12345`. Do this on both drones
   independently before trusting it in the air.
5. Confirm the existing 47 variables still decode correctly (no channel got bumped/reordered
   by adding the 48th) — rerun `decode_usd_log.py` against a bench log and diff the variable
   list against a pre-upgrade log's variable list, allowing exactly one new entry.
6. Add the host-side broadcast + `meta.json` field to all four flight scripts (list in the fix
   section above) — grep for `setParam("usd.logging", 1)` / `setParam('usd.logging', 1)` to
   find every call site so none is missed.
7. First real validation: a single 2-drone A8 hover, tag-based match end to end (index both
   cards, look up by tag, merge, confirm RMS still looks like the ~4/13 cm baseline this repo
   already has for A8). Only after that passes, resume normal C.1 data collection using the
   new tag-based workflow.

## Final step: reconcile against `docs/next_flight_card.html`

Independent of whether the firmware change has landed yet, use today's confirmed results
(`experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json`, 8 entries, plus the
"CONFIRMED unmergeable — refly required" table in `docs/lab_sessions/2026-09-21.md` §4) to
update `docs/next_flight_card.html` and `docs/25_C1_Data_Collection_Plan.md`'s block
checklist so they reflect the **true** current coverage:

- Mark A8 shakedown, A3 @ 0.30 (×2), A3 @ 0.50 (`13-04-34` only — `13-05-58` is void, still
  needs a rep), A1 @ 0.75 and @ 0.20 as **done** (each backed by a manifest entry).
- Mark A1 @ 0.30, @ 0.50 (both morning attempts and the afternoon repeat), A3 @ 0.50's second
  rep, A3 B-3 @ 0.40/0.40, A7 (never successfully flown), A2, A4, C5 as **still needed**,
  matching the refly list already in `docs/lab_sessions/2026-09-21.md` §4.
- Do not mark anything "done" that only has a `.meta.json` + radio CSV but no
  `*_merged_usd.csv` in the manifest — radio-only is QA, not training data (see
  `docs/28`'s training-data rule).

---

## Prompt for Cursor agent

Copy everything below this line into a new Cursor agent session.

---

**Context.** `flying_robot_course` is a Crazyflie multi-drone thesis project. uSD-card logs
(500 Hz, per-drone, onboard) are the training data for a residual-force model; radio CSVs are
QA-only. The 2026-09-21 lab day needed hours of desk work to pair each flight's two per-drone
uSD files correctly, because the firmware currently gives the host **zero way to identify**
which uSD file belongs to which flight — pairing has to be reconstructed after the fact by
brute-force trajectory-fit search (`merge_usd_logs.py --meta --roles`, RMS < 15 cm gate). Full
root-cause writeup and the engineering fix: `docs/39_USD_Radio_Logging_Robustness_Plan.md` in
this repo — **read it in full before doing anything**, it has exact file/line pointers already
verified against the current source. Do not re-derive the diagnosis; execute the fix.

**Task 1 — firmware: add a session tag.**
In `~/Desktop/crazyflie-firmware/src/deck/drivers/src/usddeck.c`, add a `static uint32_t
runTag;` and expose it as both `PARAM_ADD_CORE(PARAM_UINT32, runTag, &runTag)` in the existing
`PARAM_GROUP_START(usd)` block and `LOG_ADD_CORE(LOG_UINT32, runTag, &runTag)` in the existing
`LOG_GROUP_START(usd)` block (both groups already exist in this file — see doc §"The fix" for
exact line context). Add `usd.runTag` as a logged channel in
`flying_robot_course/flying_drone_stack/tools/usd_thesis_config.txt` — confirm the total
variable count stays at or under `MAX_USD_LOG_VARIABLES_PER_EVENT` (48) in that same file;
current count is 47, so this is the last free slot and nothing else may be added alongside it
without first removing a channel. Follow whatever local-modification tracking convention this
firmware fork already uses (check `firmware_app/host/LOCAL_MODIFICATIONS.md` in
`flying_robot_course`) — this is a pure instrumentation addition with no control-loop impact,
but still needs a normal build+flash cycle. Build for both study drones (cf5 and cf_second —
check `flying_drone_stack/CLAUDE.md`'s Makefile section for the per-drone build command), flash
both, and **bench-verify on the ground before any flight**: set `usd.runTag` to a known test
value, toggle `usd.logging` 1 then 0 after ~2 s, copy the resulting file with
`copy_usd_log.py`, decode it with `decode_usd_log.py`, and confirm every row's `usd.runTag`
(after adding it to `RENAME` in that script, see Task 2) equals the test value, on **both**
drones independently. Also confirm the pre-existing 47 variables still decode with unchanged
names/values (diff the variable list against a log from before this change).

**Task 2 — host scripts: generate, broadcast, and record the tag.**
In `~/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/{formation_flight.py,
run_formation.py, flight.py, simple_flight.py}` — grep each for
`setParam("usd.logging", 1)` / `setParam('usd.logging', 1)` to find every call site (there
should be exactly one broadcast call per script, already commented "ONE BROADCAST, not a
per-drone loop" in the ones that fly formations) — immediately before that call, add:
```python
run_tag = int(time.time())
allcfs.setParam("usd.runTag", run_tag)
th.sleep(0.05)
```
and add `run_tag` (or `"usd_run_tag": run_tag`, matching the key style already used in that
script's meta dict) to the `meta.json` sidecar dict the same function already writes shortly
after. Use `time.time()` (unix seconds), not a persisted counter file — it needs no shared
state and cannot collide across flights that are more than a second apart, which every flight
in this workflow already is.

**Task 3 — tooling: match by tag, keep RMS as a quality gate, not a search.**
- `flying_drone_stack/tools/decode_usd_log.py`: add `"usd.runTag": "run_tag"` to `RENAME`.
- New `flying_drone_stack/tools/index_usd_archive.py`: given an archive directory (e.g.
  `experiments/logs/usd_raw/<date>_THESIS1/`), decode each `.bin`'s `run_tag` column (reading
  the first valid sample is enough) and write a JSON index of `{run_tag: {path, n_rows,
  duration_s}}`. Loudly flag (do not silently skip) any file with no `run_tag` column at all
  (pre-upgrade firmware — expected for every archive dated 2026-09-21 or earlier) or with
  inconsistent non-constant `run_tag` values within one file.
- `flying_drone_stack/tools/merge_usd_logs.py`: add a `--run-tag <tag> --archive-t1 <dir>
  --archive-t2 <dir>` mode that looks the tag up in both archives' indexes and merges the
  matched pair directly (no candidate search). Keep the existing `--meta --roles` search path
  exactly as-is for archives with no tag. When a tag-based pair is found, still run the
  existing RMS-vs-commanded-trajectory check and print it, but do **not** use RMS to accept or
  reject the pairing in this mode — only to flag a real tracking-quality problem for separate
  investigation, since the tag has already proven which file is which.
- `flying_drone_stack/tools/copy_usd_log.py`: after a successful copy, if the copied file has
  the new `usd.runTag` channel, decode enough of it to print `run_tag` and row
  count/implied duration to the terminal immediately, and warn (not just archive silently) if
  the duration looks too short to be a real flight (e.g. under 5 s). This should catch
  short/aborted recordings the moment they're pulled off the card, not days later at a desk.
- Update `docs/28_USD_Radio_Matching_and_Session_Analysis.md`'s matching-rules section to lead
  with tag-based matching once verified end-to-end (Task 4), keeping the RMS-search recipe
  documented as the fallback for pre-tag archives (everything ≤ 2026-09-21).

**Task 4 — end-to-end validation before trusting this for real data collection.**
Fly a single 2-drone A8 hover with the new firmware+scripts. Confirm: both `meta.json`s (or
the one shared sidecar, per whichever script is used) carries `usd_run_tag`; both uSD files
decode with a `run_tag` channel that reads that exact value throughout; `index_usd_archive.py`
finds both; `merge_usd_logs.py --run-tag` finds the correct pair with zero search; the
resulting RMS is in the same ballpark as this project's existing A8 baseline (~4 cm / ~13 cm
bottom/top — see `experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json` for
reference numbers from today). Do not roll this out for a full data day until this single
flight validates cleanly.

**Task 5 — independently double-check 2026-09-21's pairs and merges (do not just trust the
manifest — verify it).**
`experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json` lists 8 merges already
produced by `experiments/analysis/merge_c1_2026_09_21.py`'s grid search. Before relying on it,
independently confirm each entry:
- For every manifest entry, re-run `merge_usd_logs.py` on the exact `bottom_idx`/`top_idx`
  pair recorded (via the symlinks already in each
  `experiments/logs/c1_2026-09-21_merged/<SCENARIO>_2026-09-21_<stamp>/` folder) and confirm
  it still reports both-side RMS < 15 cm matching the `rms_cm` field in the manifest, and that
  the `*_merged_usd.csv` in that folder is non-empty and its row count is consistent with the
  flight duration in the matching `.meta.json`.
- Cross-reference each entry's `thesis_pair` (card + index) against
  `experiments/logs/usd_raw/2026-09-21_PAIRING.md`'s "Agent grid-search results" table — they
  should agree exactly; flag any discrepancy rather than silently trusting either source.
- Confirm no physical archive file (card + index) backs more than one manifest entry — this
  was the exact bug found and fixed in `merge_c1_2026_09_21.py` earlier today (two stale
  merges, `A1_13-27-27` and `A3_13-05-58`, had each reused a file already claimed by a
  different accepted stamp; both were deleted, and the folders now correctly contain no
  `*_merged_usd.csv`). Verify those two folders are still empty of a merged CSV and have not
  been regenerated incorrectly.
- Re-run the "no pair found" stamps listed in `docs/lab_sessions/2026-09-21.md` §4's
  "CONFIRMED unmergeable from archive" table (`A1 12-40-41/12-43-34/12-49-36/13-27-27/
  13-28-49/13-30-30`, `A3 13-05-58`) through the same search once more (the script already
  implements the full ±5 index window with the uniqueness/dz-ambiguity guards — just execute
  it and read the output) and confirm they still produce no valid pair, so the refly list is
  not stale.
Report explicitly, in plain language: which scenario+stamp combinations are **confirmed
correct and training-eligible** (should match the 8 in the manifest, or fewer if verification
finds a problem — investigate and report anything that doesn't match, do not silently "fix" a
mismatch by picking whichever result looks more convenient), and which are **still
unmergeable / need a refly**.

**Task 6 — reconcile the flight card against the verified results from Task 5.**
Using the verified (not just assumed) results from Task 5, update `docs/next_flight_card.html`
and `docs/25_C1_Data_Collection_Plan.md`'s block checklist so both reflect the **true** current
state: mark exactly the scenarios backed by a verified manifest entry as done, and everything
still unmergeable (expected: A1 @ 0.30/0.50 morning + afternoon repeat, A3 @ 0.50's second rep,
A3 B-3 @ 0.40/0.40, A7, A2, A4, C5 — confirm against your own Task 5 output, not this list) as
still needed. A `.meta.json` + radio CSV with no corresponding `*_merged_usd.csv` in the
manifest is QA-only, never "done" — do not mark it complete. End with a short summary table:
scenario | status (done / needs refly / never flown) | why.

Do not touch `experiments/logs/usd_raw/` (raw archive, read-only). Do not delete or regenerate
any existing merged output under `experiments/logs/c1_2026-09-21_merged/` unless Task 5's
verification finds an actual discrepancy — if it does, fix only that discrepancy and say
exactly what was wrong and what changed. Tasks 1-4 only add new capability going forward; they
do not retroactively change 2026-09-21's data.
