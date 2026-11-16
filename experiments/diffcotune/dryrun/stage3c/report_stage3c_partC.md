
# Stage 3c, Part C: speed-up (measured on the lab PC, 12 Nov 2026)

## C1 memory (PSS from /proc/<pid>/smaps_rollup) and compile helpers

One pool of 4 workers, one task epoch and one fit epoch (1 trace per scenario) at the same point, with and without `TORCHINDUCTOR_COMPILE_THREADS=1` (set before the pool starts; `loop_lib.py` now sets it by default for every pool started from now on; the Part B check 2 job and the Stage 3b runs ran without it).

| | default (22 helpers per worker) | TORCHINDUCTOR_COMPILE_THREADS=1 |
|---|---|---|
| processes (main + tracker + 4 workers + helpers) | 98 | 6 |
| PSS of the whole tree | 7552 MB | 6117 MB (-19 %) |
| PSS per worker including its helpers | 1806 / 1856 / 1910 / 1933 MB | 1457 / 1482 / 1548 / 1582 MB |
| first call (compile), task / fit | 29.2 s / 13.3 s | 32.0 s / 22.7 s |
| steady epoch, task / fit | see C2 (about 10-11 s / 10 s) | the same |
| J_task, J_sysId, all 14 gradient components of both | | **bit-identical** (compared as repr of the floats) |

The 4 workers stay at about 1.5 GB each; the setting removes the helpers (1.4 GB in total), not the model. Two pools at the same time (2 x 6.1 GB + the 3 GB rule) do not fit in the 12.0 GB MemAvailable of the idle machine.

## C2 threads and job runner

Seconds per epoch, 4 scenarios in 4 workers, mean of 3 timed epochs after 2 warm-up epochs:

| torch threads per worker | task epoch | fit epoch | load average (1 min) at the end |
|---|---|---|---|
| 2 (default) | 11.4 s (13.8, 10.1, 10.3) | 10.1 s | 3.5 |
| 4 | 17.7 s (21.6, 11.4, 20.1) | 10.1 s | 3.7 |
| 8 | 11.4 s (13.5, 10.5, 10.4) | 9.9 s | 4.3 |

More threads give no speed-up: the epoch time is set by the serial chain of the 11 500 two-millisecond blocks of the longest scenario (A8), and the tensors are tiny. The default of 2 stays (`DIFFCOTUNE_THREADS`, default 2). The idle cores can only be used by independent jobs; the large ones (a pool, 6.1 GB) cannot run side by side in 12 GB. Small jobs can (the 35 final-evaluation flights per method are single frsim processes of a few hundred MB).

`job_runner.py`: queue of independent jobs, each with its number of CPU processes and a memory estimate, every job under `nice -n 19`, started in queue order only if (a) running + new processes <= 16, (b) 1-minute load <= 22, (c) MemAvailable - memory still to be taken by running jobs - estimate >= 3 GB. Self-test (`logs/job_runner_selftest.log`): the third of three 6-process jobs waited for a free slot (12 + 6 > 16); a job waited while the load was above its limit and started when it fell; a job with an estimate of MemAvailable - 2 GB waited (2.0 < 3.0 GB) and started when the estimate was lowered. It is not used by the loops: fit and tune of a round and the rounds of one loop depend on each other.

## C3 the four scenarios in one GPU batch (RTX 500 Ada Laptop, 4 GB)

`s0_batch.py`: rows = scenarios (and traces), padded references, per-row top-vehicle flag (`_block` takes a tensor flag, arithmetic unchanged, the CPU path is untouched), cost per row only inside its own window. Seconds per epoch (forward + backward of the mean task cost, 4 scenarios, second call; the first call includes compile):

| variant | seconds per epoch |
|---|---|
| CPU pool, 4 workers (reference) | about 10-11 |
| GPU eager, float32, forward only | 62 (forward only, 4 scenarios) |
| GPU compiled, float32, 4 rows | 30.8 |
| GPU compiled, float64, 4 rows | 37.9 |
| GPU compiled, float64, 24 rows (6 traces per scenario) | 38.6 |
| GPU compiled, float32, 5 blocks per call, 4 rows | 87.0 |
| GPU `mode="reduce-overhead"` (CUDA graphs), float64, 4 rows | no result: stopped after 36 min without finishing the first iteration |

Launch-bound: 4 or 24 rows cost the same. A round (50 + 50 epochs, 102 evaluations) would take 38 s x 102 = 64 min in float64 against 17-19 min on the CPU.

Gates (`c3_gates.py`, GPU batched against the CPU model; forward: 4 scenarios x 3 points, gate 0.5 %; gradient gate of Stage 3 on the combined cost):

| dtype | forward (max relative difference of the task 3-D RMS) | J_task gradient, default / x = (0, 0.5 log true beta) | J_sysId gradient (p + v + attitude), default / same x |
|---|---|---|---|
| float64 | pass, 3e-13 % | pass / pass (worst relative 1.1e-3 / 1e-5) | **fail** / pass (default: 4 of 14 components fail, worst relative 5.4 %, worst absolute 0.98 % of the largest component; the failing ones are the z gains and downwash / plume, i.e. the marginal A1 state of the noise-free default, known from Stage 2) |
| float32 | pass, 1e-3 % | **fail** / fail (relative differences of 0.15 to 3e5) | **fail** / fail |

Float32 is not usable (the forward is right, the gradient is not). Float64 passes all but one of the four gradient comparisons and is 3.5 x slower than the CPU pool. The CPU path stays the default. No GPU path is proposed.

## Not done in this stage (options only; estimates are NOT measured)

| option | expected gain | deviation |
|---|---|---|
| leave out the fit of the last round (`--skip-last-fit`, implemented, default off) | 8.5 min of 96 min per (d) loop (the measured last-round time of P1), about 9 % | none for theta_best; the record keeps beta of round 5 as flown (result of the fit of round 4) |
| fit beta on short pieces (2-4 s) of each flight, started from the logged state | the cost of an epoch scales with the simulated time; 23 s -> 3 s would be about 7 x less for A8, the limit of an epoch (not measured) | deviation from the paper (rollout of the whole flight) |
| plant sub-step 2 ms instead of 1 ms | the block does two plant steps; one would remove about half of the plant work in a block (not measured) | needs a new forward check (gate 0.5 %) |

## Measured wall time of the Stage 3b runs

P1 full run (4 comparisons, no skip): 14:12:37 to 19:41:35 = 5 h 29 min. (d) loop 6 rounds about 1 h 40 min (14:12:37 to 15:51) (17.1-19.1 min per round, last round 8.5 min).
