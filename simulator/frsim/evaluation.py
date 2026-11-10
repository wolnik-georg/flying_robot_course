"""Selection error and test error of a gain set, computed exactly like the tuning's final ranking.

selection error: tuning cells x the 9 selection variants x noise seeds 0, 1, 2   (episodes in variant-major, cell-minor order)
test error:      study cells x variants {nominal, ffs0.4, md10, md40, md60}      x noise seeds 0 - 4
"""
import numpy as np

from . import metrics, variants
from .episode import EpisodeConfig
from .parallel import run_many
from .scenarios import study_cells, tuning_cells

SELECTION_SEEDS = (0, 1, 2)
TEST_SEEDS = (0, 1, 2, 3, 4)


def _errors(controller, scenario, gains, cells, variant_names, seeds, procs):
    configs, keys = [], []
    for seed in seeds:
        for name in variant_names:
            for cell in cells:
                configs.append(EpisodeConfig(controller, scenario, cell, gains, name, seed, stop_on_crash=True))
                keys.append((seed, name))
    results = run_many(configs, procs)
    by = {}
    for (seed, name), res in zip(keys, results):
        by.setdefault((name, seed), []).append(metrics.episode_error(res))
    return by


def selection_error(controller, scenario, gains=None, procs=4):
    """(selection error in cm, number of crashed episodes of the 3 x 9 x cells episodes)."""
    by = _errors(controller, scenario, gains, tuning_cells(scenario), variants.SELECTION_VARIANTS, SELECTION_SEEDS, procs)
    per_seed = {s: [x for (name, seed), v in by.items() if seed == s for x in v] for s in SELECTION_SEEDS}
    # order inside a seed: variant-major, cell-minor (dict order = insertion order = seed, variant, cell)
    return metrics.selection_error(per_seed)


def test_error(controller, scenario, gains=None, procs=4):
    """{variant: test error in cm (mean over study cells and the 5 noise seeds)} and {variant: {seed: error}}."""
    by = _errors(controller, scenario, gains, study_cells(scenario), variants.TEST_VARIANTS, TEST_SEEDS, procs)
    out, per = {}, {}
    for name in variants.TEST_VARIANTS:
        seed_means, mean = metrics.test_error({s: by[(name, s)] for s in TEST_SEEDS})
        out[name], per[name] = mean, seed_means
    return out, per
