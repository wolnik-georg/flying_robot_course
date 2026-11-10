"""Selection error and test error of a gain set, computed exactly like the tuning's final ranking.

selection error: tuning cells x the 9 selection variants x noise seeds 0, 1, 2 (episodes in variant-major, cell-minor
order)
test error:      study cells x variants {nominal, ffs0.4, md10, md40, md60}    x noise seeds 0 - 4
"""

from . import metrics, variants
from .episode import EpisodeConfig
from .parallel import run_many
from .scenarios import study_cells, tuning_cells

SELECTION_SEEDS = (0, 1, 2)
TEST_SEEDS = (0, 1, 2, 3, 4)


def _run_stage(stage, controller, scenario, gains, cells, variant_names, seeds, procs):
    """Run all episodes of one stage. Returns ({(variant, seed): [episode errors]}, [one record per episode])."""
    configs, keys = [], []
    for seed in seeds:
        for name in variant_names:
            for index, cell in enumerate(cells):
                configs.append(EpisodeConfig(controller, scenario, cell, gains, name, seed, stop_on_crash=True))
                keys.append((name, seed, index, cell))
    results = run_many(configs, procs)
    by_variant_seed, records = {}, []
    for (name, seed, index, cell), res in zip(keys, results):
        error = metrics.episode_error(res)
        by_variant_seed.setdefault((name, seed), []).append(error)
        valid = res.stable  # the RMS of an episode stopped at a crash is not valid
        records.append(
            {
                'stage': stage,
                'cell_index': index,
                'cell': dict(cell),
                'variant': name,
                'seed': seed,
                'x_rms_cm': res.x_rms_cm if valid else None,
                'y_rms_cm': res.y_rms_cm if valid else None,
                'z_rms_cm': res.z_rms_cm if valid else None,
                'rms3d_cm': metrics.rms3d(res) if valid else None,
                'max_tilt_deg': res.tilt_max_deg,
                'crashed': not res.stable,
                'episode_error_cm': error,
            }
        )
    return by_variant_seed, records


def _selection(by):
    """Selection error and crashed count from the per-(variant, seed) episode errors."""
    per_seed = {s: [x for (name, seed), v in by.items() if seed == s for x in v] for s in SELECTION_SEEDS}
    return metrics.selection_error(per_seed)


def _test(by):
    """Test error per variant (mean over cells and seeds) and per seed."""
    out, per = {}, {}
    for name in variants.TEST_VARIANTS:
        seed_means, mean = metrics.test_error({s: by[(name, s)] for s in TEST_SEEDS})
        out[name], per[name] = mean, seed_means
    return out, per


def evaluate(controller, scenario, gains=None, procs=4, include_test=True):
    """Selection error (and test error) with all episode records.

    Returns a dict: selection_error_cm, crashed_selection_episodes, test_error_cm {variant: cm},
    test_error_by_seed_cm {variant: {seed: cm}}, episodes [records]."""
    by, records = _run_stage(
        'selection',
        controller,
        scenario,
        gains,
        tuning_cells(scenario),
        variants.SELECTION_VARIANTS,
        SELECTION_SEEDS,
        procs,
    )
    selection, crashes = _selection(by)
    out = {
        'selection_error_cm': selection,
        'crashed_selection_episodes': crashes,
        'test_error_cm': None,
        'test_error_by_seed_cm': None,
    }
    if include_test:
        by_test, test_records = _run_stage(
            'test', controller, scenario, gains, study_cells(scenario), variants.TEST_VARIANTS, TEST_SEEDS, procs
        )
        out['test_error_cm'], out['test_error_by_seed_cm'] = _test(by_test)
        records += test_records
    out['episodes'] = records
    return out


def selection_error(controller, scenario, gains=None, procs=4):
    """(selection error in cm, number of crashed episodes of the 3 x 9 x cells episodes)."""
    result = evaluate(controller, scenario, gains, procs, include_test=False)
    return result['selection_error_cm'], result['crashed_selection_episodes']


def test_error(controller, scenario, gains=None, procs=4):
    """{variant: test error in cm (mean over study cells and the 5 noise seeds)} and {variant: {seed: error}}."""
    by, _ = _run_stage(
        'test', controller, scenario, gains, study_cells(scenario), variants.TEST_VARIANTS, TEST_SEEDS, procs
    )
    return _test(by)
