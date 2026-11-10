"""The five controllers S0, S1, S2, S3 and FBL: which firmware switches they set and how a gain dictionary maps to firmware fields.

All five run the real firmware code (`controllerOmarIndi`, compiled into the bindings):
  S0   geometric controller (position PID + attitude control), no downwash compensation
  S1   incremental nonlinear dynamic inversion (INDI), no downwash compensation
  S2   S0 + the neural downwash compensation (NS2) switched on
  S3   S1 + NS2 switched on
  FBL  feedback linearisation with a learned downwash residual (reduced-order model + 4-neuron network), stock rate loop inside
S0 - S3 share the same 10 gains; FBL has 6 gains. Everything that is not a gain is fixed here (one place per strategy).
"""
from dataclasses import dataclass

from . import config

TREE_GAIN_NAMES = ('Pxy', 'Pz', 'Dxy', 'Dz', 'Iz', 'KRxy', 'KRz', 'Kwxy', 'Kwz', 'KIxy')
FBL_GAIN_NAMES = ('kp_xy', 'kv_xy', 'ki_xy', 'kp_z', 'kv_z', 'ki_z')

# Untuned gains: S0 - S3 = firmware defaults of the Omar controller (position P / D / I, attitude KR, rate Komega, rate integral KI);
# FBL = preset 1 (our gain set, position loop 0.25x of the authors' deployed values).
TREE_DEFAULT_GAINS = {'Pxy': 7.0, 'Pz': 7.0, 'Dxy': 4.0, 'Dz': 4.0, 'Iz': 1.5, 'KRxy': 0.007, 'KRz': 0.008, 'Kwxy': 0.00115, 'Kwz': 0.002, 'KIxy': 0.03}
FBL_DEFAULT_GAINS = {'kp_xy': 14.0, 'kv_xy': 9.5, 'ki_xy': 12.0, 'kp_z': 11.25, 'kv_z': 5.4, 'ki_z': 12.0}
# FBL gains that are never tuned (preset 1): attitude / rate gains of the stock inner loop path
FBL_FIXED = {'kr_xy': 59.5, 'kr_z': 35.0}

# Position integral gain Iz of the geometric / INDI tree is set to 1.5 before the gains are applied (it is overwritten by the gain Iz).
TREE_FIRMWARE_FIELDS = {          # gain name -> firmware struct fields (x and y are tied)
    'Pxy': ('Kpos_P.x', 'Kpos_P.y'), 'Pz': ('Kpos_P.z',), 'Dxy': ('Kpos_D.x', 'Kpos_D.y'), 'Dz': ('Kpos_D.z',), 'Iz': ('Kpos_I.z',),
    'KRxy': ('KR.x', 'KR.y'), 'KRz': ('KR.z',), 'Kwxy': ('Komega.x', 'Komega.y'), 'Kwz': ('Komega.z',), 'KIxy': ('KI.x', 'KI.y'),
}


@dataclass(frozen=True)
class Controller:
    name: str
    tree: bool            # S0 - S3 run the Omar tree, FBL the feedback-linearisation branch
    indi: int             # firmware attitude mode: 0 geometric, 3 INDI (tree only)
    ns2: int              # neural downwash compensation on (tree only)
    gain_names: tuple
    default_gains: dict
    description: str


CONTROLLERS = {
    'S0': Controller('S0', True, 0, 0, TREE_GAIN_NAMES, TREE_DEFAULT_GAINS, 'geometric controller'),
    'S1': Controller('S1', True, 3, 0, TREE_GAIN_NAMES, TREE_DEFAULT_GAINS, 'INDI controller'),
    'S2': Controller('S2', True, 0, 1, TREE_GAIN_NAMES, TREE_DEFAULT_GAINS, 'geometric controller + neural downwash compensation (NS2)'),
    'S3': Controller('S3', True, 3, 1, TREE_GAIN_NAMES, TREE_DEFAULT_GAINS, 'INDI controller + neural downwash compensation (NS2)'),
    'FBL': Controller('FBL', False, 0, 0, FBL_GAIN_NAMES, FBL_DEFAULT_GAINS, 'feedback linearisation with learned downwash residual'),
}


def get(name):
    if name not in CONTROLLERS:
        raise KeyError('unknown controller %r, choose one of %s' % (name, ', '.join(CONTROLLERS)))
    return CONTROLLERS[name]


def check_gains(controller, gains):
    """Complete gain dictionary (missing names take the default) after checking the names."""
    unknown = set(gains) - set(controller.gain_names)
    if unknown:
        raise KeyError('%s has no gain(s) %s; its gains are %s' % (controller.name, sorted(unknown), list(controller.gain_names)))
    full = dict(controller.default_gains)
    full.update(gains)
    return full


def configure(fw, omar, controller, gains):
    """Set the firmware controller state for one episode (call after controllerOmarIndiInit; same order as the plant of record)."""
    omar.mass = config.MASS_KG
    omar.indi = 0
    if controller.tree:
        omar.indi = int(controller.indi)
        omar.Kpos_I.z = 1.5
    preset = dict(FBL_DEFAULT_GAINS, **FBL_FIXED)
    for name, value in preset.items():
        setattr(omar.fbl, name, value)
    if not controller.tree:
        for name, value in gains.items():
            setattr(omar.fbl, name, value)
    fw.oot_set_peer_count(1)                       # the top vehicle is a peer; without it neither FBL nor NS2 would see it
    omar.fbl.J.x, omar.fbl.J.y, omar.fbl.J.z = 16.571710e-6, 16.655602e-6, 29.261652e-6      # controller inertia = plant inertia
    omar.fbl.en = 0 if controller.tree else 1
    omar.fbl.inner = 1                             # stock rate loop inside FBL, flown from takeoff to landing
    omar.fbl.abg_on_new = 0
    omar.fbl.nn_en = 1                             # 4-neuron force network of the FBL residual on
    omar.fbl.c_d = 0.55                            # drag coefficient of the FBL residual model
    fw.cvar.g_rnn_div = 10                         # NS2 evaluated every 10th controller tick
    fw.cvar.g_rnn_en = int(controller.ns2) if controller.tree else 0
    if controller.tree:
        for name, value in gains.items():
            for field in TREE_FIRMWARE_FIELDS[name]:
                struct, axis = field.split('.')
                setattr(getattr(omar, struct), axis, value)
