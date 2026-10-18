"""Our vehicle (Crazyflie 2.1 Brushless) as used in the controller tree and the NS2 pipeline, with the source of every number.
The bundle's simulated vehicle is a 30 g Crazyflie (mass 0.03, arm 0.046, inertia 1.43e-5/1.43e-5/2.89e-5)."""
OUR = dict(
    mass=0.0427,                    # kg   Omar C controller / NS2 labels (ctrlOmarIndi.mass)
    arm_length=0.050,               # m    ARM_LENGTH in platform_defaults_cf21bl.h
    inertia=(2.395e-5, 2.395e-5, 3.235e-5),   # kg m^2  firmware oot_inertia() (Omar's own controller uses 16.57e-6, 16.66e-6, 29.26e-6)
    k_rpm2force=3.91194027330774e-08,         # N/(rad/s)^2  MOTORRPM2FORCE (platform_defaults_cf21bl.h)
    prop_radius_assumed=0.0225,     # m    ASSUMPTION, to be verified: the bundle hard-codes 45 mm propellers, same as the 2.1
)
BUNDLE_SIM = dict(mass=0.03, arm_length=0.046, inertia=(1.43e-5, 1.43e-5, 2.89e-5), k_thrust=2.3e-08)
