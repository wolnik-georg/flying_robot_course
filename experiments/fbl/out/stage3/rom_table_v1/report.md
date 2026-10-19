# ROM table v1: 60 x 60 x 3 floats = 42.2 KiB flash, C_D 0.43
Test against the authors' DownwashResidual (ROM only), 3000 random states in z0 0.1-1.2 m, r0 0-0.5 m, random velocities up to 0.5 m/s:
- acceleration z: max error 0.0050, p99 0.0028 m/s^2 (bundle x/y output max 0.0e+00, i.e. force purely along z)
- jerk z: max error 0.136, p99 0.032, median 0.0004 m/s^3
- jerk against finite difference of the table acceleration along a pass: max abs deviation 0.236 (peak jerk 5.13) m/s^3
- gate: acceleration at z0 1.2 / 1.4 / 1.5 m = {'z0=1.2': -0.9232051968574524, 'z0=1.4': -9.04148294011975e-08, 'z0=1.5': 0.0}; step at r0 = 0.6 m: 0.00000 m/s^2 (value 0.00000)
