import sys,os,numpy as np
sys.path.insert(0,'/home/georg/Desktop/flying_robot_course/simulator')
from frsim import controllers
from frsim.episode import EpisodeConfig, run_episode
out=sys.argv[1]
res={}
for c in controllers.CONTROLLERS:
    if c.upper().startswith('S4') or 'fbl' in c.lower(): continue
    for s in ('A1','A8'):
        r=run_episode(EpisodeConfig(c,s,stop_on_crash=True))
        res[f'{c}_{s}']=np.asarray(r.trace)
        print(c,s,r.n_samples,flush=True,file=sys.stderr)
np.savez(out,**res)
