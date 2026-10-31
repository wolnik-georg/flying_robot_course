import pandas as pd, numpy as np, sys, warnings
warnings.filterwarnings('ignore')
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import uniform_filter1d
M=0.0427; g=9.81
def ana(f,name):
    df=pd.read_csv(f,comment='#'); P=[c.split('.')[0]+'.' for c in df.columns if c.startswith('cf5')][0]
    b=df[[c for c in df.columns if c.startswith(P)]].rename(columns=lambda c:c[len(P):]).copy(); b['t']=df.t.values
    tp=[c for c in df.columns if c.startswith('cf_second')][0].split('.')[0]
    for a in 'xyz': b[a+'t']=df[tp+'.'+a].values
    b=b[b.z>0.2].reset_index(drop=True)
    t=b.t.values; dt=np.median(np.diff(t))
    q=np.array(b[['est_qx','est_qy','est_qz','est_qw']].values,dtype=float)
    ez=R.from_quat(q).as_matrix()[:,:,2]
    T=b.fbl_thrust.values; w=int(0.2/dt)
    ax=np.gradient(uniform_filter1d(b.vx.values,w),dt); ay=np.gradient(uniform_filter1d(b.vy.values,w),dt); az=np.gradient(uniform_filter1d(b.vz.values,w),dt)
    Fx=M*ax-T*ez[:,0]; Fy=M*ay-T*ez[:,1]; Fz=M*az-T*ez[:,2]+M*g
    out=pd.DataFrame(dict(t=t,dx=b.x-b.xt,dy=b.y-b.yt,dz=b.z-b.zt,Fx=Fx,Fy=Fy,Fz=Fz,Tn=T))
    print(name, 'dt',dt); print(out.iloc[::400].round(3).head(16).to_string())
for f,n in [('bank_fbl_b1_ours_2026-10-26/A4_2026-10-26_17-25-15/A4_2026-10-26_17-25-15_merged_usd.csv','A4 off0.125'),('bank_fbl_b1_ours_2026-10-26/A1_2026-10-26_17-28-19/A1_2026-10-26_17-28-19_merged_usd.csv','A1')]:
    ana('experiments/logs/'+f,n)
