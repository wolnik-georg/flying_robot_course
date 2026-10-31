import numpy as np, pandas as pd, warnings
warnings.filterwarnings('ignore')
from scipy.ndimage import gaussian_filter1d
for n in ['A8_2026-10-26_17-23-37','A4_2026-10-26_17-25-15','A1_2026-10-26_17-28-19']:
    p='/home/georg/Desktop/flying_robot_course/experiments/logs/bank_fbl_b1_ours_2026-10-26/%s/%s_merged_usd.csv'%(n,n)
    df=pd.read_csv(p,comment='#'); P=[c.split('.')[0]+'.' for c in df.columns if c.startswith('cf5')][0]
    b=df[[c for c in df.columns if c.startswith(P)]].rename(columns=lambda c:c[len(P):]).copy(); b['t']=df.t.values; b=b[(b.z>0.2)&(b.t>5)].reset_index(drop=True)
    dt=0.002; tu=np.arange(b.t.iloc[0],b.t.iloc[-1],dt)
    for ax in 'xy':
        wd=gaussian_filter1d(np.interp(tu,b.t,b['fbl_w_des_'+ax]),0.15/dt); wm=gaussian_filter1d(np.radians(np.interp(tu,b.t,b['gyro_'+ax])),0.15/dt)
        best=max(((np.corrcoef(wd[:len(wd)-k],wm[k:])[0,1],k) for k in range(0,int(0.4/dt))),key=lambda z:z[0]); k=best[1]
        g=(wd[:len(wd)-k]@wm[k:])/(wd[:len(wd)-k]@wd[:len(wd)-k]); print('hw',n[:3],ax,'low-pass corr %.2f lag %.0f ms gain %.2f'%(best[0],k*dt*1000,g))
