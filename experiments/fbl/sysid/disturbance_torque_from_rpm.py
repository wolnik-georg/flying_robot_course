import numpy as np, pandas as pd, warnings
warnings.filterwarnings('ignore')
from scipy.ndimage import uniform_filter1d
ARM=0.70710678*0.05; T2T=0.00569278844371417; KF=3.91194027330774e-08
FL=['A4_2026-10-26_17-25-15','A4_2026-10-26_17-26-40','A8_2026-10-26_17-23-37','A1_2026-10-26_17-28-19']
for n in FL:
    p='/home/georg/Desktop/flying_robot_course/experiments/logs/bank_fbl_b1_ours_2026-10-26/%s/%s_merged_usd.csv'%(n,n)
    df=pd.read_csv(p,comment='#'); P=[c.split('.')[0]+'.' for c in df.columns if c.startswith('cf5')][0]
    b=df[[c for c in df.columns if c.startswith(P)]].rename(columns=lambda c:c[len(P):]).copy(); b['t']=df.t.values; b=b[b.z>0.2].reset_index(drop=True)
    F=[KF*(b['rpm_m%d'%i].values*2*np.pi/60)**2 for i in (1,2,3,4)]
    tx=ARM*(-F[0]-F[1]+F[2]+F[3]); ty=ARM*(-F[0]+F[1]+F[2]-F[3]); tz=T2T*(-F[0]+F[1]-F[2]+F[3])
    w=250  # 0.5 s at 500 Hz
    sx,sy,sz=[uniform_filter1d(v,w)*1e6 for v in (tx,ty,tz)]
    k=np.arange(0,len(b),500)
    print(n[:3],n[-8:],'torque from rpm [uNm] (0.5 s mean), every 1 s: x',np.round(sx[k][:15]).astype(int).tolist())
    print('       y',np.round(sy[k][:15]).astype(int).tolist(),' z',np.round(sz[k][:15]).astype(int).tolist())
    print('       overall mean x %.0f y %.0f z %.0f ; std of 0.5s-mean x %.0f y %.0f z %.0f (sim assumes const 80/-300/-30, var amp 300/400/60)'%(sx.mean(),sy.mean(),sz.mean(),sx.std(),sy.std(),sz.std()))
