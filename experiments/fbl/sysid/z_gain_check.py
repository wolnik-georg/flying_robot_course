import numpy as np, pandas as pd, warnings
warnings.filterwarnings('ignore')
for f in ['A4_2026-10-26_17-25-15','A4_2026-10-26_17-26-40','A8_2026-10-26_17-23-37','A1_2026-10-26_17-28-19']:
    p='/home/georg/Desktop/flying_robot_course/experiments/logs/bank_fbl_b1_ours_2026-10-26/%s/%s_merged_usd.csv'%(f,f)
    df=pd.read_csv(p,comment='#'); P=[c.split('.')[0]+'.' for c in df.columns if c.startswith('cf5')][0]
    b=df[[c for c in df.columns if c.startswith(P)]].rename(columns=lambda c:c[len(P):]).copy(); b['t']=df.t.values; b=b[b.z>0.2].reset_index(drop=True)
    t=b.t.values; dt=np.median(np.diff(t))
    e=(b.z-b.ctrltarget_z).values; ev=(b.vz-b.ctrltarget_vz).values; I=np.cumsum(e)*dt; y=b.fbl_acc_z.values
    A=np.c_[b.ctrltarget_az.values,-e,-ev,-I,b.fbl_res_az.values]; sl=slice(int(1/dt),None)
    sol,*_=np.linalg.lstsq(A[sl],y[sl],rcond=None); r2=1-((y[sl]-A[sl]@sol)**2).sum()/((y[sl]-y[sl].mean())**2).sum()
    print(f[:3],f[-8:],'R2 %.3f a_ref %.2f kp %.1f kv %.1f ki %.1f res %.2f'%(r2,*sol),' (set: kp 11.25 kv 5.4 ki 12)')
