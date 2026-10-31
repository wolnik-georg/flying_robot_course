import numpy as np, pandas as pd, warnings, glob
warnings.filterwarnings('ignore')
from scipy.ndimage import gaussian_filter1d
ARM=0.70710678*0.05; T2T=0.00569278844371417; KF=3.91194027330774e-08
FL=['bank_fbl_b1_ours_2026-10-26/A4_2026-10-26_17-25-15','bank_fbl_b1_ours_2026-10-26/A4_2026-10-26_17-26-40','bank_fbl_b1_ours_2026-10-26/A8_2026-10-26_17-23-37','bank_fbl_b1_ours_2026-10-26/A1_2026-10-26_17-28-19']
def load(p):
    n=p.split('/')[-1]; df=pd.read_csv('/home/georg/Desktop/flying_robot_course/experiments/logs/%s/%s_merged_usd.csv'%(p,n),comment='#')
    P=[c.split('.')[0]+'.' for c in df.columns if c.startswith('cf5')][0]
    b=df[[c for c in df.columns if c.startswith(P)]].rename(columns=lambda c:c[len(P):]).copy(); b['t']=df.t.values
    return b[(b.z>0.2)].reset_index(drop=True)
def resamp(b,cols,dt=0.002):
    t=b.t.values; tu=np.arange(t[0],t[-1],dt); return tu,{c:np.interp(tu,t,b[c].values) for c in cols}
res=[]
for p in FL:
    b=load(p); cols=['rpm_m1','rpm_m2','rpm_m3','rpm_m4','motor_m1','motor_m2','motor_m3','motor_m4','gyro_x','gyro_y','gyro_z']
    tu,d=resamp(b,cols); dt=0.002
    F=[KF*(d['rpm_m%d'%i]*2*np.pi/60)**2 for i in (1,2,3,4)]
    tau={'x':ARM*(-F[0]-F[1]+F[2]+F[3]),'y':ARM*(-F[0]+F[1]+F[2]-F[3]),'z':T2T*(-F[0]+F[1]-F[2]+F[3])}
    out=[p.split('/')[-1][:3]+p[-8:]]
    for ax in 'xyz':
        w=np.radians(d['gyro_'+ax]); wf=gaussian_filter1d(w,0.02/dt); al=np.gradient(wf,dt)
        tf=gaussian_filter1d(tau[ax],0.02/dt)
        best=None
        for lag in range(-10,40):
            k=lag
            if k>=0: a=al[k:]; x=tf[:len(tf)-k]
            else: a=al[:k]; x=tf[-k:]
            x=x-x.mean(); a=a-a.mean(); c=np.corrcoef(x,a)[0,1]
            if best is None or c>best[0]: best=(c,lag,(x@a)/(x@x))
        out.append('%s: corr %.2f lag %d ms 1/J=%.0f (J=%.2e)'%(ax,best[0],best[1]*2,best[2],1/best[2]))
    print(' | '.join(out))
    # command -> rpm, differential modes
    S={'x':(-1,-1,1,1),'y':(-1,1,1,-1)}
    for ax,s in S.items():
        cmd=sum(si*d['motor_m%d'%(i+1)] for i,si in enumerate(s))/65535*0.2
        rf=sum(si*F[i] for i,si in enumerate(s))
        cmd=cmd-cmd.mean(); rf=rf-rf.mean()
        best=None
        for tau_ms in (10,20,30,44,60,80,100,150):
            for dly in range(0,60,2):
                # first-order lag of delayed cmd
                a=dt/(tau_ms*1e-3+dt); y=np.zeros_like(cmd); c=np.roll(cmd,dly)
                for i in range(1,len(c)): y[i]=y[i-1]+a*(c[i]-y[i-1])
                m=slice(500,None); x=y[m]-y[m].mean(); z=rf[m]-rf[m].mean(); r=np.corrcoef(x,z)[0,1]
                if best is None or r>best[0]: best=(r,tau_ms,dly*2,(x@z)/(x@x))
        print('   cmd->rpm diff %s: corr %.2f tau %d ms delay %d ms gain %.2f'%(ax,*best))
