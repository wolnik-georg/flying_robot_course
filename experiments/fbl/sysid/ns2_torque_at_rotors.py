import sys, numpy as np, pandas as pd, warnings
warnings.filterwarnings('ignore')
sys.argv=sys.argv[:1]; sys.path.insert(0,'/home/georg/Desktop/flying_robot_course/experiments/fbl/stage3_port')
import c_in_the_loop_real as C
from scipy.ndimage import uniform_filter1d
ARM=0.70710678*0.05; T2T=0.00569278844371417; KF=3.91194027330774e-08; M=0.0427
XY=np.array([[ARM,-ARM],[-ARM,-ARM],[-ARM,ARM],[ARM,ARM]])   # rotors 1..4: (x,y) from the mixer signs
for n in ['A4_2026-10-26_17-25-15','A4_2026-10-26_17-26-40','A8_2026-10-26_17-23-37','A1_2026-10-26_17-28-19']:
    p='/home/georg/Desktop/flying_robot_course/experiments/logs/bank_fbl_b1_ours_2026-10-26/%s/%s_merged_usd.csv'%(n,n)
    df=pd.read_csv(p,comment='#'); P=[c.split('.')[0]+'.' for c in df.columns if c.startswith('cf5')][0]
    b=df[[c for c in df.columns if c.startswith(P)]].rename(columns=lambda c:c[len(P):]).copy(); b['t']=df.t.values
    tp=[c for c in df.columns if c.startswith('cf_second')][0].split('.')[0]
    for a in 'xyz': b['top_'+a]=df[tp+'.'+a].values
    for a in 'xyz': b['topv_'+a]=np.gradient(df[tp+'.'+a].values,df.t.values)
    b=b[b.z>0.2].reset_index(drop=True)
    F=[KF*(b['rpm_m%d'%i].values*2*np.pi/60)**2 for i in (1,2,3,4)]
    tx=ARM*(-F[0]-F[1]+F[2]+F[3]); ty=ARM*(-F[0]+F[1]+F[2]-F[3])
    sx=uniform_filter1d(tx,250)*1e6; sy=uniform_filter1d(ty,250)*1e6
    print(n[:3],n[-8:],'  t   dx    dy   dz | measured need(-tau_rpm)  x,y | NS2-at-rotors torque x,y  [uNm]')
    for i in range(0,len(b),500):
        r=b.iloc[i]; pos=np.array([r.x,r.y,r.z]); top=np.array([r.top_x,r.top_y,r.top_z]); v=np.array([r.vx,r.vy,r.vz]); tv=np.array([r.topv_x,r.topv_y,r.topv_z])
        Fi=[]
        for k in range(4):
            pk=pos+np.array([XY[k][0],XY[k][1],0.0]); a_nn,_=C.reference(C.W,[(top-pk,tv-v)],float(pk[2]),v,M); Fi.append(M*np.asarray(a_nn,float)[2]/4)
        tx_p=sum(XY[k][1]*Fi[k] for k in range(4))*1e6; ty_p=-sum(XY[k][0]*Fi[k] for k in range(4))*1e6
        if i<4500: print('%5.1f %5.2f %5.2f %5.2f | %7.0f %7.0f | %7.0f %7.0f'%(r.t,r.x-r.top_x,r.y-r.top_y,r.z-r.top_z,-sx[i],-sy[i],tx_p,ty_p))
