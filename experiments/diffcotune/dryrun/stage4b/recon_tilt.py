import sys, os, csv, numpy as np
sys.path.insert(0,'/home/georg/Desktop/flying_robot_course/experiments/diffcotune'); sys.path.insert(0,'/home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab')
import torch; torch.set_num_threads(2)
import usd_ingest as U, ingest, costs as C, s0_model as S, test_ingest_oct23 as T, gyro_check as G
rows=[r for r in csv.DictReader(open(T.PF)) if r['bank']=='bank_unseen_S0_2026-10-23' and r['scenario']=='A1'][:1]
r=rows[0]; cell=T.cell_of('A1',r); f=T.BANK/r['folder']/(r['folder']+'_merged_usd.csv')
for tl in (True, False):
    g=U.ingest_flight(f,'A1',cell,top_from_log=tl); ref=g['ref']; tr=g['trace']
    with torch.no_grad(): o=S.rollout(ref,np.zeros((1,10)),np.zeros((1,4)),compile=False)
    tgt=ingest.on_grid(ingest.from_trace(tr),o['t']); qm=o['q'][:,0,:].numpy(); tm=np.asarray(o['t']); wn=np.asarray(C.window(o['t'],ref.scenario_s,'sysid'),bool)
    yl,tl_=G.yaw_tilt(tgt.q); ym,tm_=G.yaw_tilt(qm)
    print('top_from_log',tl,'tilt rms real %.2f model %.2f'%(np.degrees(np.sqrt((tl_[wn]**2).mean())),np.degrees(np.sqrt((tm_[wn]**2).mean()))),'n',wn.sum(), 'ref has_top',ref.has_top, 'scenario_s',ref.scenario_s)
print('--- my formulas on the same arrays (top_from_log False run is last; redo True)')
import attitude_check as A
g=U.ingest_flight(f,'A1',cell,top_from_log=True); ref=g['ref']; tr=g['trace']
with torch.no_grad(): o=S.rollout(ref,np.zeros((1,10)),np.zeros((1,4)),compile=False)
tgt=ingest.on_grid(ingest.from_trace(tr),o['t']); qm=o['q'][:,0,:].numpy(); tm=np.asarray(o['t']); wn=np.asarray(C.window(o['t'],ref.scenario_s,'sysid'),bool)
ql=np.stack([np.interp(tm,tr[:,0],tr[:,7+j]) for j in range(4)],1)
print('max |tgt.q - ql|',np.abs(np.asarray(tgt.q)-ql).max(), 'tr time', tr[:3,0], 'tm', tm[:3], 'qm[0]',qm[0],'tgt.q[0]',np.asarray(tgt.q)[0])
for nm,q in (('tgt.q',np.asarray(tgt.q)),('ql',ql),('qm',qm)):
    r_,p_=A.rp(q); tl=np.degrees(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1)))
    print(nm,'roll rms %.2f pitch rms %.2f tilt rms %.2f'%tuple(np.sqrt((x[wn]**2).mean()) for x in (r_,p_,tl)), 'G.yaw_tilt', np.degrees(np.sqrt((G.yaw_tilt(q)[1][wn]**2).mean())))
