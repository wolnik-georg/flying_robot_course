import sys,os,numpy as np
build=sys.argv[1]; csvp=sys.argv[2]
sys.path.insert(0,build); os.environ['FRSIM_FIRMWARE_VERBOSE']=''
import cffirmware as fw
rows=[l for l in open(csvp) if not l.startswith('#')]
h=rows[0].strip().split(','); P='cf5_A1.'
D=np.array([[float(v) if v not in('','nan') else np.nan for v in r.strip().split(',')] for r in rows[1:]])
col=lambda n: D[:,h.index(P+n)]
t=D[:,0]
om=fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(om); om.indi=3
sp,se,st,co=fw.setpoint_t(),fw.sensorData_t(),fw.state_t(),fw.control_t()
sp.mode.x=sp.mode.y=sp.mode.z=fw.modeAbs; sp.mode.yaw=fw.modeAbs
def S(v,x,y,z): v.x,v.y,v.z=float(x),float(y),float(z)
sel=np.where((t>2.0)&(t<19.0))[0][::4]   # 500 Hz (log is 2 ms? step check)
dt=np.median(np.diff(t)); step=max(1,int(round(0.002/dt))); sel=np.where((t>2.0)&(t<19.0))[0][::step]
out=[]
for k,i in enumerate(sel):
    S(sp.position,col('x')[i],col('y')[i],col('z')[i]); S(sp.velocity,0,0,0); S(sp.acceleration,0,0,0); S(sp.jerk,0,0,0)
    S(st.position,col('x')[i],col('y')[i],col('z')[i]); S(st.velocity,col('vx')[i],col('vy')[i],col('vz')[i])
    st.attitudeQuaternion.x,st.attitudeQuaternion.y,st.attitudeQuaternion.z,st.attitudeQuaternion.w=[float(col(n)[i]) for n in('est_qx','est_qy','est_qz','est_qw')]
    st.attitude.roll=float(col('roll_deg')[i]); st.attitude.pitch=float(-col('pitch_deg')[i]); st.attitude.yaw=float(col('yaw_deg')[i])
    S(se.gyro,col('gyro_x')[i],col('gyro_y')[i],col('gyro_z')[i]); S(se.acc,col('acc_x')[i],col('acc_y')[i],col('acc_z')[i])
    from scipy.spatial.transform import Rotation as Rot
    R=Rot.from_quat([float(col(n)[i]) for n in('est_qx','est_qy','est_qz','est_qw')]).as_matrix()
    aw=R@np.array([col('acc_x')[i],col('acc_y')[i],col('acc_z')[i]])-np.array([0,0,1.0])
    S(st.acc,*aw)
    fw.oot_set_rpm(*[int(col(f'motor_m{m}_rpm')[i]) if np.isfinite(col(f'motor_m{m}_rpm')[i]) else 0 for m in (1,2,3,4)])
    fw.controllerOmarIndi(om,co,sp,se,st,k)
    out.append([t[i],om.f_rpm,om.a_rpm_filtered.x,om.a_rpm_filtered.y,om.a_rpm_filtered.z,om.a_imu_filtered.x,om.a_imu_filtered.y,om.a_imu_filtered.z,om.tau_rpm_filtered.x,om.tau_rpm_filtered.y,om.tau_rpm_filtered.z,
                col('omar_f_rpm')[i],col('omar_a_rpm_fz')[i],col('omar_a_imu_fz')[i],*[col(f'motor_m{m}_rpm')[i] for m in (1,2,3,4)],*[col(f'rpm_m{m}')[i] for m in (1,2,3,4)]])
np.save(sys.argv[3],np.array(out))
print('ticks',len(out),'dt log',dt)
