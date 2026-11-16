import json,sys
d=json.load(open(sys.argv[1]))
f=lambda x,n=2: 'None' if x is None else round(x,n)
for r in d:
    b=r['cf5_unseen'];t=r['cf_second_unseen']; p=r['pitch']; ro=r['roll']
    print(r['folder'][:2],r['folder'][-8:],'rms real/model roll %.1f/%.1f pitch %.1f/%.1f | pitch corr0 %s best %.2f@%.2fs fdom %.2f/%s | bottom max roll %.1f pitch %.1f p99 %.1f >25 %.2f run %.2fs fires %s gyro %.0f | top max %.1f'%(ro['rms_real_deg'],ro['rms_model_deg'],p['rms_real_deg'],p['rms_model_deg'],f(p['corr_zero_lag']),p['best_corr'],p['best_lag_s'],p['f_dom_real_hz'],f(p['f_dom_model_hz']),b['max_abs_roll_deg'],b['max_abs_pitch_deg'],b['p99_tilt_deg'],b['frac_over_25'],b['longest_run_over_25_s'],b['rule_fires_0p5s'],b['max_abs_gyro_dps'],max(t['max_abs_roll_deg'],t['max_abs_pitch_deg'])))
