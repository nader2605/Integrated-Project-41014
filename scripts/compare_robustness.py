"""Compare completed fixed suites; preserve pilot trials separately."""
import argparse
import csv
import json
from pathlib import Path


def read_suite(path):
    with (path/'summary.csv').open(newline='') as stream:
        rows=list(csv.DictReader(stream))
    config=json.loads((path/'config.json').read_text())
    expected={(c,s) for c in config['cases'] for s in config['seeds']}
    actual={(r['case'],int(r['seed'])) for r in rows}
    if actual!=expected or len(rows)!=len(expected):
        raise ValueError(f'Incomplete or duplicated suite: {path}')
    return {(r['case'],int(r['seed'])):r for r in rows},config


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,default=root/'output/robustness_initial')
    parser.add_argument('--after',type=Path,default=root/'output/robustness_final')
    parser.add_argument('--output',type=Path,default=root/'output/robustness_comparison')
    args=parser.parse_args()
    before,old_config=read_suite(args.before)
    after,new_config=read_suite(args.after)
    if before.keys()!=after.keys() or old_config['max_seconds']!=new_config['max_seconds'] or old_config['success_definition']!=new_config['success_definition']:
        raise ValueError('Cases, seeds, time limit and success definition must match.')
    args.output.mkdir(parents=True,exist_ok=True)
    cases=new_config['cases']
    passed=lambda row:row['success'].lower()=='true'
    lines=['# Robustness comparison','',
           f"Same cases, seeds {', '.join(map(str,new_config['seeds']))}, {new_config['max_seconds']:g} simulated-second limits and evaluation criteria. "
           'The revised pipeline adds denser 360-degree sensing and gyro prediction as well as control changes; '
           'the comparison does not isolate the contribution of any one change.','',
           '| Condition | Initial passes | Revised passes |','|---|---:|---:|']
    for c in cases:
        keys=[key for key in after if key[0]==c]
        lines.append(f'| {c} | {sum(passed(before[k]) for k in keys)}/{len(keys)} | {sum(passed(after[k]) for k in keys)}/{len(keys)} |')
    lines+=['','| Trial | Initial final error (m) | Revised final error (m) | Revised result |','|---|---:|---:|---|']
    keys=[(c,s) for c in cases for s in new_config['seeds']]
    with (args.output/'comparison.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['case','seed','before_success','after_success','before_final_error_m','after_final_error_m','after_completion_sim_s','after_collision_steps'])
        for key in keys:
            b,a=before[key],after[key]
            lines.append(f"| {key[0]} / {key[1]} | {float(b['actual_target_error_m']):.3f} | {float(a['actual_target_error_m']):.3f} | {'PASS' if passed(a) else 'FAIL'} |")
            writer.writerow([*key,passed(b),passed(a),b['actual_target_error_m'],a['actual_target_error_m'],a.get('completion_sim_s'),a.get('collision_physics_steps')])
    successes=[r for r in after.values() if passed(r)]
    if successes:
        times=[float(r['completion_sim_s']) for r in successes]
        errors=[float(r['actual_target_error_m']) for r in successes]
        lines+=['',f'Revised successful trials: final position error {min(errors):.3f}-{max(errors):.3f} m; completion {min(times):.1f}-{max(times):.1f} simulated seconds.']
    lines+=['','A final error for a timed-out trial is not a docking error. Success requires controller docking, '
            'actual target error <=0.15 m, actual heading error <=15 degrees and zero observed hazard contact. '
            'The initial results are retained unchanged. Pilot/adaptive runs are excluded. '
            'Two seeds per condition are exploratory evidence, not a population success-rate estimate. '
            'Sensor bias/dropout, unknown initial localisation error and Windows runtime remain unverified.','']
    (args.output/'comparison.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    x=np.arange(len(cases))
    old_counts=[sum(passed(before[k]) for k in keys if k[0]==c) for c in cases]
    new_counts=[sum(passed(after[k]) for k in keys if k[0]==c) for c in cases]
    axes[0].bar(x-.18,old_counts,width=.36,label='Initial',color='#b55b50')
    axes[0].bar(x+.18,new_counts,width=.36,label='Revised',color='#27788e')
    count=len(new_config['seeds'])
    names={'baseline':'Baseline','start_pose':'Varied start','high_noise':'2x noise','push':'Push'}
    axes[0].set(xticks=x,xticklabels=[names[c] for c in cases],yticks=list(range(count+1)),ylim=(0,count+.5),ylabel=f'Passed trials (out of {count})',title='Same evaluation criteria')
    axes[0].legend()
    for i,(o,n) in enumerate(zip(old_counts,new_counts)):
        axes[0].text(i-.18,o+.06,str(o),ha='center')
        axes[0].text(i+.18,n+.06,str(n),ha='center')
    for i,key in enumerate(keys):
        for offset,rows,colour in [(-.12,before,'#b55b50'),(.12,after,'#27788e')]:
            row=rows[key]
            axes[1].scatter(i+offset,float(row['actual_target_error_m']),color=colour,marker='o' if passed(row) else 'x',s=50)
    axes[1].axhline(.15,color='black',linestyle='--',linewidth=1,label='Position-error limit')
    labels={'baseline':'Base','start_pose':'Start','high_noise':'Noise','push':'Push'}
    axes[1].set(xticks=np.arange(len(keys)),xticklabels=[f'{labels[c]}\n{s}' for c,s in keys],ylabel='Final distance to target (m)',title='Circle: pass; cross: fail',yscale='log')
    axes[1].legend(fontsize=9)
    for ax in axes:
        ax.spines[['top','right']].set_visible(False)
        ax.grid(axis='y',alpha=.2)
    fig.suptitle(f'TurtleBot3: {len(keys)}-trial before/after comparison',fontsize=15)
    fig.savefig(args.output/'comparison.png',dpi=200)
    plt.close(fig)
    print(args.output/'comparison.md')


if __name__=='__main__':
    main()
