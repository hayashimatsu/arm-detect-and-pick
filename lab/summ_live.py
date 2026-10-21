import json,sys
d=json.load(open(sys.argv[1]+"/result.json"))
print(d.get('error')); print({k:d.get(k) for k in ('total_sim_s','wall_s','scan_s','live')})
for c in d.get('cycles',[]): print(c['k'],c['cls'],round(c['conf'],2),'retargets',c['retargets'],'grip',c['grip']['holding'],c['grip']['achieved_deg'],'t_arrive_release',c['from_arrive_to_release_s'],'holding_at_place',c['holding_at_place'])
rel=[e['sim_s'] for e in d.get('events',[]) if e['name'].endswith(':released')]
print('release_to_release',[round(b-a,2) for a,b in zip(rel,rel[1:])])
b,a=d['truth_before'],d['truth_after']
for n in a:
    if a[n]!=b[n]: print('moved',n,b[n][:2],'->',a[n][:2],'IN' if d['in_basket_after'][n] else '')
for r in d.get('tracks_vs_truth',[]): print(r)
