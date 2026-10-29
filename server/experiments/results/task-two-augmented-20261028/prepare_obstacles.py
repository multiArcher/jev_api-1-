"""Recreate the exact eight added stationary targets used on 2026-10-27."""
import copy,json,math
from pathlib import Path
root=Path(__file__).resolve().parent
base=Path('server/experiments/tasks')
task=json.loads((base/'task_two.json').read_text());existing=json.loads((base/'static_situation.json').read_text())
wp=task['params']['planWaypointList'];lon0=wp[0]['routeLon'];lat0=wp[0]['routeLat'];earth=6378137.
def xy(lon,lat):return ((lon-lon0)*math.pi/180*earth*math.cos(math.radians(lat0)),(lat-lat0)*math.pi/180*earth)
def lonlat(x,y):return (lon0+math.degrees(x/(earth*math.cos(math.radians(lat0)))),lat0+math.degrees(y/earth))
points=[xy(w['routeLon'],w['routeLat']) for w in wp]
centers=[xy(r['longitude'],r['latitude']) for r in existing]
added=[];placements=[]
for leg in (6,10,14,18,22,26,30,34,38,42):
 a,b=points[leg],points[leg+1];length=math.dist(a,b)
 if length<700:continue
 nx,ny=-(b[1]-a[1])/length,(b[0]-a[0])/length
 for fraction in (.45,.65,.30,.80,.55):
  lateral=20. if len(added)%2==0 else -20.
  point=(a[0]+fraction*(b[0]-a[0])+lateral*nx,a[1]+fraction*(b[1]-a[1])+lateral*ny)
  nearest_wp=min(math.dist(point,p) for p in points)
  nearest_obstacle=min(math.dist(point,p) for p in centers)
  if nearest_wp<200. or nearest_obstacle<400. or math.dist(point,(-50.,0.))<400.:continue
  r=copy.deepcopy(existing[-1]);r.update(platformId=4101+len(added),targetSpeed=0.,targetCourse=0.,batch=4101+len(added))
  r['longitude'],r['latitude']=lonlat(*point)
  added.append(r);centers.append(point)
  placements.append(dict(platform_id=r['platformId'],leg=f'P{leg+1} -> P{leg+2}',xy_m=point,
                         offset_from_leg_m=lateral,nearest_waypoint_m=nearest_wp,nearest_existing_obstacle_m=nearest_obstacle))
  break
 if len(added)==8:break
assert len(added)==8, placements
expected=[(544.1320248855665,-406.646571099527),(877.0872215079141,-701.6177669656516),
          (1581.8167624521325,-580.1689286861797),(1491.5719092765828,-1230.2788661515624),
          (1315.9707474761756,-1847.5812121222998),(2204.003123115982,-1676.7457263961128),
          (2825.758494786497,-1624.9267052089458),(2621.349743494917,-2370.8400886622744)]
assert all(math.dist(p['xy_m'],e)<1e-6 for p,e in zip(placements,expected))
(root/'static-situation-augmented.json').write_text(json.dumps(existing+added,ensure_ascii=False,indent=2)+'\n')
(root/'added-obstacles.json').write_text(json.dumps(added,ensure_ascii=False,indent=2)+'\n')
(root/'obstacle-placement.json').write_text(json.dumps(dict(original_count=len(existing),added_count=len(added),total_count=len(existing+added),placements=placements),ensure_ascii=False,indent=2)+'\n')
print('Recreated original layout:',len(existing),'original +',len(added),'added =',len(existing+added))
