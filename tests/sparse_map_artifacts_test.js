const fs=require('fs'),path=require('path'),assert=require('assert'),vm=require('vm');
const root=process.argv[2],d=JSON.parse(fs.readFileSync(path.join(root,'sparse-map.json'),'utf8'));
const rows=fs.readFileSync(path.join(root,'tracking.csv'),'utf8').trim().split('\n').slice(1).map(s=>s.split(',').map(Number));
const start=rows[0][0],lost=rows.filter(r=>r[1]!==2).map(r=>r[0]-start);
let prev=-1,total=0,loopMax=0;
function pose(p){assert(p.length===7&&p.every(Number.isFinite));assert(Math.abs(Math.hypot(...p.slice(3))-1)<.002,'Invalid quaternion')}
for(const s of d.snapshots){assert(Number.isFinite(s.t)&&s.t>=prev);prev=s.t;
 const jpeg=fs.readFileSync(path.join(root,s.image));assert(jpeg[0]===255&&jpeg[1]===216,'Invalid preview JPEG');
 if(s.map===null){assert(s.points.length===0&&s.keys.length===0);continue}
 assert(d.maps[s.map]&&s.state===2);assert(s.points.length<=d.point_limit&&s.points.length<=s.valid_landmarks);pose(s.camera);
 for(const p of s.points)assert(p.length===4&&p.every(Number.isFinite),'Invalid point');total+=s.points.length;
 const keys=new Map(s.keys.map(k=>{pose(k.pose);return[k.id,k]}));
 for(const[a,b,type]of s.edges)assert(keys.has(a)&&keys.has(b)&&[0,1,2].includes(type),'Invalid graph edge');
 for(const[a,b]of s.route){assert(keys.has(a)&&keys.has(b));const lo=keys.get(a).t,hi=keys.get(b).t;assert(!lost.some(t=>t>lo+1e-6&&t<hi-1e-6),'Route crosses tracking loss')}
 loopMax=Math.max(loopMax,s.edges.filter(e=>e[2]===2).length);
}
for(const[id,m]of Object.entries(d.maps)){const s=d.snapshots[m.last_snapshot];assert(s.map===id&&s.t===m.last_s);const ply=fs.readFileSync(path.join(root,'map-'+id.replace(':','-')+'.ply'),'utf8');assert(ply.includes('element vertex '+s.points.length+'\n'));assert(ply.trim().split('\n').length===s.points.length+8,'PLY vertex count mismatch')}
const html=fs.readFileSync(path.join(root,'sparse-map.html'),'utf8');assert(!html.includes('/*SPARSE_MAP_DATA*/null'));new vm.Script(html.match(/<script>([\s\S]*)<\/script>/)[1]);
const summary={mode:d.mode,frames:rows.length,tracked_frames:rows.filter(r=>r[1]===2).length,snapshots:d.snapshots.length,maps:Object.keys(d.maps).length,landmark_vertices_across_snapshots:total,maximum_valid_landmarks:Math.max(0,...d.snapshots.map(s=>s.valid_landmarks||0)),maximum_loop_edges:loopMax,last_geometry_s:Math.max(0,...d.snapshots.filter(s=>s.map!==null).map(s=>s.t)),recording_last_s:prev};
fs.writeFileSync(path.join(root,'artifact-check.json'),JSON.stringify(summary,null,2)+'\n');console.log(JSON.stringify(summary,null,2));console.log('PASS: finite geometry/quaternions, map ownership, graph edges, tracking gaps, JPEGs, PLYs and embedded viewer syntax');
