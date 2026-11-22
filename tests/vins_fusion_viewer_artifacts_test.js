const fs=require('fs'),path=require('path'),assert=require('assert');
const root=process.argv[2],data=JSON.parse(fs.readFileSync(path.join(root,'viewer-data.json'))),summary=JSON.parse(fs.readFileSync(path.join(root,'summary.json')));
assert(summary.viewer_export===true);assert(data.schema===1);assert(data.frames.length===Math.ceil(summary.frames/data.sample_stride)+((summary.frames-1)%data.sample_stride!==0?1:0));
const rows=fs.readFileSync(path.join(root,'pose_history.csv'),'utf8').trim().split('\n').slice(1).map(s=>s.split(',').map(Number)),byTime=new Map(rows.map(r=>[Math.round(r[0]*1e6),r]));
const source=fs.readFileSync(path.join(summary.dataset,'frames.csv'),'utf8').trim().split('\n').slice(1).map(s=>s.split(','));let previous=-1,images=0,paired=0;const errors=[],clouds=new Map();
const orb=fs.readFileSync(path.join(summary.dataset,'orb.yaml'),'utf8'),param=k=>Number(orb.match(new RegExp('^'+k.replaceAll('.','\\.')+': ([^\\n]+)','m'))[1]);
function inverseRotate(q,p){const[x,y,z,w]=q.map((a,i)=>i<3?-a:a),t=[2*(y*p[2]-z*p[1]),2*(z*p[0]-x*p[2]),2*(x*p[1]-y*p[0])];return[p[0]+w*t[0]+y*t[2]-z*t[1],p[1]+w*t[1]+z*t[0]-x*t[2],p[2]+w*t[2]+x*t[1]-y*t[0]];}
for(const [i,f]of data.frames.entries()){
 assert(f.t>previous);previous=f.t;assert(Number(source[f.source_index][0])===f.timestamp_s);assert(Math.abs(f.t-(f.timestamp_s-Number(source[0][0])))<1e-9);assert(f.source_index%data.sample_stride===0||f.source_index===summary.frames-1);assert(f.image==='viewer-images/'+i+'.jpg');const image=fs.readFileSync(path.join(root,f.image));assert(image[0]===255&&image[1]===216);images++;
 if(f.camera){const r=byTime.get(Math.round(f.timestamp_s*1e6));assert(r);f.camera.forEach((v,k)=>assert(Math.abs(v-r[k+1])<1e-10));}
 if(!clouds.has(f.epoch))clouds.set(f.epoch,new Map());const observations=new Map(f.features.map(p=>[p[0],p]));
 for(const p of f.points){assert(p.length===4&&p.every(Number.isFinite));clouds.get(f.epoch).set(p[0],p);const a=observations.get(p[0]);if(f.camera&&f.supported&&a){const pc=inverseRotate(f.camera.slice(3),p.slice(1).map((v,k)=>v-f.camera[k]));if(pc[2]>.2){const u=param('Camera1.fx')*pc[0]/pc[2]+param('Camera1.cx'),v=param('Camera1.fy')*pc[1]/pc[2]+param('Camera1.cy');errors.push(Math.hypot(u-a[1],v-a[2]));paired++;}}}
}
assert(data.frames[0].source_index===0);assert(data.frames.at(-1).source_index===summary.frames-1);assert(images>0&&paired>20);errors.sort((a,b)=>a-b);const p95=errors[Math.floor(.95*(errors.length-1))];assert(p95<10,'Landmark projection inconsistent with synchronized feature observations: '+p95);
for(const [epoch,points]of clouds){const ply=fs.readFileSync(path.join(root,'fusion-epoch-'+epoch+'.ply'),'utf8');assert(Number(ply.match(/element vertex (\d+)/)[1])===points.size);}
const html=fs.readFileSync(path.join(root,'viewer.html'),'utf8');assert(!html.includes('/*FUSION_VIEWER_DATA*/null'));assert(!/https?:\/\//.test(html));
console.log('PASS:',data.recording,images,'paired stereo images; camera/CSV and source timestamps agree; PLY epochs match; solved landmark reprojection p95',p95.toFixed(3),'px across',paired,'observations');
