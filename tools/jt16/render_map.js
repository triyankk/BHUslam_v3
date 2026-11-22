const fs=require('fs'),path=require('path'),assert=require('assert');
const dir=process.argv[2];if(!dir)throw Error('Output data directory required');
const data=JSON.parse(fs.readFileSync(path.join(dir,'map-data.json'),'utf8'));
assert(data.points.length>0&&data.points.length===data.summary.map_voxels&&data.points.length<=250000);
assert(data.trajectory.length===(data.summary.trajectory_export_samples??data.summary.odometry_samples)&&data.trajectory.length>1);
for(const p of data.points)assert(p.length===3&&p.every(Number.isFinite));
for(let i=0;i<data.trajectory.length;i++){const p=data.trajectory[i];assert(p.length===4&&p.every(Number.isFinite));if(i)assert(p[0]>data.trajectory[i-1][0]);}
const template=fs.readFileSync(path.join(__dirname,'../../web/jt16-map-viewer.html'),'utf8');
fs.writeFileSync(path.join(dir,'map.html'),template.replace('/*JT16_DATA*/null',JSON.stringify(data).replaceAll('<','\\u003c')));
const ply=fs.readFileSync(path.join(dir,'map.ply'),'utf8');assert(ply.includes('element vertex '+data.points.length+'\n'));
console.log('PASS: finite map points, monotonic trajectory, PLY/summary consistency and offline viewer export');
