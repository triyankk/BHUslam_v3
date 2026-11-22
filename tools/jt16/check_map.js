const fs=require('fs'),path=require('path'),assert=require('assert');const root=process.argv[2];
const read=name=>fs.readFileSync(path.join(root,'data',name),'utf8');
const csv=name=>{const lines=read(name).trim().split('\n'),keys=lines.shift().split(',');return lines.map(l=>Object.fromEntries(l.split(',').map((v,i)=>[keys[i],Number(v)])))};
const summary=JSON.parse(read('summary.json')),clouds=csv('clouds.csv'),imu=csv('imu.csv'),poses=csv('trajectory.csv'),data=JSON.parse(read('map-data.json'));
assert(!summary.camera_used&&!summary.cube_interface&&!summary.calibration_validated);
assert(clouds.length===summary.clouds_published&&imu.length===summary.imu_published&&poses.length===summary.odometry_samples);
for(const rows of [clouds,imu,poses])for(const r of rows)assert(Object.values(r).every(Number.isFinite));
for(const rows of [imu,poses])for(let i=1;i<rows.length;i++)assert(rows[i].sensor_s>rows[i-1].sensor_s);
for(const c of clouds){assert(c.sensor_end_s-c.sensor_begin_s>=.1&&c.sensor_end_s-c.sensor_begin_s<=.3);assert(c.accepted_points>=32&&c.accepted_points<=c.input_points);}
for(const p of poses){assert(Math.abs(Math.hypot(p.qx,p.qy,p.qz,p.qw)-1)<1e-5);assert([p.cov_x,p.cov_y,p.cov_z].every(v=>v>=0));assert(clouds.some(c=>p.sensor_s>=c.sensor_begin_s-1e-5&&p.sensor_s<=c.sensor_end_s+1e-5));}
assert(data.points.length===summary.map_voxels&&data.trajectory.length===poses.length);
const a=poses[0],b=poses.at(-1),displacement=Math.hypot(b.x-a.x,b.y-a.y,b.z-a.z),maxDisplacement=Math.max(...poses.map(p=>Math.hypot(p.x-a.x,p.y-a.y,p.z-a.z)));
const pcd=fs.readFileSync(path.join(root,'full-map.pcd')),header=pcd.subarray(0,1000).toString('ascii'),pointCount=Number(header.match(/^POINTS (\d+)/m)[1]);assert(pointCount>0);
const result={check:'PASS',clouds:clouds.length,imu_samples:imu.length,pose_samples:poses.length,display_map_voxels:data.points.length,pcd_points:pointCount,observed_endpoint_displacement_m:displacement,observed_max_displacement_m:maxDisplacement,ground_truth_available:false,note:'Capture classification is in operator-context.json when supplied. No external ground truth; zero startup covariance is not calibrated certainty.'};
fs.writeFileSync(path.join(root,'artifact-check.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
