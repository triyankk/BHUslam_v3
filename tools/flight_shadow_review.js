// Offline only. Never connects to Cube, ROS or camera hardware.
const fs=require('fs'),path=require('path'),cp=require('child_process');
const codeRoot=path.resolve(__dirname,'..'),root=path.resolve(process.argv[3]||codeRoot),boot=process.argv[2];
if(!/^[a-f0-9-]{36}$/.test(boot||''))throw Error('Supply a boot UUID');
const report=path.join(root,'reports/flight-shadow',boot);fs.mkdirSync(report,{recursive:true});
const dirs=p=>fs.existsSync(p)?fs.readdirSync(p,{withFileTypes:true}).filter(e=>e.isDirectory()).map(e=>path.join(p,e.name)):[];
function csv(file){
 if(!fs.existsSync(file))return [];
 const lines=fs.readFileSync(file,'utf8').trim().split('\n'),header=lines.shift().split(',');
 return lines.filter(Boolean).flatMap(line=>{const v=line.split(',').map(Number);if(v.length!==header.length||!v.every(Number.isFinite))return [];return [Object.fromEntries(header.map((k,i)=>[k,v[i]]))];});
}
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fusion=dirs(path.join(root,'runs/vins-fusion-boot')).filter(p=>fs.existsSync(path.join(p,'boot-id.txt'))&&fs.readFileSync(path.join(p,'boot-id.txt'),'utf8').trim()===boot);
const graphs=[],metrics=[];
const sessions=fusion.flatMap(run=>{const base=path.join(run,"live"),f=path.join(base,"flights");return (fs.existsSync(f)?dirs(f):[base]).map(data=>({run,data}));});
for(const {run,data:session} of sessions){
 const tracking=csv(path.join(session,"tracking.csv")),poses=csv(path.join(session,"pose_history.csv"));if(!tracking.length)continue;
 const begin=tracking[0].device_s,end=tracking.at(-1).device_s,ages=tracking.map(r=>r.age_ms).sort((a,b)=>a-b);
 let previous=begin,longest=0;for(const r of tracking)if(r.eligible){longest=Math.max(longest,r.device_s-previous);previous=r.device_s;}longest=Math.max(longest,end-previous);
 const eligible=tracking.filter(r=>r.eligible).length;
 metrics.push({run,flight_directory:session,processed:tracking.length,initialized_pose_samples:poses.length,gate_eligible:eligible,gate_fraction:eligible/tracking.length,duration_s:end-begin,max_gate_gap_including_startup_s:longest,age_p95_ms:ages[Math.floor(.95*(ages.length-1))],power_mode:fs.readFileSync(path.join(run,'power-mode.txt'),'utf8').trim(),input_recording:fs.existsSync(path.join(session,'input/recording.json'))?JSON.parse(fs.readFileSync(path.join(session,'input/recording.json'),'utf8')):null});
 const valid=poses.filter(p=>[p.timestamp_s,p.x,p.y,p.z,p.segment].every(Number.isFinite));
 graphs.push({name:path.basename(run)+"/"+path.basename(session),kind:'online VINS-Fusion shadow',first_s:valid.length?valid[0].timestamp_s-begin:0,last_s:valid.length?valid.at(-1).timestamp_s-begin:0,poses:valid.map(p=>[p.timestamp_s-begin,p.x,p.y,p.z,p.segment,p.visual_support_20]),keys:[],tracked:valid.length,processed:tracking.length,resets:Math.max(0,new Set(valid.map(p=>p.segment)).size-1)});
}
let graphFile=null;
if(graphs.length){
 const data={graphs,power_profile:'See per-run power-mode snapshots',recording_duration_s:Math.max(1,...graphs.map(g=>g.last_s))};
 let html=fs.readFileSync(path.join(codeRoot,'web/pose-graph.html'),'utf8').replace('/*POSE_DATA*/null',JSON.stringify(data).replaceAll('<','\\u003c'));
 html=html.replaceAll('Diagnostic recording','GPS-aided shadow flight').replace('Replay tracking:','Initialized pose samples:').replace('b[0]-a[0]>.051','b[0]-a[0]>.25');
 fs.writeFileSync(path.join(report,'vio-trajectory.html'),html);graphFile='vio-trajectory.html';
}
const mapping=dirs(path.join(root,'runs/flight-shadow',boot)).filter(p=>path.basename(p).startsWith('mapping-')),maps=[];
const mapSessions=mapping.flatMap(run=>{const base=path.join(run,"data"),f=path.join(base,"flights");return (fs.existsSync(f)?dirs(f):[base]).map(data=>({run,data}));});
for(const {run,data} of mapSessions){
 let source=data;
 if(!fs.existsSync(path.join(data,'map-data.json'))){
  const checkpoints=dirs(path.join(data,'checkpoints')).filter(p=>/^\d+$/.test(path.basename(p))).sort((a,b)=>Number(path.basename(b))-Number(path.basename(a)));
  source=checkpoints.find(p=>fs.existsSync(path.join(p,'map-data.json')))||data;
 }
 if(!fs.existsSync(path.join(source,'map-data.json')))continue;
 const output=path.join(report,'map-'+maps.length);fs.mkdirSync(output,{recursive:true});
 for(const name of ['map-data.json','map.ply','summary.json'])fs.copyFileSync(path.join(source,name),path.join(output,name));
 try{cp.execFileSync(process.execPath,[path.join(codeRoot,'tools/jt16/render_map.js'),output],{stdio:'pipe'});maps.push({run,source,viewer:path.relative(report,path.join(output,'map.html')),summary:JSON.parse(fs.readFileSync(path.join(output,'summary.json'),'utf8'))});}
 catch(e){maps.push({run,source,error:String(e)});}
}
const rawDir=path.join('/home/atas/vscode/optFlow_slam/data/recordings/jt16-shadow',boot),rawFiles=fs.existsSync(rawDir)?fs.readdirSync(rawDir).filter(f=>f.endsWith('.raw')).map(f=>({path:path.join(rawDir,f),bytes:fs.statSync(path.join(rawDir,f)).size})):[];
const forwarding=dirs(path.join(root,'runs/vins-forwarding',boot)).map(run=>({run,status:fs.existsSync(path.join(run,'status.json'))?JSON.parse(fs.readFileSync(path.join(run,'status.json'),'utf8')):null,events:path.join(run,'events.jsonl')}));
const ownerDirectory='/home/atas/vscode/optFlow_slam/data/recordings/vio-observation';
const ownerLogs=[];
if(fs.existsSync(ownerDirectory))for(const file of fs.readdirSync(ownerDirectory).filter(f=>f.endsWith('.jsonl'))){
 const full=path.join(ownerDirectory,file),lines=fs.readFileSync(full,'utf8').split('\n'),rows=lines.flatMap(line=>{try{return [JSON.parse(line)];}catch{return [];}});
 if(rows[0]?.boot_id!==boot)continue;
 const writes=rows.filter(r=>r.event==='odometry_serial_write'),ages=writes.map(r=>r.age_at_write_ms).filter(Number.isFinite).sort((a,b)=>a-b);
 let maxGap=0;for(let i=1;i<writes.length;i++)maxGap=Math.max(maxGap,(writes[i].host_monotonic_us-writes[i-1].host_monotonic_us)*1e-6);
 ownerLogs.push({path:full,serial_writes:writes.length,age_p95_ms:ages.length?ages[Math.floor(.95*(ages.length-1))]:null,max_gap_between_writes_s:maxGap,vision_health_samples:rows.filter(r=>r.event==='cube_vision_health').length,cube_reception_proven:false});
}
const summary={boot,mode:'GPS-aided observation; VIO fusion not validated',cube_pose_forwarding:ownerLogs.some(r=>r.serial_writes>0),forwarding:{native_runs:forwarding,owner_logs:ownerLogs,cube_reception_proven:false,required_cube_bin_records:['VISP','VISV','XKFS.SS=0']},ekf_fusion_tested:false,metric_accuracy_verified:false,vins:metrics,maps,raw_lidar:rawFiles,cube_reference_directory:'/home/atas/Desktop/Drone_Sensor_Recordings',note:'Independent native frames and unverified sensor clock alignment. No automatic VINS/LIO/GPS accuracy claim. Preserve matching Cube BIN and recorder session for later comparison.'};
fs.writeFileSync(path.join(report,'summary.json'),JSON.stringify(summary,null,2)+'\n');
const links=[graphFile?`<li><a href="${graphFile}">VINS 3D trajectory</a></li>`:'<li>No VINS pose data found.</li>',...maps.map((m,i)=>m.viewer?`<li><a href="${m.viewer}">JT16 3D map ${i+1}</a> (${m.summary.finalized?'final':'checkpoint'})</li>`:`<li>${esc(m.error)}</li>`)];
fs.writeFileSync(path.join(report,'index.html'),`<!doctype html><meta charset="utf-8"><title>Shadow flight review</title><style>body{font:16px system-ui;max-width:1100px;margin:32px auto;background:#101721;color:#e0eaf5}a{color:#7dccff}pre{white-space:pre-wrap}</style><h1>GPS-aided shadow flight</h1><p>Boot ${esc(boot)}. VIO and mapping are independent observations; gated pose reception is logged separately from EKF fusion. Forwarding health and evidence paths are listed below. Cube BIN VISP/VISV prove reception; XKFS.SS=0 verifies primary-source use. Sensor frames/clocks are not yet aligned for an accuracy comparison.</p><ul>${links.join('')}</ul><p>Keep matching Cube BIN and Desktop sensor recorder session for GPS reference. Graceful stop provides final files; power loss may leave only the last completed map checkpoint.</p><pre>${esc(JSON.stringify(summary,null,2))}</pre>`);
console.log(path.join(report,'index.html'));
