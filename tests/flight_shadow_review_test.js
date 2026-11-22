const fs=require('fs'),os=require('os'),path=require('path'),cp=require('child_process'),assert=require('assert');
const code=path.resolve(__dirname,'..'),root=fs.mkdtempSync(path.join(os.tmpdir(),'shadow-review-')),boot='00000000-0000-0000-0000-000000000001';
try{
 const fusion=path.join(root,'runs/vins-fusion-boot/fixture'),live=path.join(fusion,'live');fs.mkdirSync(live,{recursive:true});
 fs.writeFileSync(path.join(fusion,'boot-id.txt'),boot);fs.writeFileSync(path.join(fusion,'power-mode.txt'),'TEST_FIXTURE');
 fs.writeFileSync(path.join(live,'tracking.csv'),'device_s,age_ms,eligible\n10,50,0\n10.05,60,1\npartial');
 fs.writeFileSync(path.join(live,'pose_history.csv'),'timestamp_s,x,y,z,segment,visual_support_20\n10,0,0,0,0,0\n10.05,1,2,3,0,1\n');
 for(const id of ['flight-1','flight-2']){const session=path.join(live,'flights',id);fs.mkdirSync(session,{recursive:true});for(const f of ['tracking.csv','pose_history.csv'])fs.copyFileSync(path.join(live,f),path.join(session,f));}
 const checkpoint=path.join(root,'runs/flight-shadow',boot,'mapping-fixture/data/flights/flight-1/checkpoints/1');fs.mkdirSync(checkpoint,{recursive:true});
 const existing=path.join(code,'reports/jt16-mapping-20261117/loop-02/data');
 for(const name of ['map-data.json','map.ply','summary.json'])fs.copyFileSync(path.join(existing,name),path.join(checkpoint,name));
 const map=JSON.parse(fs.readFileSync(path.join(checkpoint,'map-data.json'),'utf8'));map.summary.finalized=false;
 fs.writeFileSync(path.join(checkpoint,'map-data.json'),JSON.stringify(map));fs.writeFileSync(path.join(checkpoint,'summary.json'),JSON.stringify(map.summary));
 // An incomplete newest checkpoint must not replace a completed generation.
 fs.mkdirSync(path.join(checkpoint,'../2.tmp'));
 const forwarding=path.join(root,'runs/vins-forwarding',boot,'fixture');fs.mkdirSync(forwarding,{recursive:true});
 fs.writeFileSync(path.join(forwarding,'status.json'),JSON.stringify({owner_handoffs:17,cube_reception_proven:false}));
 cp.execFileSync(process.execPath,[path.join(code,'tools/flight_shadow_review.js'),boot,root]);
 const report=path.join(root,'reports/flight-shadow',boot),summary=JSON.parse(fs.readFileSync(path.join(report,'summary.json'),'utf8'));
 assert(!summary.cube_pose_forwarding&&!summary.ekf_fusion_tested&&!summary.metric_accuracy_verified);
 assert(summary.forwarding.native_runs[0].status.owner_handoffs===17&&!summary.forwarding.cube_reception_proven);
 assert(summary.vins.length===2&&summary.vins[0].processed===2&&summary.vins[0].gate_fraction===.5);
 assert(summary.maps.length===1&&summary.maps[0].source===checkpoint&&!summary.maps[0].summary.finalized);
 assert(fs.readFileSync(path.join(report,'vio-trajectory.html'),'utf8').includes('b[0]-a[0]>.25'));
 cp.execFileSync(process.execPath,[path.join(code,'tests/jt16_map_viewer_test.js'),path.join(report,summary.maps[0].viewer)]);
 console.log('PASS: boot grouping, truncated CSV tail, metrics, checkpoint recovery and both 3D viewer exports');
}finally{fs.rmSync(root,{recursive:true,force:true});}
