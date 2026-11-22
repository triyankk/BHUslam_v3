// Check the real replay/report boundary, including camera frames and fixed calibration.
const fs=require('fs'),path=require('path'),assert=require('assert');
const root=process.argv[2];
function csv(p){const lines=fs.readFileSync(p,'utf8').trim().split('\n');return lines.slice(1).map(x=>x.split(',').map(Number));}
for(const name of ['two-minute-vins-fusion','two-minute-vins-fusion-native-noise','eighty-second-vins-fusion']){
 const dir=path.join(root,name),summary=JSON.parse(fs.readFileSync(path.join(dir,'summary.json'))),t=csv(path.join(dir,'tracking.csv')),p=csv(path.join(dir,'pose_history.csv'));
 assert(summary.failure_detector_disabled_upstream===true);assert(summary.loop_closure===false);assert(summary.calibration_validated===false);
 assert(t.length===summary.frames);assert(t.every((r,i)=>r.length===10&&r.every(Number.isFinite)&&(!i||r[0]>t[i-1][0])));
 assert(p.length===summary.initialized_frames);assert(t.filter(r=>r[7]===1).length===summary.visually_supported_frames_heuristic);
 const byTime=new Map(t.map(r=>[r[0],r]));
 for(const r of p){assert(r.length===10&&r.every(Number.isFinite));const state=byTime.get(r[0]);assert(state&&state[1]===1&&state[8]===r[8]&&state[7]===r[9]);assert(Math.abs(Math.hypot(...r.slice(4,8))-1)<1e-5);}
 assert(summary.stereo_baseline_m>.09&&summary.stereo_baseline_m<.10);
 const extrinsics=fs.readFileSync(path.join(dir,'stereo-extrinsics.yaml'),'utf8');assert(extrinsics.includes('body_T_cam0')&&extrinsics.includes('body_T_cam1'));
 const comparison=JSON.parse(fs.readFileSync(path.join(root,'../comparison/comparison.json'))),run=comparison.runs.find(r=>r.name===name);assert(run&&run.frame_count===t.length&&run.poses.length===p.length);assert(run.stereo_matches.p50>0);
 console.log('PASS:',name,'finite/monotonic CSVs, normalized camera poses, report counts, stereo support and provisional configuration');
}
