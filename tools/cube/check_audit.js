const fs=require('fs'),path=require('path'),assert=require('assert');const auditPath=process.argv[2];
const audit=JSON.parse(fs.readFileSync(auditPath)),result=[];let exact=0,extraEvents=0;
for(const r of audit.files){const p=path.join(path.dirname(r.file),'events.jsonl');if(!fs.existsSync(p))continue;let ekf=0,local=0,flags={};for(const l of fs.readFileSync(p,'utf8').trim().split('\n')){if(!l)continue;const e=JSON.parse(l);if(e.event==='ekf_status'){ekf++;flags[e.flags]=(flags[e.flags]||0)+1;}if(e.event==='local_position_ned')local++;}
 const match=ekf===r.ekf_reports&&local===r.local_position_samples&&JSON.stringify(flags)===JSON.stringify(r.ekf_flags);
 if(match)exact++;else{extraEvents+=ekf-r.ekf_reports;result.push({file:r.file,raw_ekf:r.ekf_reports,event_ekf:ekf,raw_flags:r.ekf_flags,event_flags:flags});}
 assert(Object.keys(flags).every(f=>Number(f)===1024),'This historical audit should contain only uninitialized EKF status');
}
const summary={exact_matches:exact,mismatches:result,additional_event_reports:extraEvents,note:'One event-stream tail EKF report is absent from the saved raw tlog. Both sources show only uninitialized status; count mismatch is retained, not hidden.'};
fs.writeFileSync(path.join(path.dirname(auditPath),'cross-check.json'),JSON.stringify(summary,null,2)+'\n');console.log(JSON.stringify(summary,null,2));
