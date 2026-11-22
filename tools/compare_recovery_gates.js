#!/usr/bin/env node
// Compare diagnostic decisions on identical live samples; never opens hardware.
const fs=require('fs'),path=require('path');
if(process.argv.length!==5)throw Error('Usage: compare_recovery_gates.js RUN_DIRECTORY WINDOW_JSON OUTPUT_DIRECTORY');
const [run,windowFile,out]=process.argv.slice(2),window=JSON.parse(fs.readFileSync(windowFile)),begin=window.start_host_us,end=window.end_host_us;
const lines=fs.readFileSync(path.join(run,'recovery_comparison.csv'),'utf8').trim().split('\n'),keys=lines.shift().split(',');
const all=lines.filter(Boolean).map(l=>l.split(',')).filter(c=>c.length===keys.length).map(c=>Object.fromEntries(c.map((v,i)=>[keys[i],Number(v)]))).filter(r=>r.gate_now_us<=end),rows=all.filter(r=>r.gate_now_us>=begin);
if(!rows.length||!all.every(r=>Number.isFinite(r.gate_now_us)))throw Error('Missing/invalid comparison samples');
function simulate(key){
  let pending=null,last=-Infinity;const sent=[];
  function flush(until){if(!pending)return;const due=Math.ceil(Math.max(pending.gate_now_us,last+100000)/5000)*5000;if(due>until)return;const row=pending;pending=null;if(due-row.host_capture_us>150000)return;sent.push({time:due,row});last=due;}
  for(const row of all){flush(row.gate_now_us-1);if(row[key])pending=row;flush(row.gate_now_us);}flush(end);
  const during=sent.filter(s=>s.time>=begin&&s.time<=end),gaps=during.slice(1).map((s,i)=>(s.time-during[i].time)/1000);
  let covered=0,edge=begin,healthyIntervals=[];
  for(const s of sent){const a=Math.max(begin,s.time),b=Math.min(end,s.time+300000);if(b<=a)continue;const from=Math.max(a,edge);if(b>from)covered+=b-from;edge=Math.max(edge,b);healthyIntervals.push([a,b]);}
  const coverage=covered/(end-begin),resetsKey=key==='baseline_eligible'?'baseline_reset':'candidate_reset';let resetChanges=0,maxStep=0;
  for(let i=1;i<during.length;i++){const a=during[i-1].row,b=during[i].row;if(a[resetsKey]!==b[resetsKey])resetChanges++;else if(a.epoch===b.epoch)maxStep=Math.max(maxStep,Math.hypot(b.x-a.x,b.y-a.y,b.z-a.z));}
  return {eligible_frames:rows.filter(r=>r[key]).length,eligible_percent:100*rows.filter(r=>r[key]).length/rows.length,modeled_packets:during.length,modeled_hz:during.length/((end-begin)/1e6),modeled_gaps_over_300ms:gaps.filter(g=>g>300).length,modeled_max_gap_ms:Math.max(0,...gaps),modeled_receipt_coverage_percent:100*coverage,modeled_reset_changes:resetChanges,max_same_epoch_packet_step_m:maxStep,healthyIntervals};
}
const baseline=simulate('baseline_eligible'),candidate=simulate('candidate_eligible'),extra=rows.filter(r=>r.candidate_eligible&&!r.baseline_eligible),unsafe=rows.filter(r=>(r.baseline_eligible||r.candidate_eligible)&&(r.hard_mask||r.support_mask));
if(unsafe.length)throw Error('A gate accepted a rejected sample');
const result={run,window,frames:rows.length,baseline,candidate,extra_candidate_frames:extra.length,extra_frames_all_pass_hard_and_support:true,candidate_fast_frames:rows.filter(r=>r.candidate_eligible&&r.candidate_recovery_us===100000).length,live_policy_changed:false,method:'Same real VINS samples. Offline 5 ms sender model, latest-only, 10 Hz cap and 150 ms age. Receipt coverage uses 300 ms timeout; candidate is never sent to Cube. This models transport and is not EKF/accuracy evidence.'};
fs.mkdirSync(out,{recursive:true});fs.writeFileSync(path.join(out,'comparison.json'),JSON.stringify(result,null,2)+'\n');
let svg='<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 190"><rect width="1000" height="190" fill="white"/><text x="20" y="25" font-family="sans-serif" font-size="18">Modeled Cube receipt: green=fresh within 300 ms; grey=gap</text>';
for(const [i,label,data]of [[0,'500 ms recovery',baseline],[1,'100 ms brief recovery',candidate]]){const y=65+i*55;svg+=`<text x="20" y="${y+15}" font-family="sans-serif" font-size="13">${label}</text><rect x="200" y="${y}" width="770" height="22" fill="#d1d5db"/>`;for(const [a,b]of data.healthyIntervals)svg+=`<rect x="${200+770*(a-begin)/(end-begin)}" y="${y}" width="${770*(b-a)/(end-begin)}" height="22" fill="#16a34a"/>`;}
svg+='<text x="200" y="175" font-family="sans-serif" font-size="12">0 s</text>'+`<text x="920" y="175" font-family="sans-serif" font-size="12">${((end-begin)/1e6).toFixed(0)} s</text></svg>`;fs.writeFileSync(path.join(out,'receipt-timeline.svg'),svg);
console.log(JSON.stringify({frames:result.frames,baseline:{...baseline,healthyIntervals:undefined},candidate:{...candidate,healthyIntervals:undefined},extra:extra.length},null,2));
