const fs=require('fs'),vm=require('vm'),assert=require('assert');
const page=fs.readFileSync(process.argv[2],'utf8');
const report={phase:'COMPLETE',power_profile:'FIXTURE',duration_s:2,processed:4,tracked:4,initialized_frames:2,fps:2,
 tracking_ms:{p95:30},metadata:{stop_reason:'qr_stop',video_overwrites:1},
 points:[[0,0,0,0,false,1],[.1,1,0,0,false,1],[1,5,2,3,true,2],[1.1,6,2,3,true,2]],
 metrics:{accepted_loop_closures:0,active_map_resets:1,imu_initialized_at_end:true,initialization_time_s:1}};
function run(data){
 const callbacks={},elements=new Map(),lines=[],messages=[];
 const context=new Proxy({}, {get(target,key){if(key in target)return target[key];return (...args)=>{
   if(['moveTo','lineTo','fillRect','arc'].includes(key))assert(args.every(Number.isFinite),'Nonfinite plot coordinate');
   if(key==='lineTo')lines.push(args);if(key==='fillText')messages.push(args[0]);
 }},set(target,key,value){target[key]=value;return true}});
 const canvas={clientWidth:1000,clientHeight:540,width:0,height:0,getContext:()=>context,
   addEventListener:(key,fn)=>callbacks[key]=fn,setPointerCapture:()=>{},toDataURL:()=> 'data:image/png;base64,fixture'};
 function element(id){if(id==='plot')return canvas;if(!elements.has(id))elements.set(id,{value:id==='source'?'online':'',textContent:'',appendChild(){},replaceChildren(){}});return elements.get(id)}
 const sandbox={console,Math,Number,window:{devicePixelRatio:1,addEventListener(){}},
   document:{querySelector:()=>canvas,getElementById:element,createElement:()=>({appendChild(){},click(){}})},
   fetch:()=>{throw Error('Offline report attempted network')},setTimeout(){throw Error('Offline report attempted polling')}};
 const script=page.match(/<script>([\s\S]*)<\/script>/)[1].replace('/*VIO_REPORT*/null',JSON.stringify(data));
 vm.runInNewContext(script,sandbox,{timeout:1000});
 return {callbacks,elements,lines,messages};
}
let result=run(report);
assert.strictEqual(result.elements.get('phase').textContent,'COMPLETE');
assert.strictEqual(result.elements.get('coverage').textContent,'4 / 4');
assert.strictEqual(result.lines.length,5,'Expected 3 axes + 2 segment lines, with no false line across reset');
result.callbacks.pointerdown({clientX:100,clientY:100,pointerId:1});
result.callbacks.pointermove({clientX:120,clientY:110});result.callbacks.pointerup();
result.callbacks.wheel({deltaY:100,preventDefault(){}});
result=run({...report,points:[]});assert(result.messages.some(x=>x.includes('No tracked poses')));
console.log('PASS: offline 3D plot, finite geometry, reset gaps, rotation/zoom and empty trajectory');
