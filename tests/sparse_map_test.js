const fs=require('fs'),vm=require('vm'),assert=require('assert');
const page=fs.readFileSync(process.argv[2],'utf8');
const pose=[0,0,0,0,0,0,1];
const map=(t,id)=>({t,map:id,state:2,initialized:false,image:'images/fixture.jpg',camera:pose,
 valid_landmarks:2,points:[[0,0,0,2],[1,2,3,3]],keys:[{id:1,t:t-1,pose},{id:2,t,pose:[1,0,0,0,0,0,1]}],edges:[[1,2,0],[1,2,2]],route:[[1,2]]});
const missing=t=>({t,map:null,state:3,initialized:false,image:'images/missing.jpg',points:[],keys:[],edges:[],route:[]});
const data={mode:'Stereo',inertial:false,maps:{'0:0':{label:'Map 0',last_s:2},'1:8':{label:'Map 1',last_s:6}},snapshots:[missing(0),map(2,'0:0'),missing(4),map(6,'1:8')]};
function run(data){
 const callbacks={},elements=new Map();let finiteChecks=0;
 const context=new Proxy({}, {get(target,key){if(key in target)return target[key];return(...args)=>{if(['moveTo','lineTo','fillRect'].includes(key)){assert(args.every(Number.isFinite),'Nonfinite renderer coordinate');++finiteChecks}}},set(target,key,value){target[key]=value;return true}});
 const canvas={clientWidth:1000,clientHeight:650,width:0,height:0,getContext:k=>k==='webgl'?null:context,addEventListener:(k,f)=>callbacks['canvas:'+k]=f,setPointerCapture(){},toDataURL:()=>''};
 const element=id=>{if(id==='scene')return canvas;if(!elements.has(id))elements.set(id,{value:id==='map'?'auto':'',checked:true,textContent:'',appendChild(){},addEventListener:(event,f)=>callbacks[id+':'+event]=f});return elements.get(id)};
 const sandbox={console,Math,Number,Float32Array,Map,window:{devicePixelRatio:1,addEventListener(){}},requestAnimationFrame(){},document:{getElementById:element,createElement:()=>({click(){}})},fetch(){throw Error('Viewer attempted network')}};
 vm.runInNewContext(page.match(/<script>([\s\S]*)<\/script>/)[1].replace('/*SPARSE_MAP_DATA*/null',JSON.stringify(data)),sandbox,{timeout:2000});return{elements,callbacks,finiteChecks};
}
const r=run(data);assert(r.finiteChecks>0);assert(r.elements.get('status').textContent.includes('Map 0:0'));
r.elements.get('time').value='4';r.callbacks['time:input']();assert(r.elements.get('status').textContent.includes('STALE MAP'));assert(r.elements.get('imageTime').textContent.includes('unavailable'));
r.elements.get('time').value='0';r.callbacks['time:input']();assert(r.elements.get('status').textContent.includes('No valid map'));
r.elements.get('map').value='1:8';r.callbacks['map:change']();assert(r.elements.get('status').textContent.includes('Map 1:8'));assert(!r.elements.get('status').textContent.includes('STALE'));
r.callbacks['canvas:pointerdown']({clientX:10,clientY:10,pointerId:1});r.callbacks['canvas:pointermove']({clientX:30,clientY:20});r.callbacks['canvas:pointerup']();r.callbacks['canvas:wheel']({deltaY:100,preventDefault(){}});
const empty=run({...data,maps:{},snapshots:[missing(0)]});assert(empty.elements.get('status').textContent.includes('No valid map'));
if(process.argv[3]){const real=JSON.parse(fs.readFileSync(process.argv[3],'utf8')),actual=run(real);assert(actual.finiteChecks>0);for(const id of Object.keys(real.maps)){actual.elements.get('map').value=id;actual.callbacks['map:change']();assert(actual.elements.get('status').textContent.includes('Map '+id))}actual.elements.get('map').value='auto';actual.elements.get('time').value=String(real.snapshots.at(-1).t);actual.callbacks['time:input']();console.log('PASS: real exported data render and all map selections')}
console.log('PASS: offline sparse-map geometry, tracking-loss stale label, independent map selection, orbit/zoom and empty map');
