const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8'),script=html.match(/<script>([\s\S]*)<\/script>/)[1];
const draws=[],elements=new Map(),uniforms={};let vertexCount=0;
const gl=new Proxy({VERTEX_SHADER:1,FRAGMENT_SHADER:2,COMPILE_STATUS:3,LINK_STATUS:4,ARRAY_BUFFER:5,STATIC_DRAW:6,POINTS:7,LINE_STRIP:8,FLOAT:9,DEPTH_TEST:10,COLOR_BUFFER_BIT:16,DEPTH_BUFFER_BIT:32,
 createShader:()=>({}),createProgram:()=>({}),createBuffer:()=>({}),getShaderParameter:()=>true,getProgramParameter:()=>true,getUniformLocation:(_,n)=>n,getAttribLocation:(_,n)=>n==='p'?0:1,
 bufferData:(_,data)=>{assert([...data].every(Number.isFinite));vertexCount+=data.length;},uniform3fv:(n,a)=>{assert(a.length===3&&a.every(Number.isFinite));uniforms[n]=a;},uniform1f:(_,v)=>assert(Number.isFinite(v)),drawArrays:(mode,_,n)=>{assert(n>0);draws.push([mode,n]);}}, {get:(o,k)=>k in o?o[k]:(()=>{})});
function element(id){if(!elements.has(id))elements.set(id,{checked:true,textContent:'',clientWidth:1000,clientHeight:700,getContext:()=>gl,setPointerCapture:()=>{}});return elements.get(id);}
const context=vm.createContext({document:{getElementById:element},window:{},devicePixelRatio:1,console});
vm.runInContext(script,context);assert(draws.length===2&&vertexCount>0);
const initialEye=uniforms.eye.slice();element('scene').onpointerdown({clientX:20,clientY:20,pointerId:1});element('scene').onpointermove({clientX:45,clientY:25,shiftKey:false});element('scene').onpointerup();assert(initialEye.some((v,k)=>v!==uniforms.eye[k]));
element('scene').onwheel({deltaY:100,preventDefault(){}});element('top').onclick();element('fit').onclick();
element('points').checked=false;const before=draws.length;element('points').onchange();assert(draws.length===before+1&&draws.at(-1)[0]===gl.LINE_STRIP);
assert(element('stats').textContent.includes('map voxels'));assert(!/https?:\/\//.test(html));
console.log('PASS: saved-data viewer finite buffers/camera uniforms, orbit/zoom/top/fit and layer controls under mocked WebGL; no external resources. Actual GPU rendering not tested.');
