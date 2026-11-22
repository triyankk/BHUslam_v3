const fs=require('fs'),os=require('os'),path=require('path'),cp=require('child_process'),assert=require('assert');
const root=fs.mkdtempSync(path.join(os.tmpdir(),'cube-bin-review-'));
function fmt(id,len,name,types,columns){const b=Buffer.alloc(89);b.set([0xa3,0x95,128,id,len]);b.write(name,5,4);b.write(types,9,16);b.write(columns,25,64);return b;}
function visp(){const b=Buffer.alloc(8+8+4+32+3+3);b.set([0xa3,0x95,200]);b.writeBigUInt64LE(1000000n,3);b.writeBigUInt64LE(123456789n,11);b.writeUInt32LE(1000,19);b[55]=7;b[56]=0;b[57]=65;return b;}
function source(ss){const b=Buffer.alloc(13);b.set([0xa3,0x95,201]);b.writeBigUInt64LE(1000000n,3);b[11]=0;b[12]=ss;return b;}
const binary=process.argv[2];assert(binary,'native binary argument');
try{
 const data=Buffer.concat([fmt(200,58,'VISP','QQIffffffffBBb','TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,R,P,Y,PErr,AErr,Rst,Ign,Q'),fmt(201,13,'XKFS','QBB','TimeUS,C,SS'),visp(),source(0)]);
 fs.writeFileSync(path.join(root,'good.BIN'),data);
 cp.execFileSync(binary,[path.join(root,'good.BIN'),path.join(root,'good')]);
 let r=JSON.parse(fs.readFileSync(path.join(root,'good/summary.json')));assert.equal(r.VISP,1);assert.equal(r.VISP_ignored,0);assert.equal(r.primary_source_observed_only,true);assert.equal(r.fusion_validated,false);assert.equal(r.cube_reception_proven,true);
 fs.writeFileSync(path.join(root,'switched.BIN'),Buffer.concat([data,source(1),Buffer.from([0xa3,0x95,200,0])]));
 cp.execFileSync(binary,[path.join(root,'switched.BIN'),path.join(root,'switched')]);
 r=JSON.parse(fs.readFileSync(path.join(root,'switched/summary.json')));assert.equal(r.primary_source_observed_only,false);assert.equal(r.truncated_tail,true);assert.equal(r.source_set_counts_zero_based['1'],1);
 console.log('PASS: Cube BIN reception, ignored/quality fields, source selection and truncated tail');
}finally{fs.rmSync(root,{recursive:true,force:true});}
