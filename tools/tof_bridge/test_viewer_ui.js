// Execute the actual viewer JavaScript against a real captured API snapshot.
// No browser packages or network access required.
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const path = require('path');
const elements = new Map();
function element() {
  const e = {style:{},children:[],textContent:'',value:'30',append(x){this.children.push(x)},
    getBoundingClientRect(){return {width:900,height:340}},click(){}};
  Object.defineProperty(e,'innerHTML',{set(){this.children=[element(),element()]}});
  return e;
}
function get(id){if(!elements.has(id))elements.set(id,element());return elements.get(id)}
get('chart').getContext=()=>new Proxy({}, {get:()=>()=>{},set:()=>true});
const fixture=JSON.parse(fs.readFileSync(process.argv[2] || path.join(__dirname,'../../reports/tof/live-viewer-test-01/api-snapshot.json')));
const context=vm.createContext({document:{getElementById:get,createElement:element},
  window:{devicePixelRatio:1,addEventListener(){}},performance:{now:()=>1000},
  structuredClone,Blob,URL,AbortSignal,setTimeout(){},
  fetch:async()=>{throw Error('Disconnected')},fixture});
const html=fs.readFileSync(path.join(__dirname,'viewer.html'),'utf8');
const source=html.split('<script>')[1].split('</script>')[0].replace(/\npoll\(\);\s*$/,'\n');
vm.runInContext(source,context);
vm.runInContext('update(fixture)',context);
assert.equal(get('calibration').textContent,'Valid');
assert.notEqual(get('sensor').textContent,'—');
if(fixture.sensor_only)assert.equal(get('cube').textContent,'—');
else assert.notEqual(get('cube').textContent,'—');
assert.equal(get('zones').children.length,9);
vm.runInContext('const sensorOnly=structuredClone(fixture);sensorOnly.sensor_only=true;sensorOnly.orientation="front";sensorOnly.latest.valid=true;sensorOnly.latest.transmitted=false;sensorOnly.cube=[];sensorOnly.cube_age_s=null;sensorOnly.heartbeat_age_s=null;update(sensorOnly)',context);
assert.notEqual(get('sensor').textContent,'—');
assert.equal(get('cube').textContent,'—');
assert.equal(get('status').textContent,'Live · receiving');
assert(get('connection').textContent.includes('FRONT · Sensor only'));
vm.runInContext('const mean=structuredClone(sensorOnly);mean.aggregation="zone-mean";mean.minimum_active_zones=4;mean.latest.active_zone_count=4;mean.latest.selected_mm=2500;update(mean)',context);
assert.equal(get('sensor').textContent,'2.50');
assert.equal(get('selectedLabel').textContent,'Average of active zones');
vm.runInContext('mean.latest.active_zone_count=3;mean.latest.selected_mm=null;mean.latest.valid=false;update(mean)',context);
assert.equal(get('sensor').textContent,'—');
assert(get('status').textContent.includes('Waiting for 4 active zones (3/9)'));
vm.runInContext('mean.latest.active_zone_count=5;mean.latest.selected_mm=3000;mean.latest.valid=true;update(mean)',context);
assert.equal(get('sensor').textContent,'3.00');
vm.runInContext('update(fixture)',context);
vm.runInContext('const proximity=structuredClone(fixture);proximity.orientation="rear";proximity.cube_report_kind="proximity";update(proximity)',context);
assert.equal(get('cubeLabel').textContent,'Cube rear proximity');
vm.runInContext('const disabled=structuredClone(fixture);disabled.sensor_only=false;disabled.cube_parameters={RNGFND2_TYPE:0};update(disabled)',context);
assert(get('banner').textContent.includes('rangefinder input is disabled'));
assert.notEqual(get('sensor').textContent,'—');
vm.runInContext('const directMavlink=structuredClone(fixture);directMavlink.cube_parameters={PRX1_TYPE:2,RNGFND2_TYPE:0};update(directMavlink)',context);
assert.equal(get('banner').style.display,'none');
vm.runInContext('const shared=structuredClone(fixture);shared.sensor_only=false;shared.transport="shared-usb";shared.uart=null;shared.baud=null;update(shared)',context);
assert(get('connection').textContent.includes('Cube USB · shared owner'));
assert(!get('connection').textContent.includes('detecting baud'));
vm.runInContext('const stale=structuredClone(fixture);stale.sensor_age_s=2;stale.cube_age_s=2;stale.heartbeat_age_s=4;update(stale)',context);
assert.equal(get('sensor').textContent,'—');
assert.equal(get('cube').textContent,'—');
assert.equal(get('rate').textContent,'—');
assert(get('zones').children.every(z=>z.children[0].textContent==='—'));
vm.runInContext('const invalid=structuredClone(fixture);invalid.latest.calibration_status=49;invalid.latest.transmitted=false;update(invalid)',context);
assert.equal(get('calibration').textContent,'Invalid');
assert.equal(get('sensor').textContent,'—');
assert(get('zones').children.every(z=>z.children[0].textContent==='—'));
vm.runInContext('update(fixture);paused=true;frozen=structuredClone(current);draw()',context);
vm.runInContext('poll()',context).then(()=>{
  assert.equal(get('status').textContent,'Viewer disconnected');
  assert.equal(get('sensor').textContent,'—');
  assert.equal(get('cube').textContent,'—');
  console.log('PASS: actual UI handles fresh data, nine zones, stale data, invalid calibration, chart pause, and disconnected API.');
}).catch(e=>{console.error(e);process.exitCode=1});
