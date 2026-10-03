// Run with Node.js: node tests/host/attitude_viewer_tests.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const htmlPath = process.argv[2] || path.join(__dirname, '../../tools/attitude-viewer.html');
const html = fs.readFileSync(htmlPath, 'utf8');
const scripts = [...html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)].map(m => m[1]);
const core = {module:{exports:{}}}; vm.runInNewContext(scripts[0], core);
const {parseAttitude,LineReader,rotate,quaternion} = core.module.exports;
const line = '# ATT seq=44841 t=45010654 q_u=999945/-3474/-9787/-1157 rpy_md=-397/-1122/-129 total_rpy_md=-397/-1122/359871 total_epoch=1 total_kind=body bias_u=-495/494/13 aw_m=953 dt_us=1000 run_us=73 lat_us=200 valid=1 yaw_abs=0 err=0';
const frame = parseAttitude(line);
let passed = 0;
async function test(name, fn) { await fn(); passed++; console.log('PASS '+name); }
const plain = value => JSON.parse(JSON.stringify(value));
const near = (actual, expected) => expected.forEach((v,i)=>assert.ok(Math.abs(actual[i]-v)<1e-9));
const flush = () => new Promise(resolve => setImmediate(resolve));
function page(serial) {
  let now=0, next=null;
  const draw = new Proxy({}, {get:(_,key)=>key==='setTransform' ? ()=>{} : ()=>{},set:()=>true});
  const ids = [...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]);
  const elements = Object.fromEntries(ids.map(id=>[id,{textContent:'',value:'0',max:'1',files:[],dataset:{},classList:{toggle(){}},getBoundingClientRect:()=>({width:700,height:400}),getContext:()=>draw,setPointerCapture(){}}]));
  const context = vm.createContext({document:{getElementById:id=>elements[id]},window:{isSecureContext:true,devicePixelRatio:1,addEventListener(){}},navigator:serial ? {serial} : {},performance:{now:()=>now},requestAnimationFrame:fn=>{next=fn},TextDecoder,console});
  for (const script of scripts) vm.runInContext(script,context);
  return {elements, context, state:expr=>vm.runInContext(expr,context),step:t=>{now=t;next(t);},setNow:t=>{now=t;}};
}
function mockPort() {
  const calls=[], queue=[]; let resolveRead=null;
  const streamReader = {
    read:()=>new Promise(resolve=>{if(queue.length) resolve(queue.shift());else resolveRead=resolve;}),
    cancel:async()=>{calls.push('cancel');if(resolveRead){resolveRead({done:true});resolveRead=null;}},
    releaseLock:()=>calls.push('release')
  };
  const p = {readable:{getReader:()=>{calls.push('reader');return streamReader;}},open:async()=>calls.push('open'),setSignals:async s=>calls.push(s.dataTerminalReady?'dtr-on':'dtr-off'),close:async()=>calls.push('close')};
  return {port:p,calls,push(text){const value={value:new TextEncoder().encode(text),done:false};if(resolveRead){resolveRead(value);resolveRead=null;}else queue.push(value);}};
}
function file(text) {
  let sent=false;
  return {name:'test.csv',stream:()=>({getReader:()=>({read:async()=>sent ? {done:true} : (sent=true,{done:false,value:new TextEncoder().encode(text)}),releaseLock(){}})})};
}
(async()=>{
  await test('ATT units and quaternion normalization',()=>{
    assert.deepEqual(plain(frame.rpy),[-.397,-1.122,-.129]);assert.equal(frame.t,45010654);assert.equal(frame.run_us,73);assert.ok(Math.abs(Math.hypot(...frame.q)-1)<1e-12);
  });
  await test('firmware totals replace wrapped angles in cards and chart',()=>{
    const p=page();p.state("mode='serial'");p.state('accept('+JSON.stringify(frame)+',0)');p.step(40);
    assert.equal(p.elements.yaw.textContent,'359.9°');assert.equal(p.state('history[0].angles[2]'),359.871);
    p.state('accept('+JSON.stringify({...plain(frame),t:frame.t+20000,total:[-.397,-1.122,360.129]})+',80)');p.step(100);
    assert.equal(p.elements.yaw.textContent,'360.1°');assert.equal(p.state('history.length'),2);
    p.state('accept('+JSON.stringify({...plain(frame),t:frame.t+40000,total:[0,0,0],epoch:2})+',120)');p.step(140);
    assert.equal(p.state('history.length'),1);assert.equal(p.elements.yaw.textContent,'0.0°');
  });
  await test('old logs show the model without fabricating totals',()=>{
    const old=line.replace(/ total_rpy_md=[^ ]+ total_epoch=1/,'');const a=parseAttitude(old);assert.equal(a.total,null);
    const p=page();p.state("mode='serial'");p.state('accept('+JSON.stringify(a)+',0)');p.step(40);
    assert.equal(p.elements.yaw.textContent,'—');assert.equal(p.state('history.length'),0);assert.match(p.elements.hint.textContent,/烧录新版/);
    assert.equal(parseAttitude(line.replace('359871','NaN')),null);
  });
  await test('legacy Euler totals are not mislabeled as body-axis rotation',()=>{
    const a=parseAttitude(line.replace(' total_kind=body',''));assert.equal(a.total,null);
    const p=page();p.state("mode='serial'");p.state('accept('+JSON.stringify(a)+',0)');p.step(40);
    assert.equal(p.elements.pitch.textContent,'—');assert.match(p.elements.hint.textContent,/total_kind=body/);
  });
  await test('pitch crosses vertical and two turns without Euler coupling',()=>{
    const p=page();p.state("mode='serial'");
    for(const [i,angle] of [89,90,91,180,360,720].entries()){
      const a={...plain(frame),t:i*20000,total:[0,angle,0],rpy:[180,90,180],q:plain(quaternion(.5,angle,0))};
      p.state('accept('+JSON.stringify(a)+','+(i*40)+')');p.step(i*40+30);
      assert.equal(p.elements.pitch.textContent,angle.toFixed(1)+'°');assert.equal(p.elements.roll.textContent,'0.0°');assert.equal(p.elements.yaw.textContent,'0.0°');
    }
    assert.equal(p.state('history.length'),6);
  });
  await test('every line split, CRLF and mixed IMU records',()=>{
    const text='IMU,G,1,2,3,4,5,6,7,0,1\r\n'+line+'\r\n# TEMP current_mc=32000 valid=1\n'+line+'\n';
    for (let size=1;size<=text.length;size++) {
      const found=[], reader=new LineReader(l=>{const a=parseAttitude(l);if(a)found.push(a);});
      for(let i=0;i<text.length;i+=size)reader.push(text.slice(i,i+size));
      assert.equal(found.length,2);
    }
  });
  await test('oversized incomplete line recovers with bounded tail',()=>{
    const found=[],reader=new LineReader(l=>{if(parseAttitude(l))found.push(l);});
    for(let i=0;i<30;i++){reader.push('x'.repeat(1000));assert.ok(reader.tail.length<=2048);}
    reader.push('\n'+line+'\n');assert.equal(found.length,1);
  });
  await test('truncated, NaN, zero and nonunit valid quaternions rejected',()=>{
    for(const bad of [line.slice(0,-5),line.replace('999945','NaN'),line.replace('999945/-3474/-9787/-1157','0/0/0/0'),line.replace('999945/-3474/-9787/-1157','3000000/0/0/0'),line.replace('valid=1','valid=2')])assert.equal(parseAttitude(bad),null);
    assert.equal(parseAttitude('IMU,A,1,2,3,4,5,6,7'),null);
  });
  await test('invalid zero quaternion remains an invalid diagnostic frame',()=>{
    const a=parseAttitude(line.replace('valid=1','valid=0').replace('999945/-3474/-9787/-1157','0/0/0/0'));assert.equal(a.valid,0);
  });
  await test('FRD positive roll, pitch, yaw and pitch 90 degrees',()=>{
    near(rotate(quaternion(90,0,0),[0,1,0]),[0,0,1]);
    near(rotate(quaternion(0,90,0),[1,0,0]),[0,0,-1]);
    near(rotate(quaternion(0,0,90),[1,0,0]),[0,1,0]);
    const v=rotate(quaternion(45,90,70),[1,0,0]);near(v,[0,0,-1]);
  });
  await test('invalid frames freeze model; 600 ms expires live data',()=>{
    const p=page();p.state("mode='serial'");p.state('accept('+JSON.stringify(frame)+',0)');p.step(40);assert.equal(p.elements.status.textContent,'实时 · 有效');
    p.state('accept('+JSON.stringify({...plain(frame),valid:0,q:[0,0,0,0],err:3})+',80)');p.step(100);assert.equal(p.elements.status.textContent,'实时 · 无效');assert.deepEqual(plain(p.state('good.q')),plain(frame.q));
    p.state('accept('+JSON.stringify(frame)+',120)');p.step(800);assert.equal(p.elements.status.textContent,'数据已断流');
  });
  await test('new device time clears old chart history',()=>{
    const p=page();p.state('accept('+JSON.stringify(frame)+',0)');p.state('accept('+JSON.stringify({...plain(frame),t:1000})+',40)');assert.equal(p.state('history.length'),1);assert.equal(p.state('history[0].time'),.001);
  });
  await test('USB DTR, chunk reading, cancel/release/close and reconnect',async()=>{
    let active=mockPort();const serial={requestPort:async()=>active.port,addEventListener(){}};const p=page(serial);
    await p.elements.connect.onclick();assert.deepEqual(active.calls,['open','dtr-on','reader']);
    active.push(line.slice(0,71));await flush();active.push(line.slice(71)+'\r\n');await flush();p.step(50);assert.equal(p.elements.status.textContent,'实时 · 有效');
    await p.elements.connect.onclick();assert.deepEqual(active.calls,['open','dtr-on','reader','cancel','release','dtr-off','close']);assert.equal(p.state('port'),null);p.step(100);assert.equal(p.elements.status.textContent,'已断开');
    active=mockPort();await p.elements.connect.onclick();active.push(line+'\n');await flush();p.step(150);assert.equal(p.state('count'),1);await p.elements.connect.onclick();
  });
  await test('port chooser cancellation preserves demo and unlocks controls',async()=>{
    const p=page({requestPort:async()=>{const e=new Error('cancel');e.name='NotFoundError';throw e;},addEventListener(){}});p.elements.demo.onclick();await p.elements.connect.onclick();assert.equal(p.state('mode'),'demo');assert.equal(p.elements.connect.disabled,false);
  });
  await test('DTR failure closes opened port',async()=>{
    const a=mockPort();a.port.setSignals=async()=>{throw new Error('DTR error');};const p=page({requestPort:async()=>a.port,addEventListener(){}});await p.elements.connect.onclick();assert.equal(a.calls.at(-1),'close');assert.equal(p.state('port'),null);
  });
  await test('read failure releases reader and closes port',async()=>{
    const a=mockPort();a.port.readable.getReader=()=>({read:async()=>{throw new Error('unplug');},releaseLock:()=>a.calls.push('release')});const p=page({requestPort:async()=>a.port,addEventListener(){}});await p.elements.connect.onclick();await flush();assert.deepEqual(a.calls,['open','dtr-on','release','dtr-off','close']);assert.equal(p.state('port'),null);
  });
  await test('file loading locks mode switches',async()=>{
    let resolve;const f={name:'test.csv',stream:()=>({getReader:()=>({read:()=>new Promise(r=>resolve=r),releaseLock(){}})})};const p=page();const pending=p.elements.logFile.onchange({target:{files:[f],value:'x'}});assert.equal(p.elements.connect.disabled,true);assert.equal(p.elements.demo.disabled,true);resolve({done:true});await pending;assert.equal(p.elements.connect.disabled,false);
  });
  await test('log replay, seeking and replay from end',async()=>{
    const p=page();const a=line.replace('t=45010654','t=1000000'),b=line.replace('t=45010654','t=2000000');
    await p.elements.logFile.onchange({target:{files:[file(a+'\n'+b+'\n')],value:'x'}});p.step(1200);assert.equal(p.elements.status.textContent,'回放结束');assert.equal(p.state('count'),2);
    p.elements.play.onclick();assert.equal(p.state('replayOffset'),0);p.step(1250);assert.equal(p.elements.status.textContent,'日志回放');assert.equal(p.state('count'),1);
    p.elements.replayBar.oninput({target:{value:'.5'}});assert.equal(p.state('replayOffset'),.5);
  });
  await test('no ATT and backward replay times are reported',async()=>{
    const p=page();await p.elements.logFile.onchange({target:{files:[file('IMU,G,1\n')],value:'x'}});assert.match(p.elements.hint.textContent,/未找到/);
    await p.elements.logFile.onchange({target:{files:[file(line+'\n'+line.replace('45010654','1000')+'\n')],value:'x'}});assert.match(p.elements.hint.textContent,/倒退时间/);
  });
  const logRoot=process.argv[3] || path.join(__dirname,'../../build/p3');
  if(fs.existsSync(logRoot)) await test('real P3 mixed USB recordings in arbitrary chunks',()=>{
    for(const [name,expected] of [['static',1001],['roll',400],['pitch',400],['yaw',3001]]){
      const text=fs.readFileSync(path.join(logRoot,'board-'+name+'.csv'),'utf8');let count=0,invalid=0;
      const reader=new LineReader(l=>{const a=parseAttitude(l);if(a){count++;if(!a.valid)invalid++;}});
      for(let i=0;i<text.length;i+=127)reader.push(text.slice(i,i+127));reader.push('\n');assert.equal(count,expected);assert.equal(invalid,0);
    }
  });
  console.log(passed+' tests passed');
})().catch(error=>{console.error(error);process.exitCode=1;});
