const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const path=require('node:path'),C=require('../robot-core'),A=require('../robot-arduino'),M=require('../robot-motion');
const app=fs.readFileSync(path.join(__dirname,'..','robot-app.js'),'utf8');
const contactSource=app.slice(app.indexOf('function poseFrame('),app.indexOf('function recordTrails('));
const playbackSource=app.slice(app.indexOf('function applyCodePose('),app.indexOf('// Perspective camera'));
function playback(precision='pulse',count=4){
  const config={...C.defaults(),count,gait:count===4?'trot':'tripod'},legs=C.mounts(config),start=C.gaitFrame(config,legs,0),initial={};
  for(const f of start.legs)for(const [k,v]of Object.entries(f.pose))initial[f.leg.id+'.'+k]=v;
  const parser=A.create(C.mappings(legs,1,'all'),initial),result=parser.compile(C.gaitSketch(config,legs,1,'all',32,precision));
  const env={C,RobotMotion:M,config,legs,poses:new Map(start.legs.map(f=>[f.leg.id,{...f.pose}])),body:[0,-config.height,0],anchors:new Map(),frame:null,running:true,
    runner:{api:parser,program:result.program,env:result.execution,events:result.setupEvents.concat(result.loopEvents),
      index:0,elapsed:0,from:null,target:{...initial},scope:'all',fine:precision==='pulse',phase:'setup'},
    active:()=>legs,$:()=>({checked:true,textContent:''})};
  vm.createContext(env);
  vm.runInContext([
    'function stopMotion(){running=false;}',
    'function notice(text){throw Error(text);}',
    'function currentFlat(){const out={};for(const[id,p]of poses)for(const[k,v]of Object.entries(p))out[id+"."+k]=v;return out;}',
    'function fromFlat(v){for(const l of legs)poses.set(l.id,{hip:v[l.id+".hip"],hipSide:v[l.id+".hipSide"],knee:v[l.id+".knee"]});}'
  ].join('\n')+contactSource+playbackSource,env);
  return env;
}
function advance(env,total,step){for(let elapsed=0;elapsed<total;elapsed+=step)env.advanceCode(Math.min(step,total-elapsed));}
function metrics(precision,count=4){
  const e=playback(precision,count),dt=1000/60;
  let previous=null,priorDz=0,sumY=0,samples=0,speedVariation=0,backwards=0,slip=0;
  for(let i=0;i<900;i++){
    e.advanceCode(dt);const f=e.frame;
    if(i>60){
      const ideal=C.gaitFrame(e.config,e.legs,((i+1)*dt-600)/1000),dz=f.body[2]-previous.body[2];
      if(dz<-.01)backwards++;
      speedVariation+=Math.abs(dz-priorDz);priorDz=dz;slip=Math.max(slip,f.slip||0);
      f.legs.forEach((leg,j)=>{if(ideal.legs[j].phase>.05&&ideal.legs[j].phase<.59){sumY+=leg.foot[1]**2;samples++;}});
    }
    previous=f;
  }
  return {travel:e.body[2],groundRMS:Math.sqrt(sumY/samples),speedVariation:speedVariation/839,backwards,maxSlip:slip};
}
test('sample interpolation has constant speed instead of restarting ease-in/ease-out',()=>{
  const from={a:10,b:100},to={a:30,b:50};let previous=M.interpolate(from,to,0);
  for(let i=1;i<=10;i++){const next=M.interpolate(from,to,i/10);assert.ok(Math.abs(next.a-previous.a-2)<1e-12);previous=next;}
});
test('incompatible historical anchors cannot snap the body when contacts change',()=>{
  const feet=[{id:1,local:[-10,225,0]},{id:2,local:[10,225,0]}],body=[0,-225,100];
  const previous=new Map(feet.map(f=>[f.id,{local:[...f.local],anchor:[f.local[0]+30,0,180]}]));
  const result=M.contactStep(feet,body,previous);assert.deepEqual(result.body,body);assert.ok(result.slip>50);
});
test('retained feet produce incremental travel and stable world contacts',()=>{
  const body=[0,-225,0],feet=[{id:1,local:[-50,225,0]},{id:2,local:[50,225,0]}];
  const first=M.contactStep(feet,body),next=M.contactStep(feet.map(f=>({...f,local:[f.local[0],225,-2]})),body,first.contacts);
  assert.deepEqual(next.body,[0,-225,2]);assert.equal(next.slip,0);
});
test('contact hysteresis avoids chatter near the entry tolerance',()=>{
  const f=y=>[{id:1,local:[0,y,0]}],first=M.contactStep(f(227),[0,-225,0]);
  assert.equal(first.contacts.size,1);
  const retained=M.contactStep(f(229),[0,-225,0],first.contacts);assert.equal(retained.contacts.size,1);
  assert.equal(M.contactStep(f(231),[0,-225,0],retained.contacts).contacts.size,0);
});
test('newly added contacts do not correct accumulated historical slip',()=>{
  const body=[0,-225,30],feet=[{id:1,local:[-50,225,0]},{id:2,local:[50,225,0]}];
  const start=M.contactStep(feet,body);
  const next=M.contactStep([...feet,{id:3,local:[0,225,100]}],body,start.contacts);assert.deepEqual(next.body,body);
});
test('pulse commands retain fractional angles while degree commands remain integer',()=>{
  const api=A.create({hip:'1.hip'},{'1.hip':90});
  const p=api.compile('Servo hip;void setup(){hip.attach(2);}void loop(){hip.writeMicroseconds(1500);hip.write(108.9);delay(10);}');
  assert.ok(Math.abs(p.loopEvents[0].value-(1500-544)*180/1856)<1e-12);assert.equal(p.loopEvents[1].value,108);
  for(const us of [543,2401])assert.throws(()=>api.compile('Servo hip;void setup(){}void loop(){hip.writeMicroseconds('+us+');}'),/544/);
  assert.throws(()=>api.compile('Servo hip;void setup(){hip.attach(2,1000,2000);}void loop(){hip.writeMicroseconds(1500);}'),/unsupported/);
});
test('exported cycle duration is exact and remains within interpreter statement limits',()=>{
  for(const count of [4,6,8])for(const precision of ['degrees','pulse'])for(const period of [1,1.25,2.5,5]){
    const c={...C.defaults(),count,period,gait:C.gaitChoices(count)[0].id},legs=C.mounts(c);
    const p=A.create(C.mappings(legs,1,'all'),C.flatPose(c,legs)).compile(C.gaitSketch(c,legs,1,'all',32,precision));
    assert.equal(p.loopEvents.reduce((n,e)=>n+e.ms,0),period*1000);
  }
});
test('setup positioning does not drag the whole robot over the ground',()=>{
  const e=playback();advance(e,599,17);assert.equal(e.body[0],0);assert.equal(e.body[2],0);
});
test('code poses and loop timing are independent of animation-frame partitioning',()=>{
  const a=playback(),b=playback();advance(a,6379,10);advance(b,6379,37);
  for(const [id,p]of a.poses)for(const k of Object.keys(p))assert.ok(Math.abs(p[k]-b.poses.get(id)[k])<1e-7,k);
});
test('fine generated trot greatly reduces ground and speed ripple without backwards shakes',()=>{
  const degree=metrics('degrees'),fine=metrics('pulse');
  console.log('Trot playback metrics:',JSON.stringify({degree,fine}));
  assert.ok(fine.groundRMS<.12);assert.ok(fine.groundRMS<degree.groundRMS*.25);
  assert.ok(fine.speedVariation<degree.speedVariation*.5);assert.equal(fine.backwards,0);assert.ok(fine.travel>500);
});
test('fine tripod playback remains stable over repeated cycles',()=>{
  const fine=metrics('pulse',6);console.log('Tripod playback metrics:',JSON.stringify(fine));
  assert.ok(fine.groundRMS<.12);assert.equal(fine.backwards,0);assert.ok(fine.travel>500);
});
