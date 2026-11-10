const {test}=require('node:test');
const assert=require('node:assert/strict');
const C=require('../robot-core.js'), A=require('../robot-arduino.js');
const near=(a,b,e=1e-7)=>assert.ok(Math.abs(a-b)<e,a+' != '+b);
const vec=(a,b,e=1e-7)=>b.forEach((v,i)=>near(a[i],v,e));
function setup(count=6,shape='rectangle',dof=3){
  const c={...C.defaults(),count,shape},legs=C.mounts(c).map(l=>({...l,dof}));
  c.gait=C.gaitChoices(count)[0].id;return {c,legs};
}
test('mounts stay on the rectangular sides or circular perimeter',()=>{
  for(const shape of ['rectangle','circle'])for(let count=2;count<=8;count++){
    const {c,legs}=setup(count,shape);assert.equal(legs.length,count);
    for(const l of legs){if(shape==='circle')near(Math.hypot(l.x,l.z),c.radius);else{near(Math.abs(l.x),c.width/2);assert.ok(Math.abs(l.z)<=c.length/2);}}
    assert.equal(new Set(legs.map(l=>l.id)).size,count);
  }
});
test('mount edits retain per-leg attachment and DOF configuration',()=>{
  const {c,legs}=setup();legs[2].dof=2;legs[4].enabled=false;c.shape='circle';
  const next=C.mounts(c,legs);assert.equal(next[2].dof,2);assert.equal(next[4].enabled,false);
});
test('joint signs match the single-leg simulator',()=>{
  vec(C.fk({hip:90,hipSide:90,knee:90}).foot,[0,280,0]);
  vec(C.fk({hip:180,hipSide:90,knee:90}).foot,[0,0,280]);
  vec(C.fk({hip:90,hipSide:180,knee:90}).foot,[280,0,0]);
  vec(C.fk({hip:90,hipSide:90,knee:0}).foot,[0,140,140]);
});
test('all default robot sizes and available gaits have reachable trajectories',()=>{
  for(let n=2;n<=8;n++)for(const dof of [2,3])for(const shape of ['circle','rectangle']){
    const {c,legs}=setup(n,shape,dof);
    for(const gait of C.gaitChoices(n)){c.gait=gait.id;assert.equal(C.validate(c,legs),'',n+' '+dof+' '+gait.id);}
  }
});
test('inverse kinematics reproduces targets and preserves both link lengths',()=>{
  const {c,legs}=setup();legs[0].dof=2;
  for(let t=0;t<5;t+=0.02){
    for(const f of C.gaitFrame(c,legs,t).legs){
      assert.ok(f.reachable);
      near(Math.hypot(...C.subtract(f.knee,f.hip)),140);
      near(Math.hypot(...C.subtract(f.foot,f.knee)),140);
      if(f.stance)near(f.foot[1],0);
      else assert.ok(f.foot[1]<=1e-7);
    }
  }
});
test('stance feet remain planted in world coordinates throughout each contact',()=>{
  for(const gait of ['tripod','wave','ripple']){
    const {c,legs}=setup();c.gait=gait;let previous;
    for(let t=0;t<8;t+=0.013){
      const frame=C.gaitFrame(c,legs,t);
      if(previous)for(let i=0;i<legs.length;i++){
        const a=previous.legs[i],b=frame.legs[i];
        if(a.stance&&b.stance&&b.phase>=a.phase)vec(a.foot,b.foot,1e-6);
      }
      previous=frame;
    }
  }
});
test('swing trajectories are continuous at liftoff and touchdown',()=>{
  const {c,legs}=setup();
  for(const l of legs){
    const spec=C.gaitSpec(c.gait,legs.length,l.id-1,l);
    vec(C.footTarget(c,l,spec.duty-1e-9,spec).point,C.footTarget(c,l,spec.duty+1e-9,spec).point,1e-5);
    vec(C.footTarget(c,l,1-1e-9,spec).point,C.footTarget(c,l,0,spec).point,1e-5);
  }
});
test('2-DOF legs retain their fixed lateral bracket throughout the gait',()=>{
  const {c,legs}=setup(6,'circle',2);
  for(let t=0;t<3;t+=.03)for(const f of C.gaitFrame(c,legs,t).legs)near(f.pose.hipSide,C.fixedSide(f.leg));
});
test('3-DOF lateral joints keep the swing foot at its chosen outward offset',()=>{
  const {c,legs}=setup();
  for(let t=0;t<3;t+=.03)for(const f of C.gaitFrame(c,legs,t).legs)
    near(f.foot[0]-f.hip[0],c.height*Math.tan(C.SPLAY*Math.PI/180)*f.leg.side);
});
test('unreachable and detached-only designs report errors',()=>{
  const {c,legs}=setup();c.height=205;c.lift=45;assert.match(C.validate(c,legs),/cannot reach/);
  for(const l of legs)l.enabled=false;assert.match(C.validate(c,legs),/Attach/);
});
test('support polygon excludes interior points and classifies body projection',()=>{
  const h=C.hull([[-1,0,-1],[-1,0,1],[1,0,1],[1,0,-1],[0,0,0],[-1,0,-1]]);
  assert.equal(h.length,4);assert.equal(C.supported(h,[0,-225,0]),true);assert.equal(C.supported(h,[2,0,0]),false);
  assert.equal(C.supported(h,[1,0,0]),true);assert.equal(C.supported(C.hull([[0,0,0],[1,0,0],[2,0,0]]),[1,0,0]),false);
});
test('tripod, trot, pace, bound, and wave have the intended leg groups',()=>{
  const {legs}=setup();
  near(C.gaitSpec('tripod',6,0,legs[0]).offset,C.gaitSpec('tripod',6,2,legs[2]).offset);
  assert.notEqual(C.gaitSpec('tripod',6,0,legs[0]).offset,C.gaitSpec('tripod',6,1,legs[1]).offset);
  const q=setup(4).legs;
  near(C.gaitSpec('trot',4,0,q[0]).offset,C.gaitSpec('trot',4,2,q[2]).offset);
  near(C.gaitSpec('pace',4,0,q[0]).offset,C.gaitSpec('pace',4,1,q[1]).offset);
  near(C.gaitSpec('bound',4,0,q[0]).offset,C.gaitSpec('bound',4,3,q[3]).offset);
  for(let i=0;i<6;i++)near(C.gaitSpec('wave',6,i,legs[i]).offset,i/6);
});
test('disabled legs do not appear in frames or servo mappings',()=>{
  const {c,legs}=setup();legs[1].enabled=false;c.gait='wave';
  const frame=C.gaitFrame(c,legs,0);assert.equal(frame.legs.length,5);
  assert.ok(!Object.keys(C.mappings(legs,1,'all')).some(k=>k.startsWith('leg2')));
});
test('one-leg examples compile for both 2- and 3-DOF legs',()=>{
  for(const dof of [2,3]){
    const {c,legs}=setup(6,'rectangle',dof);
    const api=A.create(C.mappings(legs,1,'one'),C.flatPose(c,legs));
    const result=api.compile(C.oneLegSketch(c,legs[0]));
    assert.ok(result.setupEvents.length>0);assert.ok(result.loopEvents.length>0);
    assert.ok(result.loopEvents.filter(e=>e.type==='write').every(e=>e.role.startsWith('1.')));
    if(dof===2)assert.ok(!result.loopEvents.some(e=>e.role==='1.hipSide'));
  }
});
test('generated whole-robot sketches compile for every supported count and gait',()=>{
  for(let n=2;n<=8;n++){
    const {c,legs}=setup(n);legs[0].dof=2;
    for(const gait of C.gaitChoices(n)){
      c.gait=gait.id;
      const api=A.create(C.mappings(legs,1,'all'),C.flatPose(c,legs));
      const result=api.compile(C.gaitSketch(c,legs,1,'all'));
      assert.equal(result.loopEvents.filter(e=>e.type==='delay').length,32);
      assert.ok(result.loopEvents.filter(e=>e.type==='write').every(e=>e.value>=0&&e.value<=180));
      assert.ok(!result.loopEvents.some(e=>e.role==='1.hipSide'));
    }
  }
});
test('generated loop ends at the setup pose for seamless repetition',()=>{
  const {c,legs}=setup();const api=A.create(C.mappings(legs,1,'all'),C.flatPose(c,legs));
  const result=api.compile(C.gaitSketch(c,legs,1,'all'));
  const last=result.loopEvents.at(-1).pose;
  for(const key of Object.keys(last))near(last[key],result.initial[key]);
});
test('multi-leg parser preserves globals and independent targets across loops',()=>{
  const api=A.create({leg1Hip:'1.hip',leg2Knee:'2.knee'},{'1.hip':90,'2.knee':90});
  const result=api.compile('Servo leg1Hip;Servo leg2Knee;int n=90;void setup(){leg2Knee.write(70);}void loop(){n=n+1;leg1Hip.write(n);delay(10);}');
  assert.equal(result.loopEvents[0].value,91);
  const next=api.execute(result.program.loop,result.execution,'loop');assert.equal(next[0].value,92);assert.equal(next[0].pose['2.knee'],70);
});
test('wrong scope, missing lateral motor, and prototype names are rejected',()=>{
  const {c,legs}=setup(6,'circle',2);const api=A.create(C.mappings(legs,1,'one'),C.flatPose(c,legs));
  for(const name of ['hipSide','leg1Hip','toString','__proto__']){
    assert.throws(()=>api.compile('Servo '+name+';void setup(){}void loop(){'+name+'.write(90);}'),/not available/);
  }
});
test('parser rejects unsupported code instead of treating it as a gait',()=>{
  const api=A.create({hip:'1.hip'},{'1.hip':90});
  for(const text of ['Servo hip;void setup(){}void loop(){hip.write(90)}',
    'Servo hip;void setup(){}void loop(){hip.write(181);}',
    'Servo hip;void setup(){}void loop(){for(int i=0;i<4;i++){hip.write(90);}}',
    'Servo hip;void setup(){}void loop(){hip.write(90);}garbage()'])assert.throws(()=>api.compile(text));
});
test('holding the body still does not silently claim forward travel',()=>{
  const {c,legs}=setup();const frame=C.gaitFrame(c,legs,3,false);near(frame.body[2],0);
});
