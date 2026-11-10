/* Robot kinematics and gait generation. No DOM, dependencies, or build step. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.RobotCore=api;})(globalThis,function(){
'use strict';
const L=140, SPLAY=18, RAD=Math.PI/180;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)), mod=(x,n=1)=>((x%n)+n)%n;
const add=(a,b)=>a.map((v,i)=>v+b[i]), subtract=(a,b)=>a.map((v,i)=>v-b[i]);
const rotateX=(v,a)=>[v[0],v[1]*Math.cos(a)-v[2]*Math.sin(a),v[1]*Math.sin(a)+v[2]*Math.cos(a)];
const rotateZ=(v,a)=>[v[0]*Math.cos(a)-v[1]*Math.sin(a),v[0]*Math.sin(a)+v[1]*Math.cos(a),v[2]];
function defaults(){return {shape:'rectangle',count:6,width:210,length:270,radius:145,height:225,stride:60,lift:25,period:2.5,gait:'tripod'};}
function mounts(config,previous=[]){
  return Array.from({length:config.count},(_,i)=>{
    const angle=2*Math.PI*(i+0.5)/config.count, sin=Math.sin(angle), cos=Math.cos(angle);
    const x=config.shape==='circle'?config.radius*sin:Math.sign(sin||1)*config.width/2;
    const z=config.shape==='circle'?config.radius*cos:cos*config.length/2;
    return {id:i+1,x,z,side:x>=0?1:-1,dof:previous[i]?.dof??3,enabled:previous[i]?.enabled??true};
  });
}
function gaitChoices(n){
  const choices=[{id:'wave',label:'Wave · one leg at a time'}];
  if(n===4)choices.unshift({id:'trot',label:'Trot · diagonal pairs'},{id:'pace',label:'Pace · left / right'},{id:'bound',label:'Bound · front / rear'});
  else if(n===6)choices.unshift({id:'tripod',label:'Tripod · alternating threes'},{id:'ripple',label:'Ripple · three pairs'});
  else if(n>=2&&n%2===0)choices.unshift({id:'alternating',label:'Alternating groups'});
  return choices;
}
function gaitSpec(name,n,index,leg){
  if(name==='wave')return {duty:n>1?1-0.8/n:0.65,offset:index/Math.max(1,n)};
  if(name==='ripple')return {duty:0.78,offset:(index%3)/3};
  if(name==='pace')return {duty:0.64,offset:leg.side>0?0:0.5};
  if(name==='bound')return {duty:0.64,offset:leg.z>=0?0:0.5};
  return {duty:0.64,offset:(index%2)*0.5};
}
function fixedSide(leg){return 90+leg.side*SPLAY;}
function fk(pose){
  const a=(pose.hip-90)*RAD,b=(90-pose.knee)*RAD,l=-(pose.hipSide-90)*RAD;
  const knee=rotateZ(rotateX([0,L,0],a),l);
  return {hip:[0,0,0],knee,foot:add(knee,rotateZ(rotateX([0,L,0],a+b),l))};
}
function ik(target,leg){
  let [x,y,z]=target;
  const lateral=leg.dof===2?leg.side*SPLAY*RAD:Math.atan2(x,y);
  const sagittalY=leg.dof===2?y/Math.cos(lateral):Math.hypot(x,y);
  const distance=Math.hypot(sagittalY,z);
  const bend=Math.acos(clamp((distance*distance-2*L*L)/(2*L*L),-1,1));
  const hip=Math.atan2(z,sagittalY)-Math.atan2(L*Math.sin(bend),L+L*Math.cos(bend));
  const raw={hip:90+hip/RAD,hipSide:90+lateral/RAD,knee:90-bend/RAD};
  const pose=Object.fromEntries(Object.entries(raw).map(([k,v])=>[k,clamp(v,0,180)]));
  const actual=fk(pose).foot, error=Math.hypot(...subtract(actual,target));
  return {pose,actual,error,reachable:error<0.001&&distance<=2*L+1e-6};
}
function footTarget(config,leg,phase,spec){
  const stance=phase<spec.duty;
  let z,y=config.height;
  if(stance)z=config.stride*(0.5-phase/spec.duty);
  else{
    const u=(phase-spec.duty)/(1-spec.duty);
    z=config.stride*(-0.5+u*u*(3-2*u));
    y-=config.lift*Math.sin(Math.PI*u)**2;
  }
  const x=(leg.dof===2?y:config.height)*Math.tan(SPLAY*RAD)*leg.side;
  return {point:[x,y,z],stance};
}
function gaitFrame(config,legs,time,travel=true){
  const active=legs.filter(l=>l.enabled), duty=gaitSpec(config.gait,active.length,0,active[0]||{side:1,z:1}).duty;
  const speed=config.stride/(duty*config.period);
  const body=[0,-config.height,travel?time*speed:0];
  const frames=active.map((leg,i)=>{
    const spec=gaitSpec(config.gait,active.length,i,leg),phase=mod(time/config.period+spec.offset);
    const target=footTarget(config,leg,phase,spec), solved=ik(target.point,leg);
    const local=fk(solved.pose), mount=add(body,[leg.x,0,leg.z]);
    return {leg,phase,duty:spec.duty,stance:target.stance,pose:solved.pose,reachable:solved.reachable,error:solved.error,
      hip:mount,knee:add(mount,local.knee),foot:add(mount,local.foot)};
  });
  return {body,legs:frames,speed};
}
function restPose(config,leg){return ik([config.height*Math.tan(SPLAY*RAD)*leg.side,config.height,0],leg).pose;}
function validate(config,legs){
  if(!legs.some(l=>l.enabled))return 'Attach at least one leg to start.';
  if(!gaitChoices(legs.filter(l=>l.enabled).length).some(g=>g.id===config.gait))return 'Choose a gait for the current leg count.';
  for(let i=0;i<64;i++){
    const frame=gaitFrame(config,legs,i*config.period/64);
    const bad=frame.legs.find(l=>!l.reachable);
    if(bad)return 'L'+bad.leg.id+' cannot reach this stride/lift with 0–180° servos. Reduce stride or lift, or adjust body height.';
  }
  return '';
}
function hull(points){
  const sorted=points.map(p=>[p[0],p[2]]).sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
  const unique=sorted.filter((p,i)=>i===0||Math.hypot(p[0]-sorted[i-1][0],p[1]-sorted[i-1][1])>1e-6);
  if(unique.length<3)return unique.map(p=>[p[0],0,p[1]]);
  const cross=(a,b,c)=>(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
  const low=[],up=[];
  for(const p of unique){while(low.length>=2&&cross(low.at(-2),low.at(-1),p)<=0)low.pop();low.push(p);}
  for(const p of [...unique].reverse()){while(up.length>=2&&cross(up.at(-2),up.at(-1),p)<=0)up.pop();up.push(p);}
  low.pop();up.pop();return low.concat(up).map(p=>[p[0],0,p[1]]);
}
function supported(polygon,point){
  if(polygon.length<3)return false;
  let sign=0;
  for(let i=0;i<polygon.length;i++){
    const a=polygon[i],b=polygon[(i+1)%polygon.length];
    const c=(b[0]-a[0])*(point[2]-a[2])-(b[2]-a[2])*(point[0]-a[0]);
    if(Math.abs(c)<1e-7)continue;
    if(sign&&Math.sign(c)!==sign)return false;sign=Math.sign(c);
  }
  return true;
}
function jointKeys(leg){return leg.dof===3?['hip','hipSide','knee']:['hip','knee'];}
function servoName(leg,key,single=false){return single?key:'leg'+leg.id+({hip:'Hip',hipSide:'Side',knee:'Knee'}[key]);}
function mappings(legs,selected,scope){
  const selectedLegs=legs.filter(l=>l.enabled&&(scope==='all'||l.id===selected)), map={};
  for(const leg of selectedLegs)for(const key of jointKeys(leg))map[servoName(leg,key,scope==='one')]=leg.id+'.'+key;
  return map;
}
function flatPose(config,legs){
  const values={};for(const leg of legs.filter(l=>l.enabled))for(const [key,value] of Object.entries(restPose(config,leg)))values[leg.id+'.'+key]=value;
  return values;
}
function gaitSketch(config,legs,selected,scope,samples=32,precision='pulse'){
  const error=validate(config,legs);if(error)throw Error(error);
  const targets=legs.filter(l=>l.enabled&&(scope==='all'||l.id===selected));
  if(!targets.length)throw Error('Select an attached leg first.');
  if(!Number.isInteger(samples)||samples<4||samples>32)throw Error('Use 4?32 samples per cycle.');
  if(!['pulse','degrees'].includes(precision))throw Error('Unknown servo precision.');
  const single=scope==='one',periodMs=Math.round(config.period*1000);
  const command=(name,angle)=>precision==='pulse'
    ? '  '+name+'.writeMicroseconds('+Math.round(544+angle*1856/180)+'); // '+angle.toFixed(2)+' degrees'
    : '  '+name+'.write('+Math.round(angle)+');';
  const lines=['// '+(single?'One-leg gait exercise':'Whole-robot '+config.gait+' gait'),'// Kinematic sample; calibrate servo directions and limits for your hardware.',
    '// Model mapping: default Servo range 544?2400 us corresponds to 0?180 degrees.',
    '// '+(precision==='pulse'?'Fine pulses reduce whole-degree rounding; physical servo response is not simulated.':'Whole-degree commands intentionally have coarser foot paths.'),
    '// Pins below are placeholders for a board with enough outputs (for example, Mega).','#include <Servo.h>'];
  for(const leg of targets)for(const key of jointKeys(leg))lines.push('Servo '+servoName(leg,key,single)+';');
  lines.push('','void setup() {');
  let pin=2;
  for(const leg of targets)for(const key of jointKeys(leg))lines.push('  '+servoName(leg,key,single)+'.attach('+(pin++)+');');
  const start=gaitFrame(config,legs,0).legs;
  for(const leg of targets){
    const pose=start.find(f=>f.leg.id===leg.id).pose;
    for(const key of jointKeys(leg))lines.push(command(servoName(leg,key,single),pose[key]));
  }
  lines.push('  delay(600);','}','','void loop() {');
  for(let i=1;i<=samples;i++){
    const frame=gaitFrame(config,legs,i*config.period/samples);
    lines.push('  // Cycle '+Math.round(i*100/samples)+'%');
    for(const leg of targets){
      const pose=frame.legs.find(f=>f.leg.id===leg.id).pose;
      for(const key of jointKeys(leg))lines.push(command(servoName(leg,key,single),pose[key]));
    }
    lines.push('  delay('+(Math.round(i*periodMs/samples)-Math.round((i-1)*periodMs/samples))+');');
  }
  lines.push('}');return lines.join('\n')+'\n';
}
function oneLegSketch(config,leg){
  if(!leg||!leg.enabled)throw Error('Select an attached leg first.');
  const lines=['// Test one leg before coordinating the robot.','#include <Servo.h>'];
  for(const key of jointKeys(leg))lines.push('Servo '+key+';');
  lines.push('','void setup() {');
  let pin=2;for(const key of jointKeys(leg))lines.push('  '+key+'.attach('+(pin++)+');');
  const rest=restPose(config,leg);
  for(const key of jointKeys(leg))lines.push('  '+key+'.write('+Math.round(rest[key])+');');
  lines.push('  delay(600);','}','','void loop() {');
  for(const [z,lift,label] of [[0,config.lift,'Lift'],[config.stride/2,config.lift,'Reach forward'],[config.stride/2,0,'Plant'],[-config.stride/2,0,'Push backward'],[0,0,'Center']]){
    const y=config.height-lift,x=(leg.dof===2?y:config.height)*Math.tan(SPLAY*RAD)*leg.side;
    const solved=ik([x,y,z],leg);if(!solved.reachable)throw Error('This one-leg exercise is out of reach; reduce stride/lift.');
    lines.push('  // '+label);
    for(const key of jointKeys(leg))lines.push('  '+key+'.write('+Math.round(solved.pose[key])+');');
    lines.push('  delay(350);');
  }
  lines.push('}');return lines.join('\n')+'\n';
}
return {L,SPLAY,clamp,mod,add,subtract,defaults,mounts,gaitChoices,gaitSpec,fixedSide,fk,ik,footTarget,gaitFrame,restPose,validate,hull,supported,jointKeys,servoName,mappings,flatPose,gaitSketch,oneLegSketch};
});
