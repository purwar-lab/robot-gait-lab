const {test}=require('node:test'),assert=require('node:assert/strict');
const C=require('../robot-core'),H=require('../robot-hardware'),A=require('../robot-arduino');
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-6,`${a} != ${b}`);
test('unequal link lengths agree between FK and IK in both leg types',()=>{
 for(const [thigh,shin] of [[100,180],[200,90],[170,160]])for(const dof of [2,3])for(const side of [-1,1]){
  const geometry={thigh,shin},leg={dof,side},pose={hip:65,hipSide:dof===2?C.fixedSide(leg):90+side*30,knee:40};
  const fk=C.fk(pose,geometry),ik=C.ik(fk.foot,leg,geometry);
  near(Math.hypot(...fk.knee),thigh);near(Math.hypot(...C.subtract(fk.foot,fk.knee)),shin);
  assert.ok(ik.reachable);ik.actual.forEach((v,i)=>near(v,fk.foot[i]));
 }
});
test('unreachable targets and off-plane 2-DOF targets report error',()=>{
 const leg={dof:3,side:1},geometry={thigh:80,shin:150};
 assert.equal(C.ik([0,500,0],leg,geometry).reachable,false);
 assert.equal(C.ik([0,0,0],leg,geometry).reachable,false);
 assert.equal(C.ik([0,220,0],{dof:2,side:1}).reachable,false);
});
test('custom dimensions affect gait, rest pose, and exported samples consistently',()=>{
 const config={...C.defaults(),thigh:170,shin:160,height:270},legs=C.mounts(config);
 assert.equal(C.validate(config,legs),'');
 for(const f of C.gaitFrame(config,legs,0).legs){near(Math.hypot(...C.subtract(f.knee,f.hip)),170);near(Math.hypot(...C.subtract(f.foot,f.knee)),160);}
 const api=A.create(C.mappings(legs,1,'one'),C.flatPose(config,legs));assert.doesNotThrow(()=>api.compile(C.gaitSketch(config,legs,1,'one')));
 const rest=C.fk(C.restPose(config,legs[0]),config);near(rest.foot[1],config.height);
});
test('calibration maps model neutral, reversal and physical limits correctly',()=>{
 const c=H.validate({pin:8,neutral:100,direction:-1,min:15,max:170});
 near(H.physical(90,c),100);near(H.physical(120,c),70);
 assert.throws(()=>H.validate({...c,min:110}));assert.throws(()=>H.validate({...c,pin:2.5}));assert.throws(()=>H.validate({...c,direction:0}));
 assert.match(H.poseError({hip:0,knee:90},{id:1,dof:2},{'1.hip':c}),/outside/);
});
function fixture(source,calibration={}){
 const config=C.defaults(),legs=C.mounts(config),mapping=C.mappings(legs,1,'one'),result=A.create(mapping,C.flatPose(config,legs)).compile(source);
 return H.hardwareSketch(source,mapping,calibration,result,config);
}
const source=`#include <Servo.h>
Servo hip; Servo hipSide; Servo knee;
int angle = 90;
void setup(){hip.attach(2); hipSide.attach(3); knee.attach(4);}
void loop(){hip.write((angle + 5)); hipSide.writeMicroseconds(1472); knee.write(90); delay(100); angle = angle + 1;}
`;
test('hardware export preserves loop expressions, remaps pins, and handles pulse commands',()=>{
 const output=fixture(source,{'1.hip':{pin:8,neutral:100,direction:-1,min:0,max:180}});
 assert.match(output,/hip.attach\(8\)/);assert.match(output,/writeCalibrated\(hip, \(angle \+ 5\), 100, -1, 0, 180\)/);
 assert.match(output,/writeCalibrated\(hipSide, \(\(1472\) - 544.0\) \* 180.0 \/ 1856.0/);
 assert.match(output,/angle = angle \+ 1/);assert.equal((output.match(/#include <Servo.h>/g)||[]).length,1);
 assert.match(output,/constrain\(neutral \+ direction/);
});
test('hardware export rejects duplicate pins and known limit violations',()=>{
 assert.throws(()=>fixture(source,{'1.hip':{pin:3,neutral:90,direction:1,min:0,max:180}}),/Duplicate pin/);
 assert.throws(()=>fixture(source,{'1.hip':{pin:8,neutral:90,direction:1,min:85,max:92}}),/exceeds/);
});
test('hardware transformation leaves comments unchanged',()=>{
 const output=fixture(source+'\n// hip.write(180);\n/* knee.attach(99); */');
 assert.match(output,/\/\/ hip.write\(180\)/);assert.match(output,/\/\* knee.attach\(99\)/);
});

test('custom mounts survive dimension changes and can return to automatic placement',()=>{
 const config=C.defaults(),legs=C.mounts(config);Object.assign(legs[0],{customMount:true,x:-120,z:70});
 const changed=C.mounts({...config,width:300},legs);assert.equal(changed[0].x,-120);assert.equal(changed[0].side,-1);assert.equal(changed[0].z,70);
 changed[0].customMount=false;assert.equal(C.mounts({...config,width:300},changed)[0].x,150);
});
