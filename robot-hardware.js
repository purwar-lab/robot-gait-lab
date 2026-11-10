/* Physical-servo calibration and Arduino hardware export. Model code stays unchanged. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.RobotHardware=api;})(globalThis,function(){
'use strict';
function entry(calibration,id,key){return calibration[id+'.'+key]||{pin:2+(id-1)*3+['hip','hipSide','knee'].indexOf(key),neutral:90,direction:1,min:0,max:180};}
function validate(c){
  if(!Number.isInteger(c.pin)||c.pin<0||c.pin>69)throw Error('Pin must be a digital pin number from 0 to 69; check your board.');
  if(![c.neutral,c.min,c.max].every(Number.isFinite)||c.min<0||c.max>180||c.min>=c.max||c.neutral<c.min||c.neutral>c.max)throw Error('Use 0 <= min < max <= 180, with neutral inside the limits.');
  if(c.direction!==1&&c.direction!==-1)throw Error('Direction must be +1 or -1.');
  return c;
}
function physical(angle,c){return c.neutral+c.direction*(angle-90);}
function poseError(pose,leg,calibration){
  for(const key of leg.dof===3?['hip','hipSide','knee']:['hip','knee']){
    const c=validate(entry(calibration,leg.id,key)),command=physical(pose[key],c);
    if(command<c.min-1e-6||command>c.max+1e-6)return 'Leg '+leg.id+' '+key+': physical '+command.toFixed(1)+' degrees is outside '+c.min+' to '+c.max+'.';
  }
  return '';
}
function hardwareSketch(source,mapping,calibration,result,config){
  const pins=new Set();
  for(const [name,role] of Object.entries(mapping)){
    const [id,key]=role.split('.'),c=validate(entry(calibration,Number(id),key));
    if(pins.has(c.pin))throw Error('Duplicate pin '+c.pin+' in this sketch. Assign a separate pin to each servo.');pins.add(c.pin);
    for(const e of result.setupEvents.concat(result.loopEvents).filter(e=>e.type==='write'&&e.role===role)){
      const command=physical(e.value,c);
      if(command<c.min-0.001||command>c.max+0.001)throw Error(name+' exceeds its calibrated limits on line '+e.line+'. Adjust the motion or calibration.');
    }
  }
  // The restricted parser validates syntax first. Replace calls in a same-length
  // comment mask, keeping source expressions and subsequent loop behavior intact.
  const masked=source.replace(/\/\*[\s\S]*?\*\//g,m=>' '.repeat(m.length)).replace(/\/\/[^\n]*/g,m=>' '.repeat(m.length));
  const edits=[],pattern=/\b([A-Za-z_]\w*)\s*\.\s*(attach|writeMicroseconds|write)\s*\(/g;
  let m;
  while((m=pattern.exec(masked))){
    if(!mapping[m[1]])throw Error('Unknown hardware servo '+m[1]);
    let end=pattern.lastIndex,depth=1;
    while(end<masked.length&&depth){if(masked[end]==='(')depth++;if(masked[end]===')')depth--;end++;}
    const arg=source.slice(pattern.lastIndex,end-1),[id,key]=mapping[m[1]].split('.'),c=entry(calibration,Number(id),key);
    const angle=m[2]==='writeMicroseconds'?'(('+arg+') - 544.0) * 180.0 / 1856.0':arg;
    const replacement=m[2]==='attach'?m[1]+'.attach('+c.pin+')':'writeCalibrated('+m[1]+', '+angle+', '+[c.neutral,c.direction,c.min,c.max].join(', ')+')';
    edits.push({start:m.index,end,replacement});pattern.lastIndex=end;
  }
  for(const e of edits.reverse())source=source.slice(0,e.start)+e.replacement+source.slice(e.end);
  const wiring=Object.entries(mapping).map(([name,role])=>{const[id,key]=role.split('.'),c=entry(calibration,Number(id),key);return '// '+name+': signal D'+c.pin+', neutral '+c.neutral+', direction '+c.direction+', limits '+c.min+'..'+c.max;}).join('\n');
  return '// Robot Gait Lab - calibrated hardware sketch\n// Thigh '+config.thigh+', shin '+config.shin+'; dimensions use one consistent unit.\n'+wiring+'\n// Use a servo-rated external supply and a common ground with Arduino.\n// Confirm board pins, servo capacity, and calibration before running.\n// Future loop values are clamped to calibrated command limits.\n// This hardware helper sketch is not accepted by the simulator parser.\n#include <Servo.h>\nvoid writeCalibrated(Servo &servo, float modelAngle, float neutral, int direction, float low, float high) {\n  float command = constrain(neutral + direction * (modelAngle - 90.0), low, high);\n  servo.writeMicroseconds((int)(544.0 + command * 1856.0 / 180.0 + 0.5));\n}\n\n'+source.replace(/^\s*#include\s*<Servo.h>\s*$/gm,'');
}
return {entry,validate,physical,poseError,hardwareSketch};
});
