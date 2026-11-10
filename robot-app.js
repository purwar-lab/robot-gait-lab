/* Robot Gait Lab browser controller and Canvas 3D renderer. */
(() => {
'use strict';
const C=RobotCore, $=id=>document.getElementById(id);
let config=C.defaults(), legs=C.mounts(config), selected=1, mode='gait', running=false, time=0;
let frame=C.gaitFrame(config,legs,0), poses=new Map(frame.legs.map(f=>[f.leg.id,{...f.pose}]));
let body=[...frame.body], runner=null, checked=null, anchors=new Map(), trails=new Map(), lastTime=0, codeSelection=null;
const canvas=$('robotScene'), ctx=canvas.getContext('2d');
let width=900,height=510,dpr=1;
const homeCamera={yaw:-0.7,pitch:0.62,zoom:1,panX:0,panY:0,perspective:true};
let camera={...homeCamera}, picks=[], reachError=C.validate(config,legs);
const colors={thigh:'#158c90',shin:'#8064ae',stance:'#239d7c',swing:'#f2a252',selected:'#e4b442',bad:'#d06153'};
const active=()=>legs.filter(l=>l.enabled), selectedLeg=()=>legs.find(l=>l.id===selected);
function notice(text,type=''){ $('notice').textContent=text;$('notice').className='notice'+(type?' '+type:''); }
function currentFlat(){
  const out={};for(const leg of active())for(const [key,value] of Object.entries(poses.get(leg.id)||C.restPose(config,leg)))out[leg.id+'.'+key]=value;return out;
}
function fromFlat(values){
  for(const leg of active()){
    const pose={...poses.get(leg.id)};
    for(const key of C.jointKeys(leg))pose[key]=values[leg.id+'.'+key];
    if(leg.dof===2)pose.hipSide=C.fixedSide(leg);
    poses.set(leg.id,pose);
  }
}
function stopMotion(){running=false;lastTime=0;$('pauseMotion').disabled=true;$('pauseMotion').textContent='Pause';}
function resetMotion(){
  stopMotion();time=0;runner=null;checked=null;anchors.clear();trails.clear();mode='gait';reachError=C.validate(config,legs);
  frame=C.gaitFrame(config,legs,0,$('moveBody').checked);body=[...frame.body];
  poses=new Map(frame.legs.map(f=>[f.leg.id,{...f.pose}]));
  $('codeLine').textContent='No sketch running';sync();render();
}
function rebuildMounts(){
  legs=C.mounts(config,legs);
  if(!active().some(l=>l.id===selected))selected=active()[0]?.id??legs[0].id;
  refreshGaits();renderMounts();resetMotion();scopeHelp();
  notice('Design updated. Existing sketch text is preserved; check its servo names before running.');
}
function refreshGaits(){
  const choices=C.gaitChoices(active().length);
  if(!choices.some(g=>g.id===config.gait))config.gait=choices[0].id;
  $('gaitType').replaceChildren(...choices.map(g=>{const o=document.createElement('option');o.value=g.id;o.textContent=g.label;return o;}));
  $('gaitType').value=config.gait;
}
function renderMounts(){
  $('mountList').replaceChildren(...legs.map(leg=>{
    const row=document.createElement('div');row.className='mount'+(leg.id===selected?' selected':'');
    const check=document.createElement('input');check.type='checkbox';check.checked=leg.enabled;check.setAttribute('aria-label','Attach leg '+leg.id);
    check.addEventListener('change',()=>{
      leg.enabled=check.checked;
      if(!active().some(l=>l.id===selected))selected=active()[0]?.id??leg.id;
      refreshGaits();renderMounts();resetMotion();scopeHelp();notice('Leg attachment changed. Recheck the sketch for this design.');
    });
    const button=document.createElement('button');button.type='button';button.dataset.leg=String(leg.id);
    button.innerHTML='Leg '+leg.id+'<small>'+leg.dof+' DOF · '+(leg.enabled?'attached':'empty')+'</small>';
    button.addEventListener('click',()=>selectLeg(leg.id));row.append(check,button);return row;
  }));
}
function selectLeg(id){
  if(mode==='code'&&$('codeScope').value==='one')stopMotion();
  selected=id;checked=null;renderMounts();scopeHelp();syncJoints();render();
}
function syncJoints(){
  const leg=selectedLeg();if(!leg)return;
  const p=poses.get(leg.id)||C.restPose(config,leg);
  $('selectedLegLabel').textContent='Leg '+leg.id+(leg.enabled?'':' · detached');
  $('selectedDof').value=String(leg.dof);
  for(const [id,key] of [['jointHip','hip'],['jointSide','hipSide'],['jointKnee','knee']]){
    $(id).value=Math.round(p[key]);$(id+'Value').textContent=Math.round(p[key])+'°';
    $(id).disabled=!leg.enabled||(key==='hipSide'&&leg.dof===2);
  }
  $('restLeg').disabled=!leg.enabled;$('insertPose').disabled=!leg.enabled;
  $('jointHint').textContent=leg.dof===2?'Two actuators: hip flexion and knee. The lateral mount is fixed at '+Math.round(C.fixedSide(leg))+'° ('+C.SPLAY+'° outward).':'Three actuators: hip flexion, lateral hip, and knee. Sliders stop motion and preview this leg.';
}
function scopeHelp(){
  const one=$('codeScope').value==='one',leg=selectedLeg();
  $('codeScopeHelp').textContent=one?'Selected leg '+selected+': use '+C.jointKeys(leg).join(', ')+'. The chassis stays fixed while you test this leg.':
    'Whole robot: use leg1Hip, leg1Side, leg1Knee, and matching names for other attached legs. Only 3-DOF legs have a Side servo. Ground contacts estimate travel.';
}
function updatePhaseTracks(){
  const rows=frame.legs;
  if($('phaseTracks').children.length!==rows.length||[...$('phaseTracks').children].some((r,i)=>Number(r.dataset.id)!==rows[i].leg.id)){
    $('phaseTracks').replaceChildren(...rows.map(f=>{
      const row=document.createElement('div');row.className='phase-row';row.dataset.id=f.leg.id;
      row.innerHTML='<b>L'+f.leg.id+'</b><div class="phase-track"><i></i></div><span class="phase-state"></span>';return row;
    }));
  }
  [...$('phaseTracks').children].forEach((row,i)=>{
    const f=rows[i],track=row.children[1];track.style.background='linear-gradient(to right,#a3d4c3 '+((f.duty??0.64)*100)+'%,#f6d1af '+((f.duty??0.64)*100)+'%)';
    track.firstChild.style.left=((f.phase??0)*100)+'%';track.firstChild.style.visibility=mode==='gait'?'visible':'hidden';
    row.lastChild.textContent=f.stance?'stance':'swing';row.lastChild.style.color=f.stance?'#208b70':'#ad672c';
  });
}
function sync(){
  const attached=active(), motors=attached.reduce((s,l)=>s+l.dof,0), planted=frame.legs.filter(f=>f.stance);
  const poly=C.hull(planted.map(f=>f.foot)), inside=C.supported(poly,frame.body);
  $('legSummary').textContent=attached.length+' legs · '+motors+' DOF';$('motorCount').textContent=motors+' servos';
  $('contactSummary').textContent=planted.length+' / '+attached.length+' feet';
  $('distanceSummary').textContent=Math.hypot(frame.body[0],frame.body[2]).toFixed(0)+' units';
  $('supportSummary').textContent=poly.length<3?'Fewer than 3 contacts':inside?'Inside support area':'Outside support area';
  $('supportSummary').style.color=inside?'#208b70':'#a2692b';
  $('sceneMode').textContent=mode==='gait'?(running?'Walking · '+config.gait:'Gait preview'):mode==='code'?($('codeScope').value==='one'?'One-leg sketch · chassis fixed':'Whole-robot sketch · contact estimate'):'Pose preview · chassis fixed';
  $('heightValue').textContent=config.height;$('strideValue').textContent=config.stride;$('liftValue').textContent=config.lift;$('periodValue').textContent=config.period+' s';
  const phase=Math.round(C.mod(time/config.period)*1000);$('cyclePhase').value=phase;$('phaseValue').textContent=Math.round(phase/10)+'%';
  const error=reachError;$('walk').disabled=!!error;
  const penetration=frame.legs.some(f=>f.foot[1]>6);
  $('reachMessage').textContent=error||(penetration?'A foot is below ground. Adjust the pose; collisions are not enforced.':
    mode==='code'&&$('codeScope').value==='all'?'Contact estimate · maximum horizontal slip '+(frame.slip||0).toFixed(1)+' units':
    mode==='gait'&&!$('moveBody').checked?'Chassis held: stance feet slide over the ground. Enable travel to keep them planted.':
    attached.length<3?'This layout needs dynamic balance, which is not simulated.':'Green = stance. Orange = swing. The shaded area shows geometric support.');
  updatePhaseTracks();syncJoints();
}
function poseFrame(estimate=false){
  const make=()=>active().map(leg=>{
    const p=poses.get(leg.id)||C.restPose(config,leg),local=C.fk(p),hip=C.add(body,[leg.x,0,leg.z]);
    const foot=C.add(hip,local.foot);
    return {leg,pose:p,hip,knee:C.add(hip,local.knee),foot,stance:Math.abs(foot[1])<6,reachable:true,phase:0,duty:0.64};
  });
  let list=make(),slip=0;
  if(estimate&&$('moveBody').checked){
    const feet=list.map(f=>({id:f.leg.id,local:C.subtract(f.foot,body)}));
    const motion=RobotMotion.contactStep(feet,body,anchors,runner?.fine?{enter:0.5,leave:0.9}:{});
    body=motion.body;anchors=motion.contacts;slip=motion.slip;list=make();
    for(const f of list)f.stance=anchors.has(f.leg.id);
  }else anchors.clear();
  return {body:[...body],legs:list,speed:0,slip};
}
function recordTrails(){
  if(!$('footTrails').checked)return;
  for(const f of frame.legs){
    const points=trails.get(f.leg.id)||[],last=points.at(-1);
    if(!last||Math.hypot(...C.subtract(last,f.foot))>1){points.push([...f.foot]);if(points.length>300)points.shift();}
    trails.set(f.leg.id,points);
  }
}
function manualJoint(key,value){
  stopMotion();mode='pose';runner=null;const leg=selectedLeg();if(!leg.enabled)return;
  poses.set(leg.id,{...poses.get(leg.id),[key]:value});frame=poseFrame();recordTrails();sync();render();
}
function configure(id,key,event='input'){
  $(id).addEventListener(event,()=>{
    const n=Number($(id).value),min=Number($(id).min),max=Number($(id).max);
    if(!Number.isFinite(n)||n<min||n>max){notice('Choose a value from '+min+' to '+max+'.','error');return;}
    config[key]=n;rebuildMounts();
  });
}
configure('plateWidth','width','change');configure('plateLength','length','change');configure('plateRadius','radius','change');
configure('bodyHeight','height');configure('stride','stride');configure('lift','lift');configure('period','period');
$('plateShape').onchange=()=>{config.shape=$('plateShape').value;$('rectangleSize').hidden=config.shape!=='rectangle';$('circleSize').hidden=config.shape!=='circle';rebuildMounts();};
$('legCount').onchange=()=>{config.count=Number($('legCount').value);rebuildMounts();};
$('applyDof').onclick=()=>{for(const l of legs)l.dof=Number($('allDof').value);rebuildMounts();};
$('selectedDof').onchange=()=>{selectedLeg().dof=Number($('selectedDof').value);rebuildMounts();};
function preset(count){
  config={...C.defaults(),count,shape:count===6?'circle':'rectangle',gait:count===6?'tripod':'trot'};
  legs=C.mounts(config);selected=1;
  for(const [id,key] of [['plateShape','shape'],['legCount','count'],['plateWidth','width'],['plateLength','length'],['plateRadius','radius'],['bodyHeight','height'],['stride','stride'],['lift','lift'],['period','period']])$(id).value=config[key];
  $('allDof').value='3';$('rectangleSize').hidden=config.shape!=='rectangle';$('circleSize').hidden=config.shape!=='circle';rebuildMounts();
}
$('presetSix').onclick=()=>preset(6);$('presetFour').onclick=()=>preset(4);
$('gaitType').onchange=()=>{config.gait=$('gaitType').value;resetMotion();};
$('walk').onclick=()=>{
  const error=C.validate(config,legs);if(error){notice(error,'error');return;}
  mode='gait';runner=null;running=true;lastTime=0;anchors.clear();$('pauseMotion').disabled=false;$('pauseMotion').textContent='Pause';
  notice('Walking with planted stance feet. Try another gait or scrub a cycle.','ok');sync();
};
$('pauseMotion').onclick=()=>{running=!running;lastTime=0;$('pauseMotion').textContent=running?'Pause':'Resume';sync();render();};
$('resetMotion').onclick=resetMotion;
$('moveBody').onchange=()=>{anchors.clear();trails.clear();if(mode==='gait'){frame=C.gaitFrame(config,legs,time,$('moveBody').checked);body=[...frame.body];}else{body=[0,-config.height,0];frame=poseFrame();}sync();render();};
$('cyclePhase').oninput=()=>{
  stopMotion();mode='gait';runner=null;time=Number($('cyclePhase').value)*config.period/1000;trails.clear();
  frame=C.gaitFrame(config,legs,time,$('moveBody').checked);body=[...frame.body];poses=new Map(frame.legs.map(f=>[f.leg.id,{...f.pose}]));sync();render();
};
for(const [id,key] of [['jointHip','hip'],['jointSide','hipSide'],['jointKnee','knee']])$(id).oninput=()=>manualJoint(key,Number($(id).value));
$('restLeg').onclick=()=>{stopMotion();mode='pose';runner=null;poses.set(selected,C.restPose(config,selectedLeg()));frame=poseFrame();recordTrails();sync();render();};
$('transparentPlate').onchange=render;$('showSupport').onchange=render;$('footTrails').onchange=()=>{trails.clear();recordTrails();render();};$('followCamera').onchange=render;

// Code workbench. Text remains in the editor when the robot design changes.
function rememberCodeSelection(){const e=$('robotCode');codeSelection={start:e.selectionStart,end:e.selectionEnd};}
['select','click','keyup','blur'].forEach(name=>$('robotCode').addEventListener(name,rememberCodeSelection));
$('robotCode').addEventListener('input',()=>{rememberCodeSelection();checked=null;if(mode==='code')stopMotion();notice('Sketch changed. Check or run it to rebuild.');});
$('codeScope').onchange=()=>{checked=null;if(mode==='code')stopMotion();scopeHelp();notice('Scope changed. Load an example or check the servo names in your sketch.');};
function setCode(text){stopMotion();checked=null;codeSelection=null;$('robotCode').value=text;notice('Sketch loaded. Check it, then run.');}
$('loadExample').onclick=()=>{try{$('codeScope').value='one';scopeHelp();setCode(C.oneLegSketch(config,selectedLeg()));}catch(e){notice(e.message,'error');}};
$('generateGait').onclick=()=>{try{setCode(C.gaitSketch(config,legs,selected,$('codeScope').value,32,$('codePrecision').value));}catch(e){notice(e.message,'error');}};
$('insertPose').onclick=()=>{
  const leg=selectedLeg();if(!leg?.enabled)return;
  const one=$('codeScope').value==='one',p=poses.get(leg.id)||C.restPose(config,leg);
  const text=C.jointKeys(leg).map(k=>'  '+C.servoName(leg,k,one)+'.write('+Math.round(p[k])+');').join('\n')+'\n  delay(350);\n';
  const e=$('robotCode'),source=e.value;
  let start=codeSelection?.start,end=codeSelection?.end;
  if(start===undefined){const match=/\bvoid\s+loop\s*\(\s*\)\s*\{/.exec(source);if(!match){notice('Add void loop() before inserting a pose.','error');return;}
    let depth=1,i=match.index+match[0].length;for(;i<source.length&&depth;i++){if(source[i]==='{')depth++;if(source[i]==='}')depth--;}if(depth){notice('loop() is missing a closing brace.','error');return;}start=end=i-1;}
  const inserted=(start>0&&source[start-1]!=='\n'?'\n':'')+text;
  e.value=source.slice(0,start)+inserted+source.slice(end);e.focus();e.setSelectionRange(start+inserted.length,start+inserted.length);rememberCodeSelection();
  checked=null;stopMotion();notice('Selected leg pose inserted. Check the sketch to validate it.');
};
function checkSketch(){
  stopMotion();
  try{
    const leg=selectedLeg();if($('codeScope').value==='one'&&!leg.enabled)throw Error('Attach the selected leg first.');
    const initial=currentFlat(),api=RobotArduino.create(C.mappings(legs,selected,$('codeScope').value),initial);
    const result=api.compile($('robotCode').value);
    checked={api,result,initial,scope:$('codeScope').value};
    const all=result.setupEvents.concat(result.loopEvents);
    notice('Sketch checked: '+all.length+' commands; '+all.reduce((n,e)=>n+e.ms,0)+' ms for setup + one loop.','ok');
    $('codeLine').textContent='Ready · '+(checked.scope==='one'?'Leg '+selected:'all attached legs');return true;
  }catch(e){checked=null;notice(e.message,'error');return false;}
}
$('checkCode').onclick=checkSketch;
$('runCode').onclick=()=>{
  if(!checkSketch())return;
  const {api,result,initial,scope}=checked;
  const writes=result.setupEvents.concat(result.loopEvents).filter(e=>e.type==='write');
  const fine=writes.length>0&&writes.every(e=>e.microseconds);
  runner={api,program:result.program,env:result.execution,setup:result.setupEvents,events:result.setupEvents.concat(result.loopEvents),index:0,elapsed:0,from:null,target:{...initial},scope,fine,phase:'setup'};
  mode='code';running=true;lastTime=0;anchors.clear();trails.clear();frame=poseFrame(false);
  $('pauseMotion').disabled=false;$('pauseMotion').textContent='Pause';notice('Playing setup once, then loop. '+(scope==='one'?'The chassis is held still.':'Travel is estimated from ground contacts.'),'ok');sync();render();
};
$('downloadCode').onclick=()=>{
  const blob=new Blob([$('robotCode').value],{type:'text/plain;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download=$('codeScope').value==='one'?'Robot_One_Leg.ino':'Robot_Gait.ino';a.click();setTimeout(()=>URL.revokeObjectURL(url),500);
};
function applyCodePose(values){fromFlat(values);frame=poseFrame(runner.scope==='all'&&runner.phase==='loop');}
function advanceCode(ms){
  runner.elapsed+=ms;
  for(let guard=0;guard<1003&&running;guard++){
    if(runner.index>=runner.events.length){
      applyCodePose(runner.target);
      if($('repeatCode').checked){
        const next=runner.api.execute(runner.program.loop,runner.env,'loop');
        if(next.length){runner.events=next;runner.index=0;runner.from=null;if(next.some(e=>e.ms>0))continue;break;}
      }
      stopMotion();notice('Sketch finished.','ok');break;
    }
    const e=runner.events[runner.index];runner.phase=e.phase;$('codeLine').textContent=e.phase+' · line '+e.line+' · '+e.description;
    if(e.type==='write'){runner.target[e.role]=e.value;runner.from=null;runner.index++;continue;}
    if(!e.ms){applyCodePose(runner.target);runner.index++;continue;}
    if(!runner.from)runner.from=currentFlat();
    // A sampled trajectory must not decelerate to zero at every write/delay pair.
    const f=Math.min(1,runner.elapsed/e.ms),blend=RobotMotion.interpolate(runner.from,runner.target,f);
    applyCodePose(blend);
    if(runner.elapsed>=e.ms){runner.elapsed-=e.ms;runner.index++;runner.from=null;continue;}
    break;
  }
}

// Perspective camera and painter-sorted 3D parts; all geometry stays in world coordinates.
function project(p){
  const follow=$('followCamera').checked,center=[follow?frame.body[0]:0,-config.height*0.45,follow?frame.body[2]:0];
  const [x,y,z]=C.subtract(p,center),cy=Math.cos(camera.yaw),sy=Math.sin(camera.yaw),cp=Math.cos(camera.pitch),sp=Math.sin(camera.pitch);
  const xx=x*cy+z*sy,zz=-x*sy+z*cy,yy=y*cp-zz*sp,depth=y*sp+zz*cp;
  const base=Math.min(width/780,height/560)*camera.zoom,scale=base*(camera.perspective?1300/Math.max(150,1300+depth):1);
  return {x:width/2+camera.panX+xx*scale,y:height*0.49+camera.panY+yy*scale,depth,scale};
}
function polygon(points,fill,stroke,lw=1){
  const q=points.map(project);if(!q.length)return;
  ctx.beginPath();ctx.moveTo(q[0].x,q[0].y);q.slice(1).forEach(p=>ctx.lineTo(p.x,p.y));ctx.closePath();
  if(fill){ctx.fillStyle=fill;ctx.fill();}if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=lw;ctx.stroke();}
}
function groundDisc(point,radius,fill){
  polygon(Array.from({length:24},(_,i)=>[point[0]+radius*Math.cos(i*Math.PI/12),0,point[2]+radius*Math.sin(i*Math.PI/12)]),fill);
}
function line3(a,b,color,lw=1){
  const A=project(a),B=project(b);ctx.strokeStyle=color;ctx.lineWidth=lw;ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(B.x,B.y);ctx.stroke();
}
function render(){
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,width,height);
  const bg=ctx.createLinearGradient(0,0,0,height);bg.addColorStop(0,'#f5f9fc');bg.addColorStop(1,'#eaf2f4');ctx.fillStyle=bg;ctx.fillRect(0,0,width,height);
  const cx=$('followCamera').checked?frame.body[0]:0,cz=$('followCamera').checked?frame.body[2]:0;
  const gx=Math.floor(cx/50)*50,gz=Math.floor(cz/50)*50;
  for(let i=-15;i<=15;i++){
    const x=gx+i*50,z=gz+i*50;
    line3([x,0,gz-750],[x,0,gz+750],i%4===0?'#b9cfd640':'#c9dce040');
    line3([gx-750,0,z],[gx+750,0,z],i%4===0?'#b9cfd640':'#c9dce040');
  }
  const planted=frame.legs.filter(f=>f.stance),support=C.hull(planted.map(f=>f.foot)),inside=C.supported(support,frame.body);
  if($('showSupport').checked){
    polygon(support,inside?'#32a38424':'#db9b3724',inside?'#329a7970':'#c48a3e90',1.3);
    groundDisc([frame.body[0],0,frame.body[2]],6,inside?'#208b70':'#c28636');
    line3([frame.body[0],-config.height,frame.body[2]],[frame.body[0],0,frame.body[2]],'#57778840',1);
  }
  if($('footTrails').checked)for(const [id,points] of trails){
    if(points.length<2)continue;
    ctx.beginPath();ctx.strokeStyle=id===selected?'#d89a46b0':'#b9a0cd70';ctx.lineWidth=id===selected?2:1;
    points.forEach((p,i)=>{const q=project(p);if(i)ctx.lineTo(q.x,q.y);else ctx.moveTo(q.x,q.y);});ctx.stroke();
  }
  for(const f of frame.legs){
    groundDisc([f.foot[0],0,f.foot[2]],f.stance?10:7,f.stance?'#249a7955':'#516f8420');
    if(!f.stance)line3(f.foot,[f.foot[0],0,f.foot[2]],'#d8a66d60',1);
  }
  const queue=[],push=(points,paint,bias=0)=>queue.push({depth:points.reduce((n,p)=>n+project(p).depth,0)/points.length+bias,paint});
  const bodyPt=(x,y,z)=>C.add(frame.body,[x,y,z]);
  function face(points,fill){push(points,()=>{ctx.save();ctx.globalAlpha=$('transparentPlate').checked?0.55:1;polygon(points,fill,'#64899c',1);ctx.restore();});}
  if(config.shape==='rectangle'){
    const x=config.width/2,z=config.length/2;
    const top=[bodyPt(-x,-16,-z),bodyPt(x,-16,-z),bodyPt(x,-16,z),bodyPt(-x,-16,z)],bottom=top.map(p=>[p[0],p[1]+24,p[2]]);
    for(let i=0;i<4;i++){const j=(i+1)%4;face([top[i],top[j],bottom[j],bottom[i]],i%2?'#85a9bd':'#9ebdce');}
    face(top,'#c9dee8');
  }else{
    const top=Array.from({length:32},(_,i)=>bodyPt(config.radius*Math.cos(i*Math.PI/16),-16,config.radius*Math.sin(i*Math.PI/16)));
    for(let i=0;i<top.length;i++){const a=top[i],b=top[(i+1)%top.length];face([a,b,[b[0],b[1]+24,b[2]],[a[0],a[1]+24,a[2]]],i<16?'#91b2c5':'#a3bfce');}
    face(top,'#c9dee8');
  }
  const arrowLen=config.shape==='circle'?config.radius*.55:config.length*.28;
  const arrow=[bodyPt(0,-18,-arrowLen*.5),bodyPt(0,-18,arrowLen)];

  function rod(a,b,color,radius){
    push([a,b],()=>{const A=project(a),B=project(b);ctx.lineCap='round';ctx.strokeStyle='#25465a';ctx.lineWidth=radius*(A.scale+B.scale)/2+2;ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(B.x,B.y);ctx.stroke();
      ctx.strokeStyle=color;ctx.lineWidth=Math.max(3,radius*(A.scale+B.scale)/2);ctx.stroke();
      ctx.strokeStyle='#ffffff45';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(A.x-2,A.y-1);ctx.lineTo(B.x-2,B.y-1);ctx.stroke();});
  }
  function joint(p,color,radius,ring=false){
    push([p],()=>{const q=project(p),r=Math.max(3,radius*q.scale),gradient=ctx.createRadialGradient(q.x-r*.3,q.y-r*.4,1,q.x,q.y,r);
      gradient.addColorStop(0,'#ffffff');gradient.addColorStop(0.35,color);gradient.addColorStop(1,'#284a60');
      ctx.beginPath();ctx.arc(q.x,q.y,r,0,Math.PI*2);ctx.fillStyle=gradient;ctx.fill();
      if(ring){ctx.strokeStyle=colors.selected;ctx.lineWidth=3;ctx.beginPath();ctx.arc(q.x,q.y,r+4,0,Math.PI*2);ctx.stroke();}
    },-1);
  }
  for(const leg of legs.filter(l=>!l.enabled)){const hip=bodyPt(leg.x,0,leg.z);joint(hip,'#a5b3bd',6);}
  for(const f of frame.legs){
    rod(f.hip,f.knee,colors.thigh,13);rod(f.knee,f.foot,colors.shin,11);
    joint(f.hip,f.leg.dof===3?'#62a9a5':'#adbfc9',9,f.leg.id===selected);
    joint(f.knee,'#c3b6dc',7);
    joint(f.foot,f.foot[1]>6||!f.reachable?colors.bad:f.stance?colors.stance:colors.swing,6);
  }
  queue.sort((a,b)=>b.depth-a.depth).forEach(item=>item.paint());
  // Surface markings and foot contact cues remain legible even where parts overlap.
  line3(arrow[0],arrow[1],'#416a80',3);
  polygon([bodyPt(0,-18,arrowLen+14),bodyPt(-10,-18,arrowLen-5),bodyPt(10,-18,arrowLen-5)],'#416a80');
  for(const f of frame.legs){
    const q=project(f.foot),color=f.foot[1]>6||!f.reachable?colors.bad:f.stance?colors.stance:colors.swing;
    ctx.beginPath();ctx.arc(q.x,q.y,Math.max(4,6*q.scale),0,Math.PI*2);ctx.fillStyle=color;ctx.fill();ctx.strokeStyle='#ffffff';ctx.lineWidth=1.5;ctx.stroke();
  }
  picks=legs.map(l=>({id:l.id,...project(bodyPt(l.x,0,l.z))}));
  const labelBoxes=[];ctx.font='600 11px system-ui';
  for(const p of [...picks].sort((a,b)=>(b.id===selected)-(a.id===selected))){
    const x=p.x+12,y=p.y-12;
    if(x<4||x>width-30||y<45||y>height-35||labelBoxes.some(b=>Math.abs(b.x-x)<35&&Math.abs(b.y-y)<18))continue;
    ctx.fillStyle=p.id===selected?'#9b6b16':'#49697e';ctx.fillText('L'+p.id,x,y);labelBoxes.push({x,y});
  }
  // Fixed orientation indicator, separate from robot labels.
  ctx.font='10px system-ui';
  for(const [x,z,label,color] of [[50,0,'X','#a47167'],[0,50,'Z','#668caf']]){
    const dx=(x*Math.cos(camera.yaw)+z*Math.sin(camera.yaw))*.65;
    const dy=-(-x*Math.sin(camera.yaw)+z*Math.cos(camera.yaw))*Math.sin(camera.pitch)*.65;
    const ox=width-62,oy=height-40;
    ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox+dx,oy+dy);ctx.stroke();ctx.fillStyle=color;ctx.fillText(label,ox+dx+3,oy+dy-3);
  }
}
function resize(){
  const rect=canvas.getBoundingClientRect();width=rect.width;height=rect.height;dpr=Math.min(2,window.devicePixelRatio||1);
  canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);render();
}
const cameraViews={iso:homeCamera,top:{yaw:0,pitch:Math.PI/2,perspective:false},front:{yaw:0,pitch:0,perspective:false},side:{yaw:Math.PI/2,pitch:0,perspective:false}};
function view(name){camera={...homeCamera,...cameraViews[name]};document.querySelectorAll('[data-camera]').forEach(b=>b.classList.toggle('selected',b.dataset.camera===name));render();}
document.querySelectorAll('[data-camera]').forEach(b=>b.onclick=()=>view(b.dataset.camera));$('resetView').onclick=()=>view('iso');
const pointers=new Map();let gesture=null,dragged=false,downPoint=null;
function point(e){const r=canvas.getBoundingClientRect();return {x:e.clientX-r.left,y:e.clientY-r.top};}
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('pointerdown',e=>{
  e.preventDefault();canvas.setPointerCapture(e.pointerId);const p=point(e);p.pan=e.shiftKey||e.button===2||e.button===1;pointers.set(e.pointerId,p);
  downPoint=p;dragged=pointers.size>1;gesture=null;canvas.style.cursor='grabbing';
});
canvas.addEventListener('pointermove',e=>{
  if(!pointers.has(e.pointerId))return;e.preventDefault();const p=point(e),prev=pointers.get(e.pointerId);
  if(Math.hypot(p.x-downPoint.x,p.y-downPoint.y)>4)dragged=true;
  if(pointers.size===1){
    const dx=p.x-prev.x,dy=p.y-prev.y;
    if(prev.pan){camera.panX+=dx;camera.panY+=dy;}else{camera.yaw+=dx*.008;camera.pitch=C.clamp(camera.pitch+dy*.008,0,Math.PI/2);}
    pointers.set(e.pointerId,{...p,pan:prev.pan});
  }else{
    dragged=true;pointers.set(e.pointerId,p);const [a,b]=[...pointers.values()],g={x:(a.x+b.x)/2,y:(a.y+b.y)/2,d:Math.hypot(a.x-b.x,a.y-b.y)};
    if(gesture){camera.panX+=g.x-gesture.x;camera.panY+=g.y-gesture.y;if(gesture.d>5)camera.zoom=C.clamp(camera.zoom*g.d/gesture.d,.4,3);}
    gesture=g;
  }
  document.querySelectorAll('[data-camera]').forEach(b=>b.classList.remove('selected'));render();
});
function release(e){
  if(!dragged&&e.type==='pointerup'){const p=point(e),nearest=picks.map(q=>({...q,d:Math.hypot(p.x-q.x,p.y-q.y)})).sort((a,b)=>a.d-b.d)[0];if(nearest&&nearest.d<30)selectLeg(nearest.id);}
  pointers.delete(e.pointerId);gesture=null;if(!pointers.size)canvas.style.cursor='grab';
}
canvas.addEventListener('pointerup',release);canvas.addEventListener('pointercancel',release);
canvas.addEventListener('wheel',e=>{e.preventDefault();camera.zoom=C.clamp(camera.zoom*Math.exp(-e.deltaY*.0015),.4,3);render();},{passive:false});
canvas.addEventListener('dblclick',()=>view('iso'));
function animate(t){
  requestAnimationFrame(animate);
  if(!running){lastTime=0;return;}
  const dt=lastTime?Math.min(0.05,(t-lastTime)/1000):0;lastTime=t;
  try{
    if(mode==='gait'){time+=dt;frame=C.gaitFrame(config,legs,time,$('moveBody').checked);body=[...frame.body];poses=new Map(frame.legs.map(f=>[f.leg.id,{...f.pose}]));}
    else if(mode==='code')advanceCode(dt*1000);
    recordTrails();sync();render();
  }catch(e){stopMotion();notice(e.message,'error');}
}
refreshGaits();renderMounts();scopeHelp();$('robotCode').value=C.oneLegSketch(config,selectedLeg());sync();
new ResizeObserver(resize).observe(canvas);resize();requestAnimationFrame(animate);
})();
