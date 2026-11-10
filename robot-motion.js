/* Code-playback geometry: continuous sample interpolation and incremental contacts. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.RobotMotion=api;})(globalThis,function(){
'use strict';
function interpolate(from,target,fraction){
  const f=Math.max(0,Math.min(1,fraction)),result={};
  for(const key of Object.keys(target))result[key]=from[key]+(target[key]-from[key])*f;
  return result;
}
function median(values){
  const v=[...values].sort((a,b)=>a-b),m=Math.floor(v.length/2);
  return v.length%2?v[m]:(v[m-1]+v[m])/2;
}
// Each foot is relative to the body, including the mount offset.
// Estimate the current displacement, never pull the body back to incompatible
// historical anchors when contacts are added or removed.
function contactStep(feet,body,previous=new Map(),options={}){
  const enter=options.enter??3,leave=options.leave??5;
  const planted=feet.filter(f=>Math.abs(body[1]+f.local[1])<=(previous.has(f.id)?leave:enter));
  const retained=planted.filter(f=>previous.has(f.id));
  const nextBody=[...body];
  if(options.travel!==false&&retained.length>=2){
    nextBody[0]-=median(retained.map(f=>f.local[0]-previous.get(f.id).local[0]));
    nextBody[2]-=median(retained.map(f=>f.local[2]-previous.get(f.id).local[2]));
  }
  const contacts=new Map();let slip=0;
  for(const f of planted){
    const world=f.local.map((v,i)=>v+nextBody[i]);
    const anchor=previous.get(f.id)?.anchor??[world[0],0,world[2]];
    slip=Math.max(slip,Math.hypot(world[0]-anchor[0],world[2]-anchor[2]));
    contacts.set(f.id,{local:[...f.local],anchor:[...anchor]});
  }
  return {body:nextBody,contacts,slip};
}
return {interpolate,contactStep};
});
