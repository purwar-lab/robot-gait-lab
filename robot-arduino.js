/* Restricted Arduino interpreter adapted from the single-leg simulator. */
(function(root){
'use strict';
function create(roleByName,initialPose){
const neutralPose=()=>({...initialPose});
function stripComments(source){
  return source.replace(/\/\*[\s\S]*?\*\//g,m=>m.replace(/[^\n]/g,' ')).replace(/\/\/[^\n]*/g,m=>' '.repeat(m.length));
}
function lineNumber(src,pos){return src.slice(0,pos).split('\n').length;}
function bodyOf(src,name){
  const m=new RegExp('\\bvoid\\s+'+name+'\\s*\\(\\s*(?:void\\s*)?\\)\\s*\\{').exec(src);
  if(!m) throw Error('Could not find void '+name+'() { ... }');
  const start=m.index+m[0].length;
  let depth=1;
  for(let i=start;i<src.length;i++){
    if(src[i]==='{')depth++;
    if(src[i]==='}')depth--;
    if(depth===0)return {text:src.slice(start,i),start,opening:m.index,end:i+1};
  }
  throw Error('Missing closing brace in '+name+'().');
}
function fail(line,message){throw Error('Line '+line+': '+message);}
function lookup(env,name,line){
  const binding=env.locals.get(name)||env.globals.get(name);
  if(!binding)fail(line,'variable "'+name+'" was not defined.');
  return binding;
}
// A small arithmetic parser; sketch text is never passed to eval or Function.
function evaluate(expression,env,line){
  const toks=expression.match(/\d+(?:\.\d*)?|\.\d+|[A-Za-z_]\w*|[()+\-*/]/g)||[];
  if(toks.join('')!==expression.replace(/\s/g,''))fail(line,'unsupported expression "'+expression+'".');
  let i=0;
  function atom(){
    const t=toks[i++];
    if(t==='('){const v=addExpr();if(toks[i++]!==')')fail(line,"missing ')'.");return v;}
    if(t==='-'||t==='+'){const v=atom();return {value:t==='-'?-v.value:v.value,integer:v.integer};}
    if(/^(?:\d|\.)/.test(t||''))return {value:Number(t),integer:!t.includes('.')};
    if(/^[A-Za-z_]\w*$/.test(t||'')){const b=lookup(env,t,line);return {value:b.value,integer:b.integer};}
    fail(line,'invalid expression "'+expression+'".');
  }
  function combine(a,b,op){
    const integer=a.integer&&b.integer;
    if(op==='/'&&b.value===0)fail(line,'division by zero.');
    let value=op==='+'?a.value+b.value:op==='-'?a.value-b.value:op==='*'?a.value*b.value:a.value/b.value;
    if(op==='/'&&integer)value=Math.trunc(value);
    return {value,integer};
  }
  function mult(){let v=atom();while(toks[i]==='*'||toks[i]==='/'){const op=toks[i++];v=combine(v,atom(),op);}return v;}
  function addExpr(){let v=mult();while(toks[i]==='+'||toks[i]==='-'){const op=toks[i++];v=combine(v,mult(),op);}return v;}
  const result=addExpr();
  if(i!==toks.length||!Number.isFinite(result.value))fail(line,'invalid or non-finite expression "'+expression+'".');
  return result;
}
function roleFromServo(name){return Object.hasOwn(roleByName,name)?roleByName[name]:null;}
function statements(text,offset,src){
  const parts=text.split(';'), result=[];
  let cursor=0;
  parts.forEach((part,i)=>{
    const s=part.trim(), line=lineNumber(src,offset+cursor+Math.max(0,part.search(/\S/)));
    cursor+=part.length+1;
    if(!s)return;
    if(i===parts.length-1)fail(line,'missing semicolon or unsupported trailing code: "'+s.slice(0,50)+'".');
    if(/[{}]/.test(s))fail(line,'nested blocks, control flow, and custom functions are not supported.');
    result.push({text:s,line});
  });
  return result;
}
function declaration(s,line){
  const m=/^(const\s+)?(int|float|double|long|unsigned\s+int)\s+([A-Za-z_]\w*)\s*=\s*(.+)$/.exec(s);
  return m ? {type:'declare',name:m[3],expression:m[4],constant:!!m[1],integer:!['float','double'].includes(m[2]),unsigned:m[2].startsWith('unsigned'),line} : null;
}
function parse(source){
  if(source.length>200000)throw Error('Sketch is too large (maximum 200,000 characters).');
  const src=stripComments(source), setup=bodyOf(src,'setup'), loop=bodyOf(src,'loop');
  // Preserve character positions so include lines cannot shift function boundaries.
  let outside=src.replace(/^[ \t]*#include[^\n]*/gm,m=>m.replace(/[^\n]/g,' '));
  for(const part of [setup,loop])outside=outside.slice(0,part.opening)+outside.slice(part.opening,part.end).replace(/[^\n]/g,' ')+outside.slice(part.end);
  const servos=new Map(), globals=[];
  for(const {text:s,line} of statements(outside,0,src)){
    const m=/^Servo\s+([A-Za-z_]\w*)$/.exec(s);
    if(m){
      if(servos.has(m[1]))fail(line,'duplicate servo declaration "'+m[1]+'".');
      const role=roleFromServo(m[1]);
      if(!role)fail(line,'servo '+m[1]+' is not available in this scope. Use: '+Object.keys(roleByName).join(', '));
      if([...servos.values()].includes(role))fail(line,'more than one servo is mapped to '+role+'.');
      servos.set(m[1],role); continue;
    }
    const d=declaration(s,line);
    if(!d)fail(line,'unsupported statement outside setup()/loop(): "'+s.slice(0,60)+'".');
    globals.push(d);
  }
  function compile(body){
    return statements(body.text,body.start,src).map(({text:s,line})=>{
      let m;
      const d=declaration(s,line); if(d)return d;
      if((m=/^([A-Za-z_]\w*)\s*=\s*(.+)$/.exec(s)))return {type:'assign',name:m[1],expression:m[2],line};
      if((m=/^([A-Za-z_]\w*)\.(write|writeMicroseconds)\s*\((.+)\)$/.exec(s))){
        if(!servos.has(m[1]))fail(line,'servo "'+m[1]+'" must be declared first.');
        return {type:'write',object:m[1],role:servos.get(m[1]),microseconds:m[2]==='writeMicroseconds',expression:m[3],line};
      }
      if((m=/^delay\s*\((.+)\)$/.exec(s)))return {type:'delay',expression:m[1],line};
      if((m=/^([A-Za-z_]\w*)\.attach\s*\(\s*([A-Za-z_]\w*|\d+)\s*\)$/.exec(s))){
        if(!servos.has(m[1]))fail(line,'servo "'+m[1]+'" must be declared first.');
        return {type:'ignore',line};
      }
      if(/^Serial\.begin\s*\(\s*\d+\s*\)$/.test(s))return {type:'ignore',line};
      fail(line,'unsupported command "'+s.slice(0,65)+'".');
    });
  }
  const compiled={globals,setup:compile(setup),loop:compile(loop)};
  if(globals.length+compiled.setup.length+compiled.loop.length>1000)throw Error('Sketch contains more than 1000 statements.');
  const prepared=prepareProgram(compiled);
  if(!prepared.setupEvents.length&&!prepared.loopEvents.length)throw Error('Sketch contains no write() or delay() commands.');
  return {program:compiled,...prepared};
}
function assignValue(binding,value,line){
  binding.value=binding.integer?Math.trunc(value):value;
  if(binding.unsigned && binding.value<0)fail(line,'negative unsigned values are not supported.');
}
function executeBlock(block,env,phase,globalScope=false){
  env.locals=new Map();
  const events=[];
  for(const op of block){
    if(op.type==='ignore')continue;
    if(op.type==='declare'){
      const scope=globalScope?env.globals:env.locals;
      if(scope.has(op.name))fail(op.line,'duplicate variable "'+op.name+'".');
      const value=evaluate(op.expression,env,op.line).value;
      const binding={constant:op.constant,integer:op.integer,unsigned:op.unsigned,value:0};
      assignValue(binding,value,op.line); scope.set(op.name,binding); continue;
    }
    if(op.type==='assign'){
      const binding=lookup(env,op.name,op.line);
      if(binding.constant)fail(op.line,'cannot assign to const variable "'+op.name+'".');
      assignValue(binding,evaluate(op.expression,env,op.line).value,op.line); continue;
    }
    const value=evaluate(op.expression,env,op.line).value;
    if(op.type==='write'){
      if(op.microseconds){
        if(value<544||value>2400)fail(op.line,'writeMicroseconds requires 544?2400 us with the supported default Servo calibration.');
      }else if(value<0||value>180)fail(op.line,'servo.write angle must be between 0 and 180.');
      const argument=Math.trunc(value),angle=op.microseconds?(argument-544)*180/1856:argument;
      env.pose[op.role]=angle;
      events.push({type:'write',role:op.role,value:angle,microseconds:op.microseconds,pose:{...env.pose},ms:0,line:op.line,phase,
        description:op.object+(op.microseconds?'.writeMicroseconds(':'.write(')+argument+')'});
    }else{
      if(value<0||value>120000)fail(op.line,'delay must be 0–120000 ms.');
      const ms=Math.trunc(value);
      events.push({type:'delay',pose:{...env.pose},ms,line:op.line,phase,description:'delay('+ms+')'});
    }
  }
  return events;
}
function prepareProgram(compiled){
  const execution={globals:new Map(),locals:new Map(),pose:neutralPose()};
  executeBlock(compiled.globals,execution,'global',true);
  const setupEvents=executeBlock(compiled.setup,execution,'setup');
  const initial={...execution.pose};
  const loopEvents=executeBlock(compiled.loop,execution,'loop');
  return {execution,setupEvents,loopEvents,initial};
}

return {compile:parse,prepare:prepareProgram,execute:executeBlock};
}
const api={create};if(typeof module==='object'&&module.exports)module.exports=api;else root.RobotArduino=api;
})(globalThis);
