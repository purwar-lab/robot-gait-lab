// No packages required: execute the page's actual script with a minimal DOM/canvas.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const { test } = require('node:test');
const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function harness() {
  const nodes = new Map(), downloads = [];
  let time = 100, clockStarted = false;
  const context2d = new Proxy({}, { get: (o, k) => o[k] ?? (() => {}), set: (o, k, v) => (o[k] = v, true) });
  function node(id) {
    if (!nodes.has(id)) nodes.set(id, {
      value: '', checked: true, width: id === 'scene' ? 820 : 250, height: id === 'scene' ? 560 : 210,
      selectionStart: 0, selectionEnd: 0, listeners: {}, classList: { toggle() {}, add() {}, remove() {} },
      addEventListener(type, fn) { (this.listeners[type] ??= []).push(fn); },
      fire(type, extra = {}) { for (const f of this.listeners[type] || []) f({ preventDefault() {}, ...extra }); },
      getContext: () => context2d,
      getBoundingClientRect() { return { left: 0, top: 0, width: this.width, height: this.height }; },
      setPointerCapture() {}, focus() {}, select() {},
      setSelectionRange(a, b) { this.selectionStart = a; this.selectionEnd = b; },
      click() { downloads.push({ name: this.download, href: this.href }); }
    });
    return nodes.get(id);
  }
  node('mode3').value = 'abduction'; node('speed').value = '1';
  node('wait').value = '350'; node('repeat').checked = false;
  const context = {
    document: { getElementById: node, querySelectorAll: () => [], createElement: () => node('download-anchor') },
    requestAnimationFrame() {}, setTimeout(fn) { fn(); }, Blob,
    URL: { createObjectURL(blob) { context.exportBlob = blob; return 'blob:test'; }, revokeObjectURL() {} },
    navigator: { clipboard: { async writeText(text) { context.copied = text; } } }
  };
  vm.createContext(context);
  const exposed = script.replace(/\}\)\(\);\s*$/, `
    globalThis.api = { computeLeg, planeProject, project3D, setCameraView, parse, build, start, animate,
      setPose, insertSnippet, recordFoot, MAX_TRAIL_POINTS,
      state: () => ({pose, trail, sequence, running, paused, targetPose, visibleIndex, camera, execution}) };
  })();`);
  vm.runInContext(exposed, context);
  return { api: context.api, node, context, downloads,
    sketch(text) { node('editor').value = text; node('editor').fire('input'); },
    tick(duration, step = 10) {
      if (!clockStarted) { context.api.animate(time); clockStarted = true; }
      for (let elapsed = 0; elapsed < duration; elapsed += step) { time += step; context.api.animate(time); }
    }
  };
}
const near = (a, b, eps = 1e-8) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);
const vectorNear = (a, b) => b.forEach((n, i) => near(a[i], n));
const sketch = (setup, loop, globals = '') => '#include <Servo.h>\nServo hip; Servo hipSide; Servo knee;\n' + globals +
  '\nvoid setup(){' + setup + '}\nvoid loop(){' + loop + '}';

test('neutral, flexion, abduction, and knee signs match world conventions', () => {
  const { api } = harness();
  vectorNear(api.computeLeg({ hip: 90, hipSide: 90, knee: 90 }).footPt, [0, 280, 0]);
  vectorNear(api.computeLeg({ hip: 180, hipSide: 90, knee: 90 }).footPt, [0, 0, 280]);
  vectorNear(api.computeLeg({ hip: 90, hipSide: 180, knee: 90 }).footPt, [280, 0, 0]);
  vectorNear(api.computeLeg({ hip: 90, hipSide: 90, knee: 0 }).footPt, [0, 140, 140]);
  vectorNear(api.computeLeg({ hip: 180, hipSide: 180, knee: 90 }, 'yaw').footPt, [-280, 0, 0]);
});
test('link lengths and reach remain invariant throughout both joint modes', () => {
  const { api } = harness();
  for (const mode of ['abduction', 'yaw']) for (let h = 0; h <= 180; h += 30)
    for (let s = 0; s <= 180; s += 30) for (let k = 0; k <= 180; k += 30) {
      const { kneePt: knee, footPt: foot } = api.computeLeg({ hip: h, hipSide: s, knee: k }, mode);
      near(Math.hypot(...knee), 140); near(Math.hypot(...foot.map((v, i) => v - knee[i])), 140);
      assert.ok(Math.hypot(...foot) <= 280 + 1e-8);
    }
});
test('single lateral-joint sweeps lie on the expected circles', () => {
  const { api } = harness(); let radius, z;
  for (let s = 0; s <= 180; s += 10) {
    const f = api.computeLeg({ hip: 110, hipSide: s, knee: 65 }, 'abduction').footPt;
    radius ??= Math.hypot(f[0], f[1]); z ??= f[2];
    near(Math.hypot(f[0], f[1]), radius); near(f[2], z);
    const yaw = api.computeLeg({ hip: 110, hipSide: s, knee: 65 }, 'yaw').footPt;
    near(Math.hypot(yaw[0], yaw[2]), z);
    vectorNear(api.computeLeg({ hip: 90, hipSide: s, knee: 90 }, 'yaw').footPt, [0, 280, 0]);
  }
});
test('orthographic presets and small panels have identical axis orientations', () => {
  const { api } = harness();
  for (const plane of ['xy', 'xz', 'yz']) {
    api.setCameraView(plane);
    const o = api.project3D([0, 0, 0]), q = api.planeProject([0, 0, 0], plane, 250, 210);
    for (const axis of [[50, 0, 0], [0, 50, 0], [0, 0, 50]]) {
      const a = api.project3D(axis), b = api.planeProject(axis, plane, 250, 210);
      const ratio = (460 / 560) / q.scale;
      near(a.x - o.x, (b.x - q.x) * ratio); near(a.y - o.y, (b.y - q.y) * ratio);
    }
  }
});
test('all reachable poses fit the small projection panels', () => {
  const { api } = harness();
  for (const mode of ['abduction', 'yaw']) for (let h = 0; h <= 180; h += 30)
    for (let s = 0; s <= 180; s += 30) for (let k = 0; k <= 180; k += 30)
      for (const plane of ['xy', 'xz', 'yz']) {
        const f = api.planeProject(api.computeLeg({ hip: h, hipSide: s, knee: k }, mode).footPt, plane, 250, 210);
        assert.ok(f.x >= 22 - 1e-8 && f.x <= 228 + 1e-8 && f.y >= 22 - 1e-8 && f.y <= 188 + 1e-8);
      }
});
test('sample parses setup and loop with accurate source lines', () => {
  const h = harness(), p = h.api.parse();
  assert.equal(p.setupEvents.length, 4); assert.equal(p.loopEvents.length, 11);
  assert.equal(p.setupEvents.reduce((n, e) => n + e.ms, 0), 700);
  const lines = h.node('editor').value.split('\n');
  for (const e of [...p.setupEvents, ...p.loopEvents]) assert.ok(lines[e.line - 1].includes(e.description));
});
test('include masking preserves declarations and recognizes alternate whitespace', () => {
  const h = harness();
  h.sketch(sketch('hip.write(a);', 'knee.write(a+1);', 'int a=95;').replace('void setup', 'void\nsetup').replace('void loop', 'void\tloop') + '\nint b=3;');
  const p = h.api.parse(); assert.equal(p.initial.hip, 95); assert.equal(p.loopEvents[0].value, 96);
});
test('unsupported trailing text, custom functions, and missing semicolons fail', () => {
  for (const suffix of ['garbage()', 'void helper(){}', 'int bad=1']) {
    const h = harness(); h.sketch(sketch('', 'delay(1);') + suffix); assert.throws(() => h.api.parse());
  }
  const h = harness(); h.sketch(sketch('', 'hip.write(90)')); assert.throws(() => h.api.parse(), /semicolon/);
});
test('variables obey scope, const protection, and integer arithmetic', () => {
  const h = harness();
  h.sketch(sketch('int local=5;', 'hip.write(90+5/2); knee.write(90+5.0/2);'));
  const p = h.api.parse(); assert.equal(p.loopEvents[0].value, 92); assert.equal(p.loopEvents[1].value, 92);
  h.sketch(sketch('int local=5;', 'hip.write(local);')); assert.throws(() => h.api.parse(), /not defined/);
  h.sketch(sketch('', 'a=4;delay(1);', 'const int a=3;')); assert.throws(() => h.api.parse(), /const/);
  h.sketch(sketch('', 'delay(5/0);')); assert.throws(() => h.api.parse(), /division by zero/);
});
test('undeclared servos and out-of-range commands fail before playback', () => {
  const h = harness(); h.sketch('void setup(){} void loop(){hip.write(90);}'); assert.throws(() => h.api.parse(), /declared/);
  for (const command of ['hip.write(180.1);', 'hip.write(-0.1);', 'delay(-1);', 'delay(120001);', 'for(int i=0;i<5;i++){delay(1);}']) {
    h.sketch(sketch('', command)); assert.throws(() => h.api.parse());
  }
});
test('setup delay is honored and setup executes only once across repeated loops', () => {
  const h = harness(); h.sketch(sketch('count=count+1;hip.write(80);delay(700);', 'count=count+1;knee.write(count);delay(100);', 'int count=90;'));
  h.node('repeat').checked = true; h.api.start(); h.tick(300);
  near(h.api.state().pose.hip, 80); near(h.api.state().pose.knee, 90);
  assert.equal(h.api.state().sequence[h.api.state().visibleIndex].phase, 'setup');
  h.tick(800); assert.equal(h.api.state().execution.globals.get('count').value, 96);
});
test('repeated single-servo writes retain the other joint targets', () => {
  const h = harness(); h.sketch(sketch('hip.write(80);', 'knee.write(60);delay(50);hip.write(110);delay(50);'));
  h.node('repeat').checked = true; h.api.start(); h.tick(100);
  const s = h.api.state(); assert.equal(s.sequence[1].pose.hip, 110);
  near(s.pose.hip, 110); assert.equal(s.targetPose.hip, 110);
});
test('loop globals persist and locals are recreated on each iteration', () => {
  const h = harness(); h.sketch(sketch('', 'int local=1;g=g+local;hip.write(g);delay(100);', 'int g=90;'));
  h.node('repeat').checked = true; h.api.start(); h.tick(300);
  assert.equal(h.api.state().execution.globals.get('g').value, 94);
  assert.equal(h.api.state().execution.locals.get('local').value, 1);
});
test('runtime range errors stop repeat playback clearly', () => {
  const h = harness(); h.sketch(sketch('', 'g=g+1;hip.write(g);delay(1);', 'int g=179;'));
  h.node('repeat').checked = true; h.api.start(); h.tick(30);
  assert.equal(h.api.state().running, false); assert.match(h.node('status').textContent, /between 0 and 180/);
});
test('trailing writes apply even with no delay', () => {
  const h = harness(); h.sketch(sketch('', 'hip.write(120);')); h.api.start(); h.tick(10);
  assert.equal(h.api.state().pose.hip, 120); assert.equal(h.api.state().running, false);
});
test('pause freezes pose and progress until resumed', () => {
  const h = harness(); h.api.start(); h.tick(100); h.node('pause').onclick();
  const pose = JSON.stringify(h.api.state().pose), progress = h.node('progress').value;
  h.tick(500); assert.equal(JSON.stringify(h.api.state().pose), pose); assert.equal(h.node('progress').value, progress);
  h.node('pause').onclick(); assert.equal(h.api.state().paused, false);
});
test('manual pose, mode change, insertion, and editing stop playback', () => {
  for (const action of ['slider', 'mode', 'insert', 'edit']) {
    const h = harness(); h.api.start();
    if (action === 'slider') { h.node('hipPitch').value = '120'; h.node('hipPitch').fire('input'); }
    if (action === 'mode') { h.node('mode3').value = 'yaw'; h.node('mode3').fire('change'); }
    if (action === 'insert') h.node('insertKnee').onclick();
    if (action === 'edit') h.node('editor').fire('input');
    assert.equal(h.api.state().running, false, action);
  }
});
test('trajectory storage is bounded, deduplicated, and unaffected by camera changes', () => {
  const h = harness(); h.api.setPose({ hip: 90, hipSide: 90, knee: 90 });
  for (let i = 0; i < 100; i++) h.api.recordFoot(); assert.equal(h.api.state().trail.length, 1);
  for (let i = 0; i < 2100; i++) h.api.setPose({ hip: i % 2 ? 100 : 90, hipSide: 90, knee: 90 });
  assert.equal(h.api.state().trail.length, h.api.MAX_TRAIL_POINTS);
  const before = JSON.stringify(h.api.state().trail); h.api.setCameraView('xz'); assert.equal(JSON.stringify(h.api.state().trail), before);
  h.node('showTrail').checked = false; h.node('showTrail').onchange(); h.api.recordFoot(); assert.equal(h.api.state().trail.length, 0);
});
test('insertion remembers a collapsed cursor and reset restores default placement', () => {
  const h = harness(), e = h.node('editor');
  const pos = e.value.indexOf('  knee.write(65)'); e.setSelectionRange(pos, pos); e.fire('blur');
  h.node('insertKnee').onclick(); assert.ok(e.value.slice(pos).startsWith('  knee.write(90);\n'));
  h.node('resetCode').onclick(); h.node('insertKnee').onclick(); assert.ok(e.value.trimEnd().endsWith('knee.write(90);\n}'));
});
test('default insertion uses loop closing brace even if setup follows loop', () => {
  const h = harness(), e = h.node('editor');
  e.value = 'Servo hip; Servo hipSide; Servo knee;\nvoid loop(){delay(5);}\nvoid setup(){hip.write(90);}';
  h.node('insertKnee').onclick(); assert.ok(e.value.indexOf('knee.write(90)') < e.value.indexOf('void setup'));
});
test('invalid insertion delay leaves code unchanged', () => {
  const h = harness(), before = h.node('editor').value;
  h.node('wait').value = '-1'; h.node('insertAll').onclick(); assert.equal(h.node('editor').value, before);
});
test('orbit, pan, and pinch controls update the camera without changing the pose', () => {
  const h = harness(), c = h.node('scene'), before = JSON.stringify(h.api.state().pose);
  c.fire('pointerdown', { pointerId: 1, button: 0, clientX: 100, clientY: 100 });
  c.fire('pointermove', { pointerId: 1, clientX: 130, clientY: 120 });
  near(h.api.state().camera.yaw, -0.56); near(h.api.state().camera.pitch, 0.61);
  c.fire('pointerup', { pointerId: 1 });
  c.fire('pointerdown', { pointerId: 2, button: 2, clientX: 100, clientY: 100 });
  c.fire('pointermove', { pointerId: 2, clientX: 130, clientY: 120 });
  near(h.api.state().camera.panX, 30); near(h.api.state().camera.panY, 20);
  c.fire('pointerup', { pointerId: 2 });
  c.fire('pointerdown', { pointerId: 3, button: 0, clientX: 100, clientY: 100 });
  c.fire('pointerdown', { pointerId: 4, button: 0, clientX: 200, clientY: 100 });
  c.fire('pointermove', { pointerId: 4, clientX: 210, clientY: 100 });
  c.fire('pointermove', { pointerId: 4, clientX: 230, clientY: 100 });
  assert.ok(h.api.state().camera.zoom > 1);
  c.fire('pointercancel', { pointerId: 3 }); c.fire('pointercancel', { pointerId: 4 });
  assert.equal(JSON.stringify(h.api.state().pose), before);
});
test('copy and .ino export preserve exact sketch text', async () => {
  const h = harness(), text = h.node('editor').value;
  await h.node('copy').onclick(); assert.equal(h.context.copied, text);
  h.node('download').onclick(); assert.equal(h.downloads[0].name, 'Three_DOF_Leg_Custom.ino');
  assert.equal(await h.context.exportBlob.text(), text);
});
