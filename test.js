const e = require('./engine.js');
const exp = require('./expected.json');
let pass = 0, fail = 0;
const close = (a, b, tol) => (a === null && b === null) || (typeof a === 'number' && typeof b === 'number' && Math.abs(a - b) <= tol * Math.max(1, Math.abs(b)));
function cmpObj(got, want, path){
  for (const k of Object.keys(want)){
    const g = got[k], w = want[k];
    if (typeof w === 'boolean'){ if (g !== w) throw new Error(path + '.' + k + ': got ' + g + ' want ' + w); }
    else if (!close(g, w, 1e-9)) throw new Error(path + '.' + k + ': got ' + g + ' want ' + w);
  }
}
exp.cases.forEach((c, i) => {
  try {
    let got;
    if (c.kind === 'sim') got = e.sim(...c.args);
    else if (c.kind === 'cmSweep') got = e.cmSweep(...c.args);
    else if (c.kind === 'weightGap') got = e.weightGap(...c.args);
    else got = e.prepSavings(...c.args);
    cmpObj(got, c.out, 'case' + i);
    pass++;
  } catch (err){ fail++; console.log('FAIL case', i, c.kind, err.message); }
});
// Anchors: frictionless, dragless, CM at rear axle reduces to textbook kinematics.
(function(){
  const r = e.sim(1.2, 4.9, 4.85, 141.75, 0, 0, 0);
  const vIdeal = Math.sqrt(2 * 9.81 * 1.2);
  if (!close(r.vExit, vIdeal, 1e-12)) throw new Error('anchor vExit');
  const aRamp = 9.81 * (1.2 / 4.9);
  if (!close(r.tRamp, Math.sqrt(2 * 4.9 / aRamp), 1e-12)) throw new Error('anchor tRamp');
  if (!close(r.tFlat, 4.85 / vIdeal, 5e-4)) throw new Error("anchor tFlat"); // fixed-step integration overshoots by less than one DT
  const th = Math.asin(1.2 / 4.9) * 180 / Math.PI;
  if (!close(r.thetaDeg, th, 1e-12)) throw new Error('anchor theta');
  pass += 4;
})();
// Properties.
(function(){
  // Rearward CM never slower than forward CM.
  const a = e.sim(1.2, 4.9, 4.85, 141.75, 0, 0.01, 0.001);
  const b = e.sim(1.2, 4.9, 4.85, 141.75, 100, 0.01, 0.001);
  if (!(a.total <= b.total)) throw new Error('CM monotonicity');
  // More friction never faster.
  const c = e.sim(1.2, 4.9, 4.85, 141.75, 25, 0.02, 0.001);
  const d = e.sim(1.2, 4.9, 4.85, 141.75, 25, 0.005, 0.001);
  if (!(d.total < c.total)) throw new Error('friction monotonicity');
  // Heavier never slower when drag exists.
  const f = e.sim(1.2, 4.9, 4.85, 141.75, 25, 0.01, 0.002);
  const g2 = e.sim(1.2, 4.9, 4.85, 100, 25, 0.01, 0.002);
  if (!(f.total <= g2.total)) throw new Error('mass monotonicity');
  // Sweep best is at least as good as current.
  const s = e.cmSweep(1.2, 4.9, 4.85, 141.75, 0.01, 0.001, 25);
  if (!(s.bestTotal <= s.currentTotal + 1e-12)) throw new Error('sweep best');
  pass += 4;
})();
// Error cases.
(function(){
  const bad = [
    () => e.sim(0, 4.9, 4.85, 141.75, 25, 0.01, 0.001),
    () => e.sim(1.2, 1.0, 4.85, 141.75, 25, 0.01, 0.001),
    () => e.sim(1.2, 4.9, 4.85, 0, 25, 0.01, 0.001),
    () => e.sim(1.2, 4.9, 4.85, 141.75, 25, 0.5, 0.001),
    () => e.weightGap(1.2, 4.9, 4.85, 141.75, 150, 25, 0.01, 0.001),
    () => e.prepSavings(1.2, 4.9, 4.85, 141.75, 25, 0.005, 0.02, 0.001),
  ];
  bad.forEach((f2, i) => {
    try { f2(); fail++; console.log('FAIL error case', i, 'did not throw'); }
    catch (err){ if (err.message.startsWith('case')) throw err; pass++; }
  });
})();
console.log(pass + '/' + (pass + fail) + ' checks pass');
process.exit(fail ? 1 : 0);
