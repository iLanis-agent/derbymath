// Derbymath engine - pinewood derby race physics.
// Model (labeled): point-mass car on a straight ramp at angle theta into a flat
// runout. Gravity and Coulomb wheel/axle friction are exact; aero drag uses a
// user-supplied CdA. The center of mass starts x_cm*sin(theta) below the start
// pin when it sits forward of the rear axle - rearward placement is free energy.
// The flat runout integrates velocity-dependent drag in fixed steps.
// Labeled simplifications: no track transition curve, constant mu, no rotational
// wheel inertia (small for derby wheels), stability not modeled - the physics
// best CM is the rear axle itself, real cars keep ~25mm ahead for stability.
const G = 9.81, RHO = 1.225, DT = 0.0005;

function sim(dropM, rampM, flatM, massG, xCmMm, mu, cdA){
  if (!(dropM > 0)) throw new Error('ramp drop must be positive');
  if (!(rampM > 0)) throw new Error('ramp length must be positive');
  if (rampM <= dropM) throw new Error('ramp length must exceed the drop');
  if (!(flatM >= 0)) throw new Error('flat length cannot be negative');
  if (!(massG > 0)) throw new Error('car mass must be positive');
  if (!(xCmMm >= 0)) throw new Error('CM position cannot be negative');
  if (!(mu >= 0)) throw new Error('friction cannot be negative');
  if (!(cdA >= 0)) throw new Error('drag area cannot be negative');
  const sinT = dropM / rampM, cosT = Math.sqrt(1 - sinT * sinT);
  const m = massG / 1000, x = xCmMm / 1000;
  const hEff = dropM - x * sinT;
  if (!(hEff > 0)) throw new Error('CM sits too far forward - no energy to race');
  const aRamp = G * (sinT - mu * cosT);
  if (!(aRamp > 0)) throw new Error('car will not roll: friction beats the slope');
  // Exit speed: energy minus Coulomb work and drag work (drag work depends on
  // v^2, so solve it fixed-point - converges in a few iterations).
  let v2 = 2 * G * (hEff - mu * cosT * rampM);
  if (!(v2 > 0)) throw new Error('friction eats the whole drop - car stalls on the ramp');
  for (let i = 0; i < 12; i++){
    const wDrag = 0.5 * RHO * cdA * (v2 / 4) * rampM;
    const nv = 2 * (G * hEff - G * mu * cosT * rampM - wDrag / m);
    if (Math.abs(nv - v2) < 1e-12) { v2 = nv; break; }
    v2 = nv;
  }
  if (!(v2 > 0)) throw new Error('friction eats the whole drop - car stalls on the ramp');
  const vExit = Math.sqrt(v2);
  const tRamp = Math.sqrt(2 * rampM / aRamp);
  // Flat runout: deceleration = g*mu + k v^2, integrate in fixed steps.
  const k = 0.5 * RHO * cdA / m;
  let v = vExit, s = 0, t = 0, finished = true;
  while (s < flatM){
    v -= (G * mu + k * v * v) * DT;
    if (v <= 0){ finished = false; break; }
    s += v * DT; t += DT;
    if (t > 300){ finished = false; break; }
  }
  const total = finished ? tRamp + t : null;
  return { thetaDeg: Math.asin(sinT) * 180 / Math.PI, hEffM: hEff,
    vExit, topKmh: vExit * 3.6, tRamp, tFlat: finished ? t : null,
    total, finished };
}

// CM sweep: physics-best CM position (stability caveat lives in the UI label).
function cmSweep(dropM, rampM, flatM, massG, mu, cdA, curXmm){
  const step = 5, maxX = Math.min(150, Math.floor(dropM * 1000 * 0.9 / 5) * 5);
  let bestX = 0, best = null;
  for (let x = 0; x <= maxX; x += step){
    const r = sim(dropM, rampM, flatM, massG, x, mu, cdA);
    if (r.finished && (best === null || r.total < best)){ best = r.total; bestX = x; }
  }
  const cur = sim(dropM, rampM, flatM, massG, curXmm, mu, cdA);
  if (best === null) throw new Error('no finishable CM position found');
  return { bestXmm: bestX, bestTotal: best,
    currentTotal: cur.finished ? cur.total : null,
    deltaS: cur.finished ? cur.total - best : null };
}

// Weight gap: what running under the weight limit costs (mass only matters via drag).
function weightGap(dropM, rampM, flatM, maxG, curG, xCmMm, mu, cdA){
  if (!(maxG > 0)) throw new Error('max weight must be positive');
  if (!(curG > 0)) throw new Error('current weight must be positive');
  if (curG > maxG) throw new Error('current weight is over the limit - add graphite, not grams');
  const a = sim(dropM, rampM, flatM, maxG, xCmMm, mu, cdA);
  const b = sim(dropM, rampM, flatM, curG, xCmMm, mu, cdA);
  if (!a.finished || !b.finished) throw new Error('car does not finish at one of those weights');
  return { maxTotal: a.total, curTotal: b.total, gapS: b.total - a.total, gapG: maxG - curG };
}

// Prep savings: time bought by better axle/wheel prep (muNow -> muNew).
function prepSavings(dropM, rampM, flatM, massG, xCmMm, muNow, muNew, cdA){
  if (!(muNow >= 0) || !(muNew >= 0)) throw new Error('friction cannot be negative');
  if (muNew > muNow) throw new Error('new friction should be the lower one');
  const a = sim(dropM, rampM, flatM, massG, xCmMm, muNow, cdA);
  const b = sim(dropM, rampM, flatM, massG, xCmMm, muNew, cdA);
  if (!a.finished || !b.finished) throw new Error('car does not finish at one of those friction levels');
  return { nowTotal: a.total, newTotal: b.total, savedS: a.total - b.total };
}

const API = { G, RHO, sim, cmSweep, weightGap, prepSavings };
if (typeof module !== 'undefined' && module.exports) module.exports = API;
if (typeof window !== 'undefined') window.Derbymath = API;
