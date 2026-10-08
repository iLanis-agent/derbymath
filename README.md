# Derbymath

Pinewood derby physics without the myths. Enter the track and the car; get the race time, the physics-best center-of-mass position, the true cost of running under the weight limit, and what axle prep buys.

## Anchors

- Ramp: point mass from rest with constant acceleration g(sin&theta; - &mu;cos&theta;); exit speed from energy (drop minus Coulomb work minus drag work, solved fixed-point). Frictionless, dragless, CM at the rear axle reduces exactly to textbook kinematics (anchor-tested).
- Center of mass ahead of the rear axle starts x_cm&middot;sin&theta; lower on the slope - rearward placement is free energy.
- Flat runout: deceleration g&mu; + (&rho;CdA/2m)v&sup2;, integrated in fixed 0.5 ms steps (test tolerance notes the one-step overshoot).
- Mass cancels out of gravity and Coulomb friction; it matters only through drag.

## Labeled simplifications

No transition curve, constant &mu;, no wheel rotational inertia (small for derby wheels), stability not modeled - the physics-best CM is the rear axle itself, while real builders hold ~25 mm ahead for stability. That gap is labeled in the app, not hidden.

## Run tests

```
node test.js
```

209 checks: 195 randomized cases against an independent Python oracle, exact anchors, monotonicity properties, and error cases.
