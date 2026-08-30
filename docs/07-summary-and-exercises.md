# Lesson 7 — Summary, Rules of Thumb, and Next Steps

**Previous:** [Lesson 6 — Observability and Limits](06-observability-and-limits.md)

---

## 1. The complete arc in one table

| Question | 1D linear answer | 2D nonlinear answer |
|---|---|---|
| What does INS-only do? | 75 m drift, honestly ($\tfrac{1}{2}\sigma_b t^2$) | 79 m drift, honestly |
| What does GPS add? | Bounded error at 1 Hz (0.56 m) | Bounded error at 1 Hz (0.69 m) — observes the full vector |
| What does one Doppler beacon add? | **0.041 m** — observes *everything* | 69 m — observes only one position + one velocity direction |
| What does two-beacon Doppler add? | — | **0.092 m** — geometry spans the state |
| Best filter | INS + GPS + Doppler (0.041 m) | INS + GPS + Doppler (0.52 m, heading 0.08°) |
| Consistent? (red ≈ blue) | Yes, all filters | No for under-observable Doppler; yes once geometry is complete |

## 2. Rules of thumb worth keeping

1. **Bias $b$ you don't estimate becomes $\tfrac{1}{2}bt^2$ of position error.** Put systematic errors in the state.
2. **Inject noise through the input channel** ($Q = G\sigma^2 G^\top$), never as an arbitrary diagonal.
3. **Joseph form, always.** It is nearly free.
4. **Wrap heading with `atan2`** after every propagation and update.
5. **Validate with Monte Carlo, judge with red-vs-blue.** Blue ≫ red = model problem, not tuning problem.
6. **A sensor's value = its information *direction* × its rate × its noise.** A datasheet number alone tells you nothing.
7. **You cannot tune your way out of an observability problem.** Add geometry: another beacon, another satellite, a manoeuvre.
8. **Aid early with absolute measurements** (GPS-like); use high-rate relative sensors (Doppler-like) to stitch between them and to expose biases.
9. **Evaluate states separately**: a filter can improve velocity/heading/bias while failing at position.
10. **Verify every Jacobian with finite differences** before trusting the filter.

## 3. Exercise index

| Lesson | Exercise | Skill |
|---|---|---|
| [1](01-why-sensor-fusion.md) | Predict trajectory endpoints; predict INS drift scaling | Physics intuition |
| [2](02-ekf-foundations.md) | Identify $H$/$R$ for GPS; hand-compute a scalar update | Equation fluency |
| [3](03-1d-linear-walkthrough.md) | Beacon at 625 m (sign flip); range-only Doppler; bias σ scaling | Measurement-model surgery |
| [4](04-monte-carlo-validation.md) | Bias-state NEES; effect of `N_MC = 5` | Consistency testing |
| [5](05-2d-nonlinear-walkthrough.md) | Finite-difference Jacobian check; stronger yaw excitation; initial heading error | EKF hygiene, excitation, alignment |
| [6](06-observability-and-limits.md) | Relocate beacon 2; range-only beacons; INS-only radial/tangential consistency | Observability reasoning |

## 4. Where to go next

- **3D attitude**: replace the single heading state with a quaternion and error-state
  formulation. J. Solà, *"Quaternion kinematics for the error-state Kalman filter"* —
  the natural extension of this repository's 2D rotation story.
- **GNSS+INS engineering practice**: P. D. Groves, *Principles of GNSS, Inertial, and
  Multisensor Integrated Navigation Systems* — the standard reference for the
  architectures you built here in miniature.
- **Estimation theory depth**: Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation
  with Applications to Tracking and Navigation* — NEES/NIS consistency testing,
  observability analysis, and the theory behind every heuristic in these lessons.
- **Beyond the EKF**: rerun `NonLinear2D.m`'s scenario with larger initial errors or
  weaker updates and compare an iterated EKF or UKF; the radial/tangential failure
  mode of Lesson 6 is a good benchmark for "does a fancier filter fix geometry?"
  (Spoiler: no.)

## 5. Repository map

```text
Demo1D.m              1D linear KF — foundations            (Lessons 2-4)
NonLinear2D.m         2D nonlinear EKF — Jacobians, limits  (Lessons 5-6)
TwoBeacon2D.m         2D observability experiment           (Lesson 6)
figures/              all figures embedded in the lessons
docs/                 this lesson series
```

All scripts are self-contained GNU Octave files with no toolbox dependencies.
Run them, change one number, and predict before you look.

---

**Previous:** [Lesson 6 — Observability and Limits](06-observability-and-limits.md)
