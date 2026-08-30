# SensorFusionDemo — Learn Sensor Fusion with EKFs, from First Linear Principles to Nonlinear Reality

A hands-on, Monte-Carlo-driven course on Extended Kalman Filter sensor fusion,
built around three self-contained GNU Octave demos. Start with a 1D linear filter
where everything is exact, then watch the same architecture meet nonlinearity and
observability limits in 2D — and learn to *diagnose* both with the filter's own
covariance.

## What you will learn

- The EKF predict/correct cycle, with every equation mapped to real code
- Why inertial drift is quadratic, and how aiding bounds it
- How to validate filters with Monte Carlo runs: RMSE, empirical vs claimed 1σ (consistency), NEES
- What "extended" means: Jacobians for rotated dynamics and range/range-rate measurements
- **Why a sensor that is spectacular in one geometry can be nearly useless in another** —
  and how a second beacon turns a 67 m error into 9 cm

## The course

| # | Lesson | One-line goal |
|---|--------|---------------|
| 1 | [Why sensor fusion?](docs/01-why-sensor-fusion.md) | The drift problem and the sensor cast |
| 2 | [EKF foundations](docs/02-ekf-foundations.md) | The five + five equations, mapped to code |
| 3 | [1D linear walkthrough](docs/03-1d-linear-walkthrough.md) | Read `Demo1D.m` end-to-end; Doppler wins by 1800× |
| 4 | [Monte Carlo validation](docs/04-monte-carlo-validation.md) | Grey/red/blue plots; consistency and overconfidence |
| 5 | [2D nonlinear walkthrough](docs/05-2d-nonlinear-walkthrough.md) | Jacobians, heading wrap, the 8-state EKF — and a surprise |
| 6 | [Observability and limits](docs/06-observability-and-limits.md) | Radial vs tangential: why one beacon fails and two fix it |
| 7 | [Summary and exercises](docs/07-summary-and-exercises.md) | Rules of thumb, exercise index, further study |

Every lesson ends with *Try it yourself* exercises (solutions included) that only
require editing one or two lines of the demo scripts.

## The demos

| Script | Model | Filters compared | Runtime |
|---|---|---|---|
| [`Demo1D.m`](Demo1D.m) | 1D linear, constant acceleration, 50 s | INS, INS+GPS, INS+Doppler, INS+GPS+Doppler | ~40 s |
| [`NonLinear2D.m`](NonLinear2D.m) | 2D nonlinear S-curve with yaw, 60 s, 8 states | same four | ~3 min |
| [`TwoBeacon2D.m`](TwoBeacon2D.m) | 2D nonlinear, beacons only (no GPS) | INS, INS+1 beacon, INS+2 beacons | ~3 min |

## Quickstart

Requires [GNU Octave](https://octave.org) ≥ 7 (tested on 11.1). No toolboxes.

```bash
# Debian/Ubuntu
sudo apt install octave
# macOS
brew install octave

git clone <this-repo>
cd SensorFusionDemo
octave Demo1D.m        # figures 1-10 + console summary
octave NonLinear2D.m   # figures 1-9  + console summary
octave TwoBeacon2D.m   # figures 1-4  + console summary
```

Prefer not to wait? All figures shown in the lessons are pre-rendered in
[`figures/`](figures/).

## How to read the error plots

All Monte Carlo plots in this repository use one colour language:

- **Light grey** — every individual run's error trajectory (50 runs, redrawn biases)
- **Red** — the filter's *claimed* 1σ ($\sqrt{P}$, averaged over runs)
- **Blue** — the *empirical* 1σ (standard deviation of the actual error across runs)

**Red ≈ blue: consistent filter. Blue ≫ red: overconfident filter** — the most
important failure signature in fusion design (see [Lesson 4](docs/04-monte-carlo-validation.md)
and [Lesson 6](docs/06-observability-and-limits.md)).

## Headline results (50-run Monte Carlo, final time)

| Scenario | INS only | Best Doppler-only | Best overall |
|---|---|---|---|
| 1D linear | 75.4 m | 0.041 m (INS+Doppler) | 0.041 m |
| 2D nonlinear | 79.1 m | 69.0 m (INS+Doppler) | 0.52 m (INS+GPS+Doppler) |
| 2D, beacons only | 75.2 m | 67.4 m (1 beacon) | **0.092 m (2 beacons)** |

Same sensors, same filter discipline — the difference is **geometry**.

## Repository layout

```text
docs/                  lesson series (start at docs/01-why-sensor-fusion.md)
figures/1d,2d,2b/      pre-rendered lesson figures
Demo1D.m               1D linear KF demo
NonLinear2D.m          2D nonlinear EKF demo
TwoBeacon2D.m          2-beacon observability demo
```

## License

[MIT](LICENSE)
