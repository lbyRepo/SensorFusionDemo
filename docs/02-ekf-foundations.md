# Lesson 2 — EKF Foundations

**Previous:** [Lesson 1 — Why Sensor Fusion?](01-why-sensor-fusion.md)
**Next:** [Lesson 3 — 1D Linear Walkthrough](03-1d-linear-walkthrough.md)

---

## Learning objectives

After this lesson you can:

1. Write down the five predict equations and five correct equations from memory and
   explain what each term means.
2. Map every symbol to the variable name used in the demo scripts.
3. Explain the role of $Q$, $R$, and the Kalman gain $K$ as a trust allocation.
4. Explain why sensor biases are put *into* the state vector instead of being tuned away.
5. Explain why the Joseph form covariance update is used.

Everything here is illustrated by the **linear 1D** demo — nothing "extended" yet.

---

## 1. What a filter believes

At every time step the EKF carries:

- $\hat{x}$ — the state estimate: our best guess of the truth.
- $P$ — the error covariance: our honest claim about how wrong $\hat{x}$ might be.

$P$ is not decoration. It is the *currency* the filter trades in: the Kalman gain is
computed purely from covariances, and $P$ is what makes "fuse things sensibly"
happen automatically instead of by hand-tuned gains.

## 2. The cycle

### Predict (once per IMU sample)

$$
\hat{x}_{k|k-1} = F\,\hat{x}_{k-1}
$$
$$
P_{k|k-1} = F\,P_{k-1}\,F^\top + Q
$$

- $F$ encodes the physics: position += velocity·dt, velocity += accel·dt, bias stays.
- $F P F^\top$ rotates and stretches existing uncertainty through the dynamics.
- $Q$ is **new** uncertainty injected by sensor noise — the filter's admission that
  the propagation itself is imperfect.

### Correct (whenever an aid reports)

With measurement $z$, predicted measurement $h = H\hat{x}$:

$$
\text{innovation:} \quad \nu = z - h
$$
$$
\text{innovation covariance:} \quad S = H P H^\top + R
$$
$$
\text{Kalman gain:} \quad K = P H^\top S^{-1}
$$
$$
\text{state update:} \quad \hat{x} \leftarrow \hat{x} + K\,\nu
$$
$$
\text{covariance update (Joseph form):} \quad P \leftarrow (I - KH)\,P\,(I - KH)^\top + K R K^\top
$$

Interpretation:

- $\nu$ is "what the sensor says vs what I predicted" — the *only* new information.
- $S$ is the uncertainty of that difference (prediction uncertainty + sensor noise).
- $K$ is the optimal blend: if the measurement is precise ($R$ small), $K$ approaches
  the direct-injection limit; if the prediction is very certain ($P$ small), $K$
  approaches zero and the sensor is politely ignored.

**A scalar feel for it** ($H = 1$):

| Prior $P$ | Sensor $R$ | Gain $K$ | Meaning |
|---|---|---|---|
| 1 | 100 | 0.0099 | "sensor is garbage, barely move" |
| 1 | 1 | 0.5 | "meet in the middle" |
| 1 | 0.01 | 0.990 | "sensor is gold, snap to it" |

No heuristics — the covariances alone produce this behaviour.

## 3. Symbol → code map

| Math | Meaning | `Demo1D.m` | `NonLinear2D.m` |
|---|---|---|---|
| $\hat{x}$ | state estimate | `X(:,f)` | `X(:,f)` (8 states) |
| $P$ | covariance | `P(:,:,f)` | `P(:,:,f)` |
| $F$ | process Jacobian | `F` ([Demo1D.m:306](../Demo1D.m#L306)) | `F` ([NonLinear2D.m:522](../NonLinear2D.m#L522)) |
| $Q$ | process noise | `Q` ([Demo1D.m:293](../Demo1D.m#L293)) | `Q` ([NonLinear2D.m:596](../NonLinear2D.m#L596)) |
| $z$ | measurement | `z` | `z` |
| $h$ | predicted measurement | `h` | `h` |
| $H$ | measurement Jacobian | `H` (Doppler: [:634](../Demo1D.m#L634), GPS: [:736](../Demo1D.m#L736)) | `H` (Doppler: [:656](../NonLinear2D.m#L656), GPS: [:710](../NonLinear2D.m#L710)) |
| $R$ | measurement noise | `R` | `R` |
| $\nu$ | innovation | `innovation` | `innovation` |
| $S$ | innovation covariance | `S` | `S` |
| $K$ | Kalman gain | `K` | `K` |

## 4. Process noise: how sensor noise enters the state

Accelerometer noise does not appear as a state — it enters *through the input
channel*. In 1D, acceleration noise $w_a$ changes velocity by $w_a \cdot dt$ and
position by $\tfrac{1}{2} w_a dt^2$:

$$
G = \begin{bmatrix} \tfrac{1}{2}dt^2 \\ dt \\ 0 \end{bmatrix}, \qquad
Q = G\,\sigma_{a}^{2}\,G^\top
$$

([Demo1D.m:265–275](../Demo1D.m#L265)). This $G$-matrix construction is the standard
way to inject continuous noise through the correct input channel, and it
automatically creates the correct **correlations** between position and velocity
uncertainty. In the 2D demo, $G$ is an 8×3 matrix because three noisy sensors
(accel-x, accel-y, gyro) inject through rotation-dependent channels.

Tuning intuition: $Q$ is the filter's model of *its own imperfection*. Too small →
filter goes deaf (measurements can't move it, errors grow). Too large → filter
nervously follows every noisy measurement and throws away the smoothing benefit.

## 5. Why biases live in the state vector

Both demos estimate constant IMU biases as states (`Demo1D.m`: 3-state
$[p,\ v,\ b]^\top$; `NonLinear2D.m`: 8-state, including accel-x, accel-y and gyro biases).

Why not just "absorb" bias into $Q$?

- A bias is **systematic**, not random: it integrates into a quadratic error.
  Modelling it as process noise would admit "yes, I drift" without giving the
  filter the ability to *learn and remove the cause*.
- As a state, the bias becomes observable through the aiding sensors: a constant
  bias produces a characteristic linear-in-time signature in velocity, which the
  Doppler/GPS velocity measurement exposes. The filter then *estimates and
  compensates it*, leaving only white noise to filter.
- This is why `Demo1D.m`'s INS+Doppler filter reaches accelerometer-bias RMSE of
  $3.6\times10^{-4}\ \mathrm{m/s^2}$ — a ~140× reduction from the $0.05\ \mathrm{m/s^2}$
  prior — while the INS-only filter of course never learns it at all.

Both demos also add a *tiny* random-walk process noise on the bias states
([Demo1D.m:278–298](../Demo1D.m#L278)). The true bias is constant; the small walk
exists purely to stop $P$ from collapsing to zero and freezing the filter.

## 6. Why the Joseph form?

The textbook shortcut $P \leftarrow (I-KH)P$ is cheaper but, in finite-precision
arithmetic, can gradually lose symmetry or even positive-definiteness — after
millions of updates (100 Hz × hours) that matters. The Joseph form

$$
P \leftarrow (I-KH)\,P\,(I-KH)^\top + K R K^\top
$$

is algebraically identical for the optimal gain but is *guaranteed* symmetric
positive semidefinite by construction. Both demos use it everywhere. This is the
form you should default to in production code.

---

> ### Learning points
> - The filter's real products are $(\hat{x}, P)$ — estimate **and** honesty.
> - $Q$ = "how much do I distrust my own propagation", $R$ = "how much do I distrust this sensor", $K$ settles the argument automatically.
> - Inject noise through the input channel with $Q = G\sigma^2G^\top$, never as an ad-hoc diagonal.
> - Put systematic sensor errors (biases) in the state so they can be *observed and removed*, not merely tolerated.
> - Use the Joseph form; it costs almost nothing and protects the covariance.

---

### Try it yourself

**Exercise 2.1** — In `Demo1D.m`, find the GPS update block. Write out $H$ and $R$
for the GPS update, and state what physical quantities they model.

<details>
<summary><em>Solution</em></summary>

$H = \begin{bmatrix}1 & 0 & 0\\ 0 & 1 & 0\end{bmatrix}$ (GPS measures position and
velocity directly), $R = \mathrm{diag}(\sigma_{gps,pos}^2, \sigma_{gps,vel}^2) =
\mathrm{diag}(2^2,\ 0.15^2)\ \mathrm{(m,\ m/s)^2}$. GPS is a *linear* sensor on this
state — no Jacobian approximation needed.

</details>

**Exercise 2.2** — A filter has $P = 4\ \mathrm{m}^2$, GPS reports with
$\sigma = 3$ m, and the innovation is $+5$ m. What is the state update for position?

<details>
<summary><em>Solution</em></summary>

$K = \frac{P}{P+R} = \frac{4}{4+9} = 0.308$, so the position estimate moves
$0.308 \times 5 = +1.54$ m toward the measurement, and $P$ shrinks to
$(1-K)P \approx 2.77\ \mathrm{m}^2$. Note the update respects *both* sides — a naive
"snap to GPS" would have moved 5 m.

</details>

---

**Previous:** [Lesson 1 — Why Sensor Fusion?](01-why-sensor-fusion.md)
**Next:** [Lesson 3 — 1D Linear Walkthrough](03-1d-linear-walkthrough.md)
