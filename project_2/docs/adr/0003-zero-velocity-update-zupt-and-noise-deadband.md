# ADR-0003: Zero-Velocity Update (ZUPT) and $0.05g$ Noise Deadband

- **Status:** Accepted
- **Date:** 2026-10-28
- **Context:** `project_2/odometry/drone_model.py`, `project_2/odometry/sim_3d.py`

---

## 1. Context & Problem Statement

In double-integration inertial odometry:
$$v_k = v_{k-1} + a_w \cdot \Delta t$$
$$p_k = p_{k-1} + v_k \cdot \Delta t$$
where $a_w$ is world-frame linear acceleration after gravity removal:
$$a_w = R(\phi, \theta, \psi) \cdot a_b - \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$$

Two physical phenomena corrupted position estimation in practice:
1. **MEMS Sensor Floor Noise:** Even when motionless on a desk, thermal and mechanical vibration produces raw noise in the range of $\pm 0.015g$ to $\pm 0.040g$. In an open-loop integrator, this integrates into massive fictitious velocity ($>1.5\text{ m/s}$) within 5 seconds.
2. **Deceleration Clipping Traps Velocity:** When the user moves the sensor by hand from Point A to Point B and stops, the initial acceleration pulse is sharp ($>0.1g$), whereas the stopping deceleration is gentle and gradual. If a naive deadband cuts off the gradual deceleration below threshold, the negative area does not cancel the positive area. By Newton's First Law in numerical integration ($v_k = v_{k-1} + 0$), the vehicle continues flying forward forever at constant speed.

---

## 2. Alternatives Considered

1. **Velocity Damping Factor ($v_k = \lambda \cdot v_{k-1}$ with $\lambda = 0.95$):**
   - *Pros:* Simple to code.
   - *Cons:* Destroys true sustained flight dynamics, introducing artificial viscous drag and causing the drone to stop moving even when subject to continuous real thrust.
2. **High-Pass Leaky Integrator on Position:**
   - *Pros:* Bounds position error.
   - *Cons:* Pulls the drone back toward origin $(0, 0, 0)$ whenever it holds position.
3. **Threshold-Gated Zero-Velocity Update (ZUPT) + $0.05g$ Deadband (Chosen):**
   - *Pros:* Ground truth physics: quadcopters at rest exhibit acceleration magnitudes nearly identical to $1g$ with near-zero angular rates. Detecting stationary state over a short temporal window ($\ge 3$ consecutive frames = $120\text{ ms}$) allows rapidly extinguishing residual integrated velocity without affecting high-dynamic maneuvers.

---

## 3. Decision

1. **Noise Deadband Threshold ($0.05g$):**
   - In `InertialPositionEstimator.update()`, any world-frame linear acceleration with Euclidean norm $\|a_w\| < 0.05g$ ($< 0.4905\text{ m/s}^2$) is clamped to zero:
     $$\text{if } \|a_w\| < 0.05g \implies a_w = [0, 0, 0]^T$$
2. **ZUPT State Machine:**
   - A stationary counter increments whenever linear acceleration is below the deadband threshold.
   - If the stationary count reaches $\ge 3$ consecutive frames ($\ge 120\text{ ms}$ at $25\text{ Hz}$):
     $$v_w \leftarrow 0.5 \times v_w \quad (\text{or } 0\text{ when } \|v_w\| < 0.01\text{ m/s})$$
   - This smoothly decays trapped velocity to absolute zero within 3 to 4 frames after movement stops.
3. **Full 64-bit Floating Point Kinematics:**
   - Removed artificial 2-decimal truncation on internal acceleration variables (`accel_decimals = None` default), preserving full sub-millimeter precision across mathematical operations.

---

## 4. Consequences & Trade-offs

### Positive
- Vehicle comes to a complete, crisp stop in 3D simulation when the physical sensor is placed stationary on a surface.
- Stationary drift at rest is completely eliminated ($0.000\text{ m/s}$ velocity, stationary position).
- Full float64 precision avoids quantization discretization artifacts.

### Negative / Trade-offs
- Very slow creeping motions below $0.05g$ ($0.49\text{ m/s}^2$) will not register as linear displacement. This is a known physical limitation of consumer MEMS IMU double-integration without external aiding (GPS, optical flow, or UWB).
