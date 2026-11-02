# ADR-0002: Dual-Rate Phase-Synchronized 10-Tick Decimation (250 Hz to 25 Hz)

- **Status:** Accepted
- **Date:** 2026-10-28
- **Context:** `project_2/code/src/main.cpp`, `project_2/code/include/drone_config.h`

---

## 1. Context & Problem Statement

The flight controller core loop executes strictly at $250\text{ Hz}$ ($4000\,\mu\text{s}$ cycle). Transmitting full ASCII telemetry lines (6 CSV floating point values formatted to 4 decimals, e.g. `R:-12.3456,P:34.5678,Y:89.0123,Ax:0.0123,Ay:-0.0456,Az:0.9987\n`) at $250\text{ Hz}$ requires transmitting $\sim 65$ bytes per frame:
$$65\text{ bytes} \times 10\text{ bits/byte} \times 250\text{ frames/s} = 162{,}500\text{ bps}$$

At standard baud rates of $57{,}600\text{ bps}$ or $115{,}200\text{ bps}$, transmitting every cycle blocks `Serial.print()` in the ATmega328P hardware UART TX ring buffer (64 bytes capacity), blowing past the $4000\,\mu\text{s}$ timing deadline and causing loop overrun LED triggers (`D12`).

---

## 2. Alternatives Considered

1. **Binary Packet Protocol (e.g. MSP / MavLink):**
   - *Pros:* Extremely compact (~16 bytes per frame).
   - *Cons:* Incompatible with standard Arduino Serial Monitor; human debugging impossible without dedicated unpacking software; breaks legacy tooling.
2. **Increase Baud Rate to 250,000 / 500,000 bps:**
   - *Pros:* Keeps 250 Hz sample rate.
   - *Cons:* High framing error rate with standard 16 MHz crystal on ATmega328P; unreliable through low-cost CH340 / CP2102 USB-UART bridges.
3. **Phase-Synchronized 10-Tick Decimation with Mid-Cycle Latching (Chosen):**
   - *Pros:* Preserves $250\text{ Hz}$ internal attitude filter and PID loop execution; transmits decimated $25\text{ Hz}$ packets comfortably within $57{,}600\text{ bps}$ bandwidth; latches transmission at `tick == 5` to isolate UART transmission from critical I2C sensor bursts at `tick == 0`.

---

## 3. Decision

1. **Decimation Counter (`main.cpp`):**
   - A static decimation counter `telemetry_tick` increments every $4000\,\mu\text{s}$ cycle ($0 \dots 9$).
   - When `telemetry_tick == 5` (halfway through the 10-cycle window), the firmware formats and sends the telemetry line:
     $$\text{Telemetry Rate} = \frac{250\text{ Hz}}{10} = 25\text{ Hz} \quad (\Delta t = 40.0\text{ ms})$$
   - Total transmission bandwidth consumed:
     $$65\text{ bytes} \times 10\text{ bits/byte} \times 25\text{ frames/s} = 16{,}250\text{ bps}$$
     which is well below the $57{,}600\text{ bps}$ line limit (leaving $>70\%$ margin).
2. **Visual Heartbeat Integration:**
   - A configurable LED toggle interval (`led_toggle_interval = 10` telemetry frames = $1.25\text{ Hz}$) toggles pin 13 (`LED_BUILTIN`), providing instantaneous physical feedback of active data acquisition without serial dependencies.

---

## 4. Consequences & Trade-offs

### Positive
- Loop execution time stays strictly under $2800\,\mu\text{s}$, leaving $1200\,\mu\text{s}$ safety margin per $4000\,\mu\text{s}$ frame.
- Telemetry remains 100% human-readable ASCII.
- PC 3D simulator receives clean, non-jittery $25\text{ Hz}$ kinematic updates matching the display refresh rate.

### Negative / Trade-offs
- PC simulation runs kinematics at $25\text{ Hz}$ rather than $250\text{ Hz}$. High-frequency rotational dynamics must rely on internal complementary filter integration on the MCU.
