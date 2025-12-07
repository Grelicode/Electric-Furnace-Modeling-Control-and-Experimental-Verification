# Electric Furnace Modeling, Control and Experimental Verification

## Overview
This project focuses on creating a complete workflow for modeling, simulating, and experimentally verifying the control of an electric furnace used as part of the heat distribution and exchange installation at the Department of Automation and Robotics.  
The work includes:

- Development of a **mathematical model** of the electric furnace  
- **Simulation** of the model in Python  
- Real-time **communication with a PLC** controller using MQTT  
- Design and implementation of a **PI controller with gain scheduling**  
- **Hardware-in-the-loop tests**  
- **Data acquisition and visualization** using Node-RED, InfluxDB and Grafana  
- Final **experimental validation** on a real furnace installation

<p align="center">
  <img src="image-url-here" alt="Electric furnace installation" width="400"/>
</p>

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Folder Structure](#folder-structure)
- [Hardware and Software](#hardware-and-software)
- [Mathematical Model](#mathematical-model)
- [MQTT Communication](#mqtt-communication)
- [Control Algorithm](#control-algorithm)
- [Installation & Setup](#installation--setup)
- [Running the Simulation](#running-the-simulation)
- [Visualization](#visualization)
- [Future Work](#future-work)
- [License](#license)
- [Contact](#contact)

---

## System Architecture

The system consists of three main layers:

1. **Simulation Layer (Python)**  
   - Mathematical model of the furnace  
   - Digital twin for the PLC  
   - MQTT client for real-time communication

2. **Control Layer (PLC – Siemens S7)**  
   - PI controller implementation  
   - Gain scheduling logic  
   - Safety interlocks and heater control  
   - Communication with Python model via MQTT broker (optionally secured with TLS)

3. **Visualization & Data Layer**  
   - Node-RED dashboard for user interface  
   - InfluxDB as a time-series database  
   - Grafana panels for real-time and historical data visualization  

---

## Features

### 🔥 1. Mathematical Model of the Furnace
- Lumped-parameter thermal model  
- First-order plus dead-time (FOPDT) approximation (configurable)  
- Adjustable parameters (thermal capacity, heat transfer coefficient, heater power)  
- Simple enough to be implemented both in Python and PLC (SCL)

### 🤖 2. PLC Control System
- PI controller with anti-windup  
- Gain scheduling based on temperature ranges  
- Safety logic: overtemperature protection, emergency stop, communication timeout  
- Implementation in TIA Portal (SCL / LAD)

### 📡 3. Python–PLC Communication
- MQTT publish/subscribe    
- Option to enable TLS encryption (Mosquitto + OpenSSL certificates)  
- Clear separation between simulation mode and real-plant mode

### 📊 4. Visualization Platform
- Node-RED dashboard for operator interface  
- InfluxDB for process data logging  
- Grafana dashboards for advanced plotting, comparison of model vs. real process  
- Export of data for further analysis

### 🧪 5. Experimental Validation
- Closed-loop tests with real furnace after successful simulations  
- Comparison of setpoint tracking and disturbance rejection  
- Evaluation of the gain-scheduled PI controller performance  
- Possibility to tune controller directly from simulation results

---

## Folder Structure

Data & Visualization: Node-RED, InfluxDB, Grafana
Debugging Tools: MQTT Explorer, Wireshark / Bettercap (optional, for security tests)

## Program logic (Python)

This section presents the most important elements of the program responsible for 
the mathematical simulation of the electric furnace and real-time communication with the PLC via MQTT.  
The Python script acts as a **digital twin** of the furnace: it receives input data from the PLC, 
executes one simulation step, and returns the outlet temperature.
### Receiving frames from the PLC (MQTT → Python)

Python subscribes to the topic `py/in` and receives three `float32` values (big-endian):

- **Tin** – inlet temperature  
- **Fout** – flow rate [L/min]  
- **Power** – heater power [%]  

```python
Tin, Fout, power = struct.unpack('>fff', msg.payload[:12])
Ph_pct = max(0.0, min(100.0, power))

print(
    f"[{ts}] IN  Tin={Tin:.3f}°C | "
    f"Fout={Fout:.3f} L/min | "
    f"Power={power:.3f} (→ {Ph_pct:.1f}%)"
)
```
### Furnace model core

- The model includes:
- thermal balance equation,
- internal states Tstar, v1, v2,
- nonlinear dependencies of gain and time constant,
- pure time delay implemented as a FIFO queue.
```python
def furnace_step(Tin, F_lmin, Ph_percent):
    Ph = min(max(Ph_percent, 0.0), 100.0)
    F = max(F_lmin, 0.0)

    k = k_of_Ph(Ph)
    tau = tau_of_F(F)
    q_el = (Ph * PNOM) / 100.0

    dTdt = (F/(60.0*V1))*(Tin - Tstar) + q_el/(CS*RHO*V1)
    dv1_dt = (Tstar * k - v1) / tau
    dv2_dt = (v1 - v2) / tau

    # State integration
    Tstar += TS * dTdt
    v1    += TS * dv1_dt
    v2    += TS * dv2_dt

    # Time delay – FIFO queue
    delay_queue.append(v2)
    return delay_queue.popleft()
```
### Dynamic gain as a function of heater power

The electrical heater exhibits nonlinear efficiency.
This is modeled using a gain dependent on the power percentage: 
```python
def k_of_Ph(Ph_percent):
    return -0.0002347 * Ph_percent + 1.012
```
### Time constant as a function of flow rate

For low flow rates the system reacts more slowly,
while for higher flow rates the dynamics become faster:
```python
def tau_of_F(F_lmin):
    F = max(F_lmin, 0.0001)
    tau = 19.08 * (F ** (-0.4293)) - 4.042
    return max(0.1, tau)
```
### Publishing model results (Python → PLC)

The model sends the outlet temperature (Thout) back to the PLC via py/out.
Additionally, a monitoring frame (Tin, Fout, Power, Thout) is published on py/mon.
```python
client.publish(TOPIC_OUT, struct.pack('>f', Thout), qos=0)

client.publish(
    TOPIC_MON,
    struct.pack('>ffff', Tin, Fout, power, Thout),
    qos=0,
)
```
### Main program loop

Python runs in an event-driven mode:
it waits for messages and processes them immediately when received.
```python
client = mqtt.Client(client_id="PIEC")
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, keepalive=60)
client.loop_start()

print("[READY] IN:py/in  OUT:py/out  MON:py/mon")
```

