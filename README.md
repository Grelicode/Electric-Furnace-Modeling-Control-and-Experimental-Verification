# Electric Furnace Modeling, Control and Experimental Verification

# Overview
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
<img width="659" height="587" alt="image" src="https://github.com/user-attachments/assets/b99d7f7e-5e6d-40b0-823e-1aeb78d83e30" />
</p>


# Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Project Scope and Key Functionalities](#Project-Scope-and-Key-Functionalities)
- [Hardware and Software](#hardware-and-software)
- [Mathematical Model](#Mathematical-model-of-the-furnace)
- [Program logic in Python](#Program-logic-in-Python) 
- [TIA Portal Structure](TIA-Portal-Structure)
- [MQTT Communication](#MQTT-Communication-in-TIA-Portal)
- [Communication Verification Between PLC and Python](#Communication-Verification-Between-PLC-and-Python)
- [Containerized System Architecture](Docker-Based-Monitoring-and-Visualization-Environment)
- [License](#license)
- [Contact](#contact)

---

# System Architecture

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
<p align="center">
<img width="2165" height="1407" alt="image" src="https://github.com/user-attachments/assets/1c73c9c5-f22b-420f-a0a6-f2e077c993e1" />
</p>


# Project Scope and Key Functionalities

## 🔥 1. Mathematical Model of the Electric Furnace

- Lumped-parameter thermal model based on the energy balance equation  
- Two first-order inertial elements and transport delay (dead time)  
- Nonlinear gain dependent on heater power  
- Time constants dependent on flow rate  
- Implemented as a Python-based digital twin  
- Real-time simulation synchronized with PLC cycle (1 s sampling)

---

## 🤖 2. PLC Control and Data Processing (TIA Portal)

- Implementation on Siemens SIMATIC S7-1500  
- PI controller prepared for gain scheduling  
- Cyclic execution in OB1  
- Manual REAL-to-BYTE serialization (REAL → DWORD → BYTE)  
- Byte slicing method for float reconstruction (BYTE → DWORD → REAL)  
- Topic validation and communication error handling  
- Time-controlled cyclic publishing using LGF_Impulse block  

---

## 📡 3. MQTT-Based PLC–Python Communication

- Bidirectional communication via Mosquitto broker  
- Float32 (Big Endian) deterministic data transmission  
- Publishing of process variables (Tin, Fout, Power)  
- Receiving simulated outlet temperature (Thout)  
- Validation using status flags (`valid`, `error`)  
- Tested in simulation (PLCSIM) and integrated architecture  

---

## 🐳 4. Containerized Monitoring Environment (Docker)

- Complete system deployed using Docker Compose  
- Independent containers for:
  - Mosquitto (MQTT broker)
  - Node-RED (data processing layer)
  - InfluxDB (time-series database)
  - Grafana (visualization platform)
- Persistent volumes for configuration and historical data  
- One-command startup of the entire environment  

---

## 🗄 5. Non-Relational Time-Series Database (InfluxDB)

- Organization: `moje_org`  
- Dedicated bucket: `moj_bucket`  
- Retention policy: Forever  
- Measurement: `piec`  
- Timestamped storage of process variables:
  - Tin
  - Thout
  - Fout
  - Power  
- API token-based authentication for secure access  

---

## 📊 6. Data Visualization in Grafana

- Dedicated dashboard: **PIEC – SYMULACJA**  
- Real-time numeric indicators  
- Time-series plots for dynamic analysis  
- Gauge panels for quick state assessment  
- Historical data tables  
- Visualization of both simulation and process behavior  

---

## 🧪 7. Virtual Commissioning and Communication Verification

- Full bidirectional testing between PLC and Python model  
- Verification via:
  - Python terminal monitoring  
  - TIA Portal Watch Tables  
- Validation of correct byte-level data transmission  
- Confirmation of stable, error-free MQTT communication  
- Demonstration of digital twin integration with industrial PLC

---

# Mathematical model of the furnace

## Energy balance equation 

The foundation of the mathematical model is an equation describing the change in the temperature of water contained in the heating chamber. In the analyzed system, it was assumed that the heating chamber behaves like a perfectly mixed tank. This means that all the water inside the chamber has the same temperature at any given moment. No regions with different temperatures are formed. Therefore, it was assumed that heat transfer occurs uniformly throughout the entire volume of the furnace.

As a result, the following equation was obtained:
<p align="center">
<img width="639" height="388" alt="image" src="https://github.com/user-attachments/assets/dfc9799e-6f55-4837-a536-afa8cc311a8e" />
</p>
The first term of the equation,

describes the effect of the flow rate on the temperature change inside the furnace chamber. The flow rate 𝐹 is expressed in liters per minute, whereas the energy balance equation operates in seconds. Hence the factor 60—this is simply a unit conversion that allows the model to be simulated in real time.

The second term of the equation,

determines how much the temperature increases due to the heater operation. The heater raises the temperature of the entire furnace volume regardless of how fast the water flows. To convert the percentage value into the actual power required for simulation, it must be divided by 100. In this way, the heater power control signal is converted from percent to a real physical quantity consistent with the units used in the equation.

### Model of additional dynamics and delay

The actual temperature measured at the furnace outlet does not correspond directly to Thout.
This effect is described by an additional transfer function:
<p align="center">
  <img width="314" height="46" alt="inercja" src="https://github.com/user-attachments/assets/5b8e7e9f-2c30-4516-908f-386e48ed2fb1" />
</p>
It represents:
- the inertia of the installation components,
- the transport delay resulting from the flow path of the heated medium.


The transfer function consists of:
- two first-order lag elements,
- a gain function 𝑘
- a dead time 𝜏0=10

Where:

Tout1 — temperature after inertial filtering (°C)

k(Pph1) — system gain dependent on heater power

𝜏1&𝜏2 — time constants dependent on flow rate

𝜏0 — transport delay

In the implementation, two parallel first-order lag elements are used to smooth the signal before it is passed through the time delay.


### Dynamics parameterization
The static gain 
𝑘(𝑃ℎ1) was described using a simple linear relationship:

𝑘(𝑃ℎ1) = − 0.0002347 ⋅ 𝑃ℎ1 + 1.012 

The gain determines how strongly a change in heater power affects the temperature measured downstream of the furnace. This function scales the signal so that it matches what the sensor indicates.

Time constants:

𝜏1(𝐹) = 𝜏2(𝐹) = 19.08 ⋅ 𝐹 − 0.4293 

The time constants describe the thermal inertia of the system as well as delays resulting from the sensors and the heat exchanger. This dependency makes it possible to reflect the fact that the thermal system responds very differently at a flow rate of 2 L/min compared to 8 L/min.

The applied gain and time constants are functions of the flow rate and heater power.

# Program logic in Python

This section presents the most important elements of the program responsible for 
the mathematical simulation of the electric furnace and real-time communication with the PLC via MQTT.  
The Python script acts as a **digital twin** of the furnace: it receives input data from the PLC, 
executes one simulation step, and returns the outlet temperature.

## Define values
```python
TS = 1.0          # [s] krok modelu (PLC wysyła dane co 1 s)
CS = 4200.0       # [J/(kg·°C)]
RHO = 1.0         # [kg/L]
V1 = 1.6          # [L]
PNOM = 12000      # [W]
T0_DELAY = 10.0   # [s] stałe opóźnienie t0
DELAY_LEN = int(round(T0_DELAY / TS))  # długość kolejki opóźnienia

# Stan modelu
Tstar = 25.0   
v1 = 25.0      
v2 = 25.0    
```

## library Import 

These imports are necessary because the script must communicate with the PLC and the visualization stack via MQTT, which is handled by the paho.mqtt client. To correctly interpret and generate message payloads in the expected format (often raw bytes), struct is used for reliable binary encoding/decoding. Additionally, time, sys, and deque support stable real-time execution by enabling loop timing, safe program control, and efficient buffering of samples (e.g., for transport delay or signal smoothing).


<img width="355" height="104" alt="image" src="https://github.com/user-attachments/assets/6f2c32f2-0944-4b5e-97f6-7a59f215c089" />


import paho.mqtt.client as mqtt - provides the MQTT client used to connect to the broker, subscribe to topics, and publish process data.

import struct - enables packing and unpacking raw binary payloads (bytes) into Python data types (e.g., floats/integers).

import time - provides timing utilities such as delays (sleep) and timestamps for loop execution and scheduling.

import sys - gives access to system-level features like program arguments and clean termination (sys.exit).

from collections import deque - provides an efficient double-ended queue used as a buffer for streaming data (e.g., for delays).


## Furnace model core

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
## Dynamic gain as a function of heater power

The electrical heater exhibits nonlinear efficiency.
This is modeled using a gain dependent on the power percentage: 
```python
def k_of_Ph(Ph_percent):
    return -0.0002347 * Ph_percent + 1.012
```
## Time constant as a function of flow rate

For low flow rates the system reacts more slowly,
while for higher flow rates the dynamics become faster:
```python
def tau_of_F(F_lmin):
    F = max(F_lmin, 0.0001)
    tau = 19.08 * (F ** (-0.4293)) - 4.042
    return max(0.1, tau)
```
## MQTT broker and topics configuration

```python
BROKER, PORT = "192.168.0.110", 1883
TOPIC_IN   = "py/in"   
TOPIC_OUT  = "py/out"  
TOPIC_MON  = "py/mon" 

def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] connected rc={rc}")
    client.subscribe(TOPIC_IN, qos=0)
    print(f"[MQTT] subscribed to {TOPIC_IN}")
```
The script defines the MQTT broker address and port (BROKER, PORT) used to establish communication with Mosquitto. It also specifies three topic names: TOPIC_IN (incoming data to the Python model), TOPIC_OUT (outgoing results back to the PLC), and TOPIC_MON (an optional monitoring/debug topic for additional telemetry).

on_connect() - callback

The on_connect(client, userdata, flags, rc) function is an MQTT callback that runs automatically after the client connects to the broker. It prints the connection return code (rc) for quick diagnostics, subscribes to the input topic (TOPIC_IN) so the script can receive process variables from the PLC, and confirms the subscription in the console output.

## Receiving frames from the PLC (MQTT → Python)
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

## Publishing model results (Python → PLC)
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

## Main program loop

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
## Main loop and shutdown
```python
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterruppt - closing...")
        client.loop_stop()
        client.disconnect()
        sys.exit(0)


if __name__ == "__main__":
    main()
```

The script keeps running using an infinite while True loop with time.sleep(1.0) to prevent the program from exiting and to reduce CPU usage while MQTT callbacks handle incoming messages in the background. If the user stops the program with Ctrl+C (KeyboardInterrupt), the exception handler prints a closing message, stops the MQTT network loop (client.loop_stop()), disconnects cleanly from the broker (client.disconnect()), and terminates the application using sys.exit(0).
Entry point
The if __name__ == "__main__": main() block ensures that main() is executed only when the script is run directly (not when it is imported as a module).

# TIA Portal Structure 
To improve readability, scalability, and maintenance, a clear hierarchical project structure was adopted in TIA Portal.
### Controller Selection
The project uses a Siemens SIMATIC S7-1500 CPU 1516-3 PN/DP, identical to the controller installed in the real system.
This ensures full compatibility between simulation, testing, and real operation.
<img width="603" height="274" alt="image" src="https://github.com/user-attachments/assets/bc151765-408f-4d5d-a761-3442be1b7477" />

## PLC Simulation
The PLC program was tested without physical hardware using S7-PLCSIM:
- TCP/IP communication mode
- Single network adapter
- Virtual PLC instance with static IPv4 address
- Network configuration consistent with TIA Portal
This setup enables seamless communication between TIA Portal and the virtual PLC.
<p align="center">
<img width="329" height="353" alt="image" src="https://github.com/user-attachments/assets/58e95db7-5d4a-4057-b363-4a9c44c3a172" />
</p>
<p align="center">
<img width="391" height="334" alt="image" src="https://github.com/user-attachments/assets/f09d845d-5215-427c-83f7-902b2e15d3ed" />
</p>

## Program Blocks Organization

The PLC program is divided into:
- Organization Blocks (OB) - main execution and cyclic tasks
- Function Blocks (FB) - stateful logic (controllers, communication)
- Functions (FC) - stateless calculations and conversions
- Data Blocks (DB) - parameter storage and internal states
All blocks are grouped into dedicated folders.

<p align="center">
<img width="206" height="276" alt="image" src="https://github.com/user-attachments/assets/49568caf-c8a3-425a-9dcf-8375062bc17e" />
<p/>

# MQTT Communication in TIA Portal 

### MQTT Client Configuration in TIA Portal using LMQTT Library

To enable bidirectional communication between the PLC and the Python-based digital twin, the **LMQTT library** was used in TIA Portal.  

The core component responsible for MQTT communication is:

- **Function Block:** `LMQTT_Client` (FB11)  
- **Instance Data Block:** `MQTT_DB` (DB9)
<p align="center">
<img width="412" height="369" alt="image" src="https://github.com/user-attachments/assets/6f07afd1-413f-4ddd-a913-b170cf8d86d0" />
<p/>
<p align="center">
  <img width="267" height="332" alt="image" src="https://github.com/user-attachments/assets/925db8f4-5d6b-4dbd-b606-50532b02f151" />
<p/>
The function block is executed cyclically in `OB1`, while all communication parameters, payload buffers, and diagnostic signals are stored in `MQTT_DB`.

### Connection Parameters (connParameters)

The `connParameters` structure defines the network configuration of the MQTT client.

Configured Parameters:

| Parameter | Value |
|------------|--------|
| broker | `"192.168.0.110"` |
| port | `1883` |
| keepAlive | `60 s` |
| tls | Not used (unencrypted connection) |

### Explanation:

- **broker** – IP address of Mosquitto broker running in Docker.
- **port** – Standard MQTT port (1883, non-TLS).
- **keepAlive** – Session keepalive interval.
- **tls** – Structure prepared for secure communication (not enabled in this implementation).

The PLC and broker must operate within the same subnet to allow proper TCP/IP communication (e.g., PLCSIM IP: `192.168.0.10`).
<p align="center">
<img width="579" height="139" alt="image" src="https://github.com/user-attachments/assets/40f85ba4-acda-4b99-90e0-ad0ad10e4993" />
<p/>
  
During testing, stable bidirectional communication was confirmed between:
- SIMATIC S7-1500 (PLCSIM)
- Mosquitto broker
- Python digital twin

## Data Conversion and Slicing Method (TIA Portal)
### REAL-to-Bytes Serialization (Byte Packing)

In Network 4 of the main program (OB1), the function `FC_Convert` (FC2) prepares process data for MQTT publishing.
<p align="center">
<img width="600" height="256" alt="image" src="https://github.com/user-attachments/assets/0f31b71d-cf50-4a4b-8d6c-7469d3981d5b" />
<p/>

The function takes three REAL process values from `DANE_DB`:

- `Tin_C` - inlet temperature  
- `Fout_Lmin` - flow rate  
- `Power` - heater power  

and writes them into a 12-byte output array `bytes[0..11]`, which is directly mapped to:

- `MQTT_DB.publishMsgPayload`

This payload is then published to the broker and consumed by the Python digital twin.
<p align="center">
<img width="468" height="416" alt="image" src="https://github.com/user-attachments/assets/b1b3f9e2-a2b2-4a8a-b33f-52b7b079052c" />
<p/>

## Why serialization is needed

MQTT payloads are byte arrays.  
TIA Portal does not automatically serialize REAL values into a deterministic byte layout, therefore a manual conversion is required.

---

## Conversion strategy (REAL → DWORD → BYTE[4])

Each REAL value is converted using two steps:

### 1) REAL → DWORD (IEEE-754 bit pattern)

```scl
#dw := REAL_TO_DWORD(#Tin_C);
#bytes[0] := SHR(IN := #dw, N := 24) AND 16#FF;
#bytes[1] := SHR(IN := #dw, N := 16) AND 16#FF;
#bytes[2] := SHR(IN := #dw, N := 8)  AND 16#FF;
#bytes[3] := #dw AND 16#FF;
```
The DWORD is split into four bytes using:
- SHR (shift right)
- AND 16#FF (masking to keep only the lowest 8 bits)
Buffer initialization

Before writing new values, the buffer is cleared:
```scl
FOR #i := 0 TO 11 DO
    #bytes[#i] := 16#00;
END_FOR;
```
Endianness (Big Endian)

The serialization uses Big Endian order:

Most Significant Byte first → Least Significant Byte last

This matches the Python encoding:
```Python
struct.pack('>f', value)
```
## Byte Slicing and REAL Reconstruction

In Network 5 of OB1, incoming MQTT data from the Python digital twin is converted back into a REAL value.

After verifying that:

- `MQTT_DB.valid = TRUE`

the function `4REAL_1` (FC3) is executed.

The purpose of this function is to reconstruct a 32-bit REAL value from the received MQTT payload.
<p align="center">
<img width="431" height="262" alt="image" src="https://github.com/user-attachments/assets/44e01e62-2742-4752-af3d-d0800dd205bb" />
<p/>
<p align="center">
<img width="446" height="284" alt="image" src="https://github.com/user-attachments/assets/c7bdbb74-720c-455d-9585-d33385045934" />
<p/>


### Step 1 – Byte Slicing (Reassembling DWORD)

The Python application sends a float32 value encoded in **Big Endian** format:

```python
struct.pack('>f', value)
```
The PLC receives four bytes stored in:
- MQTT_DB.receivedMsgPayload[0..3]
Each byte is then assigned to the corresponding byte position of a temporary DWORD variable (tdword).

Example implementation using MOVE instructions:
- Assign payload bytes to DWORD structure:
- receivedMsgPayload[0] → tdword.%B3
- receivedMsgPayload[1] → tdword.%B2
- receivedMsgPayload[2] → tdword.%B1
- receivedMsgPayload[3] → tdword.%B0
<p align="center">
<img width="421" height="199" alt="image" src="https://github.com/user-attachments/assets/b3d62513-1f9d-4609-81b0-d54231049c91" />
<p/>

This restores the original 32-bit IEEE-754 binary representation of the REAL number.

The byte order is critical:
- %B3 = Most Significant Byte
- %B0 = Least Significant Byte

This matches the Big Endian format used on the Python side.

### Step 2 - DWORD to REAL Conversion

After reconstructing the DWORD, the conversion block is executed:
<p align="center">
<img width="398" height="76" alt="image" src="https://github.com/user-attachments/assets/986b6949-156e-4be0-9e41-71cf0ae5b63a" />
<p/>

The resulting REAL value is written to:
DANE_DB.Thout_mqtt

### Why Byte Slicing Is Necessary

Byte slicing is required because:
- MQTT transmits raw byte arrays
- PLC does not automatically deserialize float values
- The exact IEEE-754 structure must be reconstructed manually
- Endianness (Big Endian) must match the sender
- Without correct slicing and byte assignment, the reconstructed REAL value would be incorrect.
### Data Flow Summary
1. Python sends float32 (Big Endian).
2. PLC receives 4 raw bytes.
3. Bytes are assigned to DWORD (%B3 → %B0).
4. DWORD is converted to REAL.
5. REAL value is written to DANE_DB.Thout_mqtt.

## Message Validation (Network 6)

In Network 6, a validation mechanism ensures that received MQTT data is processed safely before being used in the control logic. The PLC compares the received topic (`MQTT_DB.receivedTopic`) with the expected topic (`DANE_DB.ExpectedTopic`) and verifies that `MQTT_DB.valid = TRUE` and `MQTT_DB.error = FALSE`. Only when all conditions are met is the value `DANE_DB.Thout_mqtt` transferred to `DANE_DB.Thout_C`, preventing incorrect or corrupted data from affecting the furnace control algorithm.
<p align="center">
<img width="541" height="140" alt="image" src="https://github.com/user-attachments/assets/d69618f6-655a-4694-905e-51575ce77733" />
<p/>
<p align="center">
<img width="562" height="209" alt="image" src="https://github.com/user-attachments/assets/26a45b60-3040-47c5-a9b7-d6936adb0ae4" />
<p/>
  
## Cyclic Publishing (Network 7)

In Network 7, cyclic MQTT publishing is implemented using the `LGF_Impulse` block with the instance `LGF_Impulse_DB` (DB10). The block generates a time-based impulse signal that controls the publication of process data. The frequency parameter is set to `1.0`, which produces a 1 Hz impulse. The generated impulse is stored in `DANE_DB.StanImp` and subsequently sets the flag `MQTT_DB.publish`, triggering cyclic data transmission to the MQTT broker. This approach ensures deterministic, time-controlled publishing independent of the OB1 cycle time.
<p align="center">
<img width="476" height="268" alt="image" src="https://github.com/user-attachments/assets/71c4ea9f-42a5-47df-bb67-e246332981e8" />
<p/>
<p align="center">
<img width="451" height="254" alt="image" src="https://github.com/user-attachments/assets/e50a9432-41be-4c5d-97f7-4de8b6fb62ce" />
<p/>

# Communication Verification Between PLC and Python

After implementing data conversion, filtering, and cyclic publishing mechanisms in OB1, the correctness of MQTT communication between the SIMATIC S7-1500 PLC and the Python digital twin was verified.
The verification was performed simultaneously in two environments:

1. **Python side (simulation layer)** – by monitoring received MQTT frames and observing the calculated furnace outlet temperature (`Thout`) in the terminal.
2. **TIA Portal side (PLC layer)** – using Watch Tables and monitoring function blocks responsible for data conversion and communication.

---

## Verification on the Python Side

The `on_message()` function handles incoming data from the PLC, performs unpacking of float32 values, executes one simulation step of the furnace model, and publishes the computed outlet temperature back to the PLC.

Successful verification confirmed:

- Correct reception of `Tin`, `Fout`, and `Power` from topic `py/in`
- Proper calculation of `Thout`
- Correct publishing of `Thout` to topic `py/out`
- Stable and error-free communication during runtime

Terminal output confirmed consistent real-time data exchange and correct numerical values.
<p align="center">
<img width="464" height="486" alt="image" src="https://github.com/user-attachments/assets/09b12df6-8eb9-4b0e-8baf-41c1001a83c5" />
<p/>


## Verification in TIA Portal

On the PLC side, communication was monitored using Watch Tables.

The following aspects were verified:

- `MQTT_DB.status` indicates active connection
- Correct received topic (`py/out`)
- Valid message length (`receivedMsgDataLen = 4`)
- Proper reconstruction of REAL value (`Thout_mqtt`)
- Successful transfer to process variable (`Thout_C`)

The Watch Table confirmed correct binary payload reconstruction and consistent numerical values matching the Python output.
<p align="center">
<img width="591" height="294" alt="image" src="https://github.com/user-attachments/assets/2fe0a89a-34c4-46a0-a1ac-6dbd952af854" />
<p/>


## Verification Result

The communication between the PLC and the Python digital twin operated correctly in both directions:

- PLC → Python (process variables)
- Python → PLC (simulated furnace output)

No transmission errors or data corruption were observed.  
The implemented serialization, byte slicing, validation, and cyclic publishing mechanisms worked as expected.

# Docker-Based Monitoring and Visualization Environment

## Docker Compose Configuration
<p align="center">
<img width="578" height="326" alt="image" src="https://github.com/user-attachments/assets/eb48bf99-f895-4860-aa38-7e5521d9dfe1" />
<p/>
The entire monitoring and visualization environment is deployed using **Docker Compose**.  
The `docker-compose.yml` file defines four containers:

- Mosquitto (MQTT broker)
- Node-RED
- InfluxDB
- Grafana

This approach ensures that all services:

- Start automatically
- Run in isolated environments
- Communicate within a shared Docker network
- Persist data using volumes

The whole environment can be started with a single command:

```bash
docker-compose up -d
```
## Node-RED 
The nodered container provides the integration layer between MQTT and InfluxDB. Node-RED subscribes to the MQTT topic py/mon, decodes the binary payload (4 × float32 BE), converts it into JSON format, and prepares it for database storage. This layer enables flexible data transformation without complex programming.
<p align="center">
<img width="580" height="189" alt="image" src="https://github.com/user-attachments/assets/e6b41a73-f67f-40fd-824c-03e0e6c1d873" />
<p/>

## InfluxDB -  Non-Relational Time-Series Database Setup
The influxdb container is responsible for time-series data storage.
<p align="center">
<img width="485" height="397" alt="image" src="https://github.com/user-attachments/assets/7d30b9c0-fd37-41aa-a7a3-b2a1e5b42423" />
<p/>

The following elements were created:
### Organization
An organization named:
- `moje_org`
was defined to logically group database resources.
### Bucket
A dedicated bucket:
- `moj_bucket`
was created for storing process data.  
The bucket retention policy was set to:
### API Token (Access Control)
To enable secure communication between Node-RED and InfluxDB, a dedicated API token:
"PIEC" was generated.
The token provides controlled write access to:
- organization: `moje_org`
- bucket: `moj_bucket`
This mechanism ensures secure and authenticated database access.

## Data Visualization in Grafana

To visualize process data stored in InfluxDB, Grafana was deployed as a separate container within the Docker environment. Grafana serves as the presentation layer of the system, providing a clear and real-time view of process variables and their historical trends.

### Data Source Configuration
The first step was to add InfluxDB as a data source in Grafana. The connection was configured using:
- URL: `http://influxdb:8086`
- Organization: `moje_org`
- Default bucket: `moj_bucket`
- API token: `PIEC`
- Query language: **Flux**

The connection was verified using Grafana’s built-in data source test functionality.

---

### Dashboard Configuration

Based on the stored data, a dedicated dashboard named:
was created as the main visualization panel of the project.

The dashboard includes:

- Numeric panels displaying current values:
- `Tin`
- `Thout`
- `Fout`
- `Power`
- Time-series plots showing variable trends
- Gauge indicators for quick state assessment
- A table displaying recent measurement history

---

### Visualization Capabilities

The dashboard enables:

- Real-time monitoring of furnace operation
- Historical analysis of process variables
- Visual comparison of temperature, flow, and power
- Rapid detection of abnormal behavior

Grafana completes the monitoring architecture by transforming stored time-series data into an intuitive and interactive visualization interface.
<p align="center">
<img width="580" height="330" alt="image" src="https://github.com/user-attachments/assets/34d05302-b4a0-4333-b04e-151c428666b7" />
<p/>
<p align="center">
<img width="581" height="321" alt="image" src="https://github.com/user-attachments/assets/9dcd9b5b-ec26-46eb-9da2-a46c4b8b8126" />
<p/>
