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

#
