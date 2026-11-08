# Robust Metaheuristic Tuning of a Boost-Converter PI Controller: From Simulation to Direct Hardware-in-the-Loop Optimization

[![MATLAB R2024a](https://img.shields.io/badge/MATLAB-R2024a-orange.svg)](https://www.mathworks.com/)
[![Arduino Uno](https://img.shields.io/badge/Hardware-Arduino%20Uno%20(ATmega328P)-blue.svg)](https://store.arduino.cc/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Official repository accompanying the study on robust metaheuristic tuning (PSO, GWO, GA, NSGA-II) of DC-DC step-up boost converter PI controllers, featuring both high-power simulation benchmarks and direct low-cost Hardware-in-the-Loop (HIL) optimization on an Arduino Uno[cite: 1, 2, 3].

---

## 📂 Repository Overview

This repository is organized into three primary tracks:
1. **Simulation Case Study (Track A):** A 500 W boost converter evaluated across seven rigorous operating scenarios using averaged small-signal models and switching-level MATLAB/Simulink models[cite: 4, 8, 9].
2. **Hardware-in-the-Loop Optimization (Track B):** Direct optimization on a low-cost physical prototype using an Arduino Uno board and a MATLAB serial link (executing 4,320 closed-loop tests without dSPACE or expensive real-time targets)[cite: 3, 6, 8].
3. **Simulation-to-Hardware Transfer (Track C):** Quantitative analysis of the simulation-to-hardware gap caused by discontinuous conduction, sampling delay, quantization, and noise[cite: 1, 3, 7, 8].

---

## ⚙️ Key Technical Specifications

### Hardware Platform & Controller (Track B)
* **Microcontroller:** Arduino Uno (`ATmega328P`, 16 MHz)
* **Power Stage:** $22.5\,\mu\text{H}$ toroidal inductor, $470\,\mu\text{F}$ output capacitor, SR506 Schottky diode, and high-power MOSFET driver module[cite: 6, 7]
* **Switching & Sampling:** Timer 1 fast PWM at $62.5\,\text{kHz}$; 1 kHz incremental digital PI control loop via microsecond interrupts[cite: 6, 7]
* **Sensing:** 10-bit ADC via a $40\,\text{k}\Omega / 10\,\text{k}\Omega$ voltage divider network ($25\,\text{V}$ full scale, $24.4\,\text{mV/count}$)[cite: 7]

### Exact Simulation-to-Hardware Gain Conversion
To map continuous simulation gains ($K_{p,\text{sim}}, K_{i,\text{sim}}$ in duty per volt / volt-sec) to discrete Arduino units ($K_{p,\text{hw}}, K_{i,\text{hw}}$ in duty counts per ADC count), use the derived scale factor $\zeta = 6.232$[cite: 7, 8]:
$$K_{p,\text{hw}} = K_{p,\text{sim}} \frac{255}{N_v} = 6.232 \, K_{p,\text{sim}}, \quad K_{i,\text{hw}} = 6.232 \, K_{i,\text{sim}}$$

---

## 🚀 Quick Start Guide

### 1. Arduino Firmware Setup
1. Open the Arduino IDE.
2. Load the sketch located in `hardware/arduino_pi_controller/arduino_pi_controller.ino`.
3. Ensure Timer 1 is configured for 62.5 kHz fast PWM on pin D9 and upload the sketch to your Arduino Uno[cite: 7]. 
4. Keep the serial monitor closed when running MATLAB scripts to allow the serial port link.

### 2. Running Simulation Optimization (Track A)
1. Open MATLAB R2024a (with Simulink, Control System Toolbox, and Global Optimization Toolbox installed)[cite: 13, 38].
2. Run `simulation/averaged_model_script.m` to execute the 30 independent runs of PSO, GWO, GA, and NSGA-II[cite: 11, 13].
3. Inspect worst-case margins and step responses using the switching Simulink model `boost_500w_model.slx`[cite: 4, 8, 11].

### 3. Running Hardware-in-the-Loop Optimization (Track B)
1. Connect your Arduino Uno prototype via USB (noting your assigned COM port, e.g., `COM3`)[cite: 6, 7].
2. Open MATLAB and navigate to `hardware/matlab_hil_interface/`.
3. Run `hil_test_sequencer.m` to initiate the automated test chain ($8\,\text{V} \rightarrow 12\,\text{V} \rightarrow 15\,\text{V}$ reference step protocols)[cite: 14].

---

## 📊 Performance Summary

* **Robustness:** Conventional nominal-point tuners (manual / automatic PID Tuner) result in $15\text{--}19\%$ overshoot and instability under light-load conditions[cite: 1, 16].
* **Optimized Results:** The hardware-optimized Genetic Algorithm (GA) design ($K_p = 0.0601, K_i = 48.24$) reduced the validation cost by **69%** compared to the manual baseline, achieving a worst-case settling time of **15–30 ms** (compared to 410 ms for the baseline)[cite: 1, 28].

---

## 📝 License
This project is released under the MIT License - see the [LICENSE](LICENSE) file for details.