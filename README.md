# Boost-Converter PI Tuning with Metaheuristics: Simulation and Low-Cost Hardware-in-the-Loop Optimization

[![License: MIT](https://img.shields.io/badge/code-MIT-blue.svg)](LICENSE)
[![Data: CC BY 4.0](https://img.shields.io/badge/data%20%26%20figures-CC%20BY%204.0-lightgrey.svg)](LICENSE-DATA.md)
![MATLAB R2020a+](https://img.shields.io/badge/MATLAB-R2020a%2B-orange)
![Arduino Uno](https://img.shields.io/badge/Arduino-Uno-00979D)
[![DOI](https://img.shields.io/badge/DOI-pending%20(Zenodo)-lightgrey.svg)](docs/GITHUB_ZENODO_GUIDE.md)

Code, firmware, models, measured data and manuscript source for the paper

> **Robust Metaheuristic Tuning of a Boost-Converter PI Controller: From Switching-Level Simulation to Direct Hardware-in-the-Loop Optimization on a Low-Cost Arduino Platform**
> R. Kabir, J. Mondal, N. K. Roy (manuscript in preparation)

The repository shows how four metaheuristic optimizers — **PSO, GA, GWO and NSGA-II (knee point)** — can tune the proportional–integral (PI) voltage controller of a DC–DC boost converter (i) on a switching-level simulation and (ii) **directly on a physical converter** driven by an **Arduino Uno** and a **MATLAB serial link**, with no dSPACE, STM32, FPGA or other dedicated real-time hardware. It also quantifies how well gains designed in simulation transfer to the hardware after the exact unit conversion between simulation gains and Arduino gains (factor **6.232**).

![Platform architecture](docs/images/fig_arch.png)

---

## 1. Highlights

| Track | What was done | Main result |
|---|---|---|
| **A. Simulation (500 W, 12→24 V, 10 kHz)** | PSO, GWO, GA, NSGA-II, 30 runs each, 7 scenarios, robust cost, Simulink re-ranking | PSO best in every run; worst-case overshoot 0.76 % vs 15–19 % for manual / PID Tuner designs |
| **B. Hardware-in-the-loop (prototype)** | Same optimizers run on the real converter: 4 algorithms × 3 runs, 4320 closed-loop tests, 175 min | GA design (Kp = 0.0601, Ki = 48.24) cuts the validation cost by 69 % vs the manual baseline |
| **C. Simulation-to-hardware transfer** | Simulation gains × 6.232 measured on the hardware | 1.8–4.5× the cost of the best hardware design: a measurable simulation-to-hardware gap |

Hardware validation (6 designs × 3 randomized repeats, steps 8→12→15→12→8 V, worst case over the four steps, mean ± SD over repeats):

| Design | Kp | Ki | Overshoot (%) | Rise/fall time (ms) | 5 % settling (ms)¹ | Cost J |
|---|---|---|---|---|---|---|
| Baseline (manual) | 0.1000 | 20.00 | 6.89 ± 0.33 | 72.0 ± 6.0 | 410 | 2.873 ± 0.304 |
| Reference | 0.0500 | 10.00 | 4.43 ± 1.22 | 130.0 ± 3.5 | 190 | 1.886 ± 0.063 |
| PSO | 0.0474 | 104.67 | 8.52 ± 5.62 | 16.2 ± 1.1 | 20 | 1.251 ± 0.439 |
| **GA (recommended)** | **0.0601** | **48.24** | 5.11 ± 1.45 | 22.8 ± 4.5 | 30 | **0.883 ± 0.549** |
| GWO | 0.0729 | 75.37 | 6.71 ± 1.37 | 17.1 ± 0.9 | 15 | 1.712 ± 0.319 |
| NSGA-II (knee) | 0.0000 | 5.31 | 4.81 ± 1.53 | 224.7 ± 15.7 | 300 | 1.537 ± 0.120 |

¹ post-hoc 5 % settling time from the ensemble-averaged traces (the 2 % criterion used in the cost is censored by the 0.8 s record on the hardware).
Rise time is the 10–90 % rise (up steps) or fall (down steps) time, worst of the four steps. The ensemble-trace table is in `data/hardware/tables/Table_Hardware_Ensemble.csv`.

With three runs per optimizer the smallest attainable exact two-sided rank-sum p-value is 0.10, so **no significant ranking among the optimizers is claimed**; the hardware experiment is descriptive.

---

## 2. Repository structure

```
Boost_PI_SimToHardware_Repo_Data/
├── README.md                      this file
├── LICENSE                        MIT (code, firmware)
├── LICENSE-DATA.md                CC BY 4.0 (data, figures, manuscript text)
├── CITATION.cff                   citation metadata (GitHub "Cite this repository")
├── requirements.txt               Python packages for the figure scripts
├── firmware/
│   ├── boost_controller_using_arduino_matlab/   sketch used for ALL reported experiments
│   └── boost_controller_hil/                    optional variant: 5 ms telemetry, over-voltage trip, watchdog
├── matlab/
│   ├── hardware/
│   │   ├── Run_HW_PI_Optimization.m             hardware-in-the-loop optimization (script used for the paper results)
│   │   ├── Hardware_Preflight.m                 link / ADC / protocol / step-test check before optimizing
│   │   └── Run_PI_Hardware_Optimization_mock_demo.m   same idea with an emulated converter (no hardware needed)
│   ├── simulation_prototype/Run_PI_Optimization_Paper.m   prototype-scale simulation study (analytic + Simulink, Monte Carlo)
│   ├── simulation_500W/                         500 W simulation case study (Main_Run_All.m, cost, metrics, figures)
│   └── simulink_interface/                      boost_simulink.slx, boostIO.m (serial bridge), analyze_steps.m
├── simulink/PSO_PI_Controlled_Booster.slx       switching-level model used by the simulation scripts
├── data/
│   ├── hardware/                                HW_Opt_Results.mat, HW_Validation.mat, HW_Run_Log.txt, tables/
│   └── simulation_prototype/                    Table_Sim_vs_Hardware.csv (converted gains: simulation vs hardware)
├── figures/hardware_matlab/                     MATLAB figures of the hardware run (PDF/PNG)
├── paper/                                       LaTeX manuscript, Python figure/table scripts (paper/scripts), TODO_LIST.md
└── docs/                                        HARDWARE_SETUP, SERIAL_PROTOCOL, METHOD_SUMMARY, REPRODUCE, GITHUB_ZENODO_GUIDE
```

---

## 3. The experimental prototype

| Item | Value |
|---|---|
| Input | 7.2 V laboratory supply |
| Power stage | 22.5 µH inductor, 470 µF output capacitor, SR506 Schottky diode |
| Switch | High-power MOSFET driver module (5–36 V, 15 A) driven by the Arduino PWM |
| Load | resistive bank of 10 Ω, 5 W resistors (≈ 50 Ω — see `docs/HARDWARE_SETUP.md`) |
| Controller | Arduino Uno, Timer1 8-bit fast PWM on **D9, 62.5 kHz**, duty limit 200/255 = 78.4 % |
| Sensing | divider 40 kΩ / 10 kΩ on **A5**, 10-bit ADC, 25 V full scale (24.4 mV / count) |
| Control law | incremental PI, 1 kHz, gains in ADC counts (`KP`, `KI`) |
| Host link | USB serial, 115200 baud; commands `KP=`, `KI=`, `VREF=`, `RESET`; telemetry `DATA,ms,target,vout,duty,pwm` every 10 ms |

Photographs and wiring notes: `docs/images/fig_setup_photos.jpg`, [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md). Serial protocol: [docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md).

### Gain units and the scaling factor

The sketch implements `u[k] = u[k-1] + Kp (e[k]-e[k-1]) + Ki Ts e[k]` with the error in ADC counts and `u` in PWM counts (0–200 of 255):

```
Kp_hw = Kp_sim · 255 / 40.92 = 6.232 · Kp_sim        Ki_hw = 6.232 · Ki_sim
(40.92 counts per volt = 1023 / (5 V · 5);  duty = u / 255)
```

Example: the manual hardware gains (0.1, 20) correspond to simulation gains (0.0160, 3.21).

---

## 4. Quick start

### Requirements
* MATLAB R2020a or newer (`serialport`, `writeline`); no toolbox is needed for the hardware script. The simulation scripts use Control System Toolbox (+ Simulink; Simscape Electrical if your model needs it).
* Arduino IDE (to upload the sketch) and an Arduino Uno.
* Python ≥ 3.9 with the packages in `requirements.txt` (only to regenerate the paper figures).

### A. Check the hardware first (no optimization)
1. Upload `firmware/boost_controller_using_arduino_matlab/*.ino` (or the `_hil` variant) to the Uno.
2. Close the Serial Monitor and Simulink (only one program can own the COM port).
3. Set the port in `matlab/hardware/Hardware_Preflight.m` and run it. It checks telemetry, ADC vs multimeter, commands, a small 12→14 V step with the known-good gains (0.1, 20) and the watchdog.

### B. Run the hardware optimization
1. Open `matlab/hardware/Run_HW_PI_Optimization.m`, set `cfg.PORT`, and set `cfg.budgetMin` (the script chooses population / iterations / runs to fill the budget; the paper used 180 min).
2. Optional short check first: `cfg.quickTest = true` (≈ 15 min).
3. Run. Results appear in `HW_Figures/`, `HW_Tables/`, `HW_Opt_Results.mat`, `HW_Validation.mat`, `HW_Run_Log.txt`.
4. If it is interrupted: `cfg.resume = true` and run again with the same settings. **Emergency stop:** Ctrl+C, then `clear cleanupObj` (sends `VREF=5` and `RESET`).

> **Safety.** The script aborts a test if Vout exceeds 17.5 V and restores the safe gains (0.1, 20). Make sure your output capacitor is rated well above 19 V and that the supply is current-limited. You are responsible for your own hardware.

### C. Prototype-scale simulation (Track C gains)
Add `simulink/` to the MATLAB path, open `matlab/simulation_prototype/Run_PI_Optimization_Paper.m`, keep `cfg.quickTest = true` for a ten-minute pipeline check, then set it to `false` (sized to `cfg.budgetMin`). It runs PSO/GA/GWO/NSGA-II on the averaged model over 17 corner scenarios, validates on the switching model, adds a Monte Carlo and disturbance analysis and writes figures/tables.

### D. 500 W simulation case study
Run `matlab/simulation_500W/Main_Run_All.m` (settings in `Boost_Config.m`).

### E. Regenerate the paper figures and tables
```bash
cd paper/scripts
pip install -r ../../requirements.txt
./run_all.sh          # reads paper/scripts/data, writes ../figures and ../tables
```

### F. Compile the manuscript
Compile `paper/main.tex` with pdfLaTeX + BibTeX (Overleaf: upload the `paper/` folder). Open items that only the authors can fill in are marked in red and listed in `paper/TODO_LIST.md`.

---

## 5. Data

| File | Content |
|---|---|
| `data/hardware/HW_Opt_Results.mat` | all hardware optimization results: `Res` (per run: `x`, `J`, `Jre`, `hist`, NSGA-II `front`), `hwlog` (4320 tests: algorithm, run, Kp, Ki, J, F1–F3, time), `names`, `Kp_all`, `Ki_all`, `hv`, `pMat`, `plan`, `cfg` |
| `data/hardware/HW_Validation.mat` | validation traces: `VG`, `DG` (time × step × repeat, voltage and duty), `tG`, per-step metrics `MV`, gains and names (the same arrays, plus the validation costs `Jv`, are also stored in `HW_Opt_Results.mat`) |
| `data/hardware/HW_Run_Log.txt` | console log of the complete run |
| `data/hardware/tables/` | result tables as CSV and LaTeX |
| `data/simulation_prototype/Table_Sim_vs_Hardware.csv` | simulation-designed gains, their conversion and the simulated / measured overshoot, settling time and cost |

Columns of `hwlog`: `[algorithm(1 PSO,2 GA,3 GWO,4 NSGA-II), run, Kp, Ki, J, F1, F2, F3, seconds]`.

---

## 6. Reproducibility notes and known limitations

* Hardware experiment: 3 runs per optimizer, population 6, 59 iterations (360 tests per run, 2.4 s per test), 3 validation repeats per design in randomized order. The best design of every run was re-measured twice to counter the winner's-curse bias of noisy evaluations.
* The 2 % settling time of the cost is censored by the 0.8 s record on the hardware; use the post-hoc 5 % settling time for reporting.
* Telemetry every 10 ms limits the resolution of rise times below about 10 ms.
* Only reference steps were tested on the hardware (no load/line disturbances); the load could not be varied automatically.
* The prototype-scale simulation was set up around a 10 V input while the hardware ran at 7.2 V; for a 50 Ω load the prototype conducts discontinuously, which the averaged model does not capture.
* Seeds are fixed (`cfg.seed = 2026`); hardware measurements are not exactly repeatable.

---

## 7. How to cite

Please cite the paper (reference to be added when published) and this repository. GitHub's **Cite this repository** button uses `CITATION.cff`. After the Zenodo release the DOI will be added here:

```bibtex
@software{kabir_boost_pi_simtohardware_2026,
  author  = {Kabir, Razowan and Mondal, Jopotosh and Roy, Naruttam Kumar},
  title   = {Boost-Converter PI Tuning with Metaheuristics: Simulation and Low-Cost Hardware-in-the-Loop Optimization},
  year    = {2026},
  version = {1.0.0},
  url     = {https://github.com/rkrazowan/Boost_PI_SimToHardware_Repo_Data},
  doi     = {TODO: Zenodo DOI}
}
```

## 8. License

* Code and firmware: **MIT License** ([LICENSE](LICENSE)).
* Measured data, figures, tables and manuscript text: **Creative Commons Attribution 4.0 International** ([LICENSE-DATA.md](LICENSE-DATA.md)).

## 9. Authors and contact

* **Razowan Kabir** (corresponding author) — Department of Electrical and Electronic Engineering, Khulna University of Engineering and Technology (KUET), Khulna, Bangladesh — rkazowan@gmail.com — ORCID [0009-0008-6274-8117](https://orcid.org/0009-0008-6274-8117)
* **Jopotosh Mondal** — Electrical and Electronic Engineering, Gopalganj Science and Technology University (GSTU), Gopalganj, Bangladesh
* **Dr. Naruttam Kumar Roy** — Department of Electrical and Electronic Engineering, KUET, Khulna, Bangladesh

Questions and issues: please open a GitHub issue.

## 10. Repository status

Items still to add by the authors are listed in [docs/GITHUB_ZENODO_GUIDE.md](docs/GITHUB_ZENODO_GUIDE.md#6-files-still-missing-from-this-repository).
