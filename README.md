# 🚀 Solid Rocket Motor Test Stand – DAQ System Overview

Over the course of a year and a half (**Nov 2023 – April 2025**), I designed and built a data acquisition (DAQ) system for a **solid rocket motor (SRM) test stand**. This system was developed to accurately record **thrust** and **chamber pressure** data during static fire tests of **AeroTech reloadable SRMs**.

The DAQ setup interfaces with both a **load cell** and a **pressure transducer**, conditions their analog signals using amplification circuitry, and logs the data for post-processing and analysis.

---

## 🛠️ Hardware

| Component | Description |
|----------|-------------|
| [**NI cDAQ-9174**](https://www.ni.com/docs/en-US/bundle/cdaq-9174-specs/page/specs.html) | Chassis for data acquisition modules |
| [**NI 9201**](https://www.ni.com/docs/en-US/bundle/ni-9201-specs/page/specs.html) | 12-bit analog input module (used to read sensor voltages) |
| [**NI 9482**](https://www.ni.com/docs/en-US/bundle/ni-9482-sbrio-9482-specs/resource/ni-9482-sbrio-9482-specs.pdf) | Electromechanical relay module for control logic |
| [**DR-ODC5**](https://www.sensata.com/sites/default/files/a/sensata-dr-series-output-modules-datasheet.pdf) | Solid-state relay used for ignition |
| [**PS-1S400EP**](https://www.computer-world.pro/t-win-ps-1s400ep-400w-p-97811.html) | 400 W power supply unit |
| [**Computer Supply Breakout Board**](https://www.amazon.com/GeeekPi-Breakout-Adapter-Terminal-Computer/dp/B08MC389FQ) | ATX breakout board for regulated voltage output |
| [**INA110**](https://www.ti.com/lit/ds/symlink/ina110.pdf) | Instrumentation amplifier for signal conditioning (load cell) |
| [**LC113B-2K Omega S‑type Load Cell**](https://mx.omega.com/pptst_eng/LC103B.html) | Measures thrust in lbf |
| [**PX309‑3KG10V Omega Pressure Transducer**](https://assets.omega.com/pdf/test-and-measurement-equipment/pressure/pressure-transducers/PX309.pdf) | Measures chamber pressure in psi |
| [**AeroTech RMS‑75 Combustion Chamber**](https://aerotech-rocketry.com/products/product_b2ff983a-e5fe-18d7-055b-b3266c6fedc6) | Reusable RMS‑75 motor case |
| [**AeroTech L1420R‑PS RMS‑75/5120 Reload Kit**](https://aerotech-rocketry.com/products/product_3872d294-577c-353f-9773-6594597dfda3) | Solid propellant reload kit (Redline™) |
| **1 µF Tantalum Capacitors** | Used for filtering power supply noise |
| **General wiring, connectors, and test stand** | Structural and electrical integration components |

---

## 💻 Software

- **NI LabVIEW**  
  Used to build the real-time data acquisition interface. It handled:
  - Signal acquisition
  - Relay control for ignition
  - Timing control and logging of thrust and pressure data

- **Microsoft Excel**  
  LabVIEW export the data into Excel spreadsheets. Each test generates:
  - Raw load cell voltage
  - Raw pressure transducer voltage
  - Converted thrust (lbf)
  - Converted chamber pressure (psi)

---

## 📘 Additional Information

The data analyzed in this repository comes from the **first hot fire test** ever recorded using this system and is used here for demonstration purposes. The goal of this repository is to showcase what can be done with acquired SRM test data — not to claim perfection. There are always better methods to explore, and continuous improvement is part of the process.

This project serves as a hands-on opportunity for me to strengthen my skills across **mechanical**, **electrical**, and **computer engineering**, as well as to sharpen my overall **data analysis abilities**, particularly in the context of **aerospace and rocketry**.

