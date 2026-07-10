# EV Dashboard & Multi-Directional ADAS Controller (STM32F103C8t6)

A firmware implementation for an Electric Vehicle (EV) telemetry dashboard integrated with Multi-Directional Advanced Driver Assistance Systems (ADAS), designed for the STM32F103C8t6 (Blue Pill) microcontroller. This system samples real-time signals from dashboard inputs while simultaneously processing spatial proximity data using high-precision HC-SR04 ultrasonic sensor networks (Forward, Left, and Right channels). It dynamically computes metrics such as vehicle speed, remaining range, and multi-directional collision risks, streaming structured telemetry and active safety alerts via USART to a terminal screen interface.

---
## 🚀 Features

### 🛠️ Core EV Telemetry
* **Multi-Channel ADC Acquisition:** Reads raw voltage potentials from active input sliders representing vehicle physics parameters.
* **Input Capture Processing:** Tracks wheel rotation encoder frequency signals via timer capture registers to establish exact ground speed.
* **Regenerative Braking Support:** Automatically computes negative torque adjustments when the braking system is active.
* **Stable Serial Telemetry Engine:** Transmits structured parameter lines formatted for real-time VT100 virtual terminals.
  
### 🛡️ Integrated Multi-Directional ADAS Features
* **Forward Collision Warning (FCW):** Tracks forward distance via ultrasonic hardware timers to calculate Time-to-Collision (TTC) based on ground speed, triggering dynamic visual warning thresholds.
* **Emergency Brake Assist (EBA):** Overrides manual throttle configurations and injects emergency braking demands if a critical forward hazard threshold is breached.
* **Lateral Blind-Spot Monitoring (Left & Right):** Continuous digital triggering and echo-pulse capturing scan left/right lanes to flag immediate spatial overtaking hazards with integrated speed-gate validation filters.
* **Hysteresis Software Filtering:** Processes raw timer clock ticks through software noise filters to minimize false-positive safety flags caused by component tolerances or environmental drift.
---
## 🛠️ Hardware & Pin Configuration
The system architecture maps inputs and outputs to specific peripherals on the **STM32F103C8t6** chip:

| Pin Name | Peripheral Configuration | Feature Assignment | Operational Range |
| :--- | :--- | :--- | :--- |
| **PA0** | ADC1_IN0 | Accelerator Position (`acc_p`) | 0V to 3.3V (0% to 100%) |
| **PA1** | ADC1_IN1 | Brake Position (`brk_p`) | 0V to 3.3V (0% to 100%) |
| **PA2** | ADC1_IN2 | State of Charge (`soc`) | 0V to 3.3V (0% to 100%) |
| **PA3** | ADC1_IN3 | Motor Temperature (`m_tmp`) | 0V to 3.3V (0°C to 150°C) |
| **PB0** | GPIO_Output | **Front Sensor Trigger Pin** (`Trig_F`) | Digital High / Low Pulse |
| **PB1** | TIM3_CH4 (Input Capture) | **Front Sensor Echo Pin** (`Echo_F`) | Pulse Width Measurement |
| **PB2** | GPIO_Output | **Left Sensor Trigger Pin** (`Trig_L`) | Digital High / Low Pulse |
| **PB3** | TIM2_CH2 (Input Capture) | **Left Sensor Echo Pin** (`Echo_L`) | Pulse Width Measurement |
| **PB4** | GPIO_Output | **Right Sensor Trigger Pin** (`Trig_R`) | Digital High / Low Pulse |
| **PB5** | TIM3_CH2 (Input Capture) | **Right Sensor Echo Pin** (`Echo_R`) | Pulse Width Measurement |
| **PB8** | GPIO_Output | **Collision Alarm LED Indicator** (`COLL`) | Active High Warning Drive |
| **PB9** | GPIO_Output | **Right Blind-Spot Indicator LED** (`BSD_R`) | Active High Status Output |
| **PB10** | GPIO_Output | **Left Blind-Spot Indicator LED** (`BSD_L`) | Active High Status Output |
| **PA8** | TIM1_CH1 (Input Capture) | Speed Encoder Signal | Input Capture Pulse Period |
| **PA9** | USART1_TX | Telemetry Serial Transmitter | 115200 Baud |
| **PA10** | USART1_RX | Telemetry Serial Receiver | 115200 Baud |

---
## 📊 Core Mathematical Formulations
The controller converts 12-bit digital value frames ($0 \text{ to } 4095$) and timer intervals into human-readable metrics using the following calculations:

### 1. Control Percentages (ACC / BRK / SOC)
$$\text{Percentage } = \left( \frac{\text{ADC}_{\text{raw}}}{4095} \right) \times 100$$

### 2. Motor Temperature Translation (TMP)
$$\text{Temperature (°C)} = \text{Min Temp} + \left[ \left( \frac{\text{ADC}_{\text{raw}}}{4095} \right) \times (\text{Max Temp} - \text{Min Temp}) \right]$$

### 3. Estimated Remaining Range (RNG)
$$\text{Remaining Range (km)} = \left( \frac{\text{SOC}}{100} \right) \times \text{Maximum Full Battery Range (km)}$$

### 4. Ground Velocity Calculation (SPD)
The Input Capture timer evaluates elapsed timer counts between signal edges to determine frequency ($f$), which translates into speed using the wheel circumference profile:

$$f = \frac{\text{Timer Clock Frequency (Hz)}}{\text{Captured Ticks}}$$
$$\text{Speed (km/h)} = f \times \text{Wheel Calibration Constant}$$

### 5. Ultrasonic Distance Formula (Echo Pulse Width)
To calculate distance ($D$) from the HC-SR04 sensors, the Input Capture timer tracks the time the Echo pin remains high ($\text{Ticks}_{\text{echo}}$) at a given 

Timer Clock Frequency:
$$\text{Time (s)} = \frac{\text{Ticks}_{\text{echo}}}{\text{Timer Clock Frequency (Hz)}}$$
$$\text{Distance (cm)} = \frac{\text{Time (s)} \times 34300 \text{ cm/s}}{2}$$

### 6. ADAS Forward Time-to-Collision (TTC)
The forward collision avoidance framework computes real-time proximity degradation rates relative to instantaneous velocity to identify braking windows:

$$\text{Time-to-Collision (s)} = \frac{\text{Front Distance (m)}}{\text{Speed (m/s)}}$$
$$\text{Emergency State} = \begin{cases} 
      \text{ACTIVE (EBA Enabled),} & \text{if } \text{TTC} \le \text{Threshold}_{\text{critical}} \\
      \text{WARNING (FCW Alert),} & \text{if } \text{Threshold}_{\text{critical}} < \text{TTC} \le \text{Threshold}_{\text{warning}} \\
      \text{SAFE,} & \text{if } \text{TTC} > \text{Threshold}_{\text{warning}}
   \end{cases}$$
   
### 7. Side Blind-Spot Hazard Assessment

Lateral warning vectors validate independent proximity thresholds ($D_{\text{side}}$) alongside minimum velocity constraints ($V_{\text{gate}}$) to filter stationary objects during deployment:

$$\text{Left Hazard State} = \begin{cases} 1, & \text{if } D_{\text{left}} \le \text{Threshold}_{\text{lateral}} \ \ \text{AND} \ \ V_{\text{vehicle}} > V_{\text{gate}} \\ 0, & \text{otherwise} \end{cases}$$

$$\text{Right Hazard State} = \begin{cases} 1, & \text{if } D_{\text{right}} \le \text{Threshold}_{\text{lateral}} \ \ \text{AND} \ \ V_{\text{vehicle}} > V_{\text{gate}} \\ 0, & \text{otherwise} \end{cases}$$

---

## 💻 Simulation & Deployment Guide
This project is fully optimized for verification using the **PICSimLab** simulation suite.
1. Build the source workspace inside **STM32CubeIDE** (`Ctrl + B`) to generate a compiled `.bin` or `.hex` file.
2. Launch **PICSimLab** and load your custom board configuration workspace.
3. Select **File -> Load Hex/Bin** and direct it to the binary inside your project's `Debug` directory.
4. Launch **Modules -> Virtual Terminal** to view the real-time operational stream telemetry output.
