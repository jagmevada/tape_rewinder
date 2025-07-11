# Cassette Rewinder (Arduino Nano)

A DIY motorized cassette tape rewinder for standard C0 audio cassettes, powered by an Arduino Nano (ATmega328P). This project allows batch rewinding of up to four tapes simultaneously, with automatic stall detection based on current sensing, plus manual controls for starting and stopping all or individual channels.

## Features

* **4-channel PWM motor control** using Timer1 and Timer2
* **Automatic stall detection** via current-sense filtering to stop motors at tape end
* **Manual controls** for global and per-channel start/stop using buttons
* **LED indicators** for motor status (running/stopped)
* **PinChangeInterrupt** library for flexible button wiring on analog and digital pins

## Hardware Requirements

* Arduino Nano (ATmega328P, 16MHz)
* 4× DC motors (e.g., RF-300CA series)
* Current-sense resistors / wiring to measure motor current
* 10× push buttons for start/stop controls
* 5× LEDs (4 channel indicators + 1 master indicator)
* Assorted resistors, wiring, and power supply (6–12V DC)

## Pin Mapping

| Function                | Pin        | Notes                                 |
| :---------------------- | :--------- | :------------------------------------ |
| **Motor PWM & LED Ch1** | D3 (OC2B)  | PWM output drives motor; LED shows on |
| **Motor PWM & LED Ch2** | D9 (OC1A)  |                                       |
| **Motor PWM & LED Ch3** | D10 (OC1B) |                                       |
| **Motor PWM & LED Ch4** | D11 (OC2A) |                                       |
| **Master ON/OFF LED**   | D13        | Indicates global running state        |
| **START ALL button**    | D2         | External interrupt (FALLING)          |
| **STOP ALL button**     | D4         | Pin change interrupt (RISING)         |
| **START Ch1 button**    | D5         | Pin change interrupt (RISING)         |
| **START Ch2 button**    | D6         | Pin change interrupt (RISING)         |
| **START Ch3 button**    | D7         | Pin change interrupt (RISING)         |
| **START Ch4 button**    | D8         | Pin change interrupt (RISING)         |
| **STOP Ch1 button**     | D12        | Pin change interrupt (RISING)         |
| **STOP Ch2 button**     | A0         | Pin change interrupt (RISING)         |
| **STOP Ch3 button**     | A1         | Pin change interrupt (RISING)         |
| **STOP Ch4 button**     | A2         | Pin change interrupt (RISING)         |
| **Current-sense Ch1**   | A4         | Analog input for resistor voltage     |
| **Current-sense Ch2**   | A5         |                                       |
| **Current-sense Ch3**   | A6         | Analog-only input                     |
| **Current-sense Ch4**   | A7         | Analog-only input                     |

> **Note:** A6 and A7 are analog-input only pins; all other A0–A5 can serve as digital or analog.

## Software Setup (PlatformIO)

1. **Install VS Code** and the **PlatformIO IDE** extension.
2. **Create new project** in PlatformIO: select **Arduino Nano** (ATmega328P) and **Arduino** framework.
3. Copy `src/main.cpp` from this repository into your project’s `src/` folder.
4. Add the `PinChangeInterrupt` library dependency in `platformio.ini`:

   ```ini
   lib_deps =
     nickgammon/PinChangeInterrupt@^1.2.3
   ```
5. Build and upload via the PlatformIO toolbar (✅ Build, ➤ Upload).
6. Open Serial Monitor (115200 baud) to view debug output.

## Code Overview

* **`setup()`**: Configures PWM timers (Timer1, Timer2), pin modes, and attach interrupts for buttons.
* **`loop()`**: Samples current-sense inputs, updates a moving-sum filter, and cuts PWM on stall detection after a 1 s delay.
* **Interrupt handlers**:

  * **`startAll()` / `stopAll()`**: Global controls.
  * **`startLedX()` / `stopLedX()`**: Per-channel controls via PCINT.

## Operation

1. **Power on** the Nano and motors.
2. Press **START ALL** (D2) to begin rewinding all channels; LEDs D3, D9, D10, D11 light up.
3. Motors spin at configured PWM duty until either:

   * The tape end is detected (current spike) → that channel stops automatically.
   * **STOP ALL** (D4) or **STOP ChX** button is pressed.
4. To restart an individual channel after auto-stop, press its **START ChX** button (D5–D8).

## Customization

* Adjust **`DUTYx`** in code for motor speed.
* Tune **`thx`** thresholds for different motors or tape conditions.
* Expand to more channels by adding Timer0 (OC0A/OC0B) if needed, but care should be taken not to disrupt `millis()`.

## License

This project is released under the MIT License. 

---

