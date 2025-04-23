# ESP32_ADS1256


# ESP32_ADS1256

This repository contains a project that interfaces the ADS1256 Analog-to-Digital Converter (ADC) with an ESP32 microcontroller. The project is implemented in Arduino, allowing high-precision ADC functionality using the SPI protocol.

## Features

- **High Precision ADC**: Supports the ADS1256 ADC with up to 24-bit resolution.
- **Multiple Modes**:
  - Single-ended sampling
  - Differential sampling
  - Continuous cycle sampling
- **Configurable Parameters**:
  - Data rate
  - PGA (Programmable Gain Amplifier)
- **Command Interface**:
  - Serial commands to control and configure the ADC.

## Requirements

- **Hardware**:
  - ESP32 microcontroller
  - ADS1256 ADC module
  - Connection via SPI protocol
- **Software**:
  - Arduino IDE
  - ESP32 board definitions installed in the Arduino IDE

## Pin Configuration

| Pin Name     | ESP32 Pin Number |
|--------------|------------------|
| CS (Chip Select) | 15 |
| MISO         | 12 |
| MOSI         | 13 |
| SCK (Clock)  | 14 |
| DRDY (Data Ready) | 2 |

## Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/lplanckkk/ESP32_ADS1256.git
   ```
2. Open `ESP32_ADS1256.ino` in the Arduino IDE.
3. Connect the ESP32 to the ADS1256 module using the pin configuration above.
4. Upload the sketch to your ESP32.

## Serial Commands

After uploading, commands can be sent through the serial monitor at a baud rate of `2000000`. Below is a list of supported commands:

| Command | Description |
|---------|-------------|
| `SINGLE <channel>` | Single-ended sampling on the specified channel (0-7). |
| `DIFF <channel>` | Differential sampling on the specified channel pair (0-3). |
| `CYCLE` | Cycles through all single-ended channels. |
| `STOP` | Stops any ongoing sampling. |
| `READ <reg>` | Reads the specified register value in hexadecimal. |
| `WRITE <reg> <val>` | Writes a value to the specified register in hexadecimal. |
| `SET DRATE <val>` | Sets the data rate (hexadecimal, 0x00 to 0xFF). |
| `SET PGA <val>` | Sets the PGA register and updates gain settings. |
| `RESET` | Resets the ADS1256 device. |
| `STAT` | Displays the current sampling rate statistics. |
| `HELP` | Displays the help menu with all commands. |

## Example Usage

1. Start single-ended sampling on channel 0:
   ```
   SINGLE 0
   ```
2. Set the data rate to 0xF0:
   ```
   SET DRATE 0xF0
   ```
3. Read the value of a register (e.g., 0x03):
   ```
   READ 0x03
   ```
4. Stop sampling:
   ```
   STOP
   ```

## Key Functions

- **Initialization**:
  - Configures SPI communication and initializes the ADS1256.
  - Default data rate and PGA settings are applied.
- **Sampling**:
  - Single-ended, differential, and cycle sampling modes are implemented.
- **Register Operations**:
  - Commands to read and write ADS1256 registers.
- **Voltage Conversion**:
  - Converts raw ADC values to corresponding voltages based on gain and reference voltage.

## Notes

- The `DEBUG_MODE` macro can be set to `1` for additional debug output during development.
- Ensure proper grounding and wiring connections between the ESP32 and the ADS1256 module.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

Special thanks to the contributors and the open-source community for their support.

---

For any questions or issues, feel free to open an issue in this repository.
