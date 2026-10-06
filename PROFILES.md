# LoRa Device Profiles

This document details device profiles for sniffing, decoding, and interacting with specific LoRa hardware using the Flipper Zero and the Electronic Cats Sub-GHz (SX1262) shield.

---

## Profile: YoLink Vibration & Shock Sensor

* **Model Number:** `YS7201-UC` (also applies to `YS7201-UA`)
* **FCC ID:** `2ATM77201`
* **Internal Transceiver:** Semtech LLCC68 (SX1262 core architecture)
* **Application:** Smart vibration, tamper, movement, and shock detection

### RF Physical Layer Settings
| Parameter | Setting | Technical Notes |
|:---|:---|:---|
| **Center Frequency** | `910.3 MHz` | Single-channel fixed frequency for North American (`-UC`) models |
| **Modulation** | LoRa | Semtech standard chirp spread spectrum |
| **Spreading Factor (SF)** | `SF7` | Fixed rate used by sensor firmware |
| **Bandwidth (BW)** | `125 kHz` | Measured 20 dB occupied bandwidth $\approx 139.8\text{ kHz}$ in FCC compliance testing |
| **Coding Rate (CR)** | `4/5` | Standard Semtech forward error correction |
| **Sync Word** | `Public (0x34)` | Configures SX1262 registers `0x0740`/`0x0741` to `0x3444` (**must NOT be set to Private 0x12**) |
| **Preamble Length** | `8 symbols` | Standard LoRa preamble; setting receiver to 16 symbols drops packets |
| **Header Type** | Variable (Explicit) | Payload length and coding rate are encoded in header |
| **CRC** | Enabled (`0x01`) | Uplink transmissions include 16-bit CRC |
| **IQ Inversion** | Standard (`0x00`) | Non-inverted IQ used for end-node uplink transmissions |

### Operational Notes & Triggering
1. **Physical SET Button:** Pressing the SET button briefly forces the sensor to transmit a status/test packet immediately (green LED flashes). This is the most reliable way to verify RF link.
2. **Vibration Trigger:** In standby/factory state, the vibration sensor will not continuously transmit unless armed or paired.
3. **Payload Structure:** Captured frames are 43 bytes long. The packet header contains the device hardware address and sequence counter; the sensor telemetry payload is encrypted using AES-128.

---

## Profile: Standard LoRaWAN (US915 / EU868)

* **Protocol:** LoRaWAN v1.0.x / v1.1
* **Frequency Plans:**
  * **US915 Uplink:** 902.3 – 914.9 MHz (125 kHz channels 0–63), 903.0 – 914.2 MHz (500 kHz channels 64–71)
  * **EU868 Uplink:** 868.1, 868.3, 868.5 MHz
* **Sync Word:** `Public (0x34)` (`0x3444` on SX1262)
* **Preamble:** 8 symbols
* **CRC:** Enabled for uplink

---

## Profile: Meshtastic Network

* **Protocol:** Meshtastic Mesh Protocol
* **Default Frequency (US):** 906.875 MHz (Slot 20)
* **Sync Word:** `Meshtastic (0x2B)` (`0x2B44` on SX1262)
* **Default Modem Preset (LongFast):** SF11, BW 250 kHz, CR 4/5
* **Preamble:** 16 symbols
* **CRC:** Enabled

---

## Template for Adding New Device Profiles

When reverse engineering and adding support for new sensors:

```markdown
## Profile: [Manufacturer] [Device Name]
* **Model Number:** `[Model]`
* **FCC ID:** `[FCC ID]`
* **Frequency:** `[MHz]`
* **Spreading Factor:** `[SF7 - SF12]`
* **Bandwidth:** `[125 / 250 / 500 kHz]`
* **Coding Rate:** `[4/5, 4/6, 4/7, 4/8]`
* **Sync Word:** `[0x12 (Private) / 0x34 (Public) / Custom]`
* **Preamble Length:** `[8 or 16 symbols]`
* **CRC:** `[Enabled / Disabled]`
* **IQ Inversion:** `[Standard (Uplink) / Inverted (Downlink)]`
```
