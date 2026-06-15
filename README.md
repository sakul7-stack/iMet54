# iMet-54 Receiver / Decoder

ESP32-based receiver and decoder for **iMet-54 radiosondes** (weather balloons), using the **CC1101** 433MHz transceiver module. 
Captures live telemetry (GPS position, temperature, humidity) and serves it via a built-in WiFi web interface.

> **No SDR required** works with cheap CC1101 modules. Also includes a **transmitter clone** for testing without a real radiosonde.

---

## Hardware Required

| Component | Specs / Notes |
|-----------|---------------|
| **MCU** | ESP32 (DevKit v1 or similar) |
| **RF Module** | CC1101 (ELECHOUSE or compatible) |
| **Antenna** | 402 MHz tuned (¼-wave-18.7 cm) |
| **Power** | 5V USB or 3.3V regulator |

### Wiring (VSPI)

| CC1101 Pin | ESP32 GPIO | Function |
|------------|------------|----------|
| SCK | 18 | SPI Clock |
| MISO | 19 | SPI Data In |
| MOSI | 23 | SPI Data Out |
| CS | 5 | Chip Select |
| GDO0 | 4 | Serial Data Output |
| GDO2 | 2 | Serial Clock Output |
| VCC | 3.3V | Power |
| GND | GND | Ground |

---


## Two Modes: Receiver + Transmitter

This repo contains **two sketches** use whichever fits your needs:

| Sketch | Purpose | Use Case |
|--------|---------|----------|
| `iMet-54_RX.ino` | **Receiver / Decoder** | Track real iMet-54 sondes live |
| `iMet-54_TX.ino` | **Transmitter / Clone** | Test your receiver without a real sonde |

### TX Mode 

The transmitter lets you **simulate a radiosonde** for bench testing:
- Configurable frequency, GPS position, time, temperature, humidity
- Transmits one valid iMet-54 frame per second
- Web config portal at `192.168.4.1` 


**To use:**
1. Flash `iMet-54_TX.ino` to a second ESP32+CC1101
2. Connect to WiFi AP `iMet54-Config` / `imet1234`
3. Set your test parameters (freq, lat/lon, temp, etc.)
4. Start the receiver on the same frequency — frames should appear instantly

---

## Quick Start

1. **Install libraries** via Arduino Library Manager:
   - `ELECHOUSE_CC1101_SRC_DRV`
   - Built-in: `WiFi`, `WebServer`, `Preferences` (ESP32 core)

2. **Upload** `iMet-54_RX.ino` to your ESP32.

3. **Connect** to WiFi AP:
   - SSID: `iMet54-RX`
   - Password: `imet1234`

4. **Open** http://192.168.4.1 in your browser.

5. **Start RX** — default frequency is **402.000 MHz**. Use the **RSSI Scan** to find your local sonde if needed.

---

## Features

### Live Decoding
- **GFSK @ 4800 baud**, 8N1 serial framing
- Sync word: `0x24 0x24 0x24 0x24 0x42`
- **Hamming[8,4] FEC** with error correction tracking
- **Dual CRC** validation (custom LFSR + CRC32/802.3)

### Web Dashboard
- **Real-time SSE** frame streaming
- **RSSI spectrum scan** with click-to-tune
- **GPS track** display (lat/lon/alt)
- **PTU data**: temperature, relative humidity, sensor diagnostics
- **Export**: CSV, KML
- **Console log** with colour-coded hex dump

### Data Fields

| Field | Bytes | Format | Notes |
|-------|-------|--------|-------|
| Serial Number | 0x00–0x03 | `uint32` | Decimal string |
| GPS Time | 0x04–0x07 | `int32` | HHMMSS.mmm |
| Latitude | 0x08–0x0B | `int32` | DDMM.mmmm × 1e6 → decimal degrees |
| Longitude | 0x0C–0x0F | `int32` | DDDMM.mmmm × 1e6 → decimal degrees |
| Altitude | 0x10–0x13 | `int32` | 0.1 m units |
| Temperature | 0x1C–0x1F | `float32` | °C |
| RH Raw | 0x20–0x23 | `float32` | Sensor raw value |
| RH Temp | 0x24–0x27 | `float32` | °C |
| Status | 0x2A–0x2B | `uint16` | Status word |
| CRC32 | 0x34–0x37 | `uint32` | CRC32/802.3 |

---

## Protocol Details

### Frame Encoding Pipeline

```
Raw RF → 8N1 de-framing → 1728 bits
       → De-interleave (8×8 transpose per block)
       → Hamming[8,4] decode (216 codewords → 108 bytes)
       → CRC check → Parse fields
```

### GPS Coordinate Conversion

```
raw = DDMM.mmmm × 1,000,000   (big-endian int32)
deg = trunc(raw / 1e6)
min = frac(raw / 1e6) × 100
decimal_degrees = deg + (min / 60)
```

Example: `0xFE7A2BF6` = `-25,547,786` → `-25.91298°`

### Humidity Calculation

Uses **Hyland-Wexler** saturation vapour pressure formula:

```
RH% = (RH_raw × SVP(T_rh_sensor)) / SVP(T_air)
```

Clamped to 0–100%.

---

## Architecture

| Task | Core | Priority | Purpose |
|------|------|----------|---------|
| `loop()` | 1 | 1 | Web server, SSE push, scan trigger |
| `decode_task` | 0 | 2 | Bit sync, frame decode, ring buffer drain |
| `rx_isr()` | Any | ISR | Sample GDO0 on GDO2 rising edge |

- **Ring buffer**: 16 kbit circular queue (ISR → decoder)
- **Frame buffer**: 100-frame circular store (auto-overwrite oldest)

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Main dashboard (HTML) |
| `/events` | GET | SSE stream (JSON frames) |
| `/status` | GET | JSON: `{decoding, freq, frames, scanning}` |
| `/set_freq?f=402.000` | GET | Start RX on frequency (MHz) |
| `/stop_rx` | GET | Stop RX, idle radio |
| `/scan_start?start=399&stop=402&step=50` | GET | Start RSSI scan (kHz step) |
| `/scan_data` | GET | JSON scan results: `{running, points:[{f, r}]}` |

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---------|------------|-----|
| No frames decoded | Wrong frequency | Run RSSI scan, check local sonde frequency |
| CRC always `[NO]` | Weak signal / interference | Check antenna, move away from noise sources |
| ECC errors high | Marginal signal | Improve antenna, check coax |
| Web UI won't load | Not connected to AP | Join `iMet54-RX` WiFi network first |
| Serial output garbled | Baud mismatch | Set serial monitor to **115200** |

---

## Credits

- Protocol reverse-engineered and verified against **[rs1729/RS](https://github.com/rs1729/RS)** reference implementation (`imet54mod.c`).
- Hamming tables, CRC polynomials, and sample frames cross-checked for bit-exact compatibility.

---

## License

MIT — use at your own risk for educational and research purposes. Not certified for operational meteorological use.
