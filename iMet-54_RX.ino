
/*
 * iMet-54 Receiver / Decoder  —  ESP32 + CC1101  (v3.0 - fully audited)
 * =======================================================================
 * WiFi AP:  SSID "iMet54-RX"  /  password "imet1234"
 * Open:     http://192.168.4.1
 *
 * CC1101 wiring (VSPI):
 *   SCK=18  MISO=19  MOSI=23  CS=5  GDO0=4  GDO2=2
 *
 * Protocol (verified against rs1729/RS imet54mod.c reference):
 *   Modulation : GFSK, 4800 baud, 8N1
 *   Preamble   : 10x 0x00 0xAA  (alternating bits)
 *   Sync word  : 0x24 0x24 0x24 0x24 0x42
 *   Data       : 27 blocks x 64 bits = 1728 encoded bits
 *   Per block  : 8 Hamming[8,4] codewords, interleaved 8x8 transpose
 *   Payload    : 27 blocks x 4 bytes = 108 bytes
 *
 * Frame layout (108 bytes, all big-endian):
 *   [0x00..0x03]  uint32  Serial number (printed as decimal)
 *   [0x04..0x07]  int32   GPS time : HHMMSS*1000 + ms
 *   [0x08..0x0B]  int32   GPS lat  : DDMM.mmmm * 1e6  → deg + frac*100/60
 *   [0x0C..0x0F]  int32   GPS lon  : DDDMM.mmmm * 1e6 → deg + frac*100/60
 *   [0x10..0x13]  int32   GPS alt  : 0.1 m units
 *   [0x1C..0x1F]  float32 Temperature (°C)
 *   [0x20..0x23]  float32 Raw RH sensor value
 *   [0x24..0x27]  float32 RH sensor temperature (°C)
 *   [0x2A..0x2B]  uint16  Status word
 *   [0x34..0x37]  uint32  CRC32/802.3 (cont-CRC)
 *
 * GPS lat/lon encoding — CONFIRMED against all 4 sample frames:
 *   raw_int32 / 1e6  gives  DDMM.mmmm  (NOT decimal degrees)
 *   decimal_degrees = trunc(raw/1e6)  +  frac(raw/1e6) * 100 / 60
 *   Example: 0xFE7A2BF6 = -25547786
 *            -25547786 / 1e6 = -25.547786  → -25 + (-0.547786 * 100/60) = -25.91298° 
 *
 * CRC:
 *   crc_std : custom LFSR over bytes[0..103], tags at [100:102],[106:108]  → [OK]
 *   crc_cont: CRC32/802.3 over words[0..0x33] byte-swapped, stored at [0x34..0x37] → [ok]
 *

 *
 * Required libraries (Arduino Library Manager):
 *   ELECHOUSE_CC1101_SRC_DRV
 *   WiFi, WebServer, Preferences  (ESP32 Arduino core)
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// Pin definitions 
#define PIN_GDO0   4    // CC1101 serial data output  (IOCFG0 = 0x0C)
#define PIN_GDO2   2    // CC1101 serial clock output (IOCFG2 = 0x0B)
#define PIN_CS     5
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23

// Protocol constants
#define BAUD_RATE    4800
#define FRAME_BYTES  108   // 27 blocks × 4 bytes
// Encoded frame bits: 108 bytes × 2 nibbles × 8 bits/codeword = 1728 bits
// In 8N1 stream: 1728 × 10/8 = 2160 raw bits after sync word
#define FRAME_RAW_BITS  2160
#define SEARCH_MARGIN    200  // extra buffer beyond one frame

// Sync word (5 bytes) as 8N1 bit stream = 50 bits
// 8N1 framing: start(0) + D0..D7 LSB-first + stop(1)
// 0x24 = 0b00100100 → 8N1: 0,0,0,1,0,0,1,0,0,1
// 0x42 = 0b01000010 → 8N1: 0,0,1,0,0,0,0,1,0,1
#define SYNCLEN 50
static const uint8_t SYNC_BITS[SYNCLEN] = {
    0,0,0,1,0,0,1,0,0,1,   // 0x24
    0,0,0,1,0,0,1,0,0,1,   // 0x24
    0,0,0,1,0,0,1,0,0,1,   // 0x24
    0,0,0,1,0,0,1,0,0,1,   // 0x24
    0,0,1,0,0,0,0,1,0,1    // 0x42
};

// Bit ring-buffer (ISR → decode task)
#define RING_SIZE  16384
#define RING_MASK  (RING_SIZE - 1)
static volatile uint8_t  ring[RING_SIZE];
static volatile uint32_t ring_wr = 0;
static volatile uint32_t ring_rd = 0;

// Globals 
float rxFreq  = 402.000f;
bool  rxActive = false;
volatile bool isrAttached = false;

Preferences prefs;
WebServer   server(80);

// RSSI scan 
#define SCAN_MAX_POINTS 512
struct ScanPoint { float freq_mhz; int8_t rssi_dbm; };
static ScanPoint scanResults[SCAN_MAX_POINTS];
static volatile int  scanCount   = 0;
static volatile bool scanRunning = false;

struct ScanRequest { float startMHz, stopMHz, stepKHz; };
static ScanRequest   pendingScan;
static volatile bool scanRequested = false;

// Decoded frame store
#define MAX_FRAMES 100   // circular ring, oldest overwritten
struct DecodedFrame {
    char     sn[12];          // serial number (decimal string)
    char     time_str[16];    // "HH:MM:SS.mmm"
    float    lat, lon, alt;   // decimal degrees, metres
    float    temp_c;          // air temperature °C  (-999 = invalid)
    float    rh_pct;          // relative humidity % (-1 = invalid)
    float    rh_raw;          // raw sensor value
    float    trh_c;           // RH sensor temperature °C
    char     status[8];       // hex string of status word
    bool     crc_ok;          // either CRC passed
    bool     crc_std_ok;      // custom LFSR CRC
    bool     crc_cont_ok;     // CRC32/802.3 cont CRC
    int      ecc_errs;        // Hamming corrections (-1=uncorrectable)
    char     hex[FRAME_BYTES*3+1];
    uint32_t epoch_s;         // millis()/1000 at decode time
};
static DecodedFrame frames[MAX_FRAMES];
// frame_head: next write slot in circular buffer
// frame_total: total frames ever decoded (never capped — used for SSE push)
static volatile int  frame_head  = 0;
static volatile int  frame_total = 0;
static SemaphoreHandle_t frame_mutex;

// SSE
static WiFiClient sse_client;
static bool       sse_connected = false;

//  Hamming[8,4] decode tables
// Parity-check matrix (matches rs1729 imet54mod.c)
static const uint8_t H[4][8] = {
    {1,0,1,0,1,0,1,0},
    {0,1,1,0,0,1,1,0},
    {0,0,0,1,1,1,1,0},
    {1,1,1,1,1,1,1,1}
};
// 1-bit error syndromes: column j of H as 4-bit value
// He[j] = H[0][j] | H[1][j]<<1 | H[2][j]<<2 | H[3][j]<<3
static const uint8_t He[8] = {0x9,0xA,0xB,0xC,0xD,0xE,0xF,0x8};
// Valid codeword LUT: HAM_LUT[nibble] = codeword byte (bits 0..7 = D0..D7)
// Generated from generator matrix G in reference — verified against G
static const uint8_t HAM_LUT[16] = {
    0x00,0x87,0x99,0x1E,0xAA,0x2D,0x33,0xB4,
    0x4B,0xCC,0xD2,0x55,0xE1,0x66,0x78,0xFF
};

// ISR — sample data on clock rising edge
void IRAM_ATTR rx_isr() {
    uint32_t wr = ring_wr;
    ring[wr & RING_MASK] = digitalRead(PIN_GDO0) ? 1 : 0;
    ring_wr = wr + 1;
}

//  Hamming[8,4] decode 
// code8[8]: 8 bit values (0 or 1) of one codeword, LSB-first ordering
// *nib_out: decoded nibble (0..15) on success, 0xFF on failure
// returns: 0=no error, 1=corrected 1-bit error, -1=uncorrectable
static int hamming_decode(uint8_t *code8, uint8_t *nib_out) {
    // Compute syndrome
    uint8_t sv = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t s = 0;
        for (int j = 0; j < 8; j++) s ^= H[i][j] & code8[j];
        sv |= s << i;
    }
    int ecc = 0;
    if (sv) {
        ecc = -1;
        for (int j = 0; j < 8; j++) {
            if (sv == He[j]) { code8[j] ^= 1; ecc = 1; break; }
        }
    }
    // Pack bits to byte and look up nibble
    uint8_t byt = 0;
    for (int j = 0; j < 8; j++) byt |= (code8[j] & 1) << j;
    int n;
    for (n = 0; n < 16; n++) if (byt == HAM_LUT[n]) break;
    if (n >= 16) { *nib_out = 0xFF; return -1; }
    *nib_out = (uint8_t)n;
    return ecc;
}

// CRC32/802.3 (Ethernet CRC, normal poly) 
// rem initialised to 0, XOR output constant 0x63D60875 (from reference)
static uint32_t crc32_802(const uint8_t *msg, int len) {
    uint32_t rem = 0;
    for (int i = 0; i < len; i++) {
        rem ^= ((uint32_t)msg[i] << 24);
        for (int j = 0; j < 8; j++)
            rem = (rem & 0x80000000UL) ? (rem << 1) ^ 0x04C11DB7UL : (rem << 1);
    }
    return rem ^ 0x63D60875UL;
}

// crc_cont: CRC32/802.3 over bytes[0x00..0x33] with each 4-byte word byte-reversed
// Computed CRC compared with big-endian value stored at frame[0x34..0x37]
static bool crc_cont_check(const uint8_t *frame) {
    uint8_t m4[0x34];
    for (int i = 0; i < 0x34 / 4; i++)
        for (int j = 0; j < 4; j++)
            m4[4*i+j] = frame[4*i + 3 - j];
    uint32_t calc   = crc32_802(m4, 0x34);
    uint32_t stored = ((uint32_t)frame[0x34] << 24) | ((uint32_t)frame[0x35] << 16)
                    | ((uint32_t)frame[0x36] <<  8) |  frame[0x37];
    return calc == stored;
}

// crc_std: custom LFSR CRC from rs1729 imet54mod.c crc32ok()
// Covers bytes[0..103], with CRC data at [100:102] and [106:108]
static bool crc_std_check(const uint8_t *bytes) {
    const uint32_t poly0 = 0x0EDB, poly1 = 0x8260;
    int n = 104, b = 0;
    uint32_t c0 = 0x48EB, c1 = 0x1ACA, nx_c0, nx_c1;
    const uint32_t dc0 = ((uint32_t)bytes[100] << 8) | bytes[101];
    const uint32_t dc1 = ((uint32_t)bytes[106] << 8) | bytes[107];
    uint32_t crc0 = 0, crc1 = 0;

    while (n >= 0) {
        if (n < 100 || (n > 101 && n < 106))
            if ((bytes[n] >> b) & 1) { crc0 ^= c0; crc1 ^= c1; }

        nx_c0 = c0; nx_c1 = c1;
        if (c1 & 0x8000) { nx_c0 ^= poly0; nx_c1 ^= poly1; }
        nx_c0 <<= 1; nx_c1 <<= 1;
        if (   c1      & 0x8000) nx_c0 |= 1;
        if ((c1 ^ c0)  & 0x8000) nx_c1 |= 1;
        nx_c0 &= 0xFFFF;
        c0 = nx_c0; c1 = nx_c1;

        if (b < 7) b++;
        else { b = 0; if (n % 4 == 3) n -= 7; else n += 1; }
    }
    crc0 ^= dc0 ^ 0x5000;
    crc1 ^= dc1 ^ 0x1DAD;
    return (crc1 == 0 && (crc0 & 0xF000) == 0);
}

//Integer/float helpers 
static float    f4be(const uint8_t *b) {
    uint32_t u = ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
    float f; memcpy(&f, &u, 4); return f;
}
static int32_t  i4be(const uint8_t *b) {
    return (int32_t)(((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3]);
}
static uint16_t u2be(const uint8_t *b) { return ((uint16_t)b[0] << 8) | b[1]; }

// GPS lat/lon: DDMM.mmmm × 1e6 → decimal degrees 
// Confirmed against all 4 sample frames from rs1729 reference output:
//   raw=-25547786  → -25547786/1e6=-25.547786 → -25+(-0.547786×100/60)=-25.91298° ✓
//   raw= 28125553  → 28125553/1e6= 28.125553 → 28+(0.125553×100/60)= 28.20926° ✓
static float decode_latlon(int32_t raw) {
    float ddmm = raw / 1e6f;        // e.g. -25.547786 (DD=25, MM.mmmm=54.7786)
    int   deg  = (int)ddmm;         // integer degrees: -25
    float frac = ddmm - (float)deg; // fractional part:  -0.547786 (in units of minutes/100)
    return (float)deg + frac * (100.0f / 60.0f); // convert minutes to degrees: -25.91298
}

// ── Hyland-Wexler saturation vapour pressure
// Same formula as rs1729 reference
static float vaporSatP(float tc) {
    double T = tc + 273.15;
    return (float)exp(-5800.2206/T + 1.3914993 + 6.5459673*log(T)
                    - 4.8640239e-2*T + 4.1764768e-5*T*T - 1.4452093e-8*T*T*T);
}

// ── Sync word correlator 
static int sync_score(const uint8_t *bits) {
    int s = 0;
    for (int i = 0; i < SYNCLEN; i++) s += (bits[i] == SYNC_BITS[i]);
    return s;
}

//  decode pipeline
// raw_bits[]: raw 8N1 bit stream immediately after the sync word
// Must contain at least FRAME_RAW_BITS (2160) bits
//
// Pipeline:
//   1. De-8N1    : 2160 raw bits → 1728 Hamming bits
//   2. De-interleave: 27 blocks × 8×8 transpose
//   3. Hamming decode: 216 codewords → 216 nibbles → 108 bytes
//   4. CRC check
//   5. Parse fields
static bool decode_frame(const uint8_t *raw_bits, DecodedFrame *out) {

    // STEP 1: De-8N1
    // Each 10-bit 8N1 symbol: bit[0]=start, bit[1..8]=data LSB-first, bit[9]=stop
    // Take positions where (i % 10) is in {1..8}, skip 0 (start) and 9 (stop)
    // 2160 input → 1728 output  (27 blocks × 64 encoded bits)
    static uint8_t enc_bits[1728];
    int nb = 0;
    for (int i = 0; i < FRAME_RAW_BITS + SEARCH_MARGIN && nb < 1728; i++) {
        int r = i % 10;
        if (r >= 1 && r <= 8) enc_bits[nb++] = raw_bits[i];
    }
    if (nb < 1728) return false;

    // STEP 2: De-interleave
    // Each 64-bit block is stored as an 8×8 matrix in row-major order.
    // De-interleave = transpose: dst[col*8+row] = src[row*8+col]
    // (Matches reference deinter64() exactly)
    static uint8_t deint[1728];
    for (int blk = 0; blk < 27; blk++) {
        const uint8_t *src = enc_bits + blk * 64;
        uint8_t       *dst = deint    + blk * 64;
        for (int row = 0; row < 8; row++)
            for (int col = 0; col < 8; col++)
                dst[col*8+row] = src[row*8+col];
    }

    // STEP 3: Hamming[8,4] decode
    // 216 codewords (8 bits each) → 216 nibbles (4 bits each) → 108 bytes
    static uint8_t nibbles[216];
    int  ecc_errs      = 0;
    bool uncorrectable = false;
    for (int cw = 0; cw < 216; cw++) {
        uint8_t code[8];
        memcpy(code, deint + cw * 8, 8);
        uint8_t nib = 0;
        int r = hamming_decode(code, &nib);
        if      (r < 0) { uncorrectable = true; nib = 0; }
        else if (r > 0) { ecc_errs++; }
        nibbles[cw] = nib;
    }
    out->ecc_errs = uncorrectable ? -1 : ecc_errs;

    // Pack nibbles into bytes (high nibble first)
    static uint8_t frame[FRAME_BYTES + 4];
    for (int i = 0; i < FRAME_BYTES; i++)
        frame[i] = (nibbles[2*i] << 4) | (nibbles[2*i+1] & 0x0F);

    // STEP 4: CRC
    out->crc_std_ok  = crc_std_check(frame);
    out->crc_cont_ok = crc_cont_check(frame);
    out->crc_ok      = out->crc_std_ok || out->crc_cont_ok;

    // STEP 5: Parse fields

    // Serial number — decimal, matching reference "(55064506)"
    uint32_t sn = ((uint32_t)frame[0]<<24) | ((uint32_t)frame[1]<<16)
                | ((uint32_t)frame[2]<< 8) |  frame[3];
    snprintf(out->sn, sizeof(out->sn), "%u", sn);

    // GPS Time: int32 BE = HHMMSS * 1000 + ms
    // e.g. 0x06C425B0 = 113518000 → 11:35:18.000
    {
        int32_t tv = i4be(frame + 0x04);
        if (tv < 0) tv = -tv;
        int ms = tv % 1000; tv /= 1000;
        int ss = tv % 100;  tv /= 100;
        int mm = tv % 100;  tv /= 100;
        int hh = tv % 100;
        snprintf(out->time_str, sizeof(out->time_str), "%02d:%02d:%02d.%03d", hh, mm, ss, ms);
    }

    // GPS Lat/Lon: DDMM.mmmm × 1e6, convert to decimal degrees
    out->lat = decode_latlon(i4be(frame + 0x08));
    out->lon = decode_latlon(i4be(frame + 0x0C));

    // GPS Altitude: 0.1 m units
    out->alt = i4be(frame + 0x10) / 10.0f;

    // PTU
    float T_air = f4be(frame + 0x1C);
    float RH_   = f4be(frame + 0x20);  // raw sensor value
    float T_rh  = f4be(frame + 0x24);  // RH sensor temperature

    out->temp_c = (T_air > -120.0f && T_air < 80.0f) ? T_air  : -999.0f;
    out->trh_c  = (T_rh  > -120.0f && T_rh  < 80.0f) ? T_rh   : -999.0f;
    out->rh_raw = (RH_   >=   0.0f && RH_   < 200.0f) ? RH_    :   -1.0f;

    // Relative humidity via Hyland-Wexler (identical to reference get_PTU)
    out->rh_pct = -1.0f;
    if (out->temp_c > -900.0f && out->trh_c > -900.0f && out->rh_raw >= 0.0f) {
        float svpT   = vaporSatP(out->temp_c);
        float svpTrh = vaporSatP(out->trh_c);
        if (svpT > 0.0f) {
            float rh = RH_ * svpTrh / svpT;
            if (rh < 0.0f)   rh = 0.0f;
            if (rh > 100.0f) rh = 100.0f;
            out->rh_pct = rh;
        }
    }

    // Status word
    snprintf(out->status, sizeof(out->status), "%04X", u2be(frame + 0x2A));

    // Hex dump
    char *p = out->hex;
    for (int i = 0; i < FRAME_BYTES; i++) { snprintf(p, 4, "%02X ", frame[i]); p += 3; }
    if (p > out->hex) *(p-1) = '\0';

    out->epoch_s = millis() / 1000;

    // Plausibility clamp (zero out implausible GPS)
    if (out->lat < -90.0f  || out->lat >  90.0f)  out->lat = 0.0f;
    if (out->lon < -180.0f || out->lon > 180.0f)  out->lon = 0.0f;
    if (out->alt < -400.0f || out->alt > 60000.0f) out->alt = 0.0f;

    // Serial output (rs1729-compatible)
    const char *tag = out->crc_std_ok ? "[OK]" : (out->crc_cont_ok ? "[ok]" : "[NO]");
    Serial.printf("(%s) %s  lat:%.5f  lon:%.5f  alt:%.1f",
                  out->sn, out->time_str, out->lat, out->lon, out->alt);
    if (out->temp_c > -900.0f) Serial.printf("  T=%.1fC",  out->temp_c);
    if (out->rh_pct  >= 0.0f)  Serial.printf("  RH=%.0f%%", out->rh_pct);
    Serial.printf("  %s", tag);
    if (out->ecc_errs != 0)
        Serial.printf("  ecc=%s", out->ecc_errs < 0 ? "unfix" : String(out->ecc_errs).c_str());
    Serial.printf("  [%s]\n  hex: %s\n", out->status, out->hex);

    return true;
}

//  Decode task (runs on core 0)
#define SBUF_SIZE (SYNCLEN + FRAME_RAW_BITS + SEARCH_MARGIN + 64)
static uint8_t sbuf[SBUF_SIZE];
static int     sbuf_len = 0;

static void decode_task(void *) {
    for (;;) {
        if (!rxActive) { vTaskDelay(50); continue; }

        // Drain ring buffer into search buffer
        while (ring_rd != ring_wr && sbuf_len < SBUF_SIZE) {
            sbuf[sbuf_len++] = ring[ring_rd & RING_MASK];
            ring_rd++;
        }

        if (sbuf_len < SYNCLEN + FRAME_RAW_BITS) { vTaskDelay(1); continue; }

        bool found = false;
        for (int i = 0; i <= sbuf_len - SYNCLEN - FRAME_RAW_BITS; i++) {
            if (sync_score(sbuf + i) >= 46) {   // 46/50 allows up to 4 bit errors in sync
                DecodedFrame fr;
                memset(&fr, 0, sizeof(fr));
                if (decode_frame(sbuf + i + SYNCLEN, &fr)) {
                    xSemaphoreTake(frame_mutex, portMAX_DELAY);
                    frames[frame_head] = fr;
                    frame_head  = (frame_head + 1) % MAX_FRAMES;
                    frame_total++;           // never capped — used for SSE push tracking
                    xSemaphoreGive(frame_mutex);
                }
                // Consume everything up to and including this frame
                int consume = i + SYNCLEN + FRAME_RAW_BITS;
                memmove(sbuf, sbuf + consume, sbuf_len - consume);
                sbuf_len -= consume;
                found = true;
                break;
            }
        }

        if (!found) {
            // Discard old bits but keep last SYNCLEN-1 in case sync straddles a boundary
            if (sbuf_len > SYNCLEN * 2) {
                int keep = SYNCLEN - 1;
                memmove(sbuf, sbuf + sbuf_len - keep, keep);
                sbuf_len = keep;
            }
        }
        vTaskDelay(1);
    }
}

// CC1101 initialisation
void setupCC1101RX() {
    ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(rxFreq);

    // GDO2 (pin 2) = synchronous serial clock (IOCFG2 = 0x0B = CLK_XOSC/192)
    ELECHOUSE_cc1101.SpiWriteReg(0x00, 0x0B);
    // GDO0 (pin 4) = synchronous serial data  (IOCFG0 = 0x0C)
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x0C);
    // PKTCTRL0: synchronous serial mode, infinite packet length
    ELECHOUSE_cc1101.SpiWriteReg(0x08, 0x12);
    // FSCTRL1: IF frequency ~101 kHz
    ELECHOUSE_cc1101.SpiWriteReg(0x0B, 0x06);
    // MDMCFG4: channel BW = 101.5625 kHz
    ELECHOUSE_cc1101.SpiWriteReg(0x10, 0xF7);
    // MDMCFG3: data rate mantissa → ~4800 baud
    ELECHOUSE_cc1101.SpiWriteReg(0x11, 0x83);
    // MDMCFG2: GFSK, disable hardware preamble/sync detection
    ELECHOUSE_cc1101.SpiWriteReg(0x12, 0x10);
    // MDMCFG1
    ELECHOUSE_cc1101.SpiWriteReg(0x13, 0x22);
    // MDMCFG0
    ELECHOUSE_cc1101.SpiWriteReg(0x14, 0xF8);
    // DEVIATN: ±5.2 kHz GFSK frequency deviation
    ELECHOUSE_cc1101.SpiWriteReg(0x15, 0x02);
    // MCSM1: stay in RX after packet received
    ELECHOUSE_cc1101.SpiWriteReg(0x17, 0x3F);
    // MCSM0: auto-calibrate when transitioning from IDLE to RX
    ELECHOUSE_cc1101.SpiWriteReg(0x18, 0x18);
    // AGCCTRL2/1/0: tuned for GFSK narrow-band
    ELECHOUSE_cc1101.SpiWriteReg(0x1B, 0x43);
    ELECHOUSE_cc1101.SpiWriteReg(0x1C, 0x40);
    ELECHOUSE_cc1101.SpiWriteReg(0x1D, 0x91);

    pinMode(PIN_GDO0, INPUT);
    pinMode(PIN_GDO2, INPUT);

    ELECHOUSE_cc1101.SpiStrobe(0x36); delay(2);  // SIDLE
    ELECHOUSE_cc1101.SpiStrobe(0x34);             // SRX
}

static void enterRX(float freqMHz) {
    if (isrAttached) {
        detachInterrupt(digitalPinToInterrupt(PIN_GDO2));
        isrAttached = false;
    }
    ELECHOUSE_cc1101.SpiStrobe(0x36); delay(2);
    ELECHOUSE_cc1101.setMHZ(freqMHz);
    ELECHOUSE_cc1101.SpiStrobe(0x34); delay(2);
    ring_wr = ring_rd = 0;
    sbuf_len = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_GDO2), rx_isr, RISING);
    isrAttached = true;
    rxActive = true;
    Serial.printf("RX started on %.3f MHz\n", freqMHz);
}

static void exitRX() {
    rxActive = false;
    if (isrAttached) {
        detachInterrupt(digitalPinToInterrupt(PIN_GDO2));
        isrAttached = false;
    }
    ELECHOUSE_cc1101.SpiStrobe(0x36);
    Serial.println("RX stopped");
}

static int8_t readRSSI() { return (int8_t)ELECHOUSE_cc1101.getRssi(); }

// RSSI scan 
static void doScan(const ScanRequest &req) {
    scanRunning = true; scanCount = 0;
    bool wasActive = rxActive;
    exitRX();
    ELECHOUSE_cc1101.SpiStrobe(0x36); delay(2);

    float f    = req.startMHz;
    float step = req.stepKHz > 0 ? req.stepKHz / 1000.0f : 0.1f;

    while (f <= req.stopMHz + 1e-6f && scanCount < SCAN_MAX_POINTS) {
        ELECHOUSE_cc1101.setMHZ(f);
        ELECHOUSE_cc1101.SpiStrobe(0x34); delay(8);
        scanResults[scanCount].freq_mhz = f;
        scanResults[scanCount].rssi_dbm = readRSSI();
        scanCount++;
        ELECHOUSE_cc1101.SpiStrobe(0x36);
        f += step;
        if (scanCount % 20 == 0) server.handleClient();
    }
    ELECHOUSE_cc1101.SpiStrobe(0x36); delay(2);
    if (wasActive) enterRX(rxFreq);
    scanRunning = false;
}

// SSE push 
static char sse_buf[2048];

static void push_sse_frame(const DecodedFrame *f) {
    char json[1700];
    snprintf(json, sizeof(json),
        "{\"type\":\"frame\",\"frame\":{"
        "\"sn\":\"%s\","
        "\"time\":\"%s\","
        "\"lat\":%.6f,"
        "\"lon\":%.6f,"
        "\"alt\":%.1f,"
        "\"temp_c\":%.2f,"
        "\"rh_pct\":%.1f,"
        "\"rh_raw\":%.2f,"
        "\"trh_c\":%.2f,"
        "\"status\":\"%s\","
        "\"crc_ok\":%s,"
        "\"crc_std_ok\":%s,"
        "\"crc_cont_ok\":%s,"
        "\"ecc_errs\":%d,"
        "\"hex\":\"%s\""
        "}}",
        f->sn, f->time_str,
        f->lat, f->lon, f->alt,
        f->temp_c, f->rh_pct, f->rh_raw, f->trh_c,
        f->status,
        f->crc_ok      ? "true" : "false",
        f->crc_std_ok  ? "true" : "false",
        f->crc_cont_ok ? "true" : "false",
        f->ecc_errs,
        f->hex);
    snprintf(sse_buf, sizeof(sse_buf), "data: %s\n\n", json);
    if (sse_connected && sse_client.connected())
        sse_client.print(sse_buf);
}

// HTML page
static const char INDEX_HTML[] PROGMEM = R"HTMLRAW(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>iMet-54 Decoder</title>
<style>
:root{
  --bg:#f1f5f9;--surface:#ffffff;--card:#ffffff;--border:#e2e8f0;
  --accent:#2563eb;--green:#16a34a;--amber:#d97706;--red:#dc2626;--purple:#7c3aed;
  --text:#0f172a;--muted:#64748b;--light:#f8fafc;
  --mono:'JetBrains Mono','Courier New',monospace;
  --ok-bg:#dcfce7;--ok-fg:#15803d;--ok-br:#86efac;
  --no-bg:#fee2e2;--no-fg:#b91c1c;--no-br:#fca5a5;
  --sw-bg:#fef9c3;--sw-fg:#92400e;--sw-br:#fde047;
  --sh:0 1px 3px rgba(0,0,0,.07),0 1px 2px rgba(0,0,0,.04);
  --shm:0 4px 6px -1px rgba(0,0,0,.08),0 2px 4px -2px rgba(0,0,0,.04);
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font:13px/1.5 system-ui,sans-serif;
     height:100vh;display:flex;flex-direction:column;overflow:hidden}
/* Header */
header{display:flex;align-items:center;gap:12px;padding:0 20px;height:52px;
       background:var(--surface);border-bottom:1px solid var(--border);
       flex-shrink:0;box-shadow:var(--sh)}
.logo{font:700 1.05rem/1 var(--mono);letter-spacing:.03em}
.logo em{color:var(--accent);font-style:normal}
.pill{font:700 .62rem/1 var(--mono);letter-spacing:.09em;padding:3px 10px;
      border-radius:20px;border:1px solid;transition:all .25s}
.pill.on {color:var(--ok-fg);border-color:var(--ok-br);background:var(--ok-bg)}
.pill.off{color:var(--muted);border-color:var(--border);background:var(--light)}
#hdr-freq {margin-left:auto;font:600 .8rem/1 var(--mono);color:var(--accent)}
#hdr-count{font:.65rem/1 var(--mono);color:var(--muted)}
/* Layout */
.layout{display:flex;flex:1;overflow:hidden}
aside{width:258px;flex-shrink:0;background:var(--surface);
      border-right:1px solid var(--border);overflow-y:auto;
      padding:16px 14px;display:flex;flex-direction:column;gap:14px}
.sec{font:.6rem/1 system-ui;font-weight:700;text-transform:uppercase;
     letter-spacing:.1em;color:var(--muted);margin-bottom:6px}
label{font:.72rem/1.3;color:var(--muted);display:block;margin-bottom:3px;font-weight:500}
input[type=number]{width:100%;padding:7px 10px;background:var(--light);
  border:1px solid var(--border);border-radius:7px;color:var(--text);
  font-family:var(--mono);font-size:.8rem;outline:none;transition:border .15s}
input:focus{border-color:var(--accent);background:#fff}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.btn{width:100%;padding:8px 0;border:none;border-radius:8px;font:.78rem/1;
     font-weight:600;cursor:pointer;transition:all .12s;letter-spacing:.02em}
.btn:hover{filter:brightness(.92)}
.btn:active{transform:scale(.975)}
.btn-rx  {background:var(--green);color:#fff}
.btn-stop{background:var(--red);color:#fff}
.btn-scan{background:var(--light);color:var(--accent);border:1px solid var(--accent)}
.btn-dl  {background:var(--light);color:var(--text);border:1px solid var(--border);font-size:.72rem}
.btn-clr {background:transparent;border:1px solid var(--border);color:var(--muted)}
/* Live stats */
.sgrid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.stat{background:var(--light);border:1px solid var(--border);border-radius:8px;padding:9px 10px}
.sv{font:700 1rem/1.2 var(--mono);color:var(--accent);margin-bottom:2px}
.sl{font:.58rem/1;color:var(--muted);text-transform:uppercase;letter-spacing:.07em;font-weight:600}
#scan-st{font:.7rem/1.4 var(--mono);color:var(--muted);margin-top:5px}
hr{border:none;border-top:1px solid var(--border)}
/* Main */
main{flex:1;display:flex;flex-direction:column;overflow:hidden}
.tabs{display:flex;background:var(--surface);border-bottom:1px solid var(--border);
      flex-shrink:0;padding:0 4px;gap:2px}
.tab{padding:10px 16px;font:.75rem/1;font-weight:600;letter-spacing:.03em;
     cursor:pointer;color:var(--muted);border-bottom:2px solid transparent;
     transition:all .12s;margin-bottom:-1px}
.tab:hover{color:var(--text)}
.tab.on{color:var(--accent);border-bottom-color:var(--accent)}
.panel{display:none;flex:1;overflow:auto;padding:14px}
.panel.on{display:flex;flex-direction:column;gap:8px}
/* Frame cards */
.fc{background:var(--card);border:1px solid var(--border);border-radius:10px;
    padding:11px 14px;box-shadow:var(--sh);animation:fi .16s ease}
.fc.crc-ok  {border-left:3px solid var(--green)}
.fc.crc-soft{border-left:3px solid var(--amber)}
.fc.crc-bad {border-left:3px solid var(--red)}
@keyframes fi{from{opacity:0;transform:translateY(-3px)}to{opacity:1;transform:none}}
.fc-top{display:flex;align-items:center;flex-wrap:wrap;gap:7px;margin-bottom:8px}
.fc-sn {font:700 .72rem/1 var(--mono);color:var(--amber)}
.fc-t  {font:.68rem/1 var(--mono);color:var(--muted)}
.bdg{font:.6rem/1 var(--mono);font-weight:700;padding:2px 8px;border-radius:20px;border:1px solid}
.bdg.ok  {color:var(--ok-fg);border-color:var(--ok-br);background:var(--ok-bg)}
.bdg.soft{color:var(--sw-fg);border-color:var(--sw-br);background:var(--sw-bg)}
.bdg.bad {color:var(--no-fg);border-color:var(--no-br);background:var(--no-bg)}
.flds{display:grid;grid-template-columns:repeat(auto-fill,minmax(108px,1fr));gap:5px;margin-bottom:8px}
.fld{background:var(--light);border:1px solid var(--border);border-radius:6px;padding:6px 9px}
.fv{font:600 .88rem/1.2 var(--mono);color:var(--text)}
.fl{font:.58rem/1;color:var(--muted);text-transform:uppercase;letter-spacing:.07em;
    margin-top:2px;font-weight:600}
.hex{padding:7px 9px;background:var(--light);border:1px solid var(--border);
     border-radius:6px;font:.65rem/1.8 var(--mono);color:var(--muted);
     word-break:break-all;max-height:5em;overflow-y:auto}
/* Console */
#console{flex:1;font:.72rem/1.8 var(--mono);background:#f8fafc;
         border:1px solid var(--border);border-radius:8px;padding:10px;
         overflow-y:auto;color:#334155;white-space:pre-wrap}
/* Scan */
#scw{flex:1;display:flex;flex-direction:column;gap:8px}
#scc{flex:1;background:var(--card);border:1px solid var(--border);border-radius:10px;
     padding:10px;min-height:280px;box-shadow:var(--sh)}
#scv{width:100%;height:100%;display:block;cursor:crosshair}
/* Map placeholder */
#mapp{flex:1;display:flex;align-items:center;justify-content:center;
      background:var(--card);border-radius:10px;border:1px solid var(--border);box-shadow:var(--sh)}
#mapm{color:var(--muted);font-size:.85rem;text-align:center;line-height:2.5}
#mapl{color:var(--accent);font:.72rem/1 var(--mono)}
</style>
</head>
<body>
<header>
  <div class="logo">i<em>Met</em>-54</div>
  <div class="pill off" id="rx-pill">IDLE</div>
  <div id="hdr-freq">— MHz</div>
  <div id="hdr-count">0 frames</div>
</header>
<div class="layout">
<aside>
  <div>
    <div class="sec">RSSI Scan</div>
    <div class="row2">
      <div><label>Start MHz</label><input type="number" id="ss" step="0.001" value="399.000"></div>
      <div><label>Stop MHz</label> <input type="number" id="se" step="0.001" value="402.000"></div>
    </div>
    <label style="margin-top:5px">Step kHz</label>
    <input type="number" id="sk" step="1" value="50">
    <button class="btn btn-scan" style="margin-top:7px" onclick="startScan()">Scan band</button>
    <div id="scan-st"></div>
  </div>
  <hr>
  <div>
    <div class="sec">Receive</div>
    <label>Frequency MHz</label>
    <input type="number" id="freq" step="0.001" min="300" max="928" value="402.000">
    <button class="btn btn-rx"   style="margin-top:8px" onclick="startRX()">▶ Start RX</button>
    <button class="btn btn-stop" style="margin-top:5px" onclick="stopRX()">■ Stop</button>
  </div>
  <hr>
  <div>
    <div class="sec">Live</div>
    <div class="sgrid">
      <div class="stat"><div class="sv" id="s-lat">—</div><div class="sl">Lat °</div></div>
      <div class="stat"><div class="sv" id="s-lon">—</div><div class="sl">Lon °</div></div>
      <div class="stat"><div class="sv" id="s-alt">—</div><div class="sl">Alt m</div></div>
      <div class="stat"><div class="sv" id="s-tmp">—</div><div class="sl">Temp °C</div></div>
      <div class="stat"><div class="sv" id="s-rh">—</div> <div class="sl">RH %</div></div>
      <div class="stat"><div class="sv" id="s-cnt">0</div><div class="sl">OK frames</div></div>
    </div>
  </div>
  <hr>
  <div>
    <div class="sec">Export</div>
    <button class="btn btn-dl" onclick="dlCSV(false)" style="margin-bottom:5px">CSV – all frames</button>
    <button class="btn btn-dl" onclick="dlCSV(true)"  style="margin-bottom:5px">CSV – CRC-OK only</button>
    <button class="btn btn-dl" onclick="dlKML()"      style="margin-bottom:5px">KML track</button>
    <button class="btn btn-clr" onclick="clearFrames()">🗑 Clear display</button>
  </div>
</aside>
<main>
  <div class="tabs">
    <div class="tab on" onclick="tab('frames',this)">Frames</div>
    <div class="tab"    onclick="tab('console',this)">Console</div>
    <div class="tab"    onclick="tab('scan',this)">Scan</div>
    <div class="tab"    onclick="tab('map',this)">Map</div>
  </div>
  <div class="panel on" id="panel-frames"><div id="frames-list"></div></div>
  <div class="panel"    id="panel-console"><div id="console"></div></div>
  <div class="panel"    id="panel-scan">
    <div id="scw"><div id="scc"><canvas id="scv"></canvas></div></div>
  </div>
  <div class="panel"    id="panel-map">
    <div id="mapp"><div id="mapm">Sonde track appears once GPS-valid frames arrive.<br><span id="mapl"></span></div></div>
  </div>
</main>
</div>
<script>
var allFrames=[],totalFrames=0,okFrames=0,evtSrc=null,lastScan={points:[],running:false},scanPoll=null;

/* SSE */
function connectSSE(){
  if(evtSrc)evtSrc.close();
  evtSrc=new EventSource('/events');
  evtSrc.onmessage=function(e){try{dispatch(JSON.parse(e.data));}catch(x){}};
  evtSrc.onerror=function(){setTimeout(connectSSE,3000);};
}
function dispatch(m){
  if(m.type==='frame') addFrame(m.frame);
  if(m.type==='status') applyStatus(m);
}

/* Tabs */
function tab(name,el){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('on'));
  document.querySelectorAll('.panel').forEach(p=>p.classList.remove('on'));
  el.classList.add('on');
  document.getElementById('panel-'+name).classList.add('on');
  if(name==='scan') drawScan(lastScan);
}

/* RX status */
function setRX(on,freq){
  var p=document.getElementById('rx-pill');
  p.textContent=on?'RX':'IDLE'; p.className='pill '+(on?'on':'off');
  if(freq) document.getElementById('hdr-freq').textContent=parseFloat(freq).toFixed(3)+' MHz';
}
function applyStatus(s){setRX(s.decoding,s.freq);}

function startRX(){
  var f=parseFloat(document.getElementById('freq').value);
  if(isNaN(f))return;
  fetch('/set_freq?f='+f).then(function(){setRX(true,f);logC('RX started '+f.toFixed(3)+' MHz\n');});
}
function stopRX(){
  fetch('/stop_rx').then(function(){setRX(false);logC('RX stopped\n');});
}

/* Frame store */
function clearFrames(){
  allFrames=[];totalFrames=0;okFrames=0;
  document.getElementById('frames-list').innerHTML='';
  document.getElementById('s-cnt').textContent='0';
  document.getElementById('hdr-count').textContent='0 frames';
}

function addFrame(f){
  allFrames.unshift(f); totalFrames++;
  if(f.crc_ok) okFrames++;
  document.getElementById('hdr-count').textContent=totalFrames+' frames ('+okFrames+' OK)';
  document.getElementById('s-cnt').textContent=okFrames;

  if(f.crc_ok){
    if(f.lat!==0||f.lon!==0){
      document.getElementById('s-lat').textContent=f.lat.toFixed(5);
      document.getElementById('s-lon').textContent=f.lon.toFixed(5);
      document.getElementById('s-alt').textContent=Math.round(f.alt);
      document.getElementById('mapl').textContent=f.lat.toFixed(5)+'° '+f.lon.toFixed(5)+'° '+Math.round(f.alt)+'m';
    }
    if(f.temp_c>-900) document.getElementById('s-tmp').textContent=f.temp_c.toFixed(1);
    if(f.rh_pct>=0)   document.getElementById('s-rh').textContent=Math.round(f.rh_pct);
  }

  var cls,bdg,btxt;
  if(f.crc_std_ok)       {cls='crc-ok';  bdg='ok';  btxt='[OK]';}
  else if(f.crc_cont_ok) {cls='crc-soft';bdg='soft';btxt='[ok]';}
  else                   {cls='crc-bad'; bdg='bad'; btxt='[NO]';}
  var ecc=f.ecc_errs<0?' ECC:unfix':(f.ecc_errs>0?' ECC:'+f.ecc_errs:'');

  function fld(l,v,u){
    return v===null||v===undefined?'':
      '<div class="fld"><div class="fv">'+v+(u||'')+'</div><div class="fl">'+l+'</div></div>';
  }
  var flds='', hasGPS=(f.lat!==0||f.lon!==0);
  if(hasGPS){flds+=fld('Lat',f.lat.toFixed(5),'°');flds+=fld('Lon',f.lon.toFixed(5),'°');flds+=fld('Alt',Math.round(f.alt),' m');}
  if(f.temp_c>-900) flds+=fld('Temp',f.temp_c.toFixed(1),' °C');
  if(f.rh_pct>=0)   flds+=fld('RH',Math.round(f.rh_pct),' %');
  if(f.rh_raw>=0)   flds+=fld('RH raw',f.rh_raw.toFixed(1));
  if(f.trh_c>-900)  flds+=fld('T-rh',f.trh_c.toFixed(1),' °C');
  flds+=fld('Status',f.status);

  // Colour-coded hex by field offset
  var parts=(f.hex||'').trim().split(' ');
  // palette indexed by byte offset: SN amber, GPS time blue, lat green, lon sky, alt purple, T orange, RH pink, Trh teal, status indigo
  var pal={0:'#92400e',1:'#92400e',2:'#92400e',3:'#92400e',
           4:'#1d4ed8',5:'#1d4ed8',6:'#1d4ed8',7:'#1d4ed8',
           8:'#15803d',9:'#15803d',10:'#15803d',11:'#15803d',
           12:'#0369a1',13:'#0369a1',14:'#0369a1',15:'#0369a1',
           16:'#6d28d9',17:'#6d28d9',18:'#6d28d9',19:'#6d28d9',
           28:'#c2410c',29:'#c2410c',30:'#c2410c',31:'#c2410c',
           32:'#be185d',33:'#be185d',34:'#be185d',35:'#be185d',
           36:'#0f766e',37:'#0f766e',38:'#0f766e',39:'#0f766e',
           42:'#4338ca',43:'#4338ca'};
  var hexH=parts.map(function(p,i){return pal[i]?'<span style="color:'+pal[i]+'">'+p+'</span>':p;}).join(' ');

  var card='<div class="fc '+cls+'">'
    +'<div class="fc-top">'
    +'<span class="fc-sn">SN:'+f.sn+'</span>'
    +'<span class="fc-t">'+f.time+'</span>'
    +'<span class="bdg '+bdg+'">'+btxt+ecc+'</span>'
    +'</div>'
    +(flds?'<div class="flds">'+flds+'</div>':'')
    +'<div class="hex">'+hexH+'</div>'
    +'</div>';

  var list=document.getElementById('frames-list');
  list.insertAdjacentHTML('afterbegin',card);
  while(list.children.length>50) list.removeChild(list.lastChild);

  logC('['+f.time+'] SN:'+f.sn
    +(hasGPS?'  lat:'+f.lat.toFixed(5)+'  lon:'+f.lon.toFixed(5)+'  alt:'+Math.round(f.alt)+'m':'')
    +(f.temp_c>-900?'  T='+f.temp_c.toFixed(1)+'°C':'')
    +(f.rh_pct>=0?'  RH='+Math.round(f.rh_pct)+'%':'')
    +'  '+btxt+ecc+'\n  hex:'+f.hex+'\n');
}

function logC(t){
  var c=document.getElementById('console');
  c.textContent+=t;
  if(c.textContent.length>200000) c.textContent=c.textContent.slice(-100000);
  c.scrollTop=c.scrollHeight;
}

/* RSSI Scan */
function startScan(){
  var a=parseFloat(document.getElementById('ss').value);
  var b=parseFloat(document.getElementById('se').value);
  var s=parseFloat(document.getElementById('sk').value);
  if(isNaN(a)||isNaN(b)||isNaN(s))return;
  document.getElementById('scan-st').textContent='Scanning '+a+'–'+b+' MHz…';
  fetch('/scan_start?start='+a+'&stop='+b+'&step='+s).then(function(){pollScan();});
}
function pollScan(){
  if(scanPoll)clearTimeout(scanPoll);
  fetch('/scan_data').then(function(r){return r.json();}).then(function(d){
    lastScan=d; drawScan(d);
    if(d.running){
      document.getElementById('scan-st').textContent='Scanning… '+d.points.length+' pts';
      scanPoll=setTimeout(pollScan,600);
    } else {
      document.getElementById('scan-st').textContent='Done: '+d.points.length+' pts — click to pick freq';
    }
  });
}
function drawScan(d){
  var cv=document.getElementById('scv');
  var wrap=document.getElementById('scc');
  var dpr=window.devicePixelRatio||1;
  var w=wrap.clientWidth,h=wrap.clientHeight;
  cv.width=w*dpr; cv.height=h*dpr; cv.style.width=w+'px'; cv.style.height=h+'px';
  var ctx=cv.getContext('2d'); ctx.scale(dpr,dpr);
  ctx.clearRect(0,0,w,h);
  if(!d||!d.points||d.points.length<2){
    ctx.fillStyle='#94a3b8'; ctx.font='13px system-ui';
    ctx.fillText('No scan data — run a scan first.',14,28); return;
  }
  var pts=d.points;
  var fmin=pts[0].f,fmax=pts[pts.length-1].f,rmin=999,rmax=-999;
  pts.forEach(function(p){if(p.r<rmin)rmin=p.r;if(p.r>rmax)rmax=p.r;});
  if(rmax-rmin<4){rmax+=2;rmin-=2;}
  var pad={l:54,r:14,t:16,b:30};
  var pw=w-pad.l-pad.r,ph=h-pad.t-pad.b;
  function X(f){return pad.l+(f-fmin)/(fmax-fmin)*pw;}
  function Y(r){return pad.t+(1-(r-rmin)/(rmax-rmin))*ph;}
  ctx.lineWidth=1; ctx.strokeStyle='#e2e8f0'; ctx.fillStyle='#64748b'; ctx.font='10px system-ui';
  for(var i=0;i<=4;i++){
    var rv=rmin+(rmax-rmin)*i/4,y=Y(rv);
    ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(w-pad.r,y);ctx.stroke();
    ctx.fillText(rv.toFixed(0)+' dBm',2,y+4);
  }
  for(var i=0;i<=4;i++){
    var fv=fmin+(fmax-fmin)*i/4,x=X(fv);
    ctx.fillText(fv.toFixed(3),x-22,h-8);
  }
  ctx.beginPath(); ctx.moveTo(X(pts[0].f),ph+pad.t);
  pts.forEach(function(p){ctx.lineTo(X(p.f),Y(p.r));});
  ctx.lineTo(X(pts[pts.length-1].f),ph+pad.t); ctx.closePath();
  ctx.fillStyle='rgba(37,99,235,.07)'; ctx.fill();
  ctx.beginPath(); ctx.strokeStyle='#2563eb'; ctx.lineWidth=2;
  pts.forEach(function(p,i){i?ctx.lineTo(X(p.f),Y(p.r)):ctx.moveTo(X(p.f),Y(p.r));});
  ctx.stroke();
  var pk=pts.reduce(function(a,b){return b.r>a.r?b:a;},pts[0]);
  ctx.fillStyle='#d97706';
  ctx.beginPath();ctx.arc(X(pk.f),Y(pk.r),5,0,7);ctx.fill();
  ctx.fillStyle='#92400e'; ctx.font='bold 11px system-ui';
  ctx.fillText(pk.f.toFixed(3)+' MHz ('+pk.r+' dBm)',X(pk.f)-34,Y(pk.r)-14);
  cv.onclick=function(ev){
    var rect=cv.getBoundingClientRect();
    var mx=ev.clientX-rect.left;
    var best=pts.reduce(function(a,b){return Math.abs(X(b.f)-mx)<Math.abs(X(a.f)-mx)?b:a;},pts[0]);
    document.getElementById('freq').value=best.f.toFixed(3);
    logC('Selected '+best.f.toFixed(3)+' MHz ('+best.r+' dBm)\n');
  };
}

/* CSV export */
function dlCSV(okOnly){
  var rows=['"SN","Time","Lat","Lon","Alt_m","Temp_C","RH_pct","Trh_C","Status","CRC","ECC"'];
  allFrames.forEach(function(f){
    if(okOnly&&!f.crc_ok)return;
    rows.push(['"'+f.sn+'"','"'+(f.time||'')+'"',
      f.lat,f.lon,f.alt,
      f.temp_c>-900?f.temp_c.toFixed(2):'',
      f.rh_pct>=0?f.rh_pct.toFixed(1):'',
      f.trh_c>-900?f.trh_c.toFixed(2):'',
      '"'+f.status+'"',
      f.crc_std_ok?'OK':(f.crc_cont_ok?'ok':'NO'),
      f.ecc_errs].join(','));
  });
  dlBlob(rows.join('\r\n'),'text/csv','imet54_'+(okOnly?'ok_':'all_')+ts()+'.csv');
}
/* KML export */
function dlKML(){
  var pts=allFrames.filter(function(f){return f.crc_ok&&(f.lat!==0||f.lon!==0);});
  var coords=pts.map(function(f){return f.lon.toFixed(6)+','+f.lat.toFixed(6)+','+Math.round(f.alt);}).join('\n');
  var kml='<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">\n<Document>\n'
    +'<name>iMet-54 Track</name>\n'
    +'<Style id="tr"><LineStyle><color>ff0063eb</color><width>3</width></LineStyle></Style>\n'
    +'<Placemark><styleUrl>#tr</styleUrl><LineString><altitudeMode>absolute</altitudeMode>\n'
    +'<coordinates>\n'+coords+'\n</coordinates></LineString></Placemark>\n';
  pts.forEach(function(f){
    kml+='<Placemark><name>'+f.sn+' '+f.time+'</name>'
      +'<description>Alt:'+Math.round(f.alt)+'m T:'+f.temp_c.toFixed(1)+'C RH:'+Math.round(f.rh_pct)+'%</description>'
      +'<Point><altitudeMode>absolute</altitudeMode><coordinates>'
      +f.lon.toFixed(6)+','+f.lat.toFixed(6)+','+Math.round(f.alt)
      +'</coordinates></Point></Placemark>\n';
  });
  kml+='</Document></kml>';
  dlBlob(kml,'application/vnd.google-earth.kml+xml','imet54_track_'+ts()+'.kml');
}
function dlBlob(text,mime,name){
  var a=document.createElement('a');
  a.href=URL.createObjectURL(new Blob([text],{type:mime}));
  a.download=name; a.click();
}
function ts(){return new Date().toISOString().slice(0,16).replace(/:/g,'').replace('T','_');}

setInterval(function(){
  fetch('/status').then(function(r){return r.json();}).then(applyStatus).catch(function(){});
},3000);
connectSSE();
</script>
</body>
</html>
)HTMLRAW";

// ── Web handlers
void handleRoot()    { server.send_P(200, "text/html", INDEX_HTML); }

void handleSetFreq() {
    if (server.hasArg("f")) {
        float f = server.arg("f").toFloat();
        if (f > 300.0f && f < 928.0f) {
            rxFreq = f;
            prefs.begin("imet54rx", false);
            prefs.putFloat("freq", rxFreq);
            prefs.end();
            enterRX(rxFreq);
        }
    }
    server.send(200, "text/plain", "OK");
}

void handleStopRX() {
    exitRX();
    server.send(200, "text/plain", "OK");
}

void handleStatus() {
    char buf[200];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"status\",\"decoding\":%s,\"freq\":%.3f,\"frames\":%d,\"scanning\":%s}",
        rxActive    ? "true" : "false",
        rxFreq,
        frame_total,
        scanRunning ? "true" : "false");
    server.send(200, "application/json", buf);
}

void handleScanStart() {
    pendingScan.startMHz = server.hasArg("start") ? server.arg("start").toFloat() : 399.0f;
    pendingScan.stopMHz  = server.hasArg("stop")  ? server.arg("stop").toFloat()  : 402.0f;
    pendingScan.stepKHz  = server.hasArg("step")  ? server.arg("step").toFloat()  :  50.0f;
    if (pendingScan.startMHz < 300) pendingScan.startMHz = 300;
    if (pendingScan.stopMHz  > 928) pendingScan.stopMHz  = 928;
    if (pendingScan.stopMHz <= pendingScan.startMHz) pendingScan.stopMHz = pendingScan.startMHz + 1;
    if (pendingScan.stepKHz <= 0)   pendingScan.stepKHz  = 50;
    scanRequested = true;
    server.send(200, "text/plain", "OK");
}

void handleScanData() {
    String out = "{\"running\":";
    out += scanRunning ? "true" : "false";
    out += ",\"points\":[";
    int n = (int)scanCount;
    if (n > SCAN_MAX_POINTS) n = SCAN_MAX_POINTS;
    for (int i = 0; i < n; i++) {
        if (i) out += ',';
        char tmp[40];
        snprintf(tmp, sizeof(tmp), "{\"f\":%.4f,\"r\":%d}",
                 scanResults[i].freq_mhz, (int)scanResults[i].rssi_dbm);
        out += tmp;
    }
    out += "]}";
    server.send(200, "application/json", out);
}

void handleSSE() {
    sse_client     = server.client();
    sse_connected  = true;
    sse_client.println("HTTP/1.1 200 OK");
    sse_client.println("Content-Type: text/event-stream");
    sse_client.println("Cache-Control: no-cache");
    sse_client.println("Connection: keep-alive");
    sse_client.println();
    char init[160];
    snprintf(init, sizeof(init),
             "data: {\"type\":\"status\",\"decoding\":%s,\"freq\":%.3f}\n\n",
             rxActive ? "true" : "false", rxFreq);
    sse_client.print(init);
    sse_client.setNoDelay(true);
}

// Setup 
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\niMet-54 RX v3.0 starting...");

    prefs.begin("imet54rx", true);
    rxFreq = prefs.getFloat("freq", 402.000f);
    prefs.end();

    frame_mutex = xSemaphoreCreateMutex();

    WiFi.mode(WIFI_AP);
    WiFi.softAP("iMet54-RX", "imet1234");
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    server.on("/",           HTTP_GET, handleRoot);
    server.on("/set_freq",   HTTP_GET, handleSetFreq);
    server.on("/stop_rx",    HTTP_GET, handleStopRX);
    server.on("/status",     HTTP_GET, handleStatus);
    server.on("/events",     HTTP_GET, handleSSE);
    server.on("/scan_start", HTTP_GET, handleScanStart);
    server.on("/scan_data",  HTTP_GET, handleScanData);
    server.begin();

    setupCC1101RX();
    enterRX(rxFreq);

    xTaskCreatePinnedToCore(decode_task, "imet_decode", 12288, NULL, 2, NULL, 0);

    Serial.printf("Listening on %.3f MHz\n", rxFreq);
}

//Loop 
static int last_pushed = 0;

void loop() {
    server.handleClient();

    if (scanRequested) {
        scanRequested = false;
        doScan(pendingScan);
    }

    // Push newly decoded frames over SSE
    if (frame_total > last_pushed) {
        xSemaphoreTake(frame_mutex, portMAX_DELAY);
        int to_push = frame_total - last_pushed;
        if (to_push > 5) to_push = 5;
        // The circular buffer holds newest MAX_FRAMES frames
        // frame_head points to the NEXT write slot, so newest is at (frame_head-1)
        int base = (frame_head - to_push + MAX_FRAMES) % MAX_FRAMES;
        for (int i = 0; i < to_push; i++)
            push_sse_frame(&frames[(base + i) % MAX_FRAMES]);
        last_pushed = frame_total;
        xSemaphoreGive(frame_mutex);
    }

    delay(10);
}
