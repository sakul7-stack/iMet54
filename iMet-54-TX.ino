/*
 * iMet-54 Clone Transmitter for ESP32 + CC1101
 * WiFi Access Point config portal: SSID "iMet54-Config", password "imet1234"
 * Connect to http://192.168.4.1
 *
 * Configurable: TX frequency, GPS time (HH MM SS), lat, lon, alt
 * Transmits one iMet-54 compatible GFSK frame per second at ~4800 baud
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// Pin definitions 
#define PIN_GDO0  4
#define PIN_GDO2  2
#define PIN_CS    5

//  iMet-54 protocol 
#define FRAME_BYTES   108
#define NCODEWORDS    216
#define DEINTER_LEN   1728
#define OFS           24
#define DE8N1_LEN     1760
#define BITFRAME_LEN  2200
#define HEADER_LEN    40
#define PREAMBLE_LEN  160
#define TOTAL_BITS    (PREAMBLE_LEN + HEADER_LEN + BITFRAME_LEN)

// WiFi AP 
const char *AP_SSID = "iMet54-Config";
const char *AP_PASS = "imet1234";

WebServer server(80);
Preferences prefs;

//Configurable parameters (loaded from NVS)
struct Config {
    float   freq;      // MHz, e.g. 402.000
    uint8_t hh, mm, ss; // GPS time
    double  lat;       // decimal degrees
    double  lon;
    float   alt;       // metres
    float   temp;      // °C
    float   rh;        // %
    uint32_t sn;       // serial number
} cfg;

void loadConfig() {
    prefs.begin("imet54", false);
    cfg.freq = prefs.getFloat("freq", 402.000f);
    cfg.hh   = prefs.getUChar("hh", 12);
    cfg.mm   = prefs.getUChar("mm", 0);
    cfg.ss   = prefs.getUChar("ss", 0);
    cfg.lat  = prefs.getDouble("lat", 27.7172);
    cfg.lon  = prefs.getDouble("lon", 85.3240);
    cfg.alt  = prefs.getFloat("alt", 2650.0f);
    cfg.temp = prefs.getFloat("temp", 15.0f);
    cfg.rh   = prefs.getFloat("rh", 70.0f);
    cfg.sn   = prefs.getUInt("sn", 0x004D2C2A);
    prefs.end();
}

void saveConfig() {
    prefs.begin("imet54", false);
    prefs.putFloat("freq", cfg.freq);
    prefs.putUChar("hh", cfg.hh);
    prefs.putUChar("mm", cfg.mm);
    prefs.putUChar("ss", cfg.ss);
    prefs.putDouble("lat", cfg.lat);
    prefs.putDouble("lon", cfg.lon);
    prefs.putFloat("alt", cfg.alt);
    prefs.putFloat("temp", cfg.temp);
    prefs.putFloat("rh", cfg.rh);
    prefs.putUInt("sn", cfg.sn);
    prefs.end();
}

// ---- Web UI ----
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>iMet-54 Transmitter</title>
<style>
  :root { --bg:#0f1117; --card:#1a1d27; --border:#2a2d3e; --accent:#4fc3f7;
          --text:#e8eaf6; --muted:#7986cb; --green:#69f0ae; --red:#ff5252; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text); font:15px/1.6 'Segoe UI',system-ui,sans-serif;
         min-height:100vh; padding:1.5rem; }
  .header { display:flex; align-items:center; gap:12px; margin-bottom:2rem; }
  .dot { width:10px; height:10px; border-radius:50%; background:var(--green);
         box-shadow:0 0 8px var(--green); animation:pulse 1.8s infinite; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }
  h1 { font-size:1.35rem; font-weight:600; letter-spacing:.03em; }
  .sub { font-size:.8rem; color:var(--muted); letter-spacing:.05em; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:1rem; }
  .card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:1.25rem; }
  .card h2 { font-size:.75rem; text-transform:uppercase; letter-spacing:.12em;
             color:var(--muted); margin-bottom:1rem; }
  label { display:block; font-size:.82rem; color:var(--muted); margin-bottom:.25rem; margin-top:.8rem; }
  label:first-of-type { margin-top:0; }
  input[type=text],input[type=number] {
    width:100%; padding:.55rem .75rem; background:#0f1117; border:1px solid var(--border);
    border-radius:6px; color:var(--text); font-size:.9rem; outline:none;
    transition:border .2s; }
  input:focus { border-color:var(--accent); }
  .time-row { display:grid; grid-template-columns:1fr 1fr 1fr; gap:.5rem; }
  .btn { width:100%; padding:.7rem; margin-top:1.25rem; background:var(--accent);
         color:#0a1929; border:none; border-radius:8px; font-size:.9rem;
         font-weight:700; cursor:pointer; letter-spacing:.04em; transition:opacity .2s; }
  .btn:hover { opacity:.88; }
  .btn.secondary { background:transparent; border:1px solid var(--border);
                   color:var(--muted); font-weight:500; }
  .status { margin-top:1rem; padding:.65rem 1rem; border-radius:8px; font-size:.85rem;
            display:none; }
  .status.ok  { background:#1b3a2e; color:var(--green); border:1px solid #2e5c44; }
  .status.err { background:#3a1b1b; color:var(--red);   border:1px solid #5c2e2e; }
  .freq-badge { display:inline-block; background:#0f1117; border:1px solid var(--accent);
                color:var(--accent); border-radius:20px; padding:.15rem .75rem;
                font-size:.78rem; font-family:monospace; margin-left:.5rem; }
</style>
</head>
<body>
<div class="header">
  <div class="dot"></div>
  <div>
    <h1>iMet-54 Transmitter <span class="freq-badge" id="fq_badge">--- MHz</span></h1>
    <div class="sub">ESP32 + CC1101 &nbsp;·&nbsp; GFSK 4800 baud</div>
  </div>
</div>

<form id="cfg" action="/save" method="POST">
<div class="grid">

  <div class="card">
    <h2>RF &amp; Identity</h2>
    <label>Frequency (MHz)</label>
    <input type="number" name="freq" id="freq" step="0.001" min="300" max="928" value="FREQ_VAL" required>
    <label>Serial number (hex)</label>
    <input type="text" name="sn" id="sn" maxlength="8" pattern="[0-9A-Fa-f]{1,8}" value="SN_VAL" required>
  </div>

  <div class="card">
    <h2>GPS time (UTC)</h2>
    <label>Hour : Minute : Second</label>
    <div class="time-row">
      <input type="number" name="hh" min="0" max="23" value="HH_VAL" placeholder="HH" required>
      <input type="number" name="mm" min="0" max="59" value="MM_VAL" placeholder="MM" required>
      <input type="number" name="ss" min="0" max="59" value="SS_VAL" placeholder="SS" required>
    </div>
  </div>

  <div class="card">
    <h2>Position</h2>
    <label>Latitude (°, decimal, -90 .. +90)</label>
    <input type="number" name="lat" step="0.00001" min="-90" max="90" value="LAT_VAL" required>
    <label>Longitude (°, decimal, -180 .. +180)</label>
    <input type="number" name="lon" step="0.00001" min="-180" max="180" value="LON_VAL" required>
    <label>Altitude (m MSL)</label>
    <input type="number" name="alt" step="0.1" min="-500" max="60000" value="ALT_VAL" required>
  </div>

  <div class="card">
    <h2>PTU sensors</h2>
    <label>Temperature (°C)</label>
    <input type="number" name="temp" step="0.1" min="-120" max="80" value="TEMP_VAL" required>
    <label>Relative humidity (%)</label>
    <input type="number" name="rh" step="0.1" min="0" max="100" value="RH_VAL" required>
  </div>

</div>

<div style="max-width:300px; margin-top:1rem;">
  <button type="submit" class="btn">&#x2713;&nbsp; Save &amp; apply</button>
  <div class="status" id="sts"></div>
</div>
</form>

<script>
(function(){
  var fq = document.getElementById('freq');
  var badge = document.getElementById('fq_badge');
  function upd(){ badge.textContent = parseFloat(fq.value).toFixed(3)+' MHz'; }
  upd(); fq.addEventListener('input', upd);

  document.getElementById('cfg').addEventListener('submit', function(e){
    e.preventDefault();
    var fd = new FormData(this);
    fetch('/save',{method:'POST',body:new URLSearchParams(fd)})
      .then(function(r){return r.text();})
      .then(function(t){
        var s = document.getElementById('sts');
        s.style.display='block';
        if(t==='OK'){s.className='status ok';s.textContent='Saved — transmitting new config';}
        else{s.className='status err';s.textContent='Error: '+t;}
      })
      .catch(function(){
        var s = document.getElementById('sts');
        s.style.display='block';
        s.className='status err';
        s.textContent='Network error';
      });
  });
})();
</script>
</body>
</html>
)rawliteral";

String buildPage() {
    String p = INDEX_HTML;
    char buf[32];
    snprintf(buf, sizeof(buf), "%.3f", cfg.freq);  p.replace("FREQ_VAL", buf);
    snprintf(buf, sizeof(buf), "%08X", cfg.sn);    p.replace("SN_VAL",   buf);
    snprintf(buf, sizeof(buf), "%d", cfg.hh);      p.replace("HH_VAL",   buf);
    snprintf(buf, sizeof(buf), "%d", cfg.mm);      p.replace("MM_VAL",   buf);
    snprintf(buf, sizeof(buf), "%d", cfg.ss);      p.replace("SS_VAL",   buf);
    snprintf(buf, sizeof(buf), "%.5f", cfg.lat);   p.replace("LAT_VAL",  buf);
    snprintf(buf, sizeof(buf), "%.5f", cfg.lon);   p.replace("LON_VAL",  buf);
    snprintf(buf, sizeof(buf), "%.1f", cfg.alt);   p.replace("ALT_VAL",  buf);
    snprintf(buf, sizeof(buf), "%.1f", cfg.temp);  p.replace("TEMP_VAL", buf);
    snprintf(buf, sizeof(buf), "%.1f", cfg.rh);    p.replace("RH_VAL",   buf);
    return p;
}

void handleRoot() { server.send(200, "text/html", buildPage()); }

void handleSave() {
    bool changed = false;
    if (server.hasArg("freq")) { cfg.freq = server.arg("freq").toFloat(); changed=true; }
    if (server.hasArg("sn"))   { cfg.sn   = (uint32_t)strtoul(server.arg("sn").c_str(), nullptr, 16); changed=true; }
    if (server.hasArg("hh"))   { cfg.hh   = (uint8_t)server.arg("hh").toInt(); changed=true; }
    if (server.hasArg("mm"))   { cfg.mm   = (uint8_t)server.arg("mm").toInt(); changed=true; }
    if (server.hasArg("ss"))   { cfg.ss   = (uint8_t)server.arg("ss").toInt(); changed=true; }
    if (server.hasArg("lat"))  { cfg.lat  = server.arg("lat").toDouble(); changed=true; }
    if (server.hasArg("lon"))  { cfg.lon  = server.arg("lon").toDouble(); changed=true; }
    if (server.hasArg("alt"))  { cfg.alt  = server.arg("alt").toFloat(); changed=true; }
    if (server.hasArg("temp")) { cfg.temp = server.arg("temp").toFloat(); changed=true; }
    if (server.hasArg("rh"))   { cfg.rh   = server.arg("rh").toFloat(); changed=true; }

    if (changed) {
        saveConfig();
        // Rebuild bitstream with new config (outside ISR)
        buildBitstream();
        // Update CC1101 frequency if changed
        ELECHOUSE_cc1101.setMHZ(cfg.freq);
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "No fields");
    }
}

//iMet-54 encode / frame buffers 
static uint8_t deinter_buf[DEINTER_LEN];
static uint8_t de8n1_buf[DE8N1_LEN];
static uint8_t frame_bits_buf[BITFRAME_LEN];

// Double-buffered tx_bits so ISR always reads stable buffer
static uint8_t tx_bits_A[TOTAL_BITS];
static uint8_t tx_bits_B[TOTAL_BITS];
static volatile uint8_t *tx_bits = tx_bits_A;   // ISR reads this
static uint8_t          *wr_bits = tx_bits_B;   // CPU writes this

const uint8_t headerPattern[HEADER_LEN] = {
  0,0,0,0,0,0,0,0,0,1,
  0,1,0,1,0,1,0,1,0,1,
  0,0,0,1,0,0,1,0,0,1,
  0,0,0,1,0,0,1,0,0,1
};

const uint8_t G[8][4] = {
  {1,1,0,1},{1,0,1,1},{1,0,0,0},{0,1,1,1},
  {0,1,0,0},{0,0,1,0},{0,0,0,1},{1,1,1,0}
};

uint32_t crc32_802(uint8_t *msg, int len) {
    uint32_t poly32 = 0x04C11DB7, rem = 0, out = 0x63D60875;
    for (int i=0;i<len;i++) {
        rem ^= ((uint32_t)msg[i]<<24);
        for (int j=0;j<8;j++) {
            if (rem & 0x80000000UL) rem = (rem<<1)^poly32;
            else rem <<= 1;
        }
    }
    return rem ^ out;
}

void buildFrame(uint8_t *f) {
    memset(f, 0, FRAME_BYTES);

    // Serial number
    f[0]=(cfg.sn>>24); f[1]=(cfg.sn>>16); f[2]=(cfg.sn>>8); f[3]=cfg.sn;

    // GPS time: encode as HHMMSSmmm (milliseconds = 0)
    int32_t t = ((int32_t)cfg.hh*10000000 + (int32_t)cfg.mm*100000 + (int32_t)cfg.ss*1000);
    f[4]=(t>>24); f[5]=(t>>16); f[6]=(t>>8); f[7]=t;

    // Latitude  (deg + frac_min/100 stored as *1e6)
    {
        double lat = cfg.lat;
        int deg = (int)lat;
        double frac = lat - deg;
        double val_e6 = deg + frac * 60.0 / 100.0;
        int32_t v = (int32_t)(val_e6 * 1e6);
        f[8]=(v>>24); f[9]=(v>>16); f[10]=(v>>8); f[11]=v;
    }

    // Longitude
    {
        double lon = cfg.lon;
        int deg = (int)lon;
        double frac = lon - deg;
        double val_e6 = deg + frac * 60.0 / 100.0;
        int32_t v = (int32_t)(val_e6 * 1e6);
        f[12]=(v>>24); f[13]=(v>>16); f[14]=(v>>8); f[15]=v;
    }

    // Altitude (*10, integer decimetres)
    {
        int32_t v = (int32_t)(cfg.alt * 10.0f);
        f[16]=(v>>24); f[17]=(v>>16); f[18]=(v>>8); f[19]=v;
    }

    // Temperature (float32 big-endian)
    { uint32_t u; memcpy(&u,&cfg.temp,4); f[0x1C]=(u>>24);f[0x1D]=(u>>16);f[0x1E]=(u>>8);f[0x1F]=u; }

    // Raw RH (float32)
    { uint32_t u; memcpy(&u,&cfg.rh,4); f[0x20]=(u>>24);f[0x21]=(u>>16);f[0x22]=(u>>8);f[0x23]=u; }

    // Trh = same as T
    { uint32_t u; memcpy(&u,&cfg.temp,4); f[0x24]=(u>>24);f[0x25]=(u>>16);f[0x26]=(u>>8);f[0x27]=u; }

    // Status: GPS+PTU valid
    f[0x2A]=0x00; f[0x2B]=0x3E;

    // CRC32 over byte-swapped 4-byte groups of f[0..0x33]
    uint8_t m4[0x34];
    for (int i=0;i<0x34/4;i++)
        for (int j=0;j<4;j++) m4[4*i+j]=f[4*i+3-j];
    uint32_t crc = crc32_802(m4, 0x34);
    f[0x34]=(crc>>24); f[0x35]=(crc>>16); f[0x36]=(crc>>8); f[0x37]=crc;
}

void hamEncode(uint8_t nib, uint8_t *out8) {
    uint8_t m[4];
    for (int k=0;k<4;k++) m[k]=(nib>>k)&1;
    for (int c=0;c<8;c++) {
        uint8_t bit=0;
        for (int k=0;k<4;k++) bit ^= G[c][k]&m[k];
        out8[c]=bit;
    }
}

void deinter64(uint8_t *buf) {
    uint8_t tmp[DEINTER_LEN];
    memcpy(tmp, buf, DEINTER_LEN);
    for (int blk=0; blk<DEINTER_LEN/64; blk++) {
        int n=blk*64;
        for (int i=0;i<8;i++)
            for (int j=0;j<8;j++)
                buf[n+8*j+i]=tmp[n+8*i+j];
    }
}

void enc8n1(uint8_t *bits8, uint8_t *out) {
    out[0] = 0;
    for (int k=0;k<8;k++) out[1+k] = bits8[k];
    out[9] = 1;
}

void buildBitstream() {
    uint8_t frame[FRAME_BYTES];
    buildFrame(frame);

    for (int j=0;j<NCODEWORDS;j++) {
        uint8_t nib = (j%2==0) ? (frame[j/2]>>4) : (frame[j/2]&0xF);
        hamEncode(nib, &deinter_buf[8*j]);
    }
    deinter64(deinter_buf);

    memset(de8n1_buf, 0, DE8N1_LEN);
    uint8_t tailBytes[3] = {0x24,0x24,0x42};
    int idx=0;
    for (int b=0;b<3;b++)
        for (int k=0;k<8;k++)
            de8n1_buf[idx++] = (tailBytes[b]>>(7-k))&1;
    memcpy(de8n1_buf+OFS, deinter_buf, DEINTER_LEN);

    idx=0;
    for (int byteN=0; byteN<DE8N1_LEN/8; byteN++) {
        uint8_t enc[10];
        enc8n1(&de8n1_buf[byteN*8], enc);
        for (int k=0;k<10;k++) frame_bits_buf[idx++]=enc[k];
    }

    // Write into inactive buffer
    const uint8_t preambleUnit[20] = {
        0,0,0,0,0,0,0,0,0,1,
        0,1,0,1,0,1,0,1,0,1
    };
    idx=0;
    for (int r=0;r<PREAMBLE_LEN/20;r++) { memcpy(wr_bits+idx, preambleUnit, 20); idx+=20; }
    memcpy(wr_bits+idx, headerPattern, HEADER_LEN); idx+=HEADER_LEN;
    memcpy(wr_bits+idx, frame_bits_buf, BITFRAME_LEN);

    // Atomically swap pointers (no ISR lock needed: pointer write is atomic on Xtensa)
    uint8_t *tmp_ptr = (uint8_t*)tx_bits;
    tx_bits = wr_bits;
    wr_bits = tmp_ptr;
}

// ---- CC1101 TX bit-clock ISR ----
volatile int txIdx = 0;
volatile bool txActive = false;
volatile bool inGap = false;
volatile unsigned long lastBitMicros = 0;
#define FRAME_PERIOD_US 1000000UL

void IRAM_ATTR clk_isr() {
    if (txActive && !inGap) {
        digitalWrite(PIN_GDO0, tx_bits[txIdx]);
        txIdx++;
        if (txIdx >= TOTAL_BITS) {
            txIdx = 0;
            inGap = true;
            lastBitMicros = micros();
        }
    } else if (inGap) {
        digitalWrite(PIN_GDO0, 0);
        unsigned long elapsed = micros() - lastBitMicros;
        unsigned long frameDur = (unsigned long)((TOTAL_BITS / 4798.0) * 1e6);
        if (elapsed >= (FRAME_PERIOD_US - frameDur)) {
            inGap = false;
        }
    }
}

void setupCC1101TX() {
    ELECHOUSE_cc1101.setSpiPin(18, 19, 23, PIN_CS);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(cfg.freq);

    ELECHOUSE_cc1101.SpiWriteReg(0x00, 0x0B); // IOCFG2: serial clk on GDO2
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x0D); // IOCFG0: sync serial data on GDO0
    ELECHOUSE_cc1101.SpiWriteReg(0x08, 0x12); // PKTCTRL0: sync serial, infinite pkt
    ELECHOUSE_cc1101.SpiWriteReg(0x0B, 0x06); // FSCTRL1
    ELECHOUSE_cc1101.SpiWriteReg(0x10, 0xF7); // MDMCFG4: BW + DRATE_E
    ELECHOUSE_cc1101.SpiWriteReg(0x11, 0x83); // MDMCFG3: DRATE_M → ~4800 baud
    ELECHOUSE_cc1101.SpiWriteReg(0x12, 0x10); // MDMCFG2: GFSK, no HW preamble/sync
    ELECHOUSE_cc1101.SpiWriteReg(0x13, 0x22); // MDMCFG1
    ELECHOUSE_cc1101.SpiWriteReg(0x14, 0xF8); // MDMCFG0
    ELECHOUSE_cc1101.SpiWriteReg(0x15, 0x02); // DEVIATN
    ELECHOUSE_cc1101.SpiWriteReg(0x17, 0x30); // MCSM1
    ELECHOUSE_cc1101.SpiWriteReg(0x18, 0x18); // MCSM0

    pinMode(PIN_GDO0, OUTPUT);
    digitalWrite(PIN_GDO0, 0);
    pinMode(PIN_GDO2, INPUT);

    ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE
    delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x35); // STX
}

void setup() {
    Serial.begin(115200);
    loadConfig();

    // Start WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    server.on("/",     HTTP_GET,  handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.begin();
    Serial.println("Web server started");

    buildBitstream();
    setupCC1101TX();
    attachInterrupt(digitalPinToInterrupt(PIN_GDO2), clk_isr, RISING);
    txActive = true;

    Serial.printf("Transmitting on %.3f MHz\n", cfg.freq);
}

void loop() {
    server.handleClient();
}
