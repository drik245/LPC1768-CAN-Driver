/**
 * RX_CAN_WiFi.ino
 * ESP32 CAN Receiver + WiFi Dashboard
 *
 * Receives CAN frames from LPC1768 via TWAI @ 500kbps
 * and streams them to a web dashboard over WiFi (WebSocket).
 *
 * The ESP32 creates its own WiFi Access Point:
 *   SSID: CAN_Dashboard
 *   Pass: canbus123
 *   URL:  http://192.168.4.1
 *
 * Pin connections:
 *   GPIO5 (TX) → SN65HVD230 TXD
 *   GPIO4 (RX) ← SN65HVD230 RXD
 *
 * Libraries needed (install via Arduino Library Manager):
 *   - ESPAsyncWebServer  (by me-no-dev)
 *   - AsyncTCP           (by me-no-dev)
 *
 * If you cannot install those, a simpler fallback using
 * the built-in WebServer is used instead (see USE_ASYNC flag).
 */

#include "Arduino.h"
#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

/* ─── WiFi AP Settings ────────────────────────────────── */
#define WIFI_SSID   "CAN_Dashboard"
#define WIFI_PASS   "canbus123"

/* ─── Pin Definitions ─────────────────────────────────── */
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4
#define LED_PIN     2

/* ─── Globals ─────────────────────────────────────────── */
WebServer        server(80);
WebSocketsServer ws(81);

static uint32_t msg_received = 0;
static uint32_t msg_errors   = 0;

/* ─── Forward Declarations ────────────────────────────── */
bool     twai_init_bus(void);
void     handleRoot(void);
void     wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void     blink_led(int times, int on_ms, int off_ms);

/* ──────────────────────────────────────────────────────── */
/*  The entire dashboard HTML (served from ESP32 flash)    */
/* ──────────────────────────────────────────────────────── */
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN Bus Dashboard</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#0a0e1a;--card:#1a2235;--border:#2a3450;--text:#e2e8f0;
--muted:#8892a8;--accent:#3b82f6;--green:#22c55e;--red:#ef4444;
--cyan:#06b6d4;--yellow:#f59e0b}
body{font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--text);min-height:100vh}
.hdr{background:linear-gradient(135deg,#0f172a,#1e293b);border-bottom:1px solid var(--border);padding:14px 24px;display:flex;align-items:center;justify-content:space-between}
.hdr h1{font-size:1.2rem;font-weight:700}.hdr h1 span{color:var(--accent)}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:8px}
.dot.on{background:var(--green);box-shadow:0 0 8px rgba(34,197,94,0.4);animation:p 1.5s infinite}
.dot.off{background:var(--muted)}
@keyframes p{0%,100%{opacity:1}50%{opacity:.5}}
.g{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;padding:20px 24px}
.c{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:18px;transition:border-color .3s}
.c:hover{border-color:var(--accent)}
.ct{font-size:.7rem;font-weight:600;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:10px}
.sv{font-family:monospace;font-size:2rem;font-weight:700;line-height:1}
.ss{font-size:.78rem;color:var(--muted);margin-top:5px}
.sv.gr{color:var(--green)}.sv.cy{color:var(--cyan)}.sv.ac{color:var(--accent)}.sv.yl{color:var(--yellow)}
.badge{display:inline-block;padding:4px 14px;border-radius:20px;font-size:.72rem;font-weight:700}
.badge.ok{background:rgba(34,197,94,.15);color:var(--green);border:1px solid rgba(34,197,94,.3)}
.badge.fail{background:rgba(239,68,68,.15);color:var(--red);border:1px solid rgba(239,68,68,.3)}
.gb{width:100%;height:10px;background:#111827;border-radius:5px;margin-top:10px;overflow:hidden}
.gf{height:100%;border-radius:5px;background:linear-gradient(90deg,var(--cyan),var(--accent));transition:width .5s;width:0%}
.gl{display:flex;justify-content:space-between;font-size:.65rem;color:var(--muted);margin-top:3px}
.hg{display:grid;grid-template-columns:repeat(8,1fr);gap:5px;margin-top:8px}
.hc{background:#111827;border:1px solid var(--border);border-radius:6px;text-align:center;padding:7px 3px;font-family:monospace;font-size:.82rem;transition:all .3s}
.hc .lb{font-size:.55rem;color:var(--muted);display:block;margin-bottom:2px}
.hc.hl{border-color:var(--accent);background:rgba(59,130,246,.08)}
.w2{grid-column:span 2}.w4{grid-column:span 4}
.chart{position:relative;height:150px;margin-top:8px}
canvas{width:100%!important;height:100%!important}
.log{max-height:200px;overflow-y:auto;font-family:monospace;font-size:.72rem;background:#111827;border-radius:8px;padding:10px;margin-top:8px}
.log::-webkit-scrollbar{width:5px}.log::-webkit-scrollbar-thumb{background:var(--border);border-radius:3px}
.ll{padding:3px 0;border-bottom:1px solid rgba(42,52,80,.4);display:flex;gap:10px}
.lt{color:var(--muted);min-width:65px}.li{color:var(--cyan);min-width:50px}.ld{color:var(--text)}
.desc{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
.desc span{font-size:.65rem;color:var(--muted)}
.desc code{color:var(--cyan)}
@media(max-width:900px){.g{grid-template-columns:1fr 1fr}.w2,.w4{grid-column:span 2}}
@media(max-width:550px){.g{grid-template-columns:1fr}.w2,.w4{grid-column:span 1}}
</style>
</head>
<body>
<div class="hdr">
  <h1>&#x1F697; <span>CAN Bus</span> Dashboard</h1>
  <div style="display:flex;align-items:center;gap:8px">
    <span class="dot off" id="dot"></span>
    <span id="st" style="font-size:.82rem;color:var(--muted)">Connecting...</span>
  </div>
</div>
<div class="g">
  <div class="c"><div class="ct">Messages</div><div class="sv gr" id="mc">0</div><div class="ss" id="mr">0 msg/s</div></div>
  <div class="c"><div class="ct">Counter</div><div class="sv cy" id="cv">--</div><div class="ss">Byte 2</div></div>
  <div class="c"><div class="ct">Sensor</div><div class="sv ac" id="sen">--</div><div class="gb"><div class="gf" id="sb"></div></div><div class="gl"><span>0</span><span>3500</span></div></div>
  <div class="c"><div class="ct">CRC</div><div class="sv" id="crc" style="font-size:1.4rem">--</div><div class="ss" id="cs">Pass: 0 / Fail: 0</div></div>
  <div class="c w2">
    <div class="ct">Payload</div>
    <div class="hg" id="hg">
      <div class="hc"><span class="lb">B0</span><span id="b0">--</span></div>
      <div class="hc"><span class="lb">B1</span><span id="b1">--</span></div>
      <div class="hc"><span class="lb">B2</span><span id="b2">--</span></div>
      <div class="hc"><span class="lb">B3</span><span id="b3">--</span></div>
      <div class="hc"><span class="lb">B4</span><span id="b4">--</span></div>
      <div class="hc"><span class="lb">B5</span><span id="b5">--</span></div>
      <div class="hc"><span class="lb">B6</span><span id="b6">--</span></div>
      <div class="hc"><span class="lb">B7</span><span id="b7">--</span></div>
    </div>
    <div class="desc">
      <span>B0-1: Magic <code>0xABCD</code></span><span>B2: Counter</span>
      <span>B3: Status</span><span>B4-5: Sensor</span>
      <span>B6: CRC</span><span>B7: End <code>0xFF</code></span>
    </div>
  </div>
  <div class="c w2"><div class="ct">Sensor History</div><div class="chart"><canvas id="ch"></canvas></div></div>
  <div class="c w4"><div class="ct">Frame Log</div><div class="log" id="lg"></div></div>
</div>
<script>
let ws,mc=0,cp=0,cf=0,mrc=0,hist=[],ctx;
function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>{document.getElementById('dot').className='dot on';document.getElementById('st').textContent='Live';};
  ws.onclose=()=>{document.getElementById('dot').className='dot off';document.getElementById('st').textContent='Reconnecting...';setTimeout(conn,2000);};
  ws.onmessage=(e)=>{try{let d=JSON.parse(e.data);update(d)}catch(x){}};
}
function update(d){
  mc++;mrc++;
  let ok=d.crc_ok;
  if(ok)cp++;else cf++;
  document.getElementById('mc').textContent=mc;
  document.getElementById('cv').textContent=d.counter;
  document.getElementById('sen').textContent=d.sensor;
  document.getElementById('sb').style.width=Math.min(100,(d.sensor/3500)*100)+'%';
  document.getElementById('crc').innerHTML=ok?'<span class="badge ok">CRC OK</span>':'<span class="badge fail">CRC FAIL</span>';
  document.getElementById('cs').textContent='Pass: '+cp+' / Fail: '+cf;
  for(let i=0;i<8;i++){let el=document.getElementById('b'+i);if(el)el.textContent=d.hex[i];}
  document.querySelectorAll('.hc').forEach(c=>{c.classList.add('hl');setTimeout(()=>c.classList.remove('hl'),500);});
  hist.push(d.sensor);if(hist.length>60)hist.shift();
  draw();addLog(d);
}
function addLog(d){
  let lg=document.getElementById('lg');
  let t=new Date().toLocaleTimeString('en',{hour12:false});
  let div=document.createElement('div');div.className='ll';
  div.innerHTML='<span class="lt">'+t+'</span><span class="li">0x'+d.id.toString(16).toUpperCase()+'</span><span class="ld">'+d.hex.join(' ')+'</span><span>C='+d.counter+'</span><span>S='+d.sensor+'</span><span style="color:'+(d.crc_ok?'var(--green)':'var(--red)')+'">'+(d.crc_ok?'OK':'FAIL')+'</span>';
  lg.prepend(div);while(lg.children.length>80)lg.removeChild(lg.lastChild);
}
function draw(){
  if(!ctx)ctx=document.getElementById('ch').getContext('2d');
  let c=ctx,W=c.canvas.parentElement.clientWidth,H=c.canvas.parentElement.clientHeight;
  c.canvas.width=W*2;c.canvas.height=H*2;c.scale(2,2);c.clearRect(0,0,W,H);
  if(hist.length<2)return;
  let p={t:8,b:18,l:36,r:8},cw=W-p.l-p.r,ch=H-p.t-p.b;
  let mx=Math.max(...hist,500),mn=Math.min(...hist,0),rng=mx-mn||1;
  c.strokeStyle='rgba(42,52,80,.5)';c.lineWidth=.5;
  for(let i=0;i<=4;i++){let y=p.t+(ch/4)*i;c.beginPath();c.moveTo(p.l,y);c.lineTo(W-p.r,y);c.stroke();c.fillStyle='#8892a8';c.font='9px system-ui';c.fillText(Math.round(mx-(rng/4)*i),1,y+3);}
  let gr=c.createLinearGradient(0,p.t,0,H-p.b);gr.addColorStop(0,'#3b82f6');gr.addColorStop(1,'#06b6d4');
  c.strokeStyle=gr;c.lineWidth=2;c.lineJoin='round';c.beginPath();
  for(let i=0;i<hist.length;i++){let x=p.l+(i/59)*cw,y=p.t+ch-((hist[i]-mn)/rng)*ch;i===0?c.moveTo(x,y):c.lineTo(x,y);}
  c.stroke();
  let lx=p.l+((hist.length-1)/59)*cw;c.lineTo(lx,H-p.b);c.lineTo(p.l,H-p.b);c.closePath();
  let fg=c.createLinearGradient(0,p.t,0,H-p.b);fg.addColorStop(0,'rgba(59,130,246,.18)');fg.addColorStop(1,'rgba(6,182,212,.02)');
  c.fillStyle=fg;c.fill();
  let ly=p.t+ch-((hist[hist.length-1]-mn)/rng)*ch;
  c.beginPath();c.arc(lx,ly,3,0,Math.PI*2);c.fillStyle='#3b82f6';c.fill();
}
setInterval(()=>{document.getElementById('mr').textContent=mrc+' msg/s';mrc=0;},1000);
window.addEventListener('resize',draw);
conn();
</script>
</body>
</html>
)rawliteral";

/* ──────────────────────────────────────────────────────── */
void setup()
{
    Serial.begin(115200);
    delay(500);
    pinMode(LED_PIN, OUTPUT);

    Serial.println("\n=== ESP32 CAN WiFi Dashboard ===");

    /* ── Start WiFi AP ────────────────────────────────── */
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.printf("WiFi AP started: %s / %s\n", WIFI_SSID, WIFI_PASS);
    Serial.printf("Dashboard: http://%s\n", WiFi.softAPIP().toString().c_str());

    /* ── Web server: serve dashboard ──────────────────── */
    server.on("/", handleRoot);
    server.begin();
    Serial.println("HTTP server started on port 80");

    /* ── WebSocket server ─────────────────────────────── */
    ws.begin();
    ws.onEvent(wsEvent);
    Serial.println("WebSocket server started on port 81");

    /* ── Init CAN (TWAI) ─────────────────────────────── */
    if (!twai_init_bus()) {
        Serial.println("[ERROR] TWAI init failed!");
        blink_led(10, 100, 100);
        while (1) delay(1000);
    }
    Serial.println("[OK] TWAI started. Listening for CAN frames...");
    blink_led(3, 200, 200);
}

/* ──────────────────────────────────────────────────────── */
void loop()
{
    server.handleClient();
    ws.loop();

    twai_message_t message;
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(50));

    if (result == ESP_OK)
    {
        msg_received++;
        digitalWrite(LED_PIN, HIGH);

        /* ── Build JSON for WebSocket clients ─────────── */
        if (message.data_length_code == 8 &&
            message.identifier == 0x100 &&
            message.data[0] == 0xAB &&
            message.data[1] == 0xCD)
        {
            uint8_t  counter  = message.data[2];
            uint16_t sensor   = ((uint16_t)message.data[4] << 8) | message.data[5];
            uint8_t  crc_rx   = message.data[6];
            uint8_t  crc_calc = message.data[2] ^ message.data[4];
            bool     crc_ok   = (crc_rx == crc_calc);

            /* Hex strings for each byte */
            char hex[8][3];
            for (int i = 0; i < 8; i++)
                sprintf(hex[i], "%02X", message.data[i]);

            /* Build JSON */
            char json[256];
            snprintf(json, sizeof(json),
                "{\"id\":%lu,\"counter\":%d,\"sensor\":%d,\"crc_ok\":%s,"
                "\"hex\":[\"%s\",\"%s\",\"%s\",\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"],"
                "\"total\":%lu}",
                (unsigned long)message.identifier,
                counter, sensor, crc_ok ? "true" : "false",
                hex[0], hex[1], hex[2], hex[3],
                hex[4], hex[5], hex[6], hex[7],
                (unsigned long)msg_received);

            /* Send to all WebSocket clients */
            ws.broadcastTXT(json);

            /* Also print to serial */
            Serial.printf("[%lu] ID:0x%03X C=%d S=%d CRC:%s\n",
                millis(), message.identifier, counter, sensor,
                crc_ok ? "OK" : "FAIL");
        }

        digitalWrite(LED_PIN, LOW);
    }

    /* ── Bus health check ─────────────────────────────── */
    if (result == ESP_ERR_TIMEOUT) {
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            if (status.state == TWAI_STATE_BUS_OFF) {
                Serial.println("[WARN] Bus-off! Recovering...");
                twai_initiate_recovery();
                delay(500);
            }
        }
    }
}

/* ──────────────────────────────────────────────────────── */
void handleRoot() {
    server.send(200, "text/html", DASHBOARD_HTML);
}

void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED)
        Serial.printf("[WS] Client #%d connected\n", num);
    else if (type == WStype_DISCONNECTED)
        Serial.printf("[WS] Client #%d disconnected\n", num);
}

bool twai_init_bus(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
        return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

void blink_led(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH); delay(on_ms);
        digitalWrite(LED_PIN, LOW);  delay(off_ms);
    }
}
