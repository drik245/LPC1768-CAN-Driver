/**
 * CAN_Dashboard.ino — All-in-one ESP32 CAN Test Dashboard
 * 
 * WiFi AP: CAN_Dashboard / canbus123
 * URL: http://192.168.4.1
 * 
 * Modes (selectable from web):
 *   1. RX Test   — receive frames from LPC TX demo
 *   2. TX Test   — send ID=0x200 to LPC RX demo
 *   3. Filter    — send 6 IDs for LPC filter demo
 *   4. Priority  — burst 4 frames for priority demo
 */

#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#define CAN_TX  GPIO_NUM_5
#define CAN_RX  GPIO_NUM_4
#define LED_PIN 2

WebServer server(80);
WebSocketsServer ws(81);

enum Mode { MODE_RX=0, MODE_TX=1, MODE_FILTER=2, MODE_PRIORITY=3 };
volatile Mode currentMode = MODE_RX;
uint32_t txCount=0, rxCount=0, lastTxMs=0;
uint8_t txCounter=0;

/* ── CAN init/reinit ──────────────────────────────────── */
bool canRunning = false;

void canStop() {
    if (canRunning) { twai_stop(); twai_driver_uninstall(); canRunning=false; }
}

bool canStart() {
    canStop();
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    g.tx_queue_len = 10; g.rx_queue_len = 20;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g,&t,&f)!=ESP_OK) return false;
    if (twai_start()!=ESP_OK) { twai_driver_uninstall(); return false; }
    canRunning = true; return true;
}

void canRecover() {
    twai_status_info_t s;
    if (twai_get_status_info(&s)==ESP_OK && s.state==TWAI_STATE_BUS_OFF) {
        Serial.println("[WARN] Bus-off, recovering...");
        twai_initiate_recovery(); delay(200);
        canStart();
    }
}

/* ── TX helpers ───────────────────────────────────────── */
bool canSend(uint32_t id, uint8_t *data, uint8_t dlc) {
    twai_message_t m; memset(&m,0,sizeof(m));
    m.identifier=id; m.data_length_code=dlc;
    memcpy(m.data, data, dlc);
    return twai_transmit(&m, pdMS_TO_TICKS(200))==ESP_OK;
}

void sendJSON(const char* type, uint32_t id, uint8_t* data, uint8_t dlc, const char* extra) {
    char hex[64]=""; 
    for(int i=0;i<dlc&&i<8;i++) { char h[4]; sprintf(h,"%s\"%02X\"",i?",":"",data[i]); strcat(hex,h); }
    char json[256];
    snprintf(json,sizeof(json),"{\"type\":\"%s\",\"id\":%lu,\"dlc\":%d,\"hex\":[%s],\"count\":%lu%s}",
        type,(unsigned long)id,dlc,hex,(unsigned long)(txCount+rxCount),extra?extra:"");
    ws.broadcastTXT(json);
}

/* ── Dashboard HTML ───────────────────────────────────── */
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN Bus Dashboard</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#0a0e1a;--card:#1a2235;--border:#2a3450;--text:#e2e8f0;--muted:#8892a8;
--accent:#3b82f6;--green:#22c55e;--red:#ef4444;--cyan:#06b6d4;--yellow:#f59e0b;--purple:#a855f7}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);min-height:100vh}
.hdr{background:linear-gradient(135deg,#0f172a,#1e293b);border-bottom:1px solid var(--border);
padding:14px 24px;display:flex;align-items:center;justify-content:space-between}
.hdr h1{font-size:1.2rem;font-weight:700}.hdr h1 span{color:var(--accent)}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:8px}
.dot.on{background:var(--green);box-shadow:0 0 8px rgba(34,197,94,.4);animation:pulse 1.5s infinite}
.dot.off{background:var(--muted)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
.tabs{display:flex;gap:0;padding:0 24px;background:#0f1525;border-bottom:1px solid var(--border)}
.tab{padding:12px 24px;cursor:pointer;font-size:.85rem;font-weight:600;color:var(--muted);
border-bottom:2px solid transparent;transition:all .3s}
.tab:hover{color:var(--text)}.tab.active{color:var(--accent);border-bottom-color:var(--accent)}
.main{padding:20px 24px}
.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin-bottom:16px}
.sc{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:16px;transition:border-color .3s}
.sc:hover{border-color:var(--accent)}
.sl{font-size:.7rem;font-weight:600;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:6px}
.sv{font-family:monospace;font-size:1.8rem;font-weight:700}
.sv.gr{color:var(--green)}.sv.cy{color:var(--cyan)}.sv.ac{color:var(--accent)}.sv.yl{color:var(--yellow)}.sv.pu{color:var(--purple)}
.ss{font-size:.75rem;color:var(--muted);margin-top:4px}
.log{background:#111827;border:1px solid var(--border);border-radius:10px;padding:12px;
max-height:400px;overflow-y:auto;font-family:'Cascadia Code',monospace;font-size:.78rem}
.log::-webkit-scrollbar{width:5px}.log::-webkit-scrollbar-thumb{background:var(--border);border-radius:3px}
.ll{padding:4px 0;border-bottom:1px solid rgba(42,52,80,.3);display:flex;gap:8px;align-items:center}
.lt{color:var(--muted);min-width:60px}.lid{min-width:50px;font-weight:700}
.lp{font-size:.7rem;padding:2px 8px;border-radius:10px;font-weight:600}
.lp.pass{background:rgba(34,197,94,.15);color:var(--green)}.lp.reject{background:rgba(239,68,68,.15);color:var(--red)}
.lp.crit{background:rgba(168,85,247,.15);color:var(--purple)}.lp.high{background:rgba(59,130,246,.15);color:var(--accent)}
.lp.med{background:rgba(245,158,11,.15);color:var(--yellow)}.lp.low{background:rgba(100,116,139,.15);color:var(--muted)}
.ld{color:var(--cyan)}.dir{font-weight:700;font-size:.7rem}
.dir.tx{color:var(--yellow)}.dir.rx{color:var(--green)}
.mode-info{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:16px;margin-bottom:16px}
.mode-info h3{font-size:.9rem;margin-bottom:8px;color:var(--accent)}
.mode-info p{font-size:.8rem;color:var(--muted);line-height:1.5}
.mode-info code{color:var(--cyan);background:#111827;padding:1px 6px;border-radius:4px;font-size:.75rem}
@media(max-width:700px){.stats{grid-template-columns:1fr 1fr}.tabs{overflow-x:auto}}
</style></head><body>
<div class="hdr">
<h1>&#x1F697; <span>CAN Bus</span> Dashboard</h1>
<div style="display:flex;align-items:center;gap:8px">
<span class="dot off" id="dot"></span>
<span id="st" style="font-size:.82rem;color:var(--muted)">Connecting...</span>
</div></div>
<div class="tabs">
<div class="tab active" onclick="setMode(0)">&#x1F4E5; RX Test</div>
<div class="tab" onclick="setMode(1)">&#x1F4E4; TX Test</div>
<div class="tab" onclick="setMode(2)">&#x1F50D; Filter</div>
<div class="tab" onclick="setMode(3)">&#x26A1; Priority</div>
</div>
<div class="main">
<div class="mode-info" id="minfo"></div>
<div class="stats">
<div class="sc"><div class="sl">Mode</div><div class="sv ac" id="smode">RX</div></div>
<div class="sc"><div class="sl">Frames</div><div class="sv gr" id="scount">0</div><div class="ss" id="srate">0/s</div></div>
<div class="sc"><div class="sl">Last ID</div><div class="sv cy" id="sid">--</div></div>
<div class="sc"><div class="sl">Status</div><div class="sv yl" id="sstat">Idle</div></div>
</div>
<div class="log" id="lg"></div>
</div>
<script>
let wsc,fc=0,rc=0,mode=0;
const mnames=['RX Test','TX Test','Filter','Priority'];
const minfo=[
'<h3>&#x1F4E5; RX Test — Receive from LPC1768</h3><p>LPC runs <code>main_tx_demo.c</code>. ESP32 receives and displays frames here.</p>',
'<h3>&#x1F4E4; TX Test — Send to LPC1768</h3><p>ESP32 sends <code>ID=0x200</code> every 2s. LPC runs <code>main_rx_demo.c</code>.</p>',
'<h3>&#x1F50D; Filter Demo — 6 IDs for LPC Filter</h3><p>Sends 6 IDs: <code>0x080,0x100,0x150,0x200,0x2FF,0x300</code>. LPC filter passes only <code>0x100</code> and <code>0x200</code>.</p>',
'<h3>&#x26A1; Priority Demo — Burst TX</h3><p>Queues 4 frames at once: <code>0x300→0x200→0x100→0x080</code>. CAN hardware sends lowest ID first.</p>'
];
function conn(){
wsc=new WebSocket('ws://'+location.hostname+':81/');
wsc.onopen=()=>{document.getElementById('dot').className='dot on';document.getElementById('st').textContent='Live';};
wsc.onclose=()=>{document.getElementById('dot').className='dot off';document.getElementById('st').textContent='Reconnecting...';setTimeout(conn,2000);};
wsc.onmessage=(e)=>{try{let d=JSON.parse(e.data);onMsg(d)}catch(x){}};
}
function setMode(m){
mode=m; fc=0;
document.querySelectorAll('.tab').forEach((t,i)=>t.className='tab'+(i===m?' active':''));
document.getElementById('minfo').innerHTML=minfo[m];
document.getElementById('smode').textContent=mnames[m];
document.getElementById('scount').textContent='0';
document.getElementById('sid').textContent='--';
document.getElementById('lg').innerHTML='';
if(wsc&&wsc.readyState===1) wsc.send('MODE:'+m);
}
function getPrio(id){
if(id<=0xFF) return ['CRITICAL','crit'];
if(id<=0x1FF) return ['HIGH','high'];
if(id<=0x2FF) return ['MEDIUM','med'];
return ['LOW','low'];
}
function getFilter(id){ return (id===0x100||id===0x200)?['PASS','pass']:['REJECT','reject']; }
function onMsg(d){
fc++;
document.getElementById('scount').textContent=fc;
document.getElementById('sid').textContent='0x'+d.id.toString(16).toUpperCase().padStart(3,'0');
document.getElementById('sstat').textContent=d.type==='rx'?'Receiving':'Sending';
let lg=document.getElementById('lg'),div=document.createElement('div');div.className='ll';
let t=new Date().toLocaleTimeString('en',{hour12:false});
let hex=d.hex?d.hex.join(' '):'';
let idStr='0x'+d.id.toString(16).toUpperCase().padStart(3,'0');
let extra='';
if(mode===2){let f=getFilter(d.id);extra='<span class="lp '+f[1]+'">'+f[0]+'</span>';}
if(mode===3){let p=getPrio(d.id);extra='<span class="lp '+p[1]+'">'+p[0]+'</span>';}
let dir=d.type==='rx'?'<span class="dir rx">RX</span>':'<span class="dir tx">TX</span>';
div.innerHTML='<span class="lt">'+t+'</span>'+dir+
'<span class="lid" style="color:var(--cyan)">'+idStr+'</span>'+
'<span class="ld">'+hex+'</span>'+extra;
lg.prepend(div);while(lg.children.length>100)lg.removeChild(lg.lastChild);
}
setMode(0); conn();
setInterval(()=>{document.getElementById('srate').textContent=rc+'/s';rc=0;},1000);
</script></body></html>
)rawliteral";

/* ── Filter frames table ─────────────────────────────── */
struct FDef { uint32_t id; uint8_t prio; };
FDef filterFrames[] = {
    {0x080,1},{0x100,2},{0x150,2},{0x200,3},{0x2FF,3},{0x300,4}
};

/* ── WebSocket event ──────────────────────────────────── */
void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
    if (type == WStype_CONNECTED) {
        Serial.printf("[WS] Client #%d connected\n", num);
    } else if (type == WStype_TEXT) {
        String msg = (char*)payload;
        if (msg.startsWith("MODE:")) {
            int m = msg.substring(5).toInt();
            if (m >= 0 && m <= 3) {
                currentMode = (Mode)m;
                txCount = 0; rxCount = 0; txCounter = 0; lastTxMs = 0;
                Serial.printf("[MODE] Switched to %d\n", m);
            }
        }
    }
}

/* ── Mode handlers ────────────────────────────────────── */
void handleRX() {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(50)) == ESP_OK) {
        rxCount++;
        uint8_t *d = rx.data;
        sendJSON("rx", rx.identifier, d, rx.data_length_code, "");
        Serial.printf("[RX #%d] ID=0x%03X DLC=%d\n", rxCount, rx.identifier, rx.data_length_code);
    }
}

void handleTX() {
    if (millis() - lastTxMs < 2000) return;
    lastTxMs = millis();
    uint8_t d[4] = { txCounter, 0x03, 0x02, 0x00 };
    if (canSend(0x200, d, 4)) {
        txCount++; txCounter++;
        sendJSON("tx", 0x200, d, 4, "");
        Serial.printf("[TX #%d] ID=0x200 OK\n", txCount);
    } else { canRecover(); }
}

void handleFilter() {
    if (millis() - lastTxMs < 500) return;
    lastTxMs = millis();
    static int idx = 0;
    FDef &f = filterFrames[idx];
    uint8_t d[4] = { txCounter, f.prio, (uint8_t)(f.id>>8), (uint8_t)(f.id&0xFF) };
    if (canSend(f.id, d, 4)) {
        txCount++;
        char extra[64]; 
        snprintf(extra,sizeof(extra),",\"filter\":\"%s\"", (f.id==0x100||f.id==0x200)?"PASS":"REJECT");
        sendJSON("tx", f.id, d, 4, extra);
        Serial.printf("[TX #%d] ID=0x%03X %s\n", txCount, f.id, (f.id==0x100||f.id==0x200)?"PASS":"REJECT");
    } else { canRecover(); }
    idx = (idx+1) % 6;
    if (idx == 0) txCounter++;
}

void handlePriority() {
    if (millis() - lastTxMs < 3000) return;
    lastTxMs = millis();
    uint32_t ids[] = {0x300, 0x200, 0x100, 0x080}; // reverse order
    Serial.printf("[BURST #%d] Queuing reverse...\n", txCounter+1);
    for (int i = 0; i < 4; i++) {
        uint8_t d[4] = { txCounter, (uint8_t)(4-i), (uint8_t)(ids[i]>>8), (uint8_t)(ids[i]&0xFF) };
        if (canSend(ids[i], d, 4)) {
            txCount++;
            sendJSON("tx", ids[i], d, 4, "");
        } else { canRecover(); }
    }
    txCounter++;
}

/* ── Setup & Loop ─────────────────────────────────────── */
void setup() {
    Serial.begin(115200); delay(500);
    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("\n=== CAN Bus All-in-One Dashboard ===");
    
    WiFi.softAP("CAN_Dashboard", "canbus123");
    Serial.printf("WiFi AP: CAN_Dashboard / canbus123\n");
    Serial.printf("URL: http://%s\n", WiFi.softAPIP().toString().c_str());
    
    server.on("/", [](){ server.send(200, "text/html", HTML); });
    server.begin();
    ws.begin(); ws.onEvent(wsEvent);
    
    if (!canStart()) { Serial.println("[FAIL] CAN init!"); while(1) delay(1000); }
    Serial.println("[OK] CAN active. Default mode: RX\n");
}

void loop() {
    server.handleClient();
    ws.loop();
    
    switch (currentMode) {
        case MODE_RX:       handleRX(); break;
        case MODE_TX:       handleTX(); break;
        case MODE_FILTER:   handleFilter(); break;
        case MODE_PRIORITY: handlePriority(); break;
    }
    
    // Bus health
    static uint32_t lastCheck = 0;
    if (millis() - lastCheck > 5000) { lastCheck = millis(); canRecover(); }
}
