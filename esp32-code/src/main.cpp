#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include <SPI.h>
#include <GxEPD2_4C.h>
#include <Adafruit_NeoPixel.h>

// ==========================================
// PIN-DEFINITIONEN & GLOBALE SETTINGS
// ==========================================
const char* ssid = "ESP32-CAM"; 
WebServer server(80);

const int BUTTON_PIN = 5; // Pin D4
const int LED_PIN    = 6; // Pin D5 (Für die WS2812B NeoPixel LED)

// NeoPixel initialisieren (1 LED am Pin D5)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// Display Pins
const int EPD_BUSY = 1;  const int EPD_RES  = 2;
const int EPD_DC   = 3;  const int EPD_CS   = 4;
const int SPI_SCK  = 7;  const int SPI_MOSI = 9;

GxEPD2_4C<GxEPD2_154c_GDEM0154F51H, GxEPD2_154c_GDEM0154F51H::HEIGHT> display(
  GxEPD2_154c_GDEM0154F51H(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY)
);

// Kamera Pins
#define PWDN_GPIO_NUM -1   
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10   
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39   
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11     
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14     
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18     
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15     
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47   
#define PCLK_GPIO_NUM 13

// Settings 
int cropX = 60, cropY = 20; 
int brightBias = 0;
int redB = 0, yelB = 0;
float ditherStrength = 0.0;

bool triggerCapture = false;
bool useFlash = false; // Steuert, ob die LED genutzt wird
bool isFlashing = false;

// Puffer im PSRAM
int16_t* errorBuf = nullptr;      
uint8_t* previewBuf = nullptr; 

unsigned long lastProcessTime = 0;
unsigned long lastClientRequestTime = 0; 
const unsigned long CLIENT_TIMEOUT_MS = 3000; 

long sqLUT[256]; 

// ==========================================
// DAS HTML & JAVASCRIPT
// ==========================================
const char index_html[] PROGMEM = R"====(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>DitherVision 4C</title>
    <style>
        body { margin: 0; padding: 10px; background: #1e1e2e; color: #f8f8f2; display: flex; flex-direction: column; align-items: center; font-family: sans-serif; }
        h1 { margin: 0 0 10px 0; color: #ff79c6; font-size: 1.4rem; }
        .layout { display: flex; flex-direction: column; gap: 10px; width: 100%; max-width: 400px; }
        .card { background: #282a36; padding: 12px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.3); text-align: center; }
        canvas { width: 100%; max-width: 160px; border: 3px solid #44475a; border-radius: 8px; image-rendering: pixelated; background: #000; }
        .btn-group { display: flex; gap: 10px; margin-top: 10px; }
        .btn { flex: 1; padding: 12px; border: none; border-radius: 8px; font-size: 16px; font-weight: bold; cursor: pointer; }
        .btn-green { background: #50fa7b; color: #282a36; }
        .btn-yellow { background: #f1fa8c; color: #282a36; }
        .btn:active { transform: scale(0.96); }
        .slider-group { margin-bottom: 8px; text-align: left; }
        .slider-group label { display: flex; justify-content: space-between; font-size: 12px; font-weight: bold; margin-bottom: 4px; color: #bd93f9; }
        input[type=range] { width: 100%; margin: 0; accent-color: #ff79c6; }
        .status { font-size: 12px; color: #50fa7b; margin-top: 8px; min-height: 15px; font-weight: bold; }
    </style>
</head>
<body>
    <h1>DitherVision 4C</h1>
    
    <div class="layout">
        <div class="card">
            <canvas id="canvasProcessed" width="200" height="200"></canvas>
            
            <div class="btn-group">
                <button class="btn btn-green" onclick="flashToEPaper(0)">📸 Normal</button>
                <button class="btn btn-yellow" onclick="flashToEPaper(1)">💡 Blitz</button>
            </div>
            
            <div class="status" id="statusMsg"></div>
        </div>

        <div class="card">
            <div class="slider-group">
                <label>Helligkeit <span id="valBright">0%</span></label>
                <input type="range" id="bright" min="-100" max="100" value="0" 
                       oninput="document.getElementById('valBright').innerText = this.value + '%'" 
                       onchange="syncSettings(false)">
            </div>
            <div class="slider-group">
                <label>Rot Bias <span id="valRedB">0%</span></label>
                <input type="range" id="redB" min="-100" max="100" value="0" 
                       oninput="document.getElementById('valRedB').innerText = this.value + '%'" 
                       onchange="syncSettings(false)">
            </div>
            <div class="slider-group">
                <label>Gelb Bias <span id="valYelB">0%</span></label>
                <input type="range" id="yelB" min="-100" max="100" value="0" 
                       oninput="document.getElementById('valYelB').innerText = this.value + '%'" 
                       onchange="syncSettings(false)">
            </div>
            <div class="slider-group">
                <label>Dithering <span id="valDither">0%</span></label>
                <input type="range" id="dither" min="0" max="100" value="0" 
                       oninput="document.getElementById('valDither').innerText = this.value + '%'" 
                       onchange="syncSettings(false)">
            </div>
            <div class="slider-group">
                <label>Crop X <span id="valX">50%</span></label>
                <input type="range" id="offX" min="0" max="100" value="50" 
                       oninput="document.getElementById('valX').innerText = this.value + '%'" 
                       onchange="syncSettings(false)">
            </div>
            <div class="slider-group">
                <label>Crop Y <span id="valY">50%</span></label>
                <input type="range" id="offY" min="0" max="100" value="50" 
                       oninput="document.getElementById('valY').innerText = this.value + '%'" 
                       onchange="syncSettings(false)">
            </div>
        </div>
    </div>

    <script>
        const ctx = document.getElementById('canvasProcessed').getContext('2d', { willReadFrequently: true });
        let isFlashingJS = false;
        
        const palette = [ [0,0,0,255], [255,255,255,255], [255,0,0,255], [255,255,0,255] ];

        function syncSettings(triggerFlash, withLed = 0) {
            let x = document.getElementById('offX').value;
            let y = document.getElementById('offY').value;
            let b = document.getElementById('bright').value;
            let rb = document.getElementById('redB').value;
            let yb = document.getElementById('yelB').value;
            let d = document.getElementById('dither').value;

            fetch(`/${triggerFlash ? 'flash' : 'sync'}?x=${x}&y=${y}&b=${b}&rb=${rb}&yb=${yb}&d=${d}&led=${withLed}`);

            if (triggerFlash) {
                isFlashingJS = true;
                document.getElementById('statusMsg').innerText = withLed ? "Flasht mit LED... (ca. 15s)" : "Flasht... (ca. 13s)";
                setTimeout(() => { 
                    document.getElementById('statusMsg').innerText = ""; 
                    isFlashingJS = false;
                    fetchFrame(); 
                }, withLed ? 16000 : 14000); // Mehr Zeit wegen Fokus & Belichtung
            }
        }

        function flashToEPaper(withLed) { syncSettings(true, withLed); }

        async function fetchFrame() {
            if (isFlashingJS) return; 
            try {
                const res = await fetch('/frame?t=' + Date.now());
                if(res.status === 200) {
                    const buf = await res.arrayBuffer();
                    const data = new Uint8Array(buf);
                    if (data.length === 40000) {
                        let imgData = ctx.createImageData(200, 200);
                        for (let i = 0; i < 40000; i++) {
                            let c = palette[data[i]] || palette[0];
                            let idx = i * 4;
                            imgData.data[idx] = c[0]; imgData.data[idx+1] = c[1];
                            imgData.data[idx+2] = c[2]; imgData.data[idx+3] = c[3];
                        }
                        ctx.putImageData(imgData, 0, 0);
                    }
                }
            } catch (err) {}
            if (!isFlashingJS) setTimeout(fetchFrame, 100); 
        }

        fetchFrame();
    </script>
</body>
</html>
)====";

// ==========================================
// C++ BILDVERARBEITUNG FÜR PREVIEW & E-PAPER
// ==========================================

void processFrame(camera_fb_t * fb) {
    memset(errorBuf, 0, 200 * 200 * 3 * sizeof(int16_t));
    int ditherInt = (int)(ditherStrength * 256.0);

    for (int y = 0; y < 200; y++) {
        if (y % 20 == 0) yield(); 

        for (int x = 0; x < 200; x++) {
            int srcX = constrain(x + cropX, 0, 319);
            int srcY = constrain(y + cropY, 0, 239);
            int i = (srcY * 320 + srcX) * 2;
            
            uint16_t p565 = (fb->buf[i] << 8) | fb->buf[i+1];
            uint8_t origR = (p565 >> 8) & 0xF8; origR |= (origR >> 5);
            uint8_t origG = (p565 >> 3) & 0xFC; origG |= (origG >> 6);
            uint8_t origB = (p565 << 3) & 0xF8; origB |= (origB >> 5);

            int idx = (y * 200 + x) * 3;
            int rawR = origR + brightBias + errorBuf[idx];
            int rawG = origG + brightBias + errorBuf[idx+1];
            int rawB = origB + brightBias + errorBuf[idx+2];

            uint8_t r = constrain(rawR, 0, 255);
            uint8_t g = constrain(rawG, 0, 255);
            uint8_t b = constrain(rawB, 0, 255);

            uint8_t ir = 255 - r;
            uint8_t ig = 255 - g;
            uint8_t ib = 255 - b;

            long distBlack  = sqLUT[r]  + sqLUT[g]  + sqLUT[b];
            long distWhite  = sqLUT[ir] + sqLUT[ig] + sqLUT[ib];
            long distRed    = sqLUT[ir] + sqLUT[g]  + sqLUT[b]  - redB;
            long distYellow = sqLUT[ir] + sqLUT[ig] + sqLUT[b]  - yelB;

            long minDist = distBlack;
            uint8_t colorIdx = 0; 
            
            if (distWhite < minDist)  { minDist = distWhite;  colorIdx = 1; }
            if (distRed < minDist)    { minDist = distRed;    colorIdx = 2; }
            if (distYellow < minDist) { minDist = distYellow; colorIdx = 3; }

            previewBuf[y * 200 + x] = colorIdx;

            if (ditherInt > 0) {
                int cR = 0, cG = 0, cB = 0;
                if(colorIdx == 1) { cR=255; cG=255; cB=255; }
                else if(colorIdx == 2) { cR=255; }
                else if(colorIdx == 3) { cR=255; cG=255; }

                int errR = rawR - cR;
                int errG = rawG - cG;
                int errB = rawB - cB;

                int eR_base = (errR * ditherInt) / 4096;
                int eG_base = (errG * ditherInt) / 4096;
                int eB_base = (errB * ditherInt) / 4096;

                auto distribute = [&](int dx, int dy, int weight) {
                    if (x + dx >= 0 && x + dx < 200 && y + dy < 200) {
                        int nIdx = ((y + dy) * 200 + (x + dx)) * 3;
                        errorBuf[nIdx]   += eR_base * weight;
                        errorBuf[nIdx+1] += eG_base * weight;
                        errorBuf[nIdx+2] += eB_base * weight;
                    }
                };
                
                distribute(1, 0, 7); 
                distribute(-1, 1, 3);
                distribute(0, 1, 5); 
                distribute(1, 1, 1);
            }
        }
    }
}

void captureAndFlash() {
    isFlashing = true; 
    sensor_t * s = esp_camera_sensor_get();
    
    // 1. LED einschalten und Belichtungszeit anpassen lassen
    if (useFlash) {
        Serial.println("Blitz AN!");
        strip.setPixelColor(0, strip.Color(255, 255, 255)); // Volles Weiß
        strip.show();
        delay(1000); // 1 Sekunde warten, damit die Kamera das Licht misst
    }

    // 2. Autofokus triggern
    Serial.println("Fokussiere Bild...");
    s->set_reg(s, 0x3022, 0xFF, 0x03); // Single AF Trigger für OV5640
    delay(1500); // Warten bis der Linsenmotor fertig ist

    // 3. Altes Bild verwerfen, frisches Bild holen
    Serial.println("Nehme finales Bild auf...");
    camera_fb_t * fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb); 

    fb = esp_camera_fb_get();
    if (fb) {
        processFrame(fb);
        esp_camera_fb_return(fb);
    }

    // 4. LED sofort wieder ausschalten
    if (useFlash) {
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
    }

    // 5. Bild auf E-Paper flashen
    Serial.println("Flashe auf Display...");
    display.init(0); 
    display.firstPage();
    
    const uint16_t epdColors[] = {GxEPD_BLACK, GxEPD_WHITE, GxEPD_RED, GxEPD_YELLOW};
    
    do {
        for (int y = 0; y < 200; y++) {
            for (int x = 0; x < 200; x++) {
                uint8_t cIdx = previewBuf[y * 200 + x];
                display.drawPixel(x, y, epdColors[cIdx]);
            }
        }
    } while (display.nextPage());
    
    display.hibernate();
    Serial.println("Flash erfolgreich!");
    
    // Continuous AF wieder aktivieren für den Stream
    s->set_reg(s, 0x3022, 0xFF, 0x04);
    
    isFlashing = false; 
    useFlash = false; // Zurücksetzen
}

// ==========================================
// SETUP & ROUTING
// ==========================================
void handleSettingsUpdate() {
    if(server.hasArg("x")) cropX = (server.arg("x").toInt() * 120) / 100;
    if(server.hasArg("y")) cropY = (server.arg("y").toInt() * 40) / 100; 
    if(server.hasArg("b")) brightBias = server.arg("b").toInt(); 
    if(server.hasArg("rb")) redB = server.arg("rb").toInt() * 50; 
    if(server.hasArg("yb")) yelB = server.arg("yb").toInt() * 50; 
    if(server.hasArg("d")) ditherStrength = server.arg("d").toInt() / 100.0;
    if(server.hasArg("led")) useFlash = (server.arg("led").toInt() == 1);

    if(server.uri() == "/flash") triggerCapture = true;
    
    server.send(200, "text/plain", "OK");
}

void setup() {
    Serial.begin(115200);
    
    // LED Initialisieren
    strip.begin();
    strip.setPixelColor(0, strip.Color(0,0,0));
    strip.show();

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    for (int i = 0; i < 256; i++) {
        sqLUT[i] = (long)i * (long)i;
    }

    errorBuf = (int16_t*)ps_calloc(200 * 200 * 3, sizeof(int16_t));
    previewBuf = (uint8_t*)ps_calloc(200 * 200, sizeof(uint8_t));
    if(!errorBuf || !previewBuf) Serial.println("FEHLER: PSRAM konnte nicht allokiert werden!");

    SPI.begin(SPI_SCK, -1, SPI_MOSI, -1);
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565; 
    config.frame_size = FRAMESIZE_QVGA; 
    config.jpeg_quality = 10;
    
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST; 
    esp_camera_init(&config);
    
    sensor_t * s = esp_camera_sensor_get();
    s->set_vflip(s, 1); 
    s->set_hmirror(s, 0);
    s->set_reg(s, 0x3022, 0xFF, 0x04); // Continuous AF aktivieren

    WiFi.softAP(ssid);
    WiFi.setSleep(false); 
    
    Serial.print("WLAN aufgespannt! IP: "); Serial.println(WiFi.softAPIP());

    server.on("/", []() { server.send(200, "text/html", index_html); });
    server.on("/sync", handleSettingsUpdate);
    server.on("/flash", handleSettingsUpdate);
    
    server.on("/frame", []() {
        lastClientRequestTime = millis(); 

        if (isFlashing) {
            server.send(503, "text/plain", "Busy");
            return;
        }
        server.setContentLength(40000);
        server.send(200, "application/octet-stream", "");
        server.client().write(previewBuf, 40000);
    });

    server.begin();
}

void loop() {
    server.handleClient();

    bool hasActiveClient = (millis() - lastClientRequestTime) < CLIENT_TIMEOUT_MS;

    if (!isFlashing && hasActiveClient && (millis() - lastProcessTime > 50)) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (fb) {
            processFrame(fb);
            esp_camera_fb_return(fb);
        }
        lastProcessTime = millis();
    }

    // NON-BLOCKING BUTTON HANDLING MIT LANGEM DRÜCKEN
    static bool btnActive = false;
    static unsigned long btnPressTime = 0;
    bool currentButtonState = (digitalRead(BUTTON_PIN) == LOW);
    
    if (currentButtonState && !btnActive) {
        // Knopf wurde gerade gedrückt
        btnPressTime = millis();
        btnActive = true;
    } else if (!currentButtonState && btnActive) {
        // Knopf wurde losgelassen, Zeit berechnen
        unsigned long duration = millis() - btnPressTime;
        btnActive = false;
        
        if (duration > 50) { // Entprellen (Ignoriere alles unter 50ms)
            if (duration > 500) {
                // Länger als 0.5 Sekunden -> Blitz an!
                useFlash = true; 
            } else {
                // Kürzer -> Normal
                useFlash = false; 
            }
            triggerCapture = true;
        }
    }

    if (triggerCapture) {
        captureAndFlash();
        triggerCapture = false;
    }
}