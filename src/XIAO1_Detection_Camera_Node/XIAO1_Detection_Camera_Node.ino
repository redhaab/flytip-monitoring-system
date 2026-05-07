// ============================================================
// FlyTip Monitor — Node 1 Firmware
// Appendix B.1
//
// Hardware : Seeed XIAO ESP32-S3 Sense
//            OV3660 camera via internal B2B connector
//            MicroSD via internal B2B connector
//            AM312 PIR on D0 (GPIO1)
//            VL53L1X ToF on D4/D5 (GPIO5/GPIO6, I2C)
//            UART link to Node 2 on D6/D7 (GPIO43/GPIO44)
//
// Arduino IDE settings
//   Board            : XIAO_ESP32S3
//   PSRAM            : OPI PSRAM  (mandatory for camera)
//   USB CDC On Boot  : Enabled
//   Upload Mode      : UART0 / Hardware CDC
//   Upload Speed     : 921600
//
// Libraries required
//   Pololu VL53L1X   (Library Manager)
//   SD               (bundled with ESP32 core)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include <vector>
#include <algorithm>

// ── WiFi access point credentials ───────────────────────────
const char* AP_SSID     = "FlyTipMonitor";
const char* AP_PASSWORD = "flytip1234";

// ── Pin definitions ──────────────────────────────────────────
#define PIR_PIN    1    // D0  — AM312 output
#define TOF_SDA    5    // D4  — VL53L1X SDA
#define TOF_SCL    6    // D5  — VL53L1X SCL
#define SD_CS      21   // MicroSD chip select (internal B2B)
#define UART_TX    43   // D6  — to Node 2 RX
#define UART_RX    44   // D7  — from Node 2 TX

// ── OV3660 camera pins (fixed on Sense board, do not change) ─
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ── State machine thresholds ─────────────────────────────────
#define PIR_WARMUP_MS      60000UL  // PIR stabilisation period
#define WINDOW_MS          15000UL  // Max detection window
#define PERSIST_MS          5000UL  // Delta persistence required
#define DELTA_THRESHOLD_MM   200    // Ground-plane change threshold
#define TOF_POLL_MS          200    // ToF sample cadence
#define BASELINE_SAMPLES       5    // Readings for baseline average
#define COOLDOWN_MS       120000UL  // Inter-event lockout
#define ACK_TIMEOUT_MS      5000UL  // UART ACK wait limit

// ── State machine ────────────────────────────────────────────
enum State {
  STATE_WARMUP,
  STATE_IDLE,
  STATE_WINDOW,
  STATE_CONFIRMED,
  STATE_COOLDOWN
};

State         currentState  = STATE_WARMUP;
unsigned long stateStart    = 0;
unsigned long lastTofPoll   = 0;
unsigned long persistStart  = 0;
bool          persistActive = false;
int16_t       baselineMm    = 0;
int16_t       lastDeltaMm   = 0;
uint32_t      eventCount    = 0;

VL53L1X   tof;
WebServer server(80);

bool tofOk    = false;
bool cameraOk = false;
bool sdOk     = false;
bool wifiOk   = false;

// ════════════════════════════════════════════════════════════
// WEB SERVER — image gallery
// ════════════════════════════════════════════════════════════

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>FlyTip Monitor</title>";
  html += "<style>";
  html += "body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:20px}";
  html += "h1{color:#4ade80;font-size:1.4em}";
  html += "p.sub{color:#888;font-size:.85em;margin-top:-10px}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:16px;margin-top:20px}";
  html += ".card{background:#222;border-radius:8px;overflow:hidden}";
  html += ".card img{width:100%;display:block}";
  html += ".card .info{padding:10px;font-size:.8em;color:#aaa}";
  html += ".card a{display:block;text-align:center;padding:8px;background:#4ade80;";
  html += "color:#111;font-weight:bold;text-decoration:none;font-size:.85em}";
  html += ".empty{color:#666;margin-top:40px;text-align:center;font-size:1.1em}";
  html += ".stats{background:#1a1a1a;border-radius:6px;padding:12px;margin-top:16px;";
  html += "font-size:.85em;color:#888}";
  html += ".stats span{color:#4ade80}";
  html += "</style></head><body>";
  html += "<h1>FlyTip Monitor</h1>";
  html += "<p class='sub'>Event-captured images — Node 1</p>";
  html += "<div class='stats'>";
  html += "Events captured: <span>" + String(eventCount) + "</span> &nbsp;|&nbsp; ";
  html += "SD: <span>" + String(sdOk ? "OK" : "FAIL") + "</span> &nbsp;|&nbsp; ";
  html += "Camera: <span>" + String(cameraOk ? "OK" : "FAIL") + "</span> &nbsp;|&nbsp; ";
  html += "LoRa: <span>on Node 2</span>";
  html += "</div>";

  if (!sdOk) {
    html += "<p class='empty'>SD card not available.</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
    return;
  }

  File dir = SD.open("/photos");
  if (!dir || !dir.isDirectory()) {
    html += "<p class='empty'>No photos directory found.</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
    return;
  }

  std::vector<String> files;
  File entry = dir.openNextFile();
  while (entry) {
    String name = entry.name();
    if (!entry.isDirectory() && name.endsWith(".jpg")) {
      files.push_back(name);
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();

  if (files.empty()) {
    html += "<p class='empty'>No images yet. Waiting for confirmed event.</p>";
  } else {
    std::sort(files.begin(), files.end(),
              [](const String& a, const String& b){ return a > b; });
    html += "<p style='color:#888;font-size:.85em'>";
    html += String(files.size()) + " image(s) — newest first</p>";
    html += "<div class='grid'>";
    for (const String& fname : files) {
      html += "<div class='card'>";
      html += "<img src='/img?f=" + fname + "' loading='lazy' alt='" + fname + "'>";
      html += "<div class='info'>" + fname + "</div>";
      html += "<a href='/dl?f=" + fname + "'>Download</a>";
      html += "</div>";
    }
    html += "</div>";
  }
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleImage() {
  if (!server.hasArg("f")) {
    server.send(400, "text/plain", "Missing parameter");
    return;
  }
  File f = SD.open("/photos/" + server.arg("f"), FILE_READ);
  if (!f) { server.send(404, "text/plain", "Not found"); return; }
  server.sendHeader("Cache-Control", "max-age=86400");
  server.streamFile(f, "image/jpeg");
  f.close();
}

void handleDownload() {
  if (!server.hasArg("f")) {
    server.send(400, "text/plain", "Missing parameter");
    return;
  }
  String fname = server.arg("f");
  File f = SD.open("/photos/" + fname, FILE_READ);
  if (!f) { server.send(404, "text/plain", "Not found"); return; }
  server.sendHeader("Content-Disposition",
                    "attachment; filename=\"" + fname + "\"");
  server.streamFile(f, "image/jpeg");
  f.close();
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ════════════════════════════════════════════════════════════
// SUBSYSTEM INITIALISERS
// ════════════════════════════════════════════════════════════

bool initWiFi() {
  Serial.print(F("[WiFi] Starting access point... "));
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 4)) {
    Serial.println(F("FAIL"));
    return false;
  }
  delay(500);
  IPAddress ip = WiFi.softAPIP();
  Serial.println(F("OK"));
  Serial.printf("[WiFi] Network  : %s\n", AP_SSID);
  Serial.printf("[WiFi] Password : %s\n", AP_PASSWORD);
  Serial.printf("[WiFi] URL      : http://%s\n", ip.toString().c_str());
  return true;
}

void setupWebServer() {
  server.on("/",    handleRoot);
  server.on("/img", handleImage);
  server.on("/dl",  handleDownload);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("[Web] Server started on port 80"));
}

bool initToF() {
  Serial.print(F("[ToF] Initialising... "));
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println(F("FAIL — check SDA D4, SCL D5"));
    return false;
  }
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(TOF_POLL_MS);
  Serial.println(F("OK"));
  return true;
}

bool initCamera() {
  Serial.print(F("[Camera] Initialising... "));
  if (!psramFound()) {
    Serial.println(F("FAIL — Tools > PSRAM > OPI PSRAM"));
    return false;
  }
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.frame_size   = FRAMESIZE_SVGA;   // 800x600
  cfg.jpeg_quality = 10;
  cfg.fb_count     = 1;
  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println(F("FAIL"));
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
  }
  Serial.println(F("OK"));
  return true;
}

bool initSD() {
  Serial.print(F("[SD] Initialising... "));
  if (!SD.begin(SD_CS)) {
    Serial.println(F("FAIL — check FAT32 card inserted"));
    return false;
  }
  if (!SD.exists("/photos")) SD.mkdir("/photos");
  Serial.printf("OK (%llu MB)\n", SD.cardSize() / (1024 * 1024));
  return true;
}

// ════════════════════════════════════════════════════════════
// DETECTION HELPERS
// ════════════════════════════════════════════════════════════

int16_t measureBaseline() {
  long sum  = 0;
  int  good = 0;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    delay(TOF_POLL_MS);
    int16_t r = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred()) { sum += r; good++; }
  }
  return good > 0 ? (int16_t)(sum / good) : 2000;
}

String captureAndSave() {
  Serial.print(F("[Camera] Capturing... "));
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(F("FAIL — null buffer"));
    return "FAIL";
  }
  char fname[52];
  snprintf(fname, sizeof(fname),
           "/photos/EVT_%05lu_%08lu.jpg",
           eventCount, (unsigned long)millis());
  File f = SD.open(fname, FILE_WRITE);
  if (!f) {
    Serial.println(F("FAIL — cannot open file"));
    esp_camera_fb_return(fb);
    return "FAIL";
  }
  f.write(fb->buf, fb->len);
  f.close();
  Serial.printf("OK (%lu bytes — %s)\n",
                (unsigned long)fb->len, fname);

  // OCR stub — write NOREAD companion file
  // Replace contents with TFLite inference output when model available
  char tname[52];
  snprintf(tname, sizeof(tname),
           "/photos/EVT_%05lu_%08lu.txt",
           eventCount, (unsigned long)millis());
  File tf = SD.open(tname, FILE_WRITE);
  if (tf) { tf.println("NOREAD"); tf.close(); }

  esp_camera_fb_return(fb);
  return String(fname);
}

bool sendTriggerToNode2(int16_t deltaMm) {
  uint32_t uptime = millis() / 1000;
  char msg[40];
  snprintf(msg, sizeof(msg), "EVT:%d:%lu",
           deltaMm, (unsigned long)uptime);
  Serial.printf("[UART] Sending: %s\n", msg);
  Serial1.println(msg);

  // Wait for ACK
  unsigned long start = millis();
  while (millis() - start < ACK_TIMEOUT_MS) {
    if (Serial1.available()) {
      String ack = Serial1.readStringUntil('\n');
      ack.trim();
      Serial.printf("[UART] ACK: %s\n", ack.c_str());
      return ack == "ACK:OK";
    }
  }
  Serial.println(F("[UART] ACK timeout"));
  return false;
}

void logCSV(const char* state, int pir, int tofMm,
            int deltaMm, bool event,
            const char* camSt, const char* fname,
            const char* txSt) {
  Serial.printf("%lu,%s,%d,%d,%d,%d,%s,%s,%s\n",
                millis(), state, pir, tofMm, deltaMm,
                event ? 1 : 0, camSt, fname, txSt);
}

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  Serial.println(F("\n========================================"));
  Serial.println(F("  FlyTip Monitor — Node 1"));
  Serial.println(F("  Detection, Camera and Image Server"));
  Serial.println(F("========================================"));

  pinMode(PIR_PIN, INPUT);
  Wire.begin(TOF_SDA, TOF_SCL);
  Serial1.begin(9600, SERIAL_8N1, UART_RX, UART_TX);

  tofOk    = initToF();
  cameraOk = initCamera();
  sdOk     = initSD();
  wifiOk   = initWiFi();

  if (wifiOk) setupWebServer();

  Serial.println(F("\n-- Subsystem status --"));
  Serial.printf("  ToF    : %s\n", tofOk    ? "OK" : "FAIL");
  Serial.printf("  Camera : %s\n", cameraOk ? "OK" : "FAIL");
  Serial.printf("  SD     : %s\n", sdOk     ? "OK" : "FAIL");
  Serial.printf("  WiFi AP: %s\n", wifiOk   ? "OK" : "FAIL");
  Serial.println(F("  LoRa   : on Node 2"));

  if (wifiOk) {
    Serial.println(F("\n========================================"));
    Serial.println(F("  Connect to WiFi : FlyTipMonitor"));
    Serial.println(F("  Password        : flytip1234"));
    Serial.println(F("  Browser         : http://192.168.4.1"));
    Serial.println(F("========================================\n"));
  }

  Serial.println(F("ts_ms,state,pir,tof_mm,delta_mm,"
                   "event,cam_status,filename,tx_status"));

  stateStart = millis();
  Serial.printf("[WARMUP] PIR stabilising for %lu s...\n",
                PIR_WARMUP_MS / 1000);
}

// ════════════════════════════════════════════════════════════
// MAIN LOOP — four-state machine
// ════════════════════════════════════════════════════════════

void loop() {
  if (wifiOk) server.handleClient();

  unsigned long now    = millis();
  int           pirVal = digitalRead(PIR_PIN);

  switch (currentState) {

    // ── WARMUP ───────────────────────────────────────────────
    case STATE_WARMUP:
      if (now - stateStart >= PIR_WARMUP_MS) {
        Serial.println(F("[WARMUP] Complete — entering IDLE"));
        currentState = STATE_IDLE;
        stateStart   = now;
      }
      break;

    // ── IDLE ─────────────────────────────────────────────────
    case STATE_IDLE:
      logCSV("IDLE", pirVal, 0, 0, false, "-", "-", "-");
      if (pirVal == HIGH) {
        Serial.printf("\n[PIR] Motion at %lu ms\n", now);
        if (tofOk) {
          baselineMm = measureBaseline();
          Serial.printf("[BASELINE] %d mm\n", baselineMm);
        }
        currentState  = STATE_WINDOW;
        stateStart    = now;
        persistActive = false;
        lastTofPoll   = now;
      }
      delay(100);
      break;

    // ── WINDOW ───────────────────────────────────────────────
    case STATE_WINDOW:
      if (now - lastTofPoll >= TOF_POLL_MS && tofOk) {
        lastTofPoll = now;
        int16_t reading = tof.readRangeContinuousMillimeters();
        if (tof.timeoutOccurred()) break;

        lastDeltaMm     = baselineMm - reading;
        bool triggered  = abs(lastDeltaMm) >= DELTA_THRESHOLD_MM;

        logCSV("WINDOW", pirVal, reading, lastDeltaMm,
               false, "-", "-", "-");

        if (triggered) {
          if (!persistActive) {
            persistActive = true;
            persistStart  = now;
            Serial.printf("[WINDOW] Delta %d mm — persistence started\n",
                          lastDeltaMm);
          } else if (now - persistStart >= PERSIST_MS) {
            currentState = STATE_CONFIRMED;
            stateStart   = now;
            Serial.printf("[CONFIRMED] Delta %d mm held for %lu ms\n",
                          lastDeltaMm, PERSIST_MS);
            break;
          }
        } else {
          if (persistActive)
            Serial.println(F("[WINDOW] Delta dropped — persistence reset"));
          persistActive = false;
        }
      }
      if (now - stateStart >= WINDOW_MS) {
        Serial.println(F("[WINDOW] Timeout — returning to IDLE"));
        logCSV("WINDOW_TIMEOUT", pirVal, 0, 0, false, "-", "-", "-");
        currentState = STATE_IDLE;
        stateStart   = now;
      }
      break;

    // ── CONFIRMED ────────────────────────────────────────────
    case STATE_CONFIRMED: {
      eventCount++;
      String filename = "-";

      // 1. UART trigger to Node 2 — latency critical, done first
      bool txOk = sendTriggerToNode2(lastDeltaMm);

      // 2. Camera capture and SD write
      if (cameraOk && sdOk) {
        filename = captureAndSave();
      }

      const char* camSt = (filename != "-" && filename != "FAIL")
                          ? "OK" : "FAIL";

      logCSV("CONFIRMED", pirVal, 0, lastDeltaMm, true,
             camSt, filename.c_str(), txOk ? "OK" : "FAIL");

      Serial.printf("[EVENT #%lu] TX:%s  File:%s\n",
                    eventCount,
                    txOk ? "OK" : "FAIL",
                    filename.c_str());

      if (wifiOk) {
        Serial.println(F("[Web] Refresh http://192.168.4.1 to view image"));
      }

      currentState = STATE_COOLDOWN;
      stateStart   = now;
      break;
    }

    // ── COOLDOWN ─────────────────────────────────────────────
    case STATE_COOLDOWN: {
      unsigned long elapsed = now - stateStart;
      if (elapsed >= COOLDOWN_MS) {
        Serial.println(F("[COOLDOWN] Complete — returning to IDLE"));
        currentState = STATE_IDLE;
        stateStart   = now;
      } else {
        static unsigned long lastPrint = 0;
        if (now - lastPrint >= 10000) {
          Serial.printf("[COOLDOWN] %lu s remaining\n",
                        (COOLDOWN_MS - elapsed) / 1000);
          lastPrint = now;
        }
      }
      break;
    }
  }
}
