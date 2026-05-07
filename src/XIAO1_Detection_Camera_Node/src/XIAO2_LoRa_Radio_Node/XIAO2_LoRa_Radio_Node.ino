// ============================================================
// FlyTip Monitor — Node 2 Firmware
// Appendix B.2
//
// Hardware : Seeed XIAO ESP32-S3
//            Wio-SX1262 LoRa radio via internal B2B connector
//            UART link from Node 1 on D7/D6 (GPIO44/GPIO43)
//
// Arduino IDE settings
//   Board            : XIAO_ESP32S3
//   PSRAM            : OPI PSRAM
//   USB CDC On Boot  : Enabled
//   Upload Mode      : UART0 / Hardware CDC
//   Upload Speed     : 921600
//
// Libraries required
//   RadioLib by Jan Gromeš  (Library Manager)
// ============================================================

#include <Arduino.h>
#include <RadioLib.h>

// ── Wio-SX1262 pins via B2B connector ───────────────────────
// These map to the XIAO header pins exposed by the B2B footprint
#define LORA_NSS   2    // D1 — SPI chip select
#define LORA_DIO1  3    // D2 — interrupt line
#define LORA_BUSY  4    // D3 — busy indicator
#define LORA_RST   44   // D7 — hardware reset

// ── UART from Node 1 ─────────────────────────────────────────
#define UART_RX    44   // D7 — receives EVT messages from Node 1
#define UART_TX    43   // D6 — sends ACK replies to Node 1

// ── LoRa parameters (EU868) ──────────────────────────────────
#define LORA_FREQ   868.1   // MHz — EU868 band
#define LORA_SF     9       // Spreading factor
#define LORA_BW     125.0   // Bandwidth kHz
#define LORA_CR     7       // Coding rate 4/7
#define LORA_POWER  14      // Output power dBm

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

uint8_t seqNum  = 0;
bool    radioOk = false;

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  // UART from Node 1
  Serial1.begin(9600, SERIAL_8N1, UART_RX, UART_TX);

  Serial.println(F("\n========================================"));
  Serial.println(F("  FlyTip Monitor — Node 2"));
  Serial.println(F("  LoRa Radio Node"));
  Serial.println(F("========================================"));

  Serial.print(F("[LoRa] Initialising... "));
  int state = radio.begin(LORA_FREQ);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("FAIL (error %d)\n", state);
    radioOk = false;
  } else {
    radio.setSpreadingFactor(LORA_SF);
    radio.setBandwidth(LORA_BW);
    radio.setCodingRate(LORA_CR);
    radio.setOutputPower(LORA_POWER);
    radio.setSyncWord(0x34);   // LoRaWAN public sync word
    Serial.printf("OK — %.1f MHz SF%d BW%.0f %d dBm\n",
                  LORA_FREQ, LORA_SF, LORA_BW, LORA_POWER);
    radioOk = true;
  }

  Serial.println(F("\n-- Subsystem status --"));
  Serial.printf("  LoRa radio : %s\n", radioOk ? "OK" : "FAIL");
  Serial.println(F("  UART       : listening on D7 (GPIO44)"));

  Serial.println(F("\n[UART] Listening for trigger from Node 1..."));
}

// ════════════════════════════════════════════════════════════
// MAIN LOOP
// ════════════════════════════════════════════════════════════

void loop() {
  if (!Serial1.available()) return;

  String msg = Serial1.readStringUntil('\n');
  msg.trim();
  if (msg.length() == 0) return;

  Serial.printf("[UART] Received: %s\n", msg.c_str());

  // Only process EVT trigger messages
  if (!msg.startsWith("EVT:")) return;

  // Parse: EVT:delta_mm:uptime_s
  int c1 = msg.indexOf(':');
  int c2 = msg.indexOf(':', c1 + 1);
  if (c1 < 0 || c2 < 0) {
    Serial.println(F("[UART] Malformed message — ignored"));
    Serial1.println("ACK:FAIL");
    return;
  }
  int16_t  delta  = (int16_t)msg.substring(c1 + 1, c2).toInt();
  uint32_t uptime = (uint32_t)msg.substring(c2 + 1).toInt();

  Serial.printf("[EVENT] Delta=%d mm  Uptime=%lu s  Seq=%d\n",
                delta, (unsigned long)uptime, seqNum);

  if (!radioOk) {
    Serial.println(F("[LoRa] Radio unavailable — ACK:FAIL"));
    Serial1.println("ACK:FAIL");
    return;
  }

  // Build 9-byte payload
  // Byte 0    : event flag (0x01 = confirmed event)
  // Bytes 1-4 : uptime_s, little-endian uint32
  // Byte 5    : battery_pct (0xFF = unmeasured, USB powered)
  // Bytes 6-7 : tof_delta_mm, little-endian int16
  // Byte 8    : sequence number
  uint8_t payload[9];
  payload[0] = 0x01;
  payload[1] = (uptime)       & 0xFF;
  payload[2] = (uptime >>  8) & 0xFF;
  payload[3] = (uptime >> 16) & 0xFF;
  payload[4] = (uptime >> 24) & 0xFF;
  payload[5] = 0xFF;
  payload[6] = (uint8_t)(delta & 0xFF);
  payload[7] = (uint8_t)((delta >> 8) & 0xFF);
  payload[8] = seqNum;

  Serial.print(F("[LoRa] Transmitting... "));
  int txState = radio.transmit(payload, 9);

  if (txState == RADIOLIB_ERR_NONE) {
    Serial.printf("OK (ToA: %.0f ms  Seq: %d)\n",
                  radio.getTimeOnAir(9) / 1000.0, seqNum);
    Serial1.println("ACK:OK");
    seqNum++;
  } else {
    Serial.printf("FAIL (error %d)\n", txState);
    Serial1.println("ACK:FAIL");
  }
}
