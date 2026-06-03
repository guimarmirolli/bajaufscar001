#include <Arduino.h>
#include "LoRa_E220.h"

#define ENABLE_RSSI true

#define debugacl 1

static const uint32_t NOISE_SAMPLE_INTERVAL_MS = 1000;
static const uint32_t NOISE_QUIET_WINDOW_MS = 100;
static const uint32_t AMBIENT_RSSI_TIMEOUT_MS = 80;
static const uint32_t NOISE_STATUS_INTERVAL_MS = 5000;
static const uint32_t AMBIENT_RETRY_INTERVAL_MS = 30000;
static const uint32_t MAX_AMBIENT_FAILS_BEFORE_FALLBACK = 10;
static const int FALLBACK_NOISE_DBM = -117;

static uint32_t rxPktTot = 0;
static uint32_t rxBytesTot = 0;
static int8_t lastRSSI = 0;
static int lastNoiseDbm = 0;
static bool hasNoiseSample = false;
static uint32_t lastNoiseSampleAt = 0;
static uint32_t lastPacketAt = 0;
static uint32_t lastNoiseStatusAt = 0;
static uint32_t ambientReadFailTot = 0;
static bool fallbackNoiseActive = false;
static bool ambientProbeBackoff = false;
static uint32_t lastAmbientRetryAt = 0;

LoRa_E220 e220ttl(&Serial2, 36, 32, 33, UART_BPS_RATE_9600);

typedef struct __attribute__((packed)) {
  uint16_t seq;
  uint32_t t_ms;
  int16_t temperature10;
  uint16_t rpm;
  uint16_t speed10;
} TelemetryPacket;

#if debugacl
typedef struct __attribute__((packed)) {
  uint16_t seq;
  uint32_t t_ms;
  int16_t temperature10;
  uint16_t rpm;
  uint16_t speed10;
  int16_t ax_median10;
  int16_t ay_median10;
  int16_t az_median10;
} TelemetryPacketDebug;
#endif

static int rssiToDbm(int8_t rawRssi) {
  uint8_t raw = (uint8_t)rawRssi;
  return (raw == 0) ? 0 : -(256 - raw);
}

static bool ensureRssiConfiguration() {
  ResponseStructContainer c = e220ttl.getConfiguration();
  if (c.status.code != E220_SUCCESS) {
    Serial.printf("Failed to read config: %s\n", c.status.getResponseDescription().c_str());
    return false;
  }

  Configuration configuration = *(Configuration*) c.data;
  c.close();

  bool changed = false;

  if (configuration.TRANSMISSION_MODE.enableRSSI != RSSI_ENABLED) {
    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
    changed = true;
  }

  if (configuration.OPTION.RSSIAmbientNoise != RSSI_AMBIENT_NOISE_ENABLED) {
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_ENABLED;
    changed = true;
  }

  if (!changed) {
    return true;
  }

  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  if (rs.code != E220_SUCCESS) {
    Serial.printf("Failed to enable RSSI features: %s\n", rs.getResponseDescription().c_str());
    return false;
  }

  return true;
}

static bool sampleAmbientNoiseDbm(int &noiseDbm) {
  const uint8_t ambientNoiseCommand[3] = {READ_CONFIGURATION, 0x00, 0x01};
  uint8_t response[4] = {0};
  size_t received = 0;
  uint32_t start = millis();

  if (Serial2.available() > 0) {
    return false;
  }

  size_t written = Serial2.write(ambientNoiseCommand, sizeof(ambientNoiseCommand));
  if (written != sizeof(ambientNoiseCommand)) {
    return false;
  }
  Serial2.flush();

  while ((millis() - start) < AMBIENT_RSSI_TIMEOUT_MS && received < sizeof(response)) {
    if (Serial2.available() > 0) {
      response[received++] = (uint8_t)Serial2.read();
    }
  }

  if (received != sizeof(response)) {
    return false;
  }

  if (response[0] != RETURNED_COMMAND || response[1] != 0x00 || response[2] != 0x01) {
    return false;
  }

  noiseDbm = rssiToDbm((int8_t)response[3]);
  return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    e220ttl.begin();

    bool rssiReady = ensureRssiConfiguration();

    ResponseStructContainer c = e220ttl.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    Serial.print("RSSI enabled: ");
    Serial.println(configuration.TRANSMISSION_MODE.enableRSSI);
    Serial.print("Ambient noise RSSI enabled: ");
    Serial.println(configuration.OPTION.RSSIAmbientNoise);
    c.close();

    if (!rssiReady) {
      Serial.println("RSSI setup incomplete, SNR estimate may be unavailable.");
    }

    lastPacketAt = millis();
    Serial.println("RX ready");
}

void loop() 
{
  int available = e220ttl.available();

  if (available <= 0)
  {
    uint32_t now = millis();
    bool quietEnough = (now - lastPacketAt) >= NOISE_QUIET_WINDOW_MS;
    bool ambientRetryReady = !ambientProbeBackoff || ((now - lastAmbientRetryAt) >= AMBIENT_RETRY_INTERVAL_MS);
    if (ENABLE_RSSI && quietEnough && ambientRetryReady && (now - lastNoiseSampleAt) >= NOISE_SAMPLE_INTERVAL_MS)
    {
      int measuredNoiseDbm = 0;
      if (sampleAmbientNoiseDbm(measuredNoiseDbm))
      {
        lastNoiseDbm = measuredNoiseDbm;
        hasNoiseSample = true;
        fallbackNoiseActive = false;
        ambientProbeBackoff = false;
        ambientReadFailTot = 0;
        Serial.printf("[LoRa NOISE] noise: %d dBm after %lu ms idle\n",
                      lastNoiseDbm,
                      (unsigned long)(now - lastPacketAt));
      }
      else
      {
        ambientReadFailTot++;
        lastAmbientRetryAt = now;
        if (!hasNoiseSample && ambientReadFailTot >= MAX_AMBIENT_FAILS_BEFORE_FALLBACK)
        {
          lastNoiseDbm = FALLBACK_NOISE_DBM;
          hasNoiseSample = true;
          fallbackNoiseActive = true;
          ambientProbeBackoff = true;
          Serial.printf("[LoRa NOISE] using fallback noise floor: %d dBm\n", lastNoiseDbm);
        }
      }
      lastNoiseSampleAt = now;
    }

    if (!hasNoiseSample && (now - lastNoiseStatusAt) >= NOISE_STATUS_INTERVAL_MS)
    {
      if (!quietEnough)
      {
        Serial.printf("[LoRa NOISE] waiting for idle gap >= %lu ms\n",
                      (unsigned long)NOISE_QUIET_WINDOW_MS);
      }
      else
      {
        if (fallbackNoiseActive)
        {
          Serial.printf("[LoRa NOISE] fallback active (%d dBm), ambient retry in background\n", lastNoiseDbm);
        }
        else
        {
          Serial.printf("[LoRa NOISE] ambient RSSI read failed (%lu attempts)\n",
                        (unsigned long)ambientReadFailTot);
        }
      }
      lastNoiseStatusAt = now;
    }
    return;
  }

    if (available > 0) 
  {
#if debugacl
    ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(TelemetryPacketDebug));
    if (rc.status.code == E220_SUCCESS)
    {
        TelemetryPacketDebug pkt = *(TelemetryPacketDebug*)rc.data;
        rxPktTot++;
        rxBytesTot += sizeof(TelemetryPacketDebug);
        uint32_t now = millis();
        lastPacketAt = now;
        int rssiDbm = 0;
        if (ENABLE_RSSI) {
            lastRSSI = rc.rssi;
            rssiDbm = rssiToDbm(lastRSSI);
        }
        float temperature = pkt.temperature10 / 10.0f;
        float speed = pkt.speed10 / 10.0f;
        float ax = pkt.ax_median10 / 10.0f;
        float ay = pkt.ay_median10 / 10.0f;
        float az = pkt.az_median10 / 10.0f;
        Serial.printf(
            "[LoRa RX] seq=%u t=%lu Temp=%.1fC RPM=%u SPD=%.1f km/h AX=%.1f AY=%.1f AZ=%.1f RSSI=%d dBm\n",
            pkt.seq, pkt.t_ms, temperature, pkt.rpm, speed, ax, ay, az, rssiDbm
        );
        rc.close();
    }
#else
    ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(TelemetryPacket));
    if (rc.status.code == E220_SUCCESS)
    {
        TelemetryPacket pkt = *(TelemetryPacket*)rc.data;
        rxPktTot++;
        rxBytesTot += sizeof(TelemetryPacket);
        uint32_t now = millis();
        lastPacketAt = now;
        int rssiDbm = 0;
        if (ENABLE_RSSI) {
            lastRSSI = rc.rssi;
            rssiDbm = rssiToDbm(lastRSSI);
        }
        float temperature = pkt.temperature10 / 10.0f;
        float speed = pkt.speed10 / 10.0f;
        Serial.printf(
            "[LoRa RX] seq=%u t=%lu Temp=%.1fC RPM=%u SPD=%.1f km/h RSSI=%d dBm\n",
            pkt.seq, pkt.t_ms, temperature, pkt.rpm, speed, rssiDbm
        );
        rc.close();
    }
#endif
    else
    {
      Serial.printf("ERR;%lu;%s;rx_tot=%lu;bytes_tot=%lu\n",
                    millis(),
                    rc.status.getResponseDescription().c_str(),
                    (unsigned long)rxPktTot,
                    (unsigned long)rxBytesTot);
    }
  }
}