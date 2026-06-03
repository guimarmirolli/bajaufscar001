#include <Arduino.h>
#include "LoRa_E220.h"

#define ENABLE_RSSI true

static uint32_t rxPktTot = 0;
static uint32_t rxBytesTot = 0;
static int8_t lastRSSI = 0;
static uint32_t lastPacketAt = 0;

LoRa_E220 e220ttl(&Serial2, 36, 32, 33, UART_BPS_RATE_9600);

typedef struct __attribute__((packed)) {
  uint16_t seq;
  uint32_t t_ms;
  int16_t temperature10;
  uint16_t rpm;
  uint16_t speed10;
} TelemetryPacket;

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

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    e220ttl.begin();

    ensureRssiConfiguration();

    ResponseStructContainer c = e220ttl.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    Serial.print("RSSI enabled: ");
    Serial.println(configuration.TRANSMISSION_MODE.enableRSSI);
    Serial.print("Ambient noise RSSI enabled: ");
    Serial.println(configuration.OPTION.RSSIAmbientNoise);
    c.close();

    lastPacketAt = millis();
    Serial.println("RX ready");
}

void loop() 
{
  int available = e220ttl.available();

  if (available <= 0)
  {
    uint32_t now = millis();
    if (available > 0) 
    {
      ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(TelemetryPacket));
      if (rc.status.code == E220_SUCCESS)
      {
          TelemetryPacket pkt = *(TelemetryPacket*)rc.data;
          rxPktTot++;
          rxBytesTot += sizeof(TelemetryPacket);
          uint32_t now = millis();
          lastPacketAt = now;
          int rssiDbm = 0;
          if (ENABLE_RSSI) 
          {
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
      else
      {
        Serial.printf("ERRO;%lu\n",
                      millis());
      }
    }
  }
}