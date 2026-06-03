#include <Arduino.h>
#include "LoRa_E220.h"

#define ENABLE_RSSI true

LoRa_E220 e220ttl(16, 17, &Serial2, 36, 32, 33, UART_BPS_RATE_9600);

void setup() {
    Serial.begin(115200);
    delay(500);

    e220ttl.begin();
    Serial.println("RX ready");
}

void loop() {
    if (e220ttl.available() > 1) {

#ifdef ENABLE_RSSI
        ResponseContainer rc = e220ttl.receiveMessageRSSI();
#else
        ResponseContainer rc = e220ttl.receiveMessage();
#endif

        if (rc.status.code != 1) {
            Serial.println(rc.status.getResponseDescription());
            return;
        }

        Serial.print("DATA: ");
        Serial.println(rc.data);

#ifdef ENABLE_RSSI
        Serial.print("RSSI: ");
        Serial.println(rc.rssi);
#endif
        Serial.println("-----------------------");
    }
}
