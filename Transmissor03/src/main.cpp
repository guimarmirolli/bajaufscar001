#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <MAX6675.h>
#include <LoRa_E220.h>
#include <SD.h>
#include <esp_timer.h> 
#include <math.h>

#define DEBUG 0
#define debugrpm 0
#define debugacl 1

#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22

#define TIRE_RADIUS_M   0.28f
#define SPEED_PER_PPS   (2.0f * 3.14159265f * TIRE_RADIUS_M * 3.6f)

// ================== SYSTEM DATA ==================
typedef struct {
  float temperature;   // Temperatura (°C)
  float ax, ay, az;    // Aceleração (m/s²)
  float rpm;           // Rotações por minuto
  float speed;         // Velocidade
} SystemData;

// Compact telemetry packet for LoRa transport.
// Binary payloads are much smaller than ASCII strings, reducing airtime and
// improving delivery robustness at long-range settings (e.g., SF11/SF12).
// Smaller packets also lower collision probability and energy per message.
typedef struct __attribute__((packed)) {
  uint16_t seq;
  uint32_t t_ms;
  int16_t temperature10;
  uint16_t rpm;
  uint16_t speed10;
} TelemetryPacket;

static_assert(sizeof(TelemetryPacket) == 12, "TelemetryPacket must be 12 bytes");

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

static SystemData sys = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static SemaphoreHandle_t sys_mutex = NULL;  // Mutex para acesso ao sys struct

// ================== BATTERY LEVEL ==================
#define PIN_BATT_LEVEL 35  

float updateBatteryLevel() {
  uint32_t v_div_mv = analogReadMilliVolts(PIN_BATT_LEVEL); // mV calibrado no pino
  return (float)v_div_mv * (126.7f / 26.7f) / 1000.0f;     // Reconstrói tensão da bateria em V
}

// ================== ACCELEROMETER STATS ==================
typedef struct {
  float ax_median;
  float ay_median;
  float az_median;
} AccelStats;

static AccelStats accelStats = {};
// accelStats shares sys_mutex since written by taskSensors, read by taskLoRa/SDLogging

#define ACCEL_WINDOW_MAX_SAMPLES 8

typedef struct {
  float buf_ax[ACCEL_WINDOW_MAX_SAMPLES];
  float buf_ay[ACCEL_WINDOW_MAX_SAMPLES];
  float buf_az[ACCEL_WINDOW_MAX_SAMPLES];
  uint16_t count;
  uint32_t window_start_ms;
  bool started;
} AccelWindowAggregator;

static inline void accel_window_reset(AccelWindowAggregator *w, uint32_t now_ms) {
  w->count = 0;
  w->window_start_ms = now_ms;
  w->started = true;
}

static inline void accel_window_add_sample(AccelWindowAggregator *w, uint32_t now_ms, float ax, float ay, float az) {
  if (!w->started) {
    accel_window_reset(w, now_ms);
  }

  if (w->count < ACCEL_WINDOW_MAX_SAMPLES) {
    w->buf_ax[w->count] = ax;
    w->buf_ay[w->count] = ay;
    w->buf_az[w->count] = az;
    w->count++;
  }
}

static inline bool accel_window_ready(const AccelWindowAggregator *w, uint32_t now_ms, uint32_t window_ms) {
  return w->started && w->count > 0 && (now_ms - w->window_start_ms) >= window_ms;
}

static inline float accel_median_of(float *arr, uint16_t n) {
  // Insertion sort in-place (small n)
  for (uint16_t i = 1; i < n; i++) {
    float key = arr[i];
    int16_t j = (int16_t)i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  if (n == 0) return 0.0f;
  if (n % 2 == 1) return arr[n / 2];
  return (arr[n / 2 - 1] + arr[n / 2]) * 0.5f;
}

static inline void accel_window_emit_and_restart(AccelWindowAggregator *w, uint32_t now_ms, AccelStats *out) {
  if (w->count == 0) {
    return;
  }

  float tmp_ax[ACCEL_WINDOW_MAX_SAMPLES];
  float tmp_ay[ACCEL_WINDOW_MAX_SAMPLES];
  float tmp_az[ACCEL_WINDOW_MAX_SAMPLES];
  for (uint16_t i = 0; i < w->count; i++) {
    tmp_ax[i] = w->buf_ax[i];
    tmp_ay[i] = w->buf_ay[i];
    tmp_az[i] = w->buf_az[i];
  }

  out->ax_median = accel_median_of(tmp_ax, w->count);
  out->ay_median = accel_median_of(tmp_ay, w->count);
  out->az_median = accel_median_of(tmp_az, w->count);

  accel_window_reset(w, now_ms);
}

static SemaphoreHandle_t i2c_mutex = NULL;  // Mutex para I2C bus

// ================== ERROR EVENT GROUP ==================
// Bits for error signaling: bit 0 = sensor, bit 1 = LoRa, bit 2 = SD
#define ERROR_SENSOR_BIT  (1 << 0)
#define ERROR_LORA_BIT    (1 << 1)
#define ERROR_SD_BIT      (1 << 2)

static EventGroupHandle_t error_events = NULL;

// ================== HEALTH / FLAGS ==================
typedef struct {
  volatile bool init_ok;          // inicializou no boot
  volatile bool ok;               // última operação ok
  volatile uint32_t last_ok_ms;   // millis() da última vez que funcionou
  volatile uint8_t fail_count;    // falhas consecutivas (satura)
} Health_t;

static Health_t h_lora  = {false, false, 0, 0};
static Health_t h_sd    = {false, false, 0, 0};
static Health_t h_mpu   = {false, false, 0, 0};
static Health_t h_therm = {false, false, 0, 0};

static SemaphoreHandle_t health_mutex = NULL;  // Mutex para acesso aos Health_t structs

static inline void health_mark_ok(Health_t *h) {
  if (xSemaphoreTake(health_mutex, portMAX_DELAY) == pdTRUE) {
    h->ok = true;
    h->last_ok_ms = millis();
    h->fail_count = 0;
    xSemaphoreGive(health_mutex);
  }
}

static inline void health_mark_fail(Health_t *h) {
  if (xSemaphoreTake(health_mutex, portMAX_DELAY) == pdTRUE) {
    h->ok = false;
    if (h->fail_count < 255) h->fail_count++;
    xSemaphoreGive(health_mutex);
  }
}

static inline void health_set_init(Health_t *h, bool ok) {
  if (xSemaphoreTake(health_mutex, portMAX_DELAY) == pdTRUE) {
    h->init_ok = ok;
    h->ok = ok;
    h->last_ok_ms = ok ? millis() : 0;
    h->fail_count = ok ? 0 : 1;
    xSemaphoreGive(health_mutex);
  }
}

static inline bool health_alive(const Health_t *h, uint32_t timeout_ms) {
  uint32_t now = millis();
  bool result = false;
  
  if (xSemaphoreTake(health_mutex, portMAX_DELAY) == pdTRUE) {
    result = (h->init_ok && h->ok && (now - h->last_ok_ms) < timeout_ms);
    xSemaphoreGive(health_mutex);
  }
  
  return result;
}

static void i2c_recover_bus() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);

  // If SDA is held low, clock SCL up to 9 times to release the bus.
  for (int i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {
    pinMode(I2C_SCL_PIN, OUTPUT);
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(10);
  }

  // Generate a STOP condition to free the bus.
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(10);

  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
}

// ================== SENSORS ======================
Adafruit_MPU6050 mpu;
MAX6675 thermocouple(27, 26, 25);

const int spdPin = 13;
const int rpmPin = 14;
volatile uint32_t cntRPM = 0;
volatile uint32_t cntSPD = 0;
volatile uint32_t lastRpmEdgeUs = 0;
volatile uint32_t lastSpdEdgeUs = 0;
volatile uint32_t rpmPeriodUs = 0;
volatile uint32_t spdPeriodUs = 0;
portMUX_TYPE rpm_isr_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR countRPM_ISR() {
  portENTER_CRITICAL_ISR(&rpm_isr_mux);
  uint32_t nowUs = micros();
  uint32_t elapsed = nowUs - lastRpmEdgeUs;

  // Ignora pulsos que chegam em menos de 1500µs (ruído/bounce)
  // Seguro até ~10.000 RPM com 4 imãs
  if (lastRpmEdgeUs != 0 && elapsed < 1500) {
    portEXIT_CRITICAL_ISR(&rpm_isr_mux);
    return;
  }

  if (lastRpmEdgeUs != 0) rpmPeriodUs = elapsed;
  lastRpmEdgeUs = nowUs;
  cntRPM++;
  portEXIT_CRITICAL_ISR(&rpm_isr_mux);
}

void IRAM_ATTR countSPD_ISR() {
  portENTER_CRITICAL_ISR(&rpm_isr_mux);
  uint32_t nowUs = micros();
  if (lastSpdEdgeUs != 0) spdPeriodUs = nowUs - lastSpdEdgeUs;
  lastSpdEdgeUs = nowUs;
  cntSPD++;
  portEXIT_CRITICAL_ISR(&rpm_isr_mux);
}

// ================== LoRa  =========================
#define DESTINATION_ADDL 3
#define LORA_CHANNEL     23
LoRa_E220 e220ttl(&Serial2, 36, 32, 33, UART_BPS_RATE_9600);

static bool lora_init_ok() {
  ResponseStructContainer info = e220ttl.getModuleInformation();
  bool ok = (info.status.code == E220_SUCCESS);
  info.close();
  return ok;
}

// ================== SD ============================
SPIClass spiSD(VSPI);
const int SD_CS_PIN = 5;
const int SPI_SCK   = 18;
const int SPI_MISO  = 19;
const int SPI_MOSI  = 23;

// Timer for logging timestamp (ms)
static int64_t sd_startTime = 0;

// ================== TASK: Sensors ==================
void taskSensors(void *pvParameters) {
  uint32_t lastThermo = 0;

  const uint32_t MPU_SAMPLE_MS = 50;
  const uint32_t ACCEL_STATS_WINDOW_MS = 250;
  uint32_t lastMPU = 0;          // Timing guard for MPU6050 sampling
  AccelWindowAggregator accelAgg = {};

  for (;;) {
    uint32_t now = millis();

    // Thermocouple sampling (every 300ms)
    if (now - lastThermo >= 300) {
      float t = thermocouple.readCelsius();
      lastThermo = now;

      if (!isfinite(t) || t < -100.0f || t > 1000.0f) {
        health_mark_fail(&h_therm);
        xEventGroupSetBits(error_events, ERROR_SENSOR_BIT);
      } else {
        if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
          sys.temperature = t;
          xSemaphoreGive(sys_mutex);
          health_mark_ok(&h_therm);
        } else {
          health_mark_fail(&h_therm);
        }
      }
      
    }

    // MPU6050 sampling (every 50ms)
    if (h_mpu.init_ok && (now - lastMPU >= MPU_SAMPLE_MS)) {
      lastMPU = now;
      sensors_event_t accel, gyro, temp;
      bool mpu_ok = false;

      // Acquire I2C mutex for MPU6050 I2C operation
      if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        mpu.getEvent(&accel, &gyro, &temp);
        xSemaphoreGive(i2c_mutex);

        // Validate acceleration data
        mpu_ok = isfinite(accel.acceleration.x) &&
                 isfinite(accel.acceleration.y) &&
                 isfinite(accel.acceleration.z);
      }

      // Unified error handling: consolidate all failure paths
      if (!mpu_ok) {
        health_mark_fail(&h_mpu);
        xEventGroupSetBits(error_events, ERROR_SENSOR_BIT);
      } else {
        // Valid data: write to sys and update coherent window aggregate.
        if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
          float ax = accel.acceleration.x;
          float ay = accel.acceleration.y;
          float az = accel.acceleration.z;

          accel_window_add_sample(&accelAgg, now, ax, ay, az);

          // Publish coherent vector mean every fixed time window.
          if (accel_window_ready(&accelAgg, now, ACCEL_STATS_WINDOW_MS)) {
            accel_window_emit_and_restart(&accelAgg, now, &accelStats);
          }
          
          xSemaphoreGive(sys_mutex);
          health_mark_ok(&h_mpu);
        } else {
          health_mark_fail(&h_mpu);
          xEventGroupSetBits(error_events, ERROR_SENSOR_BIT);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ================== TASK: RPM / SPEED =============
void taskRPM(void *pvParameters) {
  const float RPM_PULSES_PER_REV = 4.0f;
  const uint32_t STALE_MULTIPLIER = 100;
  static float rpmFiltered = 0.0f;
  const float alpha = 0.4f;

  for (;;) {
    uint32_t pulsesRPM = 0;
    uint32_t pulsesSPD = 0;
    uint32_t rpmPeriod = 0;
    uint32_t spdPeriod = 0;
    uint32_t lastRpmUs = 0;
    uint32_t lastSpdUs = 0;

    // Snapshot and reset counters atomically for this 500ms window.
    portENTER_CRITICAL(&rpm_isr_mux);
    pulsesRPM = cntRPM;
    pulsesSPD = cntSPD;
    rpmPeriod = rpmPeriodUs;
    spdPeriod = spdPeriodUs;
    lastRpmUs = lastRpmEdgeUs;
    lastSpdUs = lastSpdEdgeUs;
    cntRPM = 0;
    cntSPD = 0;
    portEXIT_CRITICAL(&rpm_isr_mux);


    float rpm = 0.0f;
    float speed = 0.0f;
    uint32_t nowUs = micros();

    // Prefer period-based estimate.
    if (lastRpmUs != 0 && rpmPeriod > 0 && (nowUs - lastRpmUs) < ((uint64_t)rpmPeriod * STALE_MULTIPLIER))
    {
      float ppsRpm = 1000000.0f / (float)rpmPeriod;
      rpm = (ppsRpm * 60.0f) / RPM_PULSES_PER_REV;
    } 
    else if (pulsesRPM > 0) {
      rpm = (pulsesRPM * 60.0f) / (RPM_PULSES_PER_REV * 0.25f);
    }
    else 
    {
      rpm = 0.0f;
    }

    if (lastSpdUs != 0 &&
        spdPeriod > 0 &&
        (nowUs - lastSpdUs) < ((uint64_t)spdPeriod * STALE_MULTIPLIER))
    {
        float ppsSpd = 1000000.0f / (float)spdPeriod;
        speed = ppsSpd * SPEED_PER_PPS;
    }
    else if (pulsesSPD > 0)
    {
        speed = pulsesSPD * SPEED_PER_PPS;
    }
    else
    {
        speed = 0.0f;
    }

    rpmFiltered = alpha * rpm + (1.0f - alpha) * rpmFiltered;

    if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
      sys.rpm   = rpmFiltered;
      sys.speed = speed;
      xSemaphoreGive(sys_mutex);
    }

  #if debugrpm
    Serial.print("RPM: ");
    Serial.print(rpmFiltered);

    Serial.print(" | Speed: ");
    Serial.println(speed);
  #endif
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ================== TASK: LORA  ===================
void taskLoRa(void *pvParameters) {
  char payload[256];
  TelemetryPacket packet = {};
  uint32_t seq = 0;

  for (;;) {

    // snapshot consistente do sys and accelStats (same mutex)
    SystemData snap = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    AccelStats stats_snap = {};
    
    if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
      snap = sys;
      stats_snap = accelStats;
      xSemaphoreGive(sys_mutex);
    }

    // Se o modulo nao estiver disponivel, apenas sinaliza falha e nao tenta TX bloqueante.
    if (!h_lora.init_ok) {
      health_mark_fail(&h_lora);
      xEventGroupSetBits(error_events, ERROR_LORA_BIT);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    uint32_t t_ms = 0;
    if (h_sd.init_ok && sd_startTime != 0) {
      int64_t now_us = esp_timer_get_time();
      t_ms = (uint32_t)((now_us - sd_startTime) / 1000);
    } else {
      t_ms = (uint32_t)millis();
    }

    // Fill compact binary payload (12 bytes total) to reduce LoRa airtime.
    packet.seq = (uint16_t)seq;
    packet.t_ms = t_ms;
    packet.temperature10 = (int16_t)lroundf(snap.temperature * 10.0f);
    packet.rpm = (uint16_t)lroundf(snap.rpm);
    packet.speed10 = (uint16_t)lroundf(snap.speed * 10.0f);

    // Keep human-readable serial/debug output unchanged.
    int n = snprintf(payload, sizeof(payload),
      "%u;t=%lu;T=%.1fC;RPM=%.0f;SPD=%.1f",
      (unsigned long)seq,
      (unsigned long)t_ms,
      snap.temperature,
      snap.rpm,
      snap.speed
    );
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(payload)) n = sizeof(payload) - 1;

    uint32_t tx_start = millis();
#if debugacl
    TelemetryPacketDebug pktDbg = {};
    pktDbg.seq           = packet.seq;
    pktDbg.t_ms          = packet.t_ms;
    pktDbg.temperature10 = packet.temperature10;
    pktDbg.rpm           = packet.rpm;
    pktDbg.speed10       = packet.speed10;
    pktDbg.ax_median10   = (int16_t)lroundf(stats_snap.ax_median * 10.0f);
    pktDbg.ay_median10   = (int16_t)lroundf(stats_snap.ay_median * 10.0f);
    pktDbg.az_median10   = (int16_t)lroundf(stats_snap.az_median * 10.0f);
    Serial.print("ax_median:");
    Serial.print(stats_snap.ax_median);
    Serial.print(";ay_median:");
    Serial.print(stats_snap.ay_median);
    Serial.print(";az_median:");
    Serial.println(stats_snap.az_median);
    ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, LORA_CHANNEL, &pktDbg, sizeof(pktDbg));
#else
    ResponseStatus rs = e220ttl.sendFixedMessage(
      0,
      DESTINATION_ADDL,
      LORA_CHANNEL,
      &packet,
      sizeof(packet)
    );
#endif
    uint32_t tx_duration = millis() - tx_start;

    if (rs.code == 1) {
      health_mark_ok(&h_lora);
      // Clear LoRa error bit if it was set
      xEventGroupClearBits(error_events, ERROR_LORA_BIT);
    } else {
      h_lora.init_ok = false;
      health_mark_fail(&h_lora);
      xEventGroupSetBits(error_events, ERROR_LORA_BIT);
    }

    Serial.print("[LoRa TX] ");
    Serial.print(payload);
    Serial.print(" (");
    Serial.print(tx_duration);
    Serial.println("ms)");

    
    #if DEBUG
      Serial.println(payload);
      Serial.println(rs.getResponseDescription());
      Serial.println(updateBatteryLevel());
    #endif

    seq++;
    const uint32_t LORA_PERIOD_MS = 250;
    if (tx_duration < LORA_PERIOD_MS) {
      vTaskDelay(pdMS_TO_TICKS(LORA_PERIOD_MS - tx_duration));
    }
  }
}

// ================== TASK: SD Log ==================
void taskSDLogging(void *pvParameters) {
  const char* FILE_NAME = "/acelolo.txt";
  const TickType_t sample_period = pdMS_TO_TICKS(200);
  const uint32_t flush_period_ms = 1000;
  const uint8_t buffered_sample_limit = 5;
  char line[256];
  char sample_lines[5][256];

  File f;
  uint8_t lines_since_flush = 0;
  uint32_t last_reopen_ms = 0;
  uint32_t last_flush_ms = millis();
  uint8_t buffered_samples = 0;
  TickType_t lastWake = xTaskGetTickCount();

  // State tracking for failure event logging (one-shot per transition)
  // Index: 0=LoRa, 1=SD, 2=MPU, 3=Thermocouple
  bool last_ok[4] = {true, true, true, true};
  const char* module_names[4] = {"LoRa", "SD", "MPU", "Therm"};

  for (;;) {
    #if debugacl
      AccelStats stats_dbg = {};
      if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
        stats_dbg = accelStats;
        xSemaphoreGive(sys_mutex);
      }

      Serial.print("ax_median:");
      Serial.print(stats_dbg.ax_median);
      Serial.print(";ay_median:");
      Serial.print(stats_dbg.ay_median);
      Serial.print(";az_median:");
      Serial.println(stats_dbg.az_median);
    #endif
    
    if (!h_sd.init_ok) {
      health_mark_fail(&h_sd);
      xEventGroupSetBits(error_events, ERROR_SD_BIT);
      vTaskDelayUntil(&lastWake, sample_period);
      continue;
    }

    // reabrir se necessário (backoff)
    if (!f) {
      uint32_t now = millis();
      if (now - last_reopen_ms > 2000) {
        last_reopen_ms = now;
        f = SD.open(FILE_NAME, FILE_APPEND);
        if (f) {
          health_mark_ok(&h_sd);
          xEventGroupClearBits(error_events, ERROR_SD_BIT);
          last_flush_ms = millis();
        } else {
          health_mark_fail(&h_sd);
          xEventGroupSetBits(error_events, ERROR_SD_BIT);
        }
      }
      vTaskDelayUntil(&lastWake, sample_period);
      continue;
    }

    // --- Failure event logging: detect state transitions and log one-shot events ---
    int64_t now_us = esp_timer_get_time();
    uint32_t event_t_ms = 0;
    if (sd_startTime != 0) event_t_ms = (uint32_t)((now_us - sd_startTime) / 1000);
    else                   event_t_ms = millis();

    // health_alive handles its own locking; do NOT hold health_mutex here
    bool current_ok[4];
    current_ok[0] = health_alive(&h_lora, 2000);
    current_ok[1] = health_alive(&h_sd, 3000);
    current_ok[2] = health_alive(&h_mpu, 1500);
    current_ok[3] = health_alive(&h_therm, 1500);

    // Detect transitions and log events
    for (int i = 0; i < 4; i++) {
      if (last_ok[i] && !current_ok[i]) {
        // Transition: healthy → failed
        int ev_n = snprintf(line, sizeof(line),
          "t=%lu,EVENT=FAIL,MODULE=%s",
          (unsigned long)event_t_ms,
          module_names[i]
        );
        if (ev_n > 0 && (size_t)ev_n < sizeof(line)) {
          if (f.println(line) > 0) {
            lines_since_flush++;
            if (lines_since_flush >= 10) {
              f.flush();
              lines_since_flush = 0;
            }
          }
        }
        last_ok[i] = false;  // Update state immediately to prevent repeated logging
      }
      else if (!last_ok[i] && current_ok[i]) {
        // Transition: failed → healthy (recovery)
        int ev_n = snprintf(line, sizeof(line),
          "t=%lu,EVENT=RECOVER,MODULE=%s",
          (unsigned long)event_t_ms,
          module_names[i]
        );
        if (ev_n > 0 && (size_t)ev_n < sizeof(line)) {
          if (f.println(line) > 0) {
            lines_since_flush++;
            if (lines_since_flush >= 10) {
              f.flush();
              lines_since_flush = 0;
            }
          }
        }
        last_ok[i] = true;  // Update state immediately
      }
    }

    // snapshot consistente (sys and accelStats, same mutex)
    SystemData snap = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    AccelStats stats_snap = {};
    
    if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
      snap = sys;
      stats_snap = accelStats;
      xSemaphoreGive(sys_mutex);
    }

    int n = snprintf(line, sizeof(line),
      "t=%lu,T=%.1f,RPM=%.0f,SPD=%.1f,AX_MEDIAN=%.3f,AY_MEDIAN=%.3f,AZ_MEDIAN=%.3f",
      (unsigned long)event_t_ms,
      snap.temperature,
      snap.rpm,
      snap.speed,
      stats_snap.ax_median,
      stats_snap.ay_median,
      stats_snap.az_median
    );
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(line)) n = sizeof(line) - 1;

    memcpy(sample_lines[buffered_samples], line, sizeof(line));
    if (buffered_samples < buffered_sample_limit) buffered_samples++;

    uint32_t now_ms = millis();
    bool should_flush = (buffered_samples >= buffered_sample_limit) ||
                        ((now_ms - last_flush_ms) >= flush_period_ms);

    if (should_flush && buffered_samples > 0) {
      bool batch_ok = true;

      for (uint8_t i = 0; i < buffered_samples; i++) {
        if (f.println(sample_lines[i]) <= 0) {
          batch_ok = false;
          break;
        }
        lines_since_flush++;
      }

      if (batch_ok) {
        f.flush();
        lines_since_flush = 0;
        buffered_samples = 0;
        last_flush_ms = now_ms;
        health_mark_ok(&h_sd);
        xEventGroupClearBits(error_events, ERROR_SD_BIT);

        #if DEBUG
          Serial.printf("SD batch OK - %lu\n", (unsigned long)event_t_ms);
        #endif
      } else {
        health_mark_fail(&h_sd);
        xEventGroupSetBits(error_events, ERROR_SD_BIT);
        buffered_samples = 0;
        f.close();
        #if DEBUG
          Serial.println("SD batch write failed -> closing file");
        #endif
      }
    }

    vTaskDelayUntil(&lastWake, sample_period);
  }
}

// ================== TASK: Supervisor (error monitor, no reboot) ==================
void taskSupervisor(void *pvParameters) {
  const EventBits_t monitored_bits = ERROR_SENSOR_BIT | ERROR_LORA_BIT | ERROR_SD_BIT;
  EventBits_t last_bits = 0;

  for (;;) {
    // Block until any error bit is set (or timeout to periodically check error transitions)
    xEventGroupWaitBits(
      error_events,
      monitored_bits,
      pdFALSE,  // Don't clear bits automatically
      pdFALSE,  // Wait for any bit
      pdMS_TO_TICKS(500)  // Timeout to periodically check error transitions
    );  

    EventBits_t bits = xEventGroupGetBits(error_events) & monitored_bits;

    // Handle detected errors
    if ((bits & ERROR_SENSOR_BIT) != 0) {
      #if DEBUG
        Serial.println("ERROR_SENSOR: Invalid sensor data or timeout");
      #endif
      // Clear sensor error bit after logging
      xEventGroupClearBits(error_events, ERROR_SENSOR_BIT);
    }

    if ((bits & ERROR_LORA_BIT) != 0 && (last_bits & ERROR_LORA_BIT) == 0) {
      #if DEBUG
        Serial.println("ERROR_LORA: LoRa communication failed");
      #endif
    } else if ((bits & ERROR_LORA_BIT) == 0 && (last_bits & ERROR_LORA_BIT) != 0) {
      #if DEBUG
        Serial.println("ERROR_LORA cleared");
      #endif
    }

    if ((bits & ERROR_SD_BIT) != 0 && (last_bits & ERROR_SD_BIT) == 0) {
      #if DEBUG
        Serial.println("ERROR_SD: SD card operation failed");
      #endif
    } else if ((bits & ERROR_SD_BIT) == 0 && (last_bits & ERROR_SD_BIT) != 0) {
      #if DEBUG
        Serial.println("ERROR_SD cleared");
      #endif
    }

    last_bits = bits;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(rpmPin, INPUT_PULLUP);
  pinMode(spdPin, INPUT_PULLUP);
  
  pinMode(PIN_BATT_LEVEL, INPUT);

  attachInterrupt(digitalPinToInterrupt(rpmPin), countRPM_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(spdPin), countSPD_ISR, FALLING);

  // --- Create synchronization primitives
  sys_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();
  health_mutex = xSemaphoreCreateMutex();
  error_events = xEventGroupCreate();

  if (sys_mutex == NULL || i2c_mutex == NULL || health_mutex == NULL || error_events == NULL) {
    Serial.println("ERROR: Failed to create synchronization primitives");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // --- LoRa init
  e220ttl.begin();
  bool lora_ok = lora_init_ok();
  health_set_init(&h_lora, lora_ok);
  #if DEBUG
    if (!lora_ok) Serial.println("LoRa init failed");
    else Serial.println("LoRa init OK");
  #endif

  // --- SD init (ordem correta)
  spiSD.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS_PIN);
  bool sd_ok = SD.begin(SD_CS_PIN, spiSD);
  health_set_init(&h_sd, sd_ok);
  if (sd_ok) {
    sd_startTime = esp_timer_get_time();
    #if DEBUG
      Serial.println("SD init OK, timer started");
    #endif
  } else {
    #if DEBUG
      Serial.println("SD init failed");
    #endif
  }

  // --- MPU init
  bool mpu_ok = mpu.begin();
  if (mpu_ok) {
    // Increase accel range to avoid clipping at about 19.61 m/s^2 (2g).
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    // Bandwidth around 44Hz keeps impacts visible while reducing noise.
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
  health_set_init(&h_mpu, mpu_ok);
  #if DEBUG
    if (!mpu_ok) Serial.println("MPU6050 init failed");
  #endif

  // --- Thermocouple (sem begin)
  health_set_init(&h_therm, true);

  // Create RTOS tasks
  xTaskCreatePinnedToCore(taskSensors,   "Sensors",   4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(taskRPM,       "RPM",       4096, NULL, 3, NULL, 0);
  if (lora_ok) {
    xTaskCreatePinnedToCore(taskLoRa,    "LoRa",      4096, NULL, 2, NULL, 1);
  }
  xTaskCreatePinnedToCore(taskSDLogging, "SDLogging", 4096, NULL, 1, NULL, 1);

  // Supervisor aplica política de reboot (DEBUG==0 e LoRa==0 e SD==0)
  xTaskCreatePinnedToCore(taskSupervisor, "Supervisor", 4096, NULL, 2, NULL, 0);

  #if DEBUG
    Serial.println("Setup completed.");
  #endif
}

void loop() {
}