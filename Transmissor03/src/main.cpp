#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <MAX6675.h>
#include <LoRa_E220.h>
#include <SD.h>
#include <esp_timer.h> 
#include <math.h>

// ================== CONFIGURAÇÕES DE DEBUG ==================
#define DEBUG 0

// ================== PINAGEM I2C ==================
#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22

// ================== CONSTANTES DO VEÍCULO ==================
#define RAIO_PNEU   0.28f
#define VEL_POR_PULSO   (2.0f * 3.14159265f * RAIO_PNEU * 3.6f)

// ================== DADOS DO SISTEMA ==================
typedef struct {
  float temperatura;   // Temperatura (°C)
  float rpm;           // Rotações por minuto
  float speed;         // Velocidade
} SystemData;

typedef struct __attribute__((packed)) {
  uint16_t seq;
  uint32_t t_ms;
  int16_t temperatura10;
  uint16_t rpm;
  uint16_t speed10;
  int16_t ax_max10;
  int16_t ay_max10;
  int16_t az_max10;
} PacketTelemetria;

static_assert(sizeof(PacketTelemetria) == 18, "PacketTelemetria deve ter 18 bytes");

static SystemData sys = {0.0f, 0.0f, 0.0f};
static SemaphoreHandle_t sys_mutex = NULL;  // Mutex para acesso ao struct sys

// ================== NÍVEL DE BATERIA ==================
#define PIN_BATT_LEVEL 35  

float nivelBatt() {
  uint32_t v_div_mv = analogReadMilliVolts(PIN_BATT_LEVEL); // mV calibrado no pino
  return (float)v_div_mv * (126.7f / 26.7f) / 1000.0f;     // Reconstrói tensão da bateria em V
}

// ================== ESTATÍSTICAS DO ACELERÔMETRO ==================
typedef struct {
  float ax_max;
  float ay_max;
  float az_max;
} AccelStats;

static AccelStats accelStats = {};
// accelStats compartilha sys_mutex — escrito por lerSensores, lido por gravarSD

#define ACCEL_WINDOW_SIZE 5

typedef struct {
  float ax_max, ay_max, az_max;
  uint16_t count;
} AccelMaxTracker;

static inline void accel_max_reset(AccelMaxTracker *t) {
  t->ax_max = -__FLT_MAX__;
  t->ay_max = -__FLT_MAX__;
  t->az_max = -__FLT_MAX__;
  t->count = 0;
}

static inline void accel_max_add(AccelMaxTracker *t, float ax, float ay, float az) {
  if (ax > t->ax_max) t->ax_max = ax;
  if (ay > t->ay_max) t->ay_max = ay;
  if (az > t->az_max) t->az_max = az;
  t->count++;
}

static inline void accel_max_emit(AccelMaxTracker *t, AccelStats *out) {
  out->ax_max = t->ax_max;
  out->ay_max = t->ay_max;
  out->az_max = t->az_max;
  accel_max_reset(t);
}

static SemaphoreHandle_t i2c_mutex = NULL;  // Mutex para barramento I2C

// ================== GRUPO DE EVENTOS DE ERRO ==================
#define ERROR_SENSOR_BIT  (1 << 0)
#define ERROR_LORA_BIT    (1 << 1)
#define ERROR_SD_BIT      (1 << 2)

static EventGroupHandle_t error_events = NULL;

// ================== SAÚDE / FLAGS DOS MÓDULOS ==================
typedef struct {
  volatile bool init_ok;          // inicializou no boot
  volatile bool ok;               // última operação ok
  volatile uint32_t last_ok_ms;   // millis() da última vez que funcionou
  volatile uint8_t fail_count;    // falhas consecutivas (satura em 255)
} Health_t;

static Health_t h_lora  = {false, false, 0, 0};
static Health_t h_sd    = {false, false, 0, 0};
static Health_t h_mpu   = {false, false, 0, 0};
static Health_t h_therm = {false, false, 0, 0};

static SemaphoreHandle_t health_mutex = NULL;  // Mutex para acesso aos structs Health_t

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

// ================== SENSORES ==================
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

// ISR do sensor Hall de RPM
void IRAM_ATTR contRPM_ISR() {
  portENTER_CRITICAL_ISR(&rpm_isr_mux);
  uint32_t nowUs = micros();

  if (lastRpmEdgeUs != 0) rpmPeriodUs = nowUs - lastRpmEdgeUs;
  lastRpmEdgeUs = nowUs;
  cntRPM++;
  portEXIT_CRITICAL_ISR(&rpm_isr_mux);
}

// ISR do sensor Hall de velocidade
void IRAM_ATTR contSPD_ISR() {
  portENTER_CRITICAL_ISR(&rpm_isr_mux);
  uint32_t nowUs = micros();
  if (lastSpdEdgeUs != 0) spdPeriodUs = nowUs - lastSpdEdgeUs;
  lastSpdEdgeUs = nowUs;
  cntSPD++;
  portEXIT_CRITICAL_ISR(&rpm_isr_mux);
}

// ================== LoRa ==================
#define DESTINATION_ADDL 3
#define LORA_CHANNEL     23
LoRa_E220 e220ttl(&Serial2, 36, 32, 33, UART_BPS_RATE_9600);

static bool lora_init_ok() {
  ResponseStructContainer info = e220ttl.getModuleInformation();
  bool ok = (info.status.code == E220_SUCCESS);
  info.close();
  return ok;
}

// ================== CARTÃO SD ==================
SPIClass spiSD(VSPI);
const int SD_CS_PIN = 5;
const int SPI_SCK   = 18;
const int SPI_MISO  = 19;
const int SPI_MOSI  = 23;

// Referência de tempo para timestamps do log (ms)
static int64_t sd_startTime = 0;

// ================== TAREFA: LEITURA DE SENSORES ==================
void lerSensores(void *pvParameters) {
  uint32_t lastThermo = 0;

  const uint32_t MPU_SAMPLE_MS = 50;
  uint32_t lastMPU = 0;
  AccelMaxTracker accelTracker = {};
  accel_max_reset(&accelTracker);

  for (;;) {
    uint32_t now = millis();

    // Amostragem do termopar (a cada 300ms)
    if (now - lastThermo >= 300) {
      float t = thermocouple.readCelsius();
      lastThermo = now;

      if (!isfinite(t) || t < -100.0f || t > 1000.0f) {
        health_mark_fail(&h_therm);
        xEventGroupSetBits(error_events, ERROR_SENSOR_BIT);
      } else {
        if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
          sys.temperatura = t;
          xSemaphoreGive(sys_mutex);
          health_mark_ok(&h_therm);
        } else {
          health_mark_fail(&h_therm);
        }
      }
    }

    // Amostragem do MPU6050 (a cada 50ms)
    if (h_mpu.init_ok && (now - lastMPU >= MPU_SAMPLE_MS)) {
      lastMPU = now;
      sensors_event_t accel, gyro, temp;
      bool mpu_ok = false;

      // Adquire mutex I2C para operação do MPU6050
      if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        mpu.getEvent(&accel, &gyro, &temp);
        xSemaphoreGive(i2c_mutex);

        // Valida dados de aceleração
        mpu_ok = isfinite(accel.acceleration.x) &&
                 isfinite(accel.acceleration.y) &&
                 isfinite(accel.acceleration.z);
      }

      // Tratamento unificado de erro
      if (!mpu_ok) {
        health_mark_fail(&h_mpu);
        xEventGroupSetBits(error_events, ERROR_SENSOR_BIT);
      } else {
        // Dados válidos: atualiza rastreador de máximo
        if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
          accel_max_add(&accelTracker,
            accel.acceleration.x,
            accel.acceleration.y,
            accel.acceleration.z);

          // Publica valor máximo a cada ACCEL_WINDOW_SIZE amostras
          if (accelTracker.count >= ACCEL_WINDOW_SIZE) {
            accel_max_emit(&accelTracker, &accelStats);
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

// ================== TAREFA: RPM / VELOCIDADE ==================
void contarHall(void *pvParameters) {
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

    // Snapshot atômico dos contadores e reset
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

    // Preferência por estimativa baseada no período entre pulsos
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
        speed = ppsSpd * VEL_POR_PULSO;
    }
    else if (pulsesSPD > 0)
    {
        speed = pulsesSPD * VEL_POR_PULSO;
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

    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ================== TAREFA: TRANSMISSÃO LoRa ==================
void transmitirLoRa(void *pvParameters) {
  char payload[256];
  PacketTelemetria packet = {};
  uint32_t seq = 0;

  for (;;) {

    // Snapshot consistente do sys e accelStats
    SystemData snap = {};
    AccelStats stats_snap = {};
    
    if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
      snap = sys;
      stats_snap = accelStats;
      xSemaphoreGive(sys_mutex);
    }

    // Se o módulo não estiver disponível, sinaliza falha e não tenta TX bloqueante
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

    // Monta payload binário compacto para reduzir tempo de ar do LoRa
    packet.seq = (uint16_t)seq;
    packet.t_ms = t_ms;
    packet.temperatura10 = (int16_t)lroundf(snap.temperatura * 10.0f);
    packet.rpm = (uint16_t)lroundf(snap.rpm);
    packet.speed10 = (uint16_t)lroundf(snap.speed * 10.0f);
    packet.ax_max10 = (int16_t)lroundf(stats_snap.ax_max * 10.0f);
    packet.ay_max10 = (int16_t)lroundf(stats_snap.ay_max * 10.0f);
    packet.az_max10 = (int16_t)lroundf(stats_snap.az_max * 10.0f);

    // Saída serial legível para debug
    snprintf(payload, sizeof(payload),
      "%u;t=%lu;T=%.1fC;RPM=%.0f;SPD=%.1f",
      (unsigned long)seq,
      (unsigned long)t_ms,
      snap.temperatura,
      snap.rpm,
      snap.speed
    );

    uint32_t tx_start = millis();

    ResponseStatus rs = e220ttl.sendFixedMessage(
      0,
      DESTINATION_ADDL,
      LORA_CHANNEL,
      &packet,
      sizeof(packet)
    );

    uint32_t tx_duration = millis() - tx_start;

    if (rs.code == 1) {
      health_mark_ok(&h_lora);
      // Limpa bit de erro LoRa se estava setado
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
      Serial.println(nivelBatt());
    #endif

    seq++;
    const uint32_t LORA_PERIOD_MS = 250;
    if (tx_duration < LORA_PERIOD_MS) {
      vTaskDelay(pdMS_TO_TICKS(LORA_PERIOD_MS - tx_duration));
    }
  }
}

// ================== TAREFA: GRAVAÇÃO NO SD ==================
void gravarSD(void *pvParameters) {
  const char* FILE_NAME = "/bajaUFSCAR.txt";
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

  // Rastreamento de estado para log de eventos de falha (disparo único por transição)
  // 0=LoRa, 1=SD, 2=MPU, 3=Termopar
  bool last_ok[4] = {true, true, true, true};
  const char* module_names[4] = {"LoRa", "SD", "MPU", "Therm"};

  for (;;) {
    
    if (!h_sd.init_ok) {
      health_mark_fail(&h_sd);
      xEventGroupSetBits(error_events, ERROR_SD_BIT);
      vTaskDelayUntil(&lastWake, sample_period);
      continue;
    }

    // Reabrir arquivo se necessário (com backoff de 2s)
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

    // --- Log de eventos de falha: detecta transições de estado ---
    int64_t now_us = esp_timer_get_time();
    uint32_t event_t_ms = 0;
    if (sd_startTime != 0) event_t_ms = (uint32_t)((now_us - sd_startTime) / 1000);
    else                   event_t_ms = millis();

    // health_alive faz seu próprio lock; NÃO segurar health_mutex aqui
    bool current_ok[4];
    current_ok[0] = health_alive(&h_lora, 2000);
    current_ok[1] = health_alive(&h_sd, 3000);
    current_ok[2] = health_alive(&h_mpu, 1500);
    current_ok[3] = health_alive(&h_therm, 1500);

    // Detecta transições e grava eventos
    for (int i = 0; i < 4; i++) {
      if (last_ok[i] && !current_ok[i]) {
        // Transição: saudável → falha
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
        last_ok[i] = false;  // Atualiza estado para evitar log repetido
      }
      else if (!last_ok[i] && current_ok[i]) {
        // Transição: falha → recuperado
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
        last_ok[i] = true;  // Atualiza estado
      }
    }

    // Snapshot consistente (sys e accelStats, mesmo mutex)
    SystemData snap = {};
    AccelStats stats_snap = {};
    
    if (xSemaphoreTake(sys_mutex, portMAX_DELAY) == pdTRUE) {
      snap = sys;
      stats_snap = accelStats;
      xSemaphoreGive(sys_mutex);
    }

    snprintf(line, sizeof(line),
      "t=%lu,T=%.1f,RPM=%.0f,SPD=%.1f,AX_MAX=%.3f,AY_MAX=%.3f,AZ_MAX=%.3f",
      (unsigned long)event_t_ms,
      snap.temperatura,
      snap.rpm,
      snap.speed,
      stats_snap.ax_max,
      stats_snap.ay_max,
      stats_snap.az_max
    );

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

      } else {
        health_mark_fail(&h_sd);
        xEventGroupSetBits(error_events, ERROR_SD_BIT);
        buffered_samples = 0;
        f.close();
      }
    }

    vTaskDelayUntil(&lastWake, sample_period);
  }
}

// ================== TAREFA: SUPERVISOR DE ERROS ==================
void verificarErros(void *pvParameters) {
  const EventBits_t monitored_bits = ERROR_SENSOR_BIT | ERROR_LORA_BIT | ERROR_SD_BIT;
  EventBits_t last_bits = 0;

  for (;;) {
    xEventGroupWaitBits(
      error_events,
      monitored_bits,
      pdFALSE,
      pdFALSE,
      pdMS_TO_TICKS(500)
    );  

    EventBits_t bits = xEventGroupGetBits(error_events) & monitored_bits;

    // Trata erros detectados
    if ((bits & ERROR_SENSOR_BIT) != 0) {
      #if DEBUG
        Serial.println("ERROR_SENSOR");
      #endif
      xEventGroupClearBits(error_events, ERROR_SENSOR_BIT);
    }

    if ((bits & ERROR_LORA_BIT) != 0 && (last_bits & ERROR_LORA_BIT) == 0) {
      #if DEBUG
        Serial.println("ERROR_LORA");
      #endif
    } else if ((bits & ERROR_LORA_BIT) == 0 && (last_bits & ERROR_LORA_BIT) != 0) {
      #if DEBUG
        Serial.println("ERROR_LORA cleared");
      #endif
    }

    if ((bits & ERROR_SD_BIT) != 0 && (last_bits & ERROR_SD_BIT) == 0) {
      #if DEBUG
        Serial.println("ERROR_SD");
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

  attachInterrupt(digitalPinToInterrupt(rpmPin), contRPM_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(spdPin), contSPD_ISR, FALLING);

  sys_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();
  health_mutex = xSemaphoreCreateMutex();
  error_events = xEventGroupCreate();

  if (sys_mutex == NULL || i2c_mutex == NULL || health_mutex == NULL || error_events == NULL) {
    Serial.println("ERRO: Falha em criar primitivas de sincronizacao");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // --- Inicialização LoRa ---
  e220ttl.begin();
  bool lora_ok = lora_init_ok();
  health_set_init(&h_lora, lora_ok);
  #if DEBUG
    if (!lora_ok) Serial.println("LoRa init falhou");
    else Serial.println("LoRa init OK");
  #endif

  // --- Inicialização SD ---
  spiSD.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS_PIN);
  bool sd_ok = SD.begin(SD_CS_PIN, spiSD);
  health_set_init(&h_sd, sd_ok);
  if (sd_ok) {
    sd_startTime = esp_timer_get_time();
    #if DEBUG
      Serial.println("SD init OK, timestamp comecou");
    #endif
  } else {
    #if DEBUG
      Serial.println("SD init falhou");
    #endif
  }

  // --- Inicialização MPU6050 ---
  bool mpu_ok = mpu.begin();
  if (mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
  health_set_init(&h_mpu, mpu_ok);
  #if DEBUG
    if (!mpu_ok) Serial.println("MPU6050 init falhou");
  #endif

  // --- Termopar (assumido OK no boot) ---
  health_set_init(&h_therm, true);

  // --- Criação das tarefas RTOS ---
  xTaskCreatePinnedToCore(lerSensores,        "Sensores",     4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(contarHall,         "RPM",          4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(transmitirLoRa,     "LoRa",         4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(gravarSD,           "GravacaoSD",   4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(verificarErros,     "Supervisor",   4096, NULL, 2, NULL, 0);

  #if DEBUG
    Serial.println("Setup concluido com sucesso");
  #endif
}

void loop() {
}