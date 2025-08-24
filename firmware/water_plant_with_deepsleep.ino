#include <ESP8266WiFi.h>
#include <user_interface.h> // for system_rtc_mem_read/write

// ====== CONFIG ======
const int AirValue = 600;     // Dry calibration value
const int WaterValue = 260;   // Wet calibration value
const int SENSOR_POWER = D1;  // GPIO5 controls sensor power
const int PUMP_PIN = D6;      // Relay/pump control pin

// Sleep parameters 
// ESP.deepSleep() on the ESP8266 takes microseconds. the ESP8266 hardware limit is ~71 minutes per sleep cycle (2^32 µs ≈ 71 min)
const uint32_t sleepCycleUS = 8000000; //71ULL * 60ULL * 1000000ULL; // ~71 min max
const int totalCycles = 7; // 7 × 71 min ≈ 8h 17m

// RTC memory struct (must be 4-byte aligned)
struct {
  uint32_t marker;
  uint32_t cycleCount;
} rtcData;

// ====== SETUP ======  
void setup() {
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  Serial.begin(115200);
  delay(100);

  // Read RTC memory
  system_rtc_mem_read(64, &rtcData, sizeof(rtcData));

  if (rtcData.marker != 0xDEADBEEF) {
    // First boot
    rtcData.marker = 0xDEADBEEF;
    rtcData.cycleCount = 0;
  }

  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.printf("Cycle count: %u / %u\n", rtcData.cycleCount, totalCycles);

  if (rtcData.cycleCount >= totalCycles) {
    // Time to run moisture check
    rtcData.cycleCount = 0; // reset for next long sleep

    int moisture = readMoisture();
    Serial.printf("Raw moisture: %d\n", moisture);

    int soilPercent = constrain(
      map(moisture, AirValue, WaterValue, 0, 100),
      0, 100
    );

    Serial.printf("Soil moisture: %d%%\n", soilPercent);

    if (soilPercent < 10) {
      Serial.println("Soil dry — watering...");
      triggerRelay();
    } else {
      Serial.println("Soil wet — no watering");
    }
  } else {
    Serial.println("Skipping moisture check this cycle");
    rtcData.cycleCount++;
  }

  // Save RTC memory
  system_rtc_mem_write(64, &rtcData, sizeof(rtcData));

  Serial.println("Entering deep sleep...");
  ESP.deepSleep(sleepCycleUS);
}

void loop() {
  // Never reached
}

// ====== FUNCTIONS ======
int readMoisture() {
  const int SAMPLES = 15;
  const int SETTLE_DELAY = 300; // ms
  const int SAMPLE_DELAY = 10;  // ms

  digitalWrite(SENSOR_POWER, HIGH);
  pinMode(SENSOR_POWER, OUTPUT);
  delay(SETTLE_DELAY);

  float average = analogRead(A0);

  for (int i = 1; i < SAMPLES; i++) {
    delay(SAMPLE_DELAY);
    float newValue = analogRead(A0);
    average = (0.4f * average) + (0.6f * newValue);
  }

  digitalWrite(SENSOR_POWER, LOW);
  return (int)average;
}

void triggerRelay() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);
  delay(2000); // 2 seconds
  digitalWrite(PUMP_PIN, LOW);
  Serial.println("Watering complete!");
}
