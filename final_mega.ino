#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BH1750.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT31.h>

// ===== 주소/하드웨어 설정 =====
#define TCAADDR     0x70
#define LCD_ADDR    0x27
#define SHT31_ADDR  0x45
#define INA219_ADDR 0x40

// ===== 서보 핀 =====
Servo servo_updown;
Servo servo_rightleft;
const int PIN_UD = 11;
const int PIN_RL = 3;

// ===== BH1750 (TCA9548A 경유) =====
BH1750 lightMeter;
const uint8_t tca_channels[4] = {1, 2, 3, 4}; // 좌상, 우상, 좌하, 우하
#define NUM_SENSORS 4
int threshold_value = 500;
int max_diff = 15000;

const int UPDOWN_MIN = 20;
const int UPDOWN_MAX = 70;
const int RIGHTLEFT_MIN = 30;
const int RIGHTLEFT_MAX = 180;

// ===== 표시/센서 =====
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Adafruit_INA219 ina219;
Adafruit_SHT31 sht = Adafruit_SHT31();

// ===== 평균/표시 =====
float sum_temp = 0, sum_hum = 0, sum_volt = 0, sum_current = 0;
int count = 0;
unsigned long lastAvgTime = 0;   // 1초 주기

// ===== SD / UART / 통신 =====
const int SD_CS = 4;                 // SD CS (모듈에 따라 4 또는 10)
const unsigned long BAUD = 9600;     // USB시리얼/Serial1 통일
String lastDateTime = "";            // "YYYY-MM-DD HH:MM:SS" (ESP32가 TIME, ... 로 전달)

// ---------- 유틸 ----------
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

int readLux(uint8_t index) {
  tcaselect(tca_channels[index]);
  delay(50);
  float lux = lightMeter.readLightLevel();
  Serial.print("채널 "); Serial.print(tca_channels[index]);
  Serial.print(" 조도: "); Serial.print(lux);
  Serial.println(" lux");
  return int(lux);
}

// ---------- SD ----------
void writeHeaderIfNeeded() {
  if (!SD.exists("data.csv")) {
    File f = SD.open("data.csv", FILE_WRITE);
    if (f) {
      f.println("datetime,millis,volt_V,curr_mA,power_mW,temp_C,hum_%");
      f.close();
    }
  }
}

void appendDataToSD(const String& dt, float volt, float curr_mA, float power_mW, float tempC, float hum) {
  File f = SD.open("data.csv", FILE_WRITE);
  if (f) {
    f.print(dt);           f.print(',');
    f.print(millis());     f.print(',');
    f.print(volt, 3);      f.print(',');
    f.print(curr_mA, 1);   f.print(',');
    f.print(power_mW, 0);  f.print(',');
    f.print(tempC, 2);     f.print(',');
    f.println(hum, 2);
    f.flush();
    f.close();
  } else {
    Serial.println("⚠️ SD open 실패");
  }
}

// ---------- 센서 읽기(평균 누적) ----------
void readSensors() {
  float temp = sht.readTemperature();
  float hum  = sht.readHumidity();
  float volt = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();  // mA

  if (!isnan(temp) && !isnan(hum) && volt > 0.1) {
    sum_temp += temp;
    sum_hum  += hum;
    sum_volt += volt;
    sum_current += current;
    count++;
  } else {
    Serial.println("센서 값 오류: T=" + String(temp) + ", H=" + String(hum));
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(BAUD);
  Wire.begin();

  // 서보
  servo_rightleft.attach(PIN_RL);
  servo_updown.attach(PIN_UD);
  servo_rightleft.write(105);
  servo_updown.write(10);
  delay(500);

  // BH1750 초기화 (각 채널에서 시작)
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaselect(tca_channels[i]);
    delay(10);
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      Serial.print("센서 "); Serial.print(i); Serial.println(" 초기화 성공");
    } else {
      Serial.print("센서 "); Serial.print(i); Serial.println(" 초기화 실패");
    }
  }

  // INA219
  if (!ina219.begin()) {
    Serial.println("INA219 초기화 실패");
  }
  ina219.setCalibration_32V_1A();

  // SHT31
  if (!sht.begin(SHT31_ADDR)) {
    Serial.println("SHT31 초기화 실패");
  }

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Loading...");

  // SD (MEGA는 53 OUTPUT 고정)
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD init FAIL");
  } else {
    Serial.println("✅ SD init OK");
    writeHeaderIfNeeded();
  }

  // ESP32와 UART (ESP32 TX0 -> MEGA RX1 D19, MEGA TX1 D18 -> ESP32 RX0, GND 공통)
  Serial1.begin(BAUD);
  Serial.println("UART ready on Serial1");
}

// ===================== LOOP =====================
void loop() {
  // --- (A) ESP32 시간 수신: "TIME,YYYY-MM-DD,HH:MM:SS" ---
  while (Serial1.available()) {
    static String line;
    char c = (char)Serial1.read();
    if (c == '\n') {
      line.trim();
      if (line.startsWith("TIME,")) {
        // 예: TIME,2025-08-14,13:25:07
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > c1) {
          String d = line.substring(c1 + 1, c2);
          String t = line.substring(c2 + 1);
          lastDateTime = d + " " + t;
          Serial.print("[TIME] "); Serial.println(lastDateTime);
        }
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }

  // --- (B) 조도 읽기 & 서보 제어(실시간) ---
  int topl = readLux(0);
  int topr = readLux(1);
  int botl = readLux(2);
  int botr = readLux(3);

  int topAvg   = (topl + topr) / 2;
  int botAvg   = (botl + botr) / 2;
  int leftAvg  = (topl + botl) / 2;
  int rightAvg = (topr + botr) / 2;

  int currentUD = servo_updown.read();
  int currentRL = servo_rightleft.read();

  int diffUD = topAvg - botAvg;
  if (abs(diffUD) > threshold_value) {
    int move = map(abs(diffUD), threshold_value, max_diff, 1, 10);
    move = constrain(move, 1, 10);
    if (diffUD > 0 && currentUD < UPDOWN_MAX)
      servo_updown.write(min(currentUD + move, UPDOWN_MAX));
    else if (diffUD < 0 && currentUD > UPDOWN_MIN)
      servo_updown.write(max(currentUD - move, UPDOWN_MIN));
  }

  int diffRL = leftAvg - rightAvg;
  if (abs(diffRL) > threshold_value) {
    int move = map(abs(diffRL), threshold_value, max_diff, 1, 10);
    move = constrain(move, 1, 10);
    if (diffRL > 0 && currentRL > RIGHTLEFT_MIN)
      servo_rightleft.write(max(currentRL - move, RIGHTLEFT_MIN));
    else if (diffRL < 0 && currentRL < RIGHTLEFT_MAX)
      servo_rightleft.write(min(currentRL + move, RIGHTLEFT_MAX));
  }

  // --- (C) 환경센서 평균 누적 ---
  readSensors();

  // --- (D) 1초마다: 평균 산출 + LCD 표시 + SD 저장 + ESP32 업로드 ---
  if (millis() - lastAvgTime >= 1000) {
    float avg_temp    = (count > 0) ? (sum_temp / count)    : NAN;
    float avg_hum     = (count > 0) ? (sum_hum / count)     : NAN;
    float avg_volt    = (count > 0) ? (sum_volt / count)    : NAN;
    float avg_current = (count > 0) ? (sum_current / count) : NAN; // mA (보정 전)

    // 전력 계산 (mW)
    float power_mW = (isnan(avg_volt) || isnan(avg_current)) ? NAN
                      : (avg_volt * (avg_current / 1000.0) * 1000.0);

    // LCD 표시(전류는 네가 쓰던 보정만 LCD에 반영)
    float corrected_current = isnan(avg_current) ? NAN : (avg_current * 10.0);

    Serial.print("전류 평균(보정 후): ");
    if (!isnan(corrected_current)) { Serial.print(corrected_current); Serial.println(" mA"); }
    else { Serial.println("NaN"); }

    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    if (!isnan(avg_temp)) { lcd.print(avg_temp, 1); lcd.print(" C     "); }
    else { lcd.print("--.- C     "); }

    lcd.setCursor(0, 2);
    lcd.print("Humi: ");
    if (!isnan(avg_hum)) { lcd.print(avg_hum, 1); lcd.print(" %     "); }
    else { lcd.print("--.- %     "); }

    lcd.setCursor(0, 3);
    lcd.print("Curr: ");
    if (!isnan(corrected_current)) {
      if (corrected_current >= 1000.0) {
        lcd.print(corrected_current / 1000.0, 2); lcd.print(" A   ");
      } else {
        lcd.print(corrected_current, 1); lcd.print(" mA  ");
      }
    } else {
      lcd.print("--.- mA  ");
    }

    // SD 저장 (원데이터 사용: 보정 없이 avg_current, power_mW 기록)
    if (!isnan(avg_volt) && !isnan(avg_current) && !isnan(power_mW) &&
        !isnan(avg_temp) && !isnan(avg_hum)) {
      appendDataToSD(lastDateTime, avg_volt, avg_current, power_mW, avg_temp, avg_hum);
    }

    // ESP32 업로드용 송신: MEAS,<volt>,<curr_mA>,<power_mW>,<tempC>,<hum>
    {
      String out = "MEAS,";
      out += String(isnan(avg_volt) ? 0.0 : avg_volt, 3); out += ",";
      out += String(isnan(avg_current) ? 0.0 : avg_current, 1); out += ",";
      out += String(isnan(power_mW) ? 0.0 : power_mW, 0); out += ",";
      out += String(isnan(avg_temp) ? 0.0 : avg_temp, 2); out += ",";
      out += String(isnan(avg_hum) ? 0.0 : avg_hum, 2);
      out.replace(" ", ""); // 혹시 모를 공백 제거
      Serial1.println(out);

      Serial.print("[MEGA->ESP32] ");
      Serial.println(out);
    }

    // 누산 리셋
    sum_temp = sum_hum = sum_volt = sum_current = 0;
    count = 0;
    lastAvgTime = millis();
  }

  delay(200);
}
