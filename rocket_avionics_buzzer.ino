#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LittleFS.h>
#include <ESP32Servo.h>

// ============================================================
//              ESP32 + BMP280 + LORA + SERVO
// ============================================================
//
// FEATURES
// 1. ESP32-WROOM-32
// 2. SX1278 / RA-02 LoRa
// 3. BMP280
// 4. Servo motor
// 5. Altitude-difference detection
// 6. LoRa telemetry at 2 packets/sec
// 7. Internal LittleFS flight logging
// 8. Serial flight-data dump ('D')
// 9. Serial log erase ('E')
//
// BUZZER REMOVED COMPLETELY
//
// ============================================================


// ============================================================
//                     PIN CONFIGURATION
// ============================================================

// ---------------- LoRa SX1278 ----------------
constexpr uint8_t LORA_CS   = 5;
constexpr uint8_t LORA_RST  = 14;
constexpr uint8_t LORA_DIO0 = 26;

constexpr uint8_t LORA_SCK  = 18;
constexpr uint8_t LORA_MISO = 19;
constexpr uint8_t LORA_MOSI = 23;


// ---------------- BMP280 ----------------
constexpr uint8_t BMP_SDA = 21;
constexpr uint8_t BMP_SCL = 22;


// ---------------- Servo ----------------
constexpr uint8_t SERVO_PIN = 25;


// ---------------- Status LED ----------------
constexpr uint8_t STATUS_LED = 2;


// ============================================================
//                    LORA PARAMETERS
// ============================================================

constexpr long LORA_FREQUENCY = 434500000L;
constexpr long LORA_BANDWIDTH = 125000L;

constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;
constexpr uint16_t LORA_PREAMBLE = 8;

constexpr int8_t LORA_TX_POWER = 10;


// ============================================================
//                    TIMING PARAMETERS
// ============================================================

constexpr unsigned long TELEMETRY_PERIOD = 500UL;
// 500 ms = 2 packets/sec


// ============================================================
//                    ALTITUDE PARAMETERS
// ============================================================

constexpr float ALTITUDE_THRESHOLD = 10.0F;

// Servo positions
constexpr int SERVO_INITIAL_POSITION = 0;
constexpr int SERVO_TEST_POSITION    = 90;


// ============================================================
//                    BMP280 / FLASH
// ============================================================

constexpr float SEA_LEVEL_PRESSURE = 1013.25F;

const char *FLIGHT_LOG = "/flight.csv";


// ============================================================
//                     OBJECTS
// ============================================================

Adafruit_BMP280 bmp;

Servo testServo;


// ============================================================
//                    SYSTEM STATUS
// ============================================================

bool bmpOK = false;
bool loraOK = false;
bool flashOK = false;
bool servoOK = false;


// ============================================================
//                    ALTITUDE STATE
// ============================================================

float referenceAltitude = NAN;
float currentAltitude = NAN;
float altitudeDifference = NAN;

bool altitudeThresholdReached = false;


// ============================================================
//                    SYSTEM TIMERS
// ============================================================

unsigned long bootTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long packetID = 0;


// ============================================================
//                    LORA STATUS
// ============================================================

String lastLoRaStatus = "NOT_STARTED";


// ============================================================
//                    BMP280
// ============================================================

bool initializeBMP280()
{
  Wire.begin(BMP_SDA, BMP_SCL);

  if (bmp.begin(0x76))
  {
    Serial.println("[OK] BMP280 detected at 0x76");
    return true;
  }

  if (bmp.begin(0x77))
  {
    Serial.println("[OK] BMP280 detected at 0x77");
    return true;
  }

  Serial.println("[WARNING] BMP280 NOT detected.");

  return false;
}


void configureBMP280()
{
  if (!bmpOK)
    return;

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,

    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,

    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_1
  );
}


float getTemperature()
{
  if (!bmpOK)
    return NAN;

  return bmp.readTemperature();
}


float getPressure()
{
  if (!bmpOK)
    return NAN;

  return bmp.readPressure() / 100.0F;
}


float getAltitude()
{
  if (!bmpOK)
    return NAN;

  return bmp.readAltitude(SEA_LEVEL_PRESSURE);
}


// ============================================================
//                    LORA INITIALIZATION
// ============================================================

bool initializeLoRa()
{
  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_CS
  );

  LoRa.setPins(
    LORA_CS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("[WARNING] LoRa initialization FAILED.");

    lastLoRaStatus = "INIT_FAILED";

    return false;
  }

  LoRa.setSignalBandwidth(LORA_BANDWIDTH);

  LoRa.setSpreadingFactor(
    LORA_SPREADING_FACTOR
  );

  LoRa.setCodingRate4(
    LORA_CODING_RATE
  );

  LoRa.setPreambleLength(
    LORA_PREAMBLE
  );

  LoRa.enableCrc();

  LoRa.setTxPower(
    LORA_TX_POWER
  );

  lastLoRaStatus = "READY";

  Serial.println("[OK] LoRa initialized.");

  Serial.println("Frequency : 434.500 MHz");
  Serial.println("Bandwidth : 125 kHz");
  Serial.println("Spreading : SF7");
  Serial.println("Coding    : 4/5");
  Serial.println("Preamble  : 8");
  Serial.println("CRC       : ENABLED");
  Serial.println("TX Power  : 10 dBm");

  return true;
}


// ============================================================
//                    SERVO INITIALIZATION
// ============================================================

bool initializeServo()
{
  testServo.setPeriodHertz(50);

  if (!testServo.attach(
        SERVO_PIN,
        500,
        2400
      ))
  {
    Serial.println("[WARNING] Servo attach FAILED.");

    return false;
  }

  testServo.write(
    SERVO_INITIAL_POSITION
  );

  Serial.println("[OK] Servo initialized.");

  Serial.print("Servo GPIO : ");
  Serial.println(SERVO_PIN);

  Serial.print("Initial position : ");
  Serial.print(SERVO_INITIAL_POSITION);
  Serial.println(" deg");

  return true;
}


// ============================================================
//                    FLASH INITIALIZATION
// ============================================================

bool initializeFlash()
{
  // false = do not automatically format filesystem
  if (!LittleFS.begin(false))
  {
    Serial.println("[WARNING] LittleFS mount FAILED.");

    return false;
  }

  Serial.println("[OK] ESP32 internal flash mounted.");

  if (!LittleFS.exists(FLIGHT_LOG))
  {
    File file = LittleFS.open(
      FLIGHT_LOG,
      FILE_WRITE
    );

    if (!file)
    {
      Serial.println(
        "[WARNING] Cannot create flight log."
      );

      return false;
    }

    file.println(
      "TIME_MS,"
      "PACKET_ID,"
      "TEMPERATURE_C,"
      "PRESSURE_HPA,"
      "ALTITUDE_M,"
      "REFERENCE_ALTITUDE_M,"
      "ALTITUDE_DIFFERENCE_M,"
      "ALTITUDE_THRESHOLD,"
      "SERVO_POSITION,"
      "ALTITUDE_TRIGGERED,"
      "BMP_OK,"
      "LORA_OK,"
      "LORA_STATUS,"
      "RECORD_TYPE"
    );

    file.flush();
    file.close();

    Serial.println("[OK] New flight log created.");
  }
  else
  {
    Serial.println("[OK] Existing flight log found.");
  }

  return true;
}


// ============================================================
//                    FLASH WRITE
// ============================================================

bool writeFlashRecord(
  const String &record
)
{
  if (!flashOK)
    return false;

  File file = LittleFS.open(
    FLIGHT_LOG,
    FILE_APPEND
  );

  if (!file)
  {
    Serial.println(
      "[FLASH ERROR] Cannot open log."
    );

    return false;
  }

  file.println(record);

  file.flush();
  file.close();

  return true;
}


// ============================================================
//                    SYSTEM EVENT LOG
// ============================================================

void logSystemEvent(
  const char *eventName
)
{
  if (!flashOK)
    return;

  String record;

  record += String(millis());
  record += ",-,-,-,-,-,-,";

  record += String(
    ALTITUDE_THRESHOLD,
    2
  );

  record += ",";

  record += String(
    servoOK
      ? testServo.read()
      : -1
  );

  record += ",";

  record += altitudeThresholdReached
              ? "1"
              : "0";

  record += ",";

  record += bmpOK
              ? "1"
              : "0";

  record += ",";

  record += loraOK
              ? "1"
              : "0";

  record += ",";

  record += lastLoRaStatus;

  record += ",";

  record += eventName;

  writeFlashRecord(record);
}


// ============================================================
//                 ESTABLISH REFERENCE ALTITUDE
// ============================================================

bool establishReferenceAltitude()
{
  if (!bmpOK)
  {
    Serial.println(
      "[ERROR] Cannot establish altitude reference."
    );

    return false;
  }

  constexpr uint8_t SAMPLE_COUNT = 10;

  float sum = 0.0F;

  for (
    uint8_t i = 0;
    i < SAMPLE_COUNT;
    i++
  )
  {
    float altitude = getAltitude();

    if (isnan(altitude))
    {
      Serial.println(
        "[ERROR] Invalid BMP280 altitude."
      );

      return false;
    }

    sum += altitude;

    delay(100);
  }

  referenceAltitude =
    sum / SAMPLE_COUNT;

  currentAltitude =
    referenceAltitude;

  altitudeDifference = 0.0F;

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "      ALTITUDE REFERENCE SET"
  );

  Serial.println(
    "========================================"
  );

  Serial.print(
    "Reference altitude : "
  );

  Serial.print(
    referenceAltitude,
    2
  );

  Serial.println(" m");

  Serial.print(
    "Trigger difference : "
  );

  Serial.print(
    ALTITUDE_THRESHOLD,
    2
  );

  Serial.println(" m");

  Serial.println();

  logSystemEvent(
    "REFERENCE_ALTITUDE_SET"
  );

  return true;
}


// ============================================================
//                 ALTITUDE MEASUREMENT
// ============================================================

void updateAltitude()
{
  if (!bmpOK)
    return;

  currentAltitude =
    getAltitude();

  if (isnan(currentAltitude))
    return;

  if (isnan(referenceAltitude))
    return;

  altitudeDifference =
    currentAltitude -
    referenceAltitude;
}


// ============================================================
//                 SERVO ALTITUDE LOGIC
// ============================================================

void processAltitudeCondition()
{
  if (!servoOK)
    return;

  if (isnan(altitudeDifference))
    return;

  float absoluteDifference =
    fabs(altitudeDifference);

  if (
    !altitudeThresholdReached &&
    absoluteDifference >= ALTITUDE_THRESHOLD
  )
  {
    altitudeThresholdReached = true;

    Serial.println();

    Serial.println(
      "========================================"
    );

    Serial.println(
      "       ALTITUDE DIFFERENCE REACHED"
    );

    Serial.println(
      "========================================"
    );

    Serial.print(
      "Reference altitude : "
    );

    Serial.print(
      referenceAltitude,
      2
    );

    Serial.println(" m");

    Serial.print(
      "Current altitude   : "
    );

    Serial.print(
      currentAltitude,
      2
    );

    Serial.println(" m");

    Serial.print(
      "Altitude difference: "
    );

    Serial.print(
      altitudeDifference,
      2
    );

    Serial.println(" m");

    Serial.print(
      "Servo position     : "
    );

    Serial.print(
      SERVO_TEST_POSITION
    );

    Serial.println(" deg");

    // Move servo
    testServo.write(
      SERVO_TEST_POSITION
    );

    // Status LED ON
    digitalWrite(
      STATUS_LED,
      HIGH
    );

    logSystemEvent(
      "ALTITUDE_THRESHOLD_REACHED"
    );
  }
}


// ============================================================
//                    TELEMETRY PACKET
// ============================================================

String createTelemetryPacket()
{
  float temperature =
    getTemperature();

  float pressure =
    getPressure();

  float altitude =
    getAltitude();

  unsigned long elapsed =
    millis() - bootTime;

  packetID++;

  String packet;

  packet.reserve(220);

  packet += "PKT=1";

  packet += ",ID=";
  packet += String(packetID);

  packet += ",T=";
  packet += String(
    temperature,
    2
  );

  packet += ",P=";
  packet += String(
    pressure,
    2
  );

  packet += ",ALT=";
  packet += String(
    altitude,
    2
  );

  packet += ",REF=";
  packet += String(
    referenceAltitude,
    2
  );

  packet += ",DIFF=";
  packet += String(
    altitudeDifference,
    2
  );

  packet += ",THR=";
  packet += String(
    ALTITUDE_THRESHOLD,
    2
  );

  packet += ",SERVO=";

  if (servoOK)
  {
    packet += String(
      testServo.read()
    );
  }
  else
  {
    packet += "-1";
  }

  packet += ",TRIG=";

  packet += altitudeThresholdReached
              ? "1"
              : "0";

  packet += ",TIME=";

  packet += String(
    elapsed
  );

  return packet;
}


// ============================================================
//                    FLASH TELEMETRY RECORD
// ============================================================

String createFlashTelemetryRecord()
{
  String record;

  record += String(millis());

  record += ",";

  record += String(
    packetID
  );

  record += ",";

  record += String(
    getTemperature(),
    2
  );

  record += ",";

  record += String(
    getPressure(),
    2
  );

  record += ",";

  record += String(
    currentAltitude,
    2
  );

  record += ",";

  record += String(
    referenceAltitude,
    2
  );

  record += ",";

  record += String(
    altitudeDifference,
    2
  );

  record += ",";

  record += String(
    ALTITUDE_THRESHOLD,
    2
  );

  record += ",";

  if (servoOK)
  {
    record += String(
      testServo.read()
    );
  }
  else
  {
    record += "-1";
  }

  record += ",";

  record += altitudeThresholdReached
              ? "1"
              : "0";

  record += ",";

  record += bmpOK
              ? "1"
              : "0";

  record += ",";

  record += loraOK
              ? "1"
              : "0";

  record += ",";

  record += lastLoRaStatus;

  record += ",TELEMETRY";

  return record;
}


// ============================================================
//                    SEND TELEMETRY
// ============================================================

void sendTelemetry()
{
  String packet =
    createTelemetryPacket();

  Serial.println();
  Serial.println(
    "[TELEMETRY]"
  );

  Serial.println(packet);

  // ----------------------------------------------------------
  // LORA
  // ----------------------------------------------------------

  if (loraOK)
  {
    LoRa.beginPacket();

    LoRa.print(packet);

    int result =
      LoRa.endPacket();

    if (result == 1)
    {
      lastLoRaStatus =
        "TX_OK";

      Serial.println(
        "LoRa: TRANSMITTED"
      );
    }
    else
    {
      lastLoRaStatus =
        "TX_ERROR";

      Serial.println(
        "LoRa: TRANSMISSION ERROR"
      );
    }
  }
  else
  {
    lastLoRaStatus =
      "NOT_AVAILABLE";

    Serial.println(
      "LoRa: NOT AVAILABLE"
    );
  }

  // ----------------------------------------------------------
  // FLASH
  // ----------------------------------------------------------

  if (flashOK)
  {
    if (
      !writeFlashRecord(
        createFlashTelemetryRecord()
      )
    )
    {
      Serial.println(
        "[WARNING] Telemetry flash write failed."
      );
    }
  }
}


// ============================================================
//                    DUMP FLIGHT LOG
// ============================================================

void dumpFlightLog()
{
  if (!flashOK)
  {
    Serial.println(
      "[ERROR] Flash unavailable."
    );

    return;
  }

  File file =
    LittleFS.open(
      FLIGHT_LOG,
      FILE_READ
    );

  if (!file)
  {
    Serial.println(
      "[ERROR] Cannot open flight log."
    );

    return;
  }

  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "          STORED FLIGHT DATA"
  );

  Serial.println(
    "========================================"
  );

  while (file.available())
  {
    Serial.write(
      file.read()
    );
  }

  file.close();

  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "          END OF FLIGHT DATA"
  );

  Serial.println(
    "========================================"
  );
}


// ============================================================
//                    ERASE FLIGHT LOG
// ============================================================

void eraseFlightLog()
{
  if (!flashOK)
  {
    Serial.println(
      "[ERROR] Flash unavailable."
    );

    return;
  }

  if (
    LittleFS.remove(
      FLIGHT_LOG
    )
  )
  {
    Serial.println(
      "[OK] Flight log erased."
    );

    // Re-create empty log with header
    File file =
      LittleFS.open(
        FLIGHT_LOG,
        FILE_WRITE
      );

    if (file)
    {
      file.println(
        "TIME_MS,"
        "PACKET_ID,"
        "TEMPERATURE_C,"
        "PRESSURE_HPA,"
        "ALTITUDE_M,"
        "REFERENCE_ALTITUDE_M,"
        "ALTITUDE_DIFFERENCE_M,"
        "ALTITUDE_THRESHOLD,"
        "SERVO_POSITION,"
        "ALTITUDE_TRIGGERED,"
        "BMP_OK,"
        "LORA_OK,"
        "LORA_STATUS,"
        "RECORD_TYPE"
      );

      file.flush();
      file.close();

      Serial.println(
        "[OK] New flight log created."
      );
    }
    else
    {
      Serial.println(
        "[ERROR] Could not recreate flight log."
      );
    }
  }
  else
  {
    Serial.println(
      "[ERROR] Failed to erase flight log."
    );
  }
}


// ============================================================
//                    SERIAL COMMANDS
// ============================================================

void processSerialCommands()
{
  if (!Serial.available())
    return;

  char command =
    Serial.read();

  // Dump flight data
  if (
    command == 'D' ||
    command == 'd'
  )
  {
    dumpFlightLog();
  }

  // Erase flight data
  else if (
    command == 'E' ||
    command == 'e'
  )
  {
    eraseFlightLog();
  }

  // Status command
  else if (
    command == 'S' ||
    command == 's'
  )
  {
    Serial.println();
    Serial.println(
      "========== SYSTEM STATUS =========="
    );

    Serial.print("BMP280 : ");
    Serial.println(
      bmpOK ? "OK" : "FAIL"
    );

    Serial.print("LoRa   : ");
    Serial.println(
      loraOK ? "OK" : "FAIL"
    );

    Serial.print("Flash  : ");
    Serial.println(
      flashOK ? "OK" : "FAIL"
    );

    Serial.print("Servo  : ");
    Serial.println(
      servoOK ? "OK" : "FAIL"
    );

    Serial.print("Reference altitude : ");
    Serial.print(
      referenceAltitude,
      2
    );

    Serial.println(" m");

    Serial.print("Current altitude   : ");
    Serial.print(
      currentAltitude,
      2
    );

    Serial.println(" m");

    Serial.print("Altitude difference: ");
    Serial.print(
      altitudeDifference,
      2
    );

    Serial.println(" m");

    Serial.print("Servo position     : ");
  }
}