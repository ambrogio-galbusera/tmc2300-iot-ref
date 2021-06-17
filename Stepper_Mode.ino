#define BLYNK_PRINT Serial
//#define BLYNK_BLE
//#define SPEED_RAMP_TEST

#ifdef BLYNK_BLE
#define BLYNK_USE_DIRECT_CONNECT
#endif

#define PIN_VCC_IO 5
#define PIN_EN 32
#define PIN_DIAG 34
#define PIN_RED 23
#define PIN_GREEN 18
#define PIN_STEP 21
#define PIN_DIR 22

#define TX1_pin  10
#define RX1_pin  9

#define USC     4
#define FSC     256
#define V_RPS  (V_HZ / USC / FSC)
#define FCLK   12000000
#define FLCK_DIV (2^24)

#ifdef BLYNK_BLE
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#else
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#endif

#include "include/Functions.h"
#include "include/TMC2300.h"
#include "include/CRC.h"

/******************************************************************************/
#ifndef BLYNK_BLE
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "<Wifi SSID>";
char pass[] = "<Wifi password>";

char auth[] = "<Blynk token>";
#else
// Auth Token for the Blynk App.
char auth[] = "<Blynk token>";
#endif

/******************************************************************************/
#define VIRTUAL_PIN_CURRENT    1
#define VIRTUAL_PIN_VELOCITY   2
#define VIRTUAL_PIN_DIRECTION  3
#define VIRTUAL_PIN_ENABLE     4

BlynkTimer timer;

int targetVelocity = 0;
int targetVActual = 0;
int prevMillis;
bool direction = true;
bool enable = false;

int tmc_compute_vactual(int rpm)
{
  double vhz = rpm * FSC / USC;
  double vact = vhz / 0.715;
  Serial.print("RPM: "); Serial.print(rpm); Serial.print(" --> "); Serial.print(vhz); Serial.print(" --> "); Serial.println(vact);
  
  return (int)vact;
}

/******************************************************************************/
// These functions are called whenever the corresponding virtual pin is updated
// in the Blynk app

BLYNK_WRITE(VIRTUAL_PIN_CURRENT)
{
  Serial.print("New MaxCurrent set: ");
  Serial.println(param.asInt());
  
  uint32_t value = 1 << TMC2300_IHOLDDELAY_SHIFT
                 | ((param.asInt() << TMC2300_IRUN_SHIFT) & TMC2300_IRUN_MASK)
                 | 8 << TMC2300_IHOLD_SHIFT;
  tmc2300_writeInt(TMC2300_IHOLD_IRUN, value);
}

BLYNK_WRITE(VIRTUAL_PIN_VELOCITY)
{
  Serial.print("New TargetVelocity set: ");
  Serial.println(param.asInt());

  targetVelocity = param.asInt();
  targetVActual = tmc_compute_vactual(param.asInt());
  
  tmc2300_writeInt(TMC2300_VACTUAL, direction? targetVActual : -targetVActual);
}

BLYNK_WRITE(VIRTUAL_PIN_DIRECTION)
{
  Serial.println("Changed Direction");
  
  direction = param.asInt() != 0;

  tmc2300_writeInt(TMC2300_VACTUAL, direction? targetVActual : -targetVActual);
}


BLYNK_WRITE(VIRTUAL_PIN_ENABLE)
{
  enable = param.asInt() != 0;
  
  if (enable)
  {
    Serial.println("Enable Motor: True");
  }
  else
  {
    Serial.println("Enable Motor: False");
  }
  
  digitalWrite(PIN_EN, enable? HIGH:LOW);
}

// Called once per second by the Blynk timer
void periodicJob()
{
  if (enable)
  {
    // Toggle the status LED while the motor is active
    digitalWrite(PIN_GREEN, HIGH);
    delay(250);
    digitalWrite(PIN_GREEN, LOW);
    delay(250);
    digitalWrite(PIN_GREEN, HIGH);
    delay(250);
    digitalWrite(PIN_GREEN, LOW);
  }

  // Re-write the CHOPCONF register periodically
  tmc2300_writeInt(TMC2300_CHOPCONF, 0x14008001);
}

void tmp_linear_ramp()
{
  
}
void tmc_setup()
{
  // pull down EN
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);

  pinMode(PIN_STEP, OUTPUT);    
  digitalWrite(PIN_STEP, LOW);   
      
  pinMode(PIN_DIR, OUTPUT);    
  digitalWrite(PIN_DIR, LOW);     
  
  // power on TMC
  pinMode(PIN_VCC_IO, OUTPUT);
  digitalWrite(PIN_VCC_IO, LOW);

  // Initialize CRC calculation for TMC2300 UART datagrams
  Serial.print("Initializing CRC...");
  tmc_fillCRC8Table(0x07, true, 0);
  Serial.println("DONE");

  // check communication
  Serial.print("Reading IFCNT...");
  int32_t reg = tmc2300_readInt(TMC2300_IFCNT);
  Serial.print("DONE ("); Serial.print(reg, HEX); Serial.println(")");

  // lower down current (hold=0, run=10)
  Serial.print("Setting IHOLD_IRUN...");
  tmc2300_writeInt(TMC2300_IHOLD_IRUN, 0x00000A00);
  tmc2300_writeInt(TMC2300_CHOPCONF, 0x14008001);     // Re-write the CHOPCONF register periodically    
  Serial.println("DONE");
  
  reg = tmc2300_readInt(TMC2300_IFCNT);
  Serial.print("DONE ("); Serial.print(reg, HEX); Serial.println(")");
}

void setup()
{
  // Debug console
  Serial.begin(115200);

  // TMC2300 IC UART connection
  Serial1.begin(115200, SERIAL_8N1, RX1_pin, TX1_pin);

  // Status LED
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_RED, OUTPUT);

  tmc_setup();
  
  // Connect to the WiFi access point
  Serial.println("Connecting to the WiFi access point");
#ifdef BLYNK_BLE
  Blynk.setDeviceName("Blynk");

  Blynk.begin(auth);
#else
  // Connect to the WiFi access point
  Serial.println("Connecting to the WiFi access point");
  Blynk.connectWiFi(ssid, pass);

  // Connect to the Blynk server
  Serial.println("Authentificating with the Blynk server");
  Blynk.config(auth);
#endif

  Serial.println("Initialization complete");

  // Start the timer for the periodic function
  timer.setInterval(1000L, periodicJob);

  prevMillis = millis();

#ifdef SPEED_RAMP_TEST
  tmc2300_writeInt(TMC2300_IHOLD_IRUN, 0x00001400);

  digitalWrite(PIN_EN, HIGH);
  delay(1000);
  targetVActual = tmc_compute_vactual(100);

  Serial.println("Homming at 100 RPM");
  tmc2300_writeInt(TMC2300_VACTUAL, targetVActual);
  delay(2000);
  Serial.println("Stopping");
  tmc2300_writeInt(TMC2300_VACTUAL, 0);  
#endif
}

int delayCntr = 0;
int rampCntr = 0;

void loop()
{
  Blynk.run();
  timer.run();

#ifdef SPEED_RAMP_TEST
  int currMillis = millis();
  int delta = currMillis - prevMillis;
  if (delta > 5000)
  {
    if (targetVelocity < 200)
    {
      targetVelocity += 10;
      targetVActual = tmc_compute_vactual(targetVelocity);
  
      Serial.print("Running at "); Serial.print(targetVelocity); Serial.println(" RPM");
      tmc2300_writeInt(TMC2300_VACTUAL, targetVActual);
    }
    
    prevMillis = currMillis;
  }
#endif

  if (Serial.available())
  {
    int c = Serial.read();

    if (c == '0')
    {
      int32_t reg1 = tmc2300_readInt(TMC2300_IFCNT);
      int32_t reg2 = tmc2300_readInt(TMC2300_GSTAT);
      int32_t reg3 = tmc2300_readInt(TMC2300_GCONF);
      int32_t reg4 = tmc2300_readInt(TMC2300_IOIN);
      int32_t reg5 = tmc2300_readInt(TMC2300_TSTEP);
      int32_t reg6 = tmc2300_readInt(TMC2300_SG_VALUE);
      int32_t reg7 = tmc2300_readInt(TMC2300_CHOPCONF);
      int32_t reg8 = tmc2300_readInt(TMC2300_DRVSTATUS);
      int32_t reg9 = tmc2300_readInt(TMC2300_PWMCONF);
      int32_t reg10 = tmc2300_readInt(TMC2300_PWMSCALE);
      int32_t reg11 = tmc2300_readInt(TMC2300_PWM_AUTO);
    
      Serial.println("--------------------------------------");
      Serial.print("IFCNT:    ");  Serial.println(reg1, HEX);
      Serial.print("GSTAT:    ");  Serial.println(reg2, HEX);
      Serial.print("GCONF:    ");  Serial.println(reg3, HEX);
      Serial.print("IOIN:     ");  Serial.println(reg4, HEX);
      Serial.print("TSTEP:    ");  Serial.println(reg5, HEX);
      Serial.print("SG:       ");  Serial.println(reg6, HEX);
      Serial.print("CHOPCONF: ");  Serial.println(reg7, HEX);
      Serial.print("DRV_STAT: ");  Serial.println(reg8, HEX);
      Serial.print("PWM_CONF: ");  Serial.println(reg9, HEX);
      Serial.print("PWM_SCALE:");  Serial.println(reg10, HEX);
      Serial.print("PWM_AUTO: ");  Serial.println(reg11, HEX);
    }
    else if (c == '1')
    {
      Serial.println(targetVelocity);
      targetVelocity += 5;
      Serial.println(targetVelocity);
      targetVActual = tmc_compute_vactual(targetVelocity);
      tmc2300_writeInt(TMC2300_VACTUAL, direction? targetVActual : -targetVActual);
    }
    else if (c == '2')
    {
      Serial.println(targetVelocity);
      targetVelocity -= 5;
      Serial.println(targetVelocity);
      targetVActual = tmc_compute_vactual(targetVelocity);
      tmc2300_writeInt(TMC2300_VACTUAL, direction? targetVActual : -targetVActual);
    }
    
  }
}
