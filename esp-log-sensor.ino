// This code is a NMEA 0183 boatspeed instrument
// Mike Holden mholden@csum.edu
// began 3/17/10
// Changes:
//  05/02/10 added factory reset to program eeprom initially (instead of
//  commenting in/out stuff)
//           put checksum in eeprom to tell if it's been programmed.
//           fixed up calibration routine to work better
//  05/17/10 Added fix so missing pulses make speed go to zero.
//  05/24/12 updated for arduino 1.0
//  09/02/17 Updated for arduino 1.8
//           PWM test pin frequency using tone instead of hacking PWM registers
//           EEPROM uses built in functions rather than "eeprom write anything"
//           old example.

#include <EEPROM.h>

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include <DallasTemperature.h>
#include <OneWire.h>

// Bibliothek für ESP-Now
extern "C" {
#include <espnow.h>
}

#ifndef STASSID
#define STASSID "paddlewheel"
#define STAPSK "Macanache"
#endif

#define WIFI_CHANNEL 1

#define SAMPLE_MIN_WIDTH 40000
#define SAMPLE_MAX_WIDTH 500000
#define UPDATE_INTERVAL 500
#define SMOOTHING_WEIGHT 10
#define NMEA_MAX_LENGTH 85

const char *ssid = STASSID;
const char *password = STAPSK;

// MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

// Hardware:
const uint8_t pulsePin = D1;     // Pulse connected to this digital pin
const uint8_t tempPin = D2;
const uint8_t LEDPin = LED_BUILTIN; // there should be an LED on 13 already

long baud;

// factory defaults
// update these before programming
// if you don't want to use the calibration routines
// the calibration routines can be used to update these without reprogramming
#define DEFAULT_BAUD 115200
#define DEFAULT_KMI_PER_PULSE                                                  \
  (1. / 20000.) // nautical miles per pulse (Airmar says 1/20e3)
#define DEFAULT_SPEED_SCALE                                                    \
  1.00 // scale factor for calibration. (multiplies kmi_per_pulse)
#define DEFAULT_A_FILT                                                         \
  1 // filter pole radians/sec.  time constant = 1/a seconds
#define DEFAULT_DT_PRINT 1 // s between output strings

// speed calcs
float kmi_per_pulse;  // nautical miles per pulse (airmar spec is 1/20e3)
float speed_scale;    // scale factor for calibration.  Multiplies kmi_per_pulse
float speed_raw = 0;  // unfiltered speed
float speed_filt = 0; // low-pass filtered speed
float a_filt;         // filter pole radians/sec.  time constant = 1/a seconds
float adt = 0;
long dt_update = 0; //
unsigned long last_update_time;

// pulse timing calcs
// volatile so they can be updated in an interrupt
volatile long dtpulse;
volatile unsigned long edgetime;
volatile unsigned char gotdata = 0;
unsigned char lastpin = 0;

// output stuff
float dt_print;    // s between output strings
float dt_char = 3; // ms between output characters ~10/baud*1000
float print_time = 0, char_time = 0;
char nmeastring[NMEA_MAX_LENGTH];
int kt, kttenths, kph, kphtenths;
char checksum = 0xAA;
int ii, jj;

// testing timer stuff
extern unsigned long timer0_overflow_count;
int verflowed = 0;

// data is stored here in eeprom
// this is a structure
struct save_t {
  unsigned int checkcode;
  long baud;           // baud rate
  float kmi_per_pulse; // this should be in eeprom later for calibration
  float speed_scale;   // scale factor for calibration.
  float a_filt;        // filter pole radians/sec.  time constant = 1/a seconds
  float dt_print;      // s between output strings
} saved;

OneWire tempWire(tempPin);
DallasTemperature tempSensor(&tempWire);

void setupOTA() {
  WiFi.softAP(ssid, password, WIFI_CHANNEL);
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   delay(5000);
  //   ESP.restart();
  // }
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.begin();
}

// The setup() method runs once, when the sketch starts
void setup() {
  // initialize the digital pin as an output:
  pinMode(LEDPin, OUTPUT);
  pinMode(pulsePin, INPUT_PULLUP); // newer way
  pinMode(tempPin, INPUT_PULLUP); // newer way

  // Load data from eeprom
  digitalWrite(LEDPin, HIGH); // set the LED on
  delay(100);                 // wait for a second
  digitalWrite(LEDPin, LOW);  // set the LED off
  delay(100);                 // wait for a second
  digitalWrite(LEDPin, HIGH); // set the LED on
  delay(100);                 // wait for a second
  digitalWrite(LEDPin, LOW);  // set the LED off
  delay(500);                 // wait for a second

  EEPROM.get(0, saved);
  baud = saved.baud;
  kmi_per_pulse = saved.kmi_per_pulse;
  speed_scale = saved.speed_scale;
  a_filt = saved.a_filt;
  dt_print = saved.dt_print;

  if (saved.checkcode != 0x1234)
  // handle no eeprom here
  {
    baud = DEFAULT_BAUD;
    kmi_per_pulse =
        DEFAULT_KMI_PER_PULSE; // nautical miles per pulse (Airmar says 1/20e3)
    speed_scale = DEFAULT_SPEED_SCALE; // scale factor for calibration.
                                       // (multiplies kmi_per_pulse)
    a_filt =
        DEFAULT_A_FILT; // filter pole radians/sec.  time constant = 1/a seconds
    dt_print = DEFAULT_DT_PRINT; // s between output strings
  }

  dt_char = 20000 /
            (float)baud; // ms between output characters; minimum ~10/baud*1000
  
  Serial.begin(baud);
  
  setupOTA();
  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    delay(5000);
    ESP.restart();
  } else {
    Serial.println("*** ESP_Now init OK");
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  }
  tempSensor.begin();

  if (saved.checkcode == 0x1234) {
    Serial.println("--Loaded Data from EEPROM--");
  } else {
    Serial.println("--Bad data from EEPROM, resetting--");
  }
  Serial.print("  Baud = "); // testing
  Serial.println(baud);
  Serial.print("  1/kmi_per_pulse="); // testing
  Serial.println(1. / kmi_per_pulse);
  Serial.print("  speed_scale="); // testing
  Serial.println(speed_scale);
  Serial.print("  Filter pole = "); // testing
  Serial.println(a_filt);
  Serial.print("  output interval = ");
  Serial.println(dt_print);
  Serial.println("---");

  digitalWrite(LEDPin, HIGH); // set the LED on
  delay(1000);                // wait for a second
  digitalWrite(LEDPin, LOW);  // set the LED off
  delay(1000);                // wait for a second

  calibration();

  attachInterrupt(
      1, falling_edge,
      FALLING); // catch falling edge and handle with falling_edge();
}

void calibration(void)
// This routine asks for input
{
  Serial.println("Enter +++ to calibrate");
  print_time = millis();
  ii = 0;
  while (millis() < print_time + 5000) {
    if (Serial.available() > 0) {
      if (Serial.read() == '+') {
        ii++;
      } else {
        ii--;
      }
    }
    if (ii == 3) {
      Serial.println("--Calibration---");

      // factory resets:
      Serial.println("\r\n--\r\nFactory Calibration:");
      Serial.println(
          "Press + to start with factory calibration, any other key to skip");
      ii = -1;
      while (ii == -1) {
        ii = Serial.read();
      }
      if (ii == '+') {
        baud = DEFAULT_BAUD;
        kmi_per_pulse = DEFAULT_KMI_PER_PULSE; // nautical miles per pulse
                                               // (Airmar says 1/20e3)
        speed_scale = DEFAULT_SPEED_SCALE;     // scale factor for calibration.
                                               // (multiplies kmi_per_pulse)
        a_filt = DEFAULT_A_FILT; // filter pole radians/sec.  time constant =
                                 // 1/a seconds
        dt_print = DEFAULT_DT_PRINT; // s between output strings
      }

      ii = 0;
      Serial.println("\r\n--\r\nSpeed Scale:");
      Serial.println("Use + and - keys to change, Enter to accept");
      while (ii != 13) {
        Serial.print("  \r");
        Serial.print(speed_scale);
        ii = Serial.read();
        if (ii == '+') {
          speed_scale += 0.01;
        }
        if (ii == '-') {
          speed_scale -= 0.01;
        }
        if (speed_scale > 20)
          speed_scale = 20;
        if (speed_scale < 0.1)
          speed_scale = 0.1;
      }

      ii = 0;
      Serial.println("\r\n--\r\nFilter Pole (rad/sec):");
      Serial.println("  Time constant = 1/pole (seconds):");
      Serial.println("Use + and - keys to change, Enter to accept");
      while (ii != 13) {
        Serial.print("  \r");
        Serial.print(a_filt);
        ii = Serial.read();
        if (ii == '+') {
          a_filt += 0.1;
        }
        if (ii == '-') {
          a_filt -= 0.1;
        }
        if (a_filt > 10)
          a_filt = 10;
        if (a_filt < 0.1)
          a_filt = 0.1;
      }

      ii = 0;
      Serial.println("\r\n--\r\nOutput interval (seconds):");
      Serial.println("Use + and - keys to change, Enter to accept");
      while (ii != 13) {
        Serial.print("  \r");
        Serial.print(dt_print);
        ii = Serial.read();
        if (ii == '+') {
          dt_print += 0.1;
        }
        if (ii == '-') {
          dt_print -= 0.1;
        }
        if (dt_print > 10)
          dt_print = 10;
        if (dt_print < 0.1)
          dt_print = 0.1;
      }

      ii = 0;
      Serial.println("\r\n--\r\nBaud Rate: ");
      Serial.println("Use + and - keys to change, Enter to accept");
      while (ii != 13) {
        Serial.print("  \r");
        Serial.print(baud);
        ii = Serial.read();
        if (ii == '+') {
          baud *= 2;
        }
        if (ii == '-') {
          baud /= 2;
        }
        if (baud > 115200)
          baud = 115200;
        if (baud < 1200)
          baud = 1200;
      }
      Serial.println(
          "\r\nCycle power or reset to see new baud rate (save first)");

      Serial.println("Press Enter to save, x (or power down now) to abort");
      ii = 0;
      while ((ii != 13) && (ii != 'x')) {
        ii = Serial.read();
        if (ii == 13) {
          saved.checkcode = 0x1234;
          saved.baud = baud;
          saved.kmi_per_pulse =
              kmi_per_pulse; // nautical miles per pulse (Airmar says 1/20e3)
          saved.speed_scale = speed_scale; // scale factor for calibration.
                                           // (multiplies kmi_per_pulse)
          saved.a_filt =
              a_filt; // filter pole radians/sec.  time constant = 1/a seconds
          saved.dt_print = dt_print;
          EEPROM.put(0, saved);
          Serial.println("Data saved...");
        }
      }

      print_time = 0; // exit loop sooner
    }
  }
}

ICACHE_RAM_ATTR void falling_edge(void)
// what to do on a falling edge
{
  dtpulse = micros() - edgetime;
  edgetime = micros();
  gotdata = 1;
}

// the loop() method runs over and over again,
// as long as the Arduino has power
void loop() {
  ArduinoOTA.handle();
  WiFi.disconnect();

  tempSensor.requestTemperatures(); // Send the command to get temperatures
  float tempC = tempSensor.getTempCByIndex(0);
  // Check if reading was successful
  if(tempC != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Temperature is: ");
    Serial.println(tempC);
  } 

  // if it's been too long, fake some 0 kt data
  if ((float)(micros() - last_update_time) * 1e-6 > 2.0 * dt_print) {
    dtpulse = (long)(kmi_per_pulse / .001 * 3.60e9 *
                     speed_scale); // should give 0.001 kt
    gotdata = 1;
  }

  // when done timing, do filtering
  // the millis timer overflows after 50 days.
  // This will give some spurious data, which will take a few time constants to
  // filter out.
  if (gotdata) {
    speed_raw = kmi_per_pulse / (float)dtpulse * 3.60e9 * speed_scale; // kt
    dt_update = (micros() - last_update_time); // seconds
    last_update_time = micros();               // ms

    adt = a_filt * (float)dt_update * 1.0e-6;
    if (adt < 1.0) {
      speed_filt =
          speed_filt + adt * (speed_raw - speed_filt); // for stability a*dt < 1
    } else {
      speed_filt = speed_raw; // un-filterable (shouldn't happen if set up
                              // properly and working)
    }

    // gotdata = 0; // HACK!!! should reset to 0 (uncomment for non-fake data)

    // make NMEA string
    // speed_filt = 8*cos(float(millis())*6.0/10000.0)+8; //HACK!!! fake data
    kt = (int)speed_filt;
    kttenths = speed_filt * 100 - kt * 100;
    // this is a hack to check filtering
    // kph = (int)speed_raw;
    // kphtenths = speed_raw*100-kph*100;
    // this is the correct kph values
    kph = (int)(speed_filt * 1.852);
    kphtenths = (speed_filt * 185.2 - kph * 100);

    // Compose NMEA stentence
    memset(nmeastring, 0, NMEA_MAX_LENGTH);
    sprintf(nmeastring, "$VWVHW,,T,,M,%1d.%02d,N,%1d.%02d,K*", kt, kttenths,
            kph, kphtenths);
    checksum = 0;
    jj = 1;
    while (nmeastring[jj + 1] != 0) {
      checksum ^= nmeastring[jj];
      jj++;
    }
    sprintf(nmeastring + strlen(nmeastring), "%0X\r\n", checksum);
  }

  // Print NMEA sentence.
  // periodically send the string
  // $VWVHW,,T,,M,4.14,N,7.65,K*51 (heading true) (heading mag) (speed knts)
  // (speed KPH)
  // === VHW - Water speed and heading ===
  //
  //------------------------------------------------------------------------------
  //        1   2 3   4 5   6 7   8 9
  //        |   | |   | |   | |   | |
  // $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
  //------------------------------------------------------------------------------
  // Field Number:
  // 1. Degress True
  // 2. T = True
  // 3. Degrees Magnetic
  // 4. M = Magnetic
  // 5. Knots (speed of vessel relative to the water)
  // 6. N = Knots
  // 7. Kilometers (speed of vessel relative to the water)
  // 8. K = Kilometers
  // 9. Checksum
  //    To do:  Add wind output as a possiblility for wind paddlewheel sensors.
  //    This is the wind sentence:
  //    === MWV - Wind Speed and Angle ===
  //    ------------------------------------------------------------------------------
  //            1   2 3   4 5
  //            |   | |   | |
  //     $--MWV,x.x,a,x.x,a*hh<CR><LF>
  //    ------------------------------------------------------------------------------
  //    Field Number:
  //    1. Wind Angle, 0 to 360 degrees
  //    2. Reference, R = Relative, T = True
  //    3. Wind Speed
  //    4. Wind Speed Units, K/M/N (kph or m/s or Knots respectively)
  //    5. Status, A = Data Valid
  //    6. Checksum

  if (millis() > print_time) {

    // if (millis() > char_time) {
    //   if (nmeastring[ii] == 0) { // end of string
    //                              // Serial.print(speed_filt);
    //                              // Serial.print(' ');
    //                              // Serial.print(speed_raw);
    //                              // Serial.print(' ');
    //     if (checksum < 0x10)
    //       Serial.print('0');
    //     Serial.println(checksum, HEX);
    //     ii = 0;
    //     print_time = (float)millis() + dt_print * 1000;
    //   } else {
    //     Serial.print(nmeastring[ii]);
    //     ii++;
    //   }
    //   char_time = millis() + dt_char;
    // }

    uint32_t bsize = strlen(nmeastring);
    char sendBuffer[NMEA_MAX_LENGTH];
    memset(sendBuffer, 0, NMEA_MAX_LENGTH);
    memcpy(sendBuffer, nmeastring, bsize);
    Serial.print(sendBuffer);
    Serial.printf("Size %d\n", bsize);
    int sendStatus = esp_now_send(NULL, (u8*)sendBuffer, bsize + 2);
    switch (sendStatus)
    {
    case 0:
      Serial.println("ESP send OK");
      break;
    case 1:
      Serial.println("ESP send FAILED");
      break;
    }
    print_time = (float)millis() + dt_print * 1000;
  }

} // end of loop
