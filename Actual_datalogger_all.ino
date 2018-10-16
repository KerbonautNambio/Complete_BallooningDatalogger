#include "Adafruit_Sensor.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "DHT.h"
#include "Adafruit_SI1145.h"
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

#define DHTPIN 3 // what pin we're connected to
#define DHTTYPE DHT22

// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs

// The analog pins that connect to the sensors
#define BANDGAPREF 14            // special indicator that we want to measure the bandgap

#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SI1145 uv = Adafruit_SI1145();
MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

double dewPoint(double celsius, double humidity)
{
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

  // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;

  // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP / 0.61078); // temp var
  return (241.88 * T) / (17.558 - T);
}


void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while (1);
}

void setup(void)
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("DHT22 test!");
  Serial.println("Adafruit SI1145 tested");
  
  if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
  Serial.println();

  dht.begin();
 
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelGyroMag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

  // use debugging LEDs

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }


  logfile.println("millis,stamp,datetime,light,temp,vcc");
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,light,temp,vcc");
#endif //ECHO_TO_SERIAL

  // If you want to set the aref to something other than 5v
  analogReference(EXTERNAL);
}

void loop(void)
{
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");
#endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL
  {
    float sensor_volt;
    float RS_gas; // Get value of RS in a GAS
    float ratio; // Get ratio RS_GAS/RS_air
    int sensorValue = analogRead(A0);
    sensor_volt = (float)sensorValue / 1024 * 5.0;
    RS_gas = (5.0 - sensor_volt) / sensor_volt; // omit *RL

    /*-Replace the name "R0" with the value of R0 show at measure apporx -*/
    ratio = RS_gas / 1.82; // ratio = RS/R0
    /*24/06/2017 in my room, RO = 1.15-----------------------------------------------------------------------*/
    logfile.print("RS_ratio = ");
    logfile.println(RS_gas);
    logfile.print("Rs/R0");
    logfile.println(ratio);
#if ECHO_TO_SERIAL
    Serial.print("RS_ratio = ");
    Serial.println(RS_gas);
    Serial.print("Rs/R0 = ");
    Serial.println(ratio);
    Serial.print("\n\n");
#endif

    delay(1000);
  }

  {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true);

    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    float hi = dht.computeHeatIndex(f, h);
    float hiDegC = dht.convertFtoC(hi);

    logfile.print("Humidity: ");
    logfile.print(h);
    logfile.print(" %\t");
    logfile.print("Temperature 1: ");
    logfile.print(t);
    logfile.print(" *C ");
    logfile.print(f);
    logfile.print(" *F\t");
    logfile.print("Temperature 2: ");
    logfile.print(hiDegC);
    logfile.print(" *C ");
    logfile.print(hi);
    logfile.print(" *F ");
    logfile.print("Dew Point (*C): ");
    logfile.println(dewPoint(t, h));
#if ECHO_TO_SERIAL
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature 1: ");
    Serial.print(t);
    Serial.print(" *C ");
    Serial.print(f);
    Serial.print(" *F\t");
    Serial.print("Temperature 2: ");
    Serial.print(hiDegC);
    Serial.print(" *C ");
    Serial.print(hi);
    Serial.print(" *F ");
    Serial.print("Dew Point (*C): ");
    Serial.println(dewPoint(t, h));
#endif
  }
{

  logfile.print("Visible UV: "); Serial.println(uv.readVisible());
  logfile.print("IR: "); Serial.println(uv.readIR());
#if ECHO_TO_SERIAL
  Serial.println("===================");
  Serial.print("Visible UV: "); Serial.println(uv.readVisible());
  Serial.print("IR: "); Serial.println(uv.readIR());
    delay(200);
#endif
}
  {
    delay(50);
    // read raw accel/gyro/mag measurements from device
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    accelGyroMag.getAcceleration(&ax, &ay, &az);
    accelGyroMag.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro/mag x/y/z values
    logfile.print("a/g/m:\t");
    logfile.print(ax); logfile.print("\t");
    logfile.print(ay); logfile.print("\t");
    logfile.print(az); logfile.print("\t");
    logfile.print(gx); logfile.print("\t");
    logfile.print(gy); logfile.print("\t");
    logfile.print(gz); logfile.print("\t");
    logfile.print(int(mx)*int(mx)); logfile.print("\t");
    logfile.print(int(my)*int(my)); logfile.print("\t");
    logfile.print(int(mz)*int(mz)); logfile.print("\t | ");
#if ECHO_TO_SERIAL    
    Serial.print("a/g/m:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print(int(mx)*int(mx)); Serial.print("\t");
    Serial.print(int(my)*int(my)); Serial.print("\t");
    Serial.print(int(mz)*int(mz)); Serial.print("\t | ");
#endif
    const float N = 256;
    float mag = mx*mx/N + my*my/N + mz*mz/N;

    Serial.print(mag); Serial.print("\t");
    for (int i=0; i<mag; i++)
        Serial.print("*");
    Serial.print("\n");
}

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

}


