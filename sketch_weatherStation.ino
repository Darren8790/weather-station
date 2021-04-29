#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <Blynk.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BlynkSimpleEsp8266.h>
#define SEALEVELPRESSURE_HPA (1013.25)
//Setup connection of the sensor
Adafruit_BME280 bme; // I2C

static const int RXPin = 13, TXPin = 12;   // GPIO 15=D2(conneect Tx of GPS) and GPIO 13=D1(Connect Rx of GPS
static const uint32_t GPSBaud = 9600; //if Baud rate 9600 didn't work in your case then use 4800

TinyGPSPlus gps; // The TinyGPS++ object
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget

SoftwareSerial ss(RXPin, TXPin);  // The serial connection to the GPS device

char auth[] = "azA_0g0UkLVgb_yCDDxdR2j16jhT9Mu1";
char ssid[] = "SKYYW59G";
char pass[] = "dkkbVWQQ3sxN";

BlynkTimer timer;

//Variables
float pressure;     //To store the barometric pressure (Pa)
float temperature;  //To store the temperature (oC)
float humidity;      //To store the humidity (%) (you can also use it as a float variable)
float altitude;
float spd;       //Variable  to store the speed
float sats;      //Variable to store no. of satellites response
String bearing;  //Variable to store orientation or direction of GPS

//unsigned int move_index;         // moving index, to be used later
unsigned int move_index = 1;       // fixed location for now

void setup() {
  bme.begin(0x76);    //Begin the sensor
  Serial.begin(9600); //Begin serial communication at 9600bps
  Serial.println();
  ss.begin(GPSBaud);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(2000L, ReadSensors);   // read sensor every 5s
  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
}

void checkGPS(){
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
      Blynk.virtualWrite(V4, "GPS ERROR");  // Value Display widget  on V4 if GPS not detected
  }
}

void ReadSensors() {
  //Read values from the sensor:
  pressure = bme.readPressure() / 100.0F;
  temperature = bme.readTemperature();
  humidity = bme.readHumidity ();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  Blynk.virtualWrite(V6, pressure);   // write pressure to V1 value display widget
  Blynk.virtualWrite(V7, temperature);  // write temperature to V2 value display widget
  Blynk.virtualWrite(V8, humidity);    // write altimeter to V3 value display widget
  Blynk.virtualWrite(V9, altitude);

  //Print values to serial monitor:
  Serial.print(F("Pressure: "));
  Serial.print(pressure);
  Serial.print(" hPa");
  Serial.print("\t");
  Serial.print(("Temp: "));
  Serial.print(temperature);
  Serial.print(" °C");
  Serial.print("\t");
  Serial.print("Humidity: ");
  Serial.print(humidity); // this should be adjusted to your local forcase
  Serial.println(" %");
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  //delay(2000); //Update every 5 sec
}

void loop() {
  while (ss.available() > 0) 
    {
      // sketch displays information every time a new sentence is correctly encoded.
      if (gps.encode(ss.read()))
        displayInfo();
  }
  Blynk.run();
  timer.run();
}

void displayInfo()
{ 
  if (gps.location.isValid() ) 
  {    
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon. 
    float longitude = (gps.location.lng()); 
    
    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);
    Blynk.virtualWrite(V1, String(latitude, 6));   
    Blynk.virtualWrite(V2, String(longitude, 6));  
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();               //get speed
       Blynk.virtualWrite(V3, spd);
       
       sats = gps.satellites.value();    //get number of satellites
       Blynk.virtualWrite(V4, sats);

       bearing = TinyGPSPlus::cardinal(gps.course.value()); // get the direction
       Blynk.virtualWrite(V5, bearing);                   
  }
  
 Serial.println();
}
