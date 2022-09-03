#include <SimpleDHT.h>     // Data ---> D3 VCC ---> 3V3 GND ---> GND
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>  // http web access library
#include <ArduinoJson.h>        // JSON decoding library
#include <WiFiClient.h>

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//MQ135 Sensor Data processing
#include <MQ135.h>
#define PIN_MQ135 A0
#define Rain_PIN 14
MQ135 mq135_sensor(PIN_MQ135, 45);


#include <SFE_BMP180.h>

#include <Wire.h>
#include <SimpleDHT.h>
//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 16, 2);


// WiFi parameters
const char *ssid     = "TP-LINK_9544";
const char *password = "smoothspider130";

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME  "Atikul"
#define AIO_KEY       "aio_uOuI658yLc4qF94NaAJBfG6yybu4"
//#define AIO_USERNAME  "saimur_arnab"
//#define AIO_KEY       "aio_mGry21FSfZwdx0Fh6xOtGVt8u4bU"

WiFiClient wifiClient;
WiFiClient client;

// DATA for referencing
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish temp_ref = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp_ref");
Adafruit_MQTT_Publish hum_ref = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hum_ref");
Adafruit_MQTT_Publish pressure_ref = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure_ref");
// DATA from the NODE MCU
Adafruit_MQTT_Publish Temperature_DHT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature_DHT");
Adafruit_MQTT_Publish Humidity_DHT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity1_DHT");
Adafruit_MQTT_Publish Air_quality_MQ_135 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Air_quality_MQ_135");
Adafruit_MQTT_Publish Pressure_BMP180 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure_BMP180");
Adafruit_MQTT_Publish rain_sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rain_sensor");


int pinDHT11 = 0;
SimpleDHT11 dht11(pinDHT11);
byte hum = 0;  //Stores humidity value
byte temp = 0; //Stores temperature value
#define PHOTOCELL_PIN A0
 SFE_BMP180 pressure;
 #define ALTITUDE 32

const unsigned char buzzer = 16;

void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.print("Connecting.");

  WiFi.begin(ssid, password);   // access Wi-FI point

  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected\r\n");
  // connect to adafruit io
  connect();
  pinMode (buzzer, OUTPUT);

  //Serial.begin(9600);
  pressure.begin();
  //lcd.begin();
  //lcd.backlight();

}


//.........................VOID CONNECT() ...........................


// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if (ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

int counter=1;
void loop() {
  Serial.println(counter);
if (counter > 7){
  counter=1;
}
  // ping adafruit io a few times to make sure we remain connected
  if (! mqtt.ping(3)) {
    //Serial.printf("WTF");
    // reconnect to adafruit io
    if (! mqtt.connected())
      connect();
  }



  // Openweather strats here
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status

    HTTPClient http;  //Declare an object of class HTTPClient

    // specify request destination
    http.begin(wifiClient, "http://api.openweathermap.org/data/2.5/weather?q=Dhaka,BD&APPID=c25108944bc7981074839ba08f296a53");
    //begin()
    int httpCode = http.GET();  // send the request

    if (httpCode > 0) { // check the returning code

      String payload = http.getString();   //Get the request response payload

      DynamicJsonBuffer jsonBuffer(512);

      // Parse JSON object
      JsonObject& root = jsonBuffer.parseObject(payload);
      if (!root.success()) {
        Serial.println(F("Parsing failed!"));
        return;
      }

      // Read the humidity and temperature data
      dht11.read(&temp, &hum, NULL);

      //Read the MQ 135 data
      float correctedRZero = mq135_sensor.getCorrectedRZero(temp, hum);
      float correctedPPM = mq135_sensor.getCorrectedPPM(temp, hum);

      int H = (int)hum;
      int b = (int)temp;
      int Gas = correctedPPM;


      //Read the BMP 180 data
      char status;
      double T, P, p0, a;
      status = pressure.startTemperature();
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getTemperature(T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("temperature: ");
          Serial.print(T,2);
          Serial.print(" deg C, ");

          // Start a pressure measurement:
          // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
          // If request is successful, the number of ms to wait is returned.
          // If request is unsuccessful, 0 is returned.

          status = pressure.startPressure(3);
          if (status != 0)
          {
            // Wait for the measurement to complete:
            delay(status);

            // Retrieve the completed pressure measurement:
            // Note that the measurement is stored in the variable P.
            // Note also that the function requires the previous temperature measurement (T).
            // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
            // Function returns 1 if successful, 0 if failure.

            status = pressure.getPressure(P, T);
            if (status != 0)
            {
              // Print out the measurement:
              Serial.print("absolute pressure: ");
              Serial.print(P, 2);
              P = P / 1000;
              Serial.print(" bar, ");


              // The pressure sensor returns abolute pressure, which varies with altitude.
              // To remove the effects of altitude, use the sealevel function and your current altitude.
              // This number is commonly used in weather reports.
              // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
              // Result: p0 = sea-level compensated pressure in mb

              p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
              Serial.print("relative (sea-level) pressure: ");
              Serial.print(p0, 2);
              p0 = p0 / 1000;
              Serial.print("bar, ");
              // Serial.print(p0*0.0295333727,2);
              //Serial.println(" inHg");

              // On the other hand, if you want to determine your altitude from the pressure reading,
              // use the altitude function along with a baseline pressure (sea-level or other).
              // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
              // Result: a = altitude in m.

              a = pressure.altitude(P, p0);
              Serial.print("computed altitude: ");
              Serial.print(a, 0);
              Serial.print(" meters, ");

            }
            else Serial.println("error retrieving pressure measurement\n");
          }
          else Serial.println("error starting pressure measurement\n");
        }
        else Serial.println("error retrieving temperature measurement\n");
      }
      else Serial.println("error starting temperature measurement\n");
      //delay(5000);
      //data from rain sensor
      int R=digitalRead(Rain_PIN);
      String rain;
      if (R==1){
        rain="It's dry outside";
      }
      else {
        rain="it's raining";
      }
      // Read the reference datas
      float temperature_reference = (float)(root["main"]["temp"]) - 273.15;        // get temperature
      int   humidity_reference = root["main"]["humidity"];                  // get humidity
      float pressure_reference = (float)(root["main"]["pressure"]) / 1000;  // get pressure


      // print data from reference
      Serial.printf("Temperature_Reference = %.2f°C\r\n", temperature_reference);
      Serial.printf("Humidity_Reference    = %d %%\r\n", humidity_reference);
      Serial.printf("Pressure_Reference    = %.3f bar\r\n", pressure_reference);

      // print data from NODEMCU
      Serial.print((int)temp); Serial.print(" *C, ");
      Serial.print((int)hum); Serial.print(" H,  ");
      Serial.print(correctedRZero); Serial.print(" ohm,  ");
      Serial.print((int)correctedPPM); Serial.println(" ppm ");
       Serial.print(rain);




      //adafruit strats here
     switch (counter)
    {
      case 1:temp_ref.publish(temperature_reference);break;
      case 2:hum_ref.publish(humidity_reference); break;
      case 3:pressure_ref.publish(pressure_reference);
      case 4:Temperature_DHT.publish(temp);
      case 5:Humidity_DHT.publish(hum);
      case 6:Air_quality_MQ_135.publish(correctedPPM); 
      case 7:Pressure_BMP180.publish(P);break;
      default: temp_ref.publish(temperature_reference);break;
    }




      
      //Publish Reference
    /*  if (! temp_ref.publish(temperature_reference)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      } 
      if (! hum_ref.publish(humidity_reference)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }
      if (! pressure_ref.publish(pressure_reference)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }

            
     // Publish Our data
      if (! Temperature_DHT.publish(temp)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }
      if (! Humidity_DHT.publish(hum)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }
      if (! Air_quality_MQ_135.publish(correctedPPM)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }
      if (! Pressure_BMP180.publish(P)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }
      if (! rain_sensor.publish(R)) {                     //Publish to Adafruit
        Serial.println(F("Failed"));
      }
      
      else {
        Serial.println(F("Sent!"));
      }
      */
      

      // The threshold for buzzer is 350
      if (correctedPPM > 350)
      {
        digitalWrite(buzzer, HIGH);
      }

      else
      {
        digitalWrite(buzzer, LOW);
      }
      //data from rain sensor
      //int R=digitalRead(Rain_PIN);

      //sending to LCD
   /* lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HUM : ");
    lcd.print(H);
    lcd.print(" %");
    //delay(2000);
     //lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("TEMP : ");
    lcd.print(b);
    lcd.print(" *C");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("AIRQuality : ");
    lcd.print(Gas);
    lcd.print(" %");
     //delay(2000);
      //lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("pressure:");
    lcd.print(P,2);
    lcd.print("bar");
    delay(2000);
     if(R==1)
    {
      lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" it's hot day ");
   //lcd.print(rain);
    }
    else
    {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("it's raaining ");
    
    lcd.setCursor(0, 1);
    lcd.print("outside ");
    }*/
   
    //delay(000);

    }

    //http.end();   //Close connection

  }// openweather end here


  counter+=1;
  delay(5000);   // wait 1 minute

}
// End of code.
