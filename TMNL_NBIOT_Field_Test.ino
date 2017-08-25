/***************************************************************************
	This is a library for the HTS221 Humidity Temperature Sensor
		Originally written by speirano for SmartEverything
		Adjusted by Gregory Knauff of SODAQ for the NB-IoT shield
	Adjusted by Jan van Loenen to work on Sodaq Explorer and Arduino Leonardo
  Adjusted by Eric Barten for the TMNL field test (added LCD)


***************************************************************************/

/*****
 * ATT Settings
 *
 * create a new asset as Number
 *
 * device decoding:
 
  {
  "sense": [{
      "asset": "my_temperature",
      "value": {
        "byte": 0,
        "bytelength": 2,
        "type": "integer",
        "calculation": "val / 100"
      }
    },
    {
      "asset": "my_humidity",
      "value": {
        "byte": 2,
        "bytelength": 2,
        "type": "integer",
        "calculation": "val / 100"
      }
    },
    {
      "asset": "my_pressure",
      "value": {
        "byte": 4,
        "bytelength": 2,
        "type": "integer"
      }
    },
    {
      "asset": "my_gps",
      "value": {
        "latitude": {
          "byte": 6,
          "bytelength": 4,
          "type": "integer",
          "calculation": "val / 100000"
        },
        "longitude": {
          "byte": 10,
          "bytelength": 4,
          "type": "integer",
          "calculation": "val / 100000"
        }
      }
    },
    {
      "asset": "my_date_time",
      "value": {
        "year": {
          "byte": 14,
          "bytelength": 2,
          "type": "integer"
        },
        "month": {
          "byte": 16,
          "bytelength": 2,
          "type": "integer"
        },
        "day": {
          "byte": 18,
          "bytelength": 2,
          "type": "integer"
        },
        "hour": {
          "byte": 20,
          "bytelength": 2,
          "type": "integer"
        },
        "minutes": {
          "byte": 22,
          "bytelength": 2,
          "type": "integer"
        },
        "seconds": {
          "byte": 24,
          "bytelength": 2,
          "type": "integer"
        }
      }
    },
    {
      "asset": "my_rssi",
      "value": {
        "byte": 26,
        "bytelength": 1,
        "type": "integer"
      }
    }
  ]
 }
 */

#include <Arduino.h>
#include <Wire.h>
#include <TMNL_Sodaq_nbIOT.h>
 // #include <SoftwareSerial.h> // Uno
#include "Sodaq_HTS221.h"
#include "Sodaq_LPS22HB.h"
#include "Sodaq_UBlox_GPS.h"
#include "LiquidCrystal_I2C.h"

#if defined(ARDUINO_AVR_LEONARDO)
#define DEBUG_STREAM Serial 
#define MODEM_STREAM Serial1

#elif defined(ARDUINO_AVR_UNO)
 SoftwareSerial softSerial(10, 11); // RX, TX
 // You can connect an uartsbee or other board (e.g. 2nd Uno) to connect the softserial.
#define DEBUG_STREAM softSerial 
#define MODEM_STREAM Serial

#elif defined(ARDUINO_SODAQ_EXPLORER)
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial

#elif defined(ARDUINO_SAM_ZERO)
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1

#else
#error "Please select a Sodaq ExpLoRer, Arduino Leonardo or add your board."
#endif

#define I2C_LCD         0x20  //2.6" LCD Keypad Shield I2C LCD 1602 Display Module w/ Touch Keys


 Sodaq_nbIOT nbiot;
 Sodaq_LPS22HB lps22hb;
 LiquidCrystal_I2C lcd(I2C_LCD, 16, 2);


 uint32_t lat = 0;
 uint32_t lon = 0;
 String fixStr;
 String last_fix_dt;
 int16_t yy = 2000;
 int16_t MM = 0;
 int16_t dd = 0;
 int16_t hh = 0;
 int16_t mm = 0;
 int16_t ss= 0;
 int16_t att_rssi=0;

 void setup();
 bool connectToNetwork();
 void initHumidityTemperature();
 void initPressureSensor();
 void initGPS();
 void loop();
 void do_flash_led(int pin);
 void PrintStringAt(String Text,int col=0, int row=0);


 void setup()
 {
	 pinMode(13, OUTPUT);
	 digitalWrite(13, HIGH);

	 DEBUG_STREAM.begin(9600);
	 MODEM_STREAM.begin(nbiot.getDefaultBaudrate());

	 //while ((!DEBUG_STREAM) || millis() < 10000) {
		 // Wait for serial monitor for 10 seconds
	 //}
   delay(2000);
 
   //Initiliaze Display
   lcd.init();
   lcd.backlight();
   PrintStringAt("TMNL NB-IOTester",0,0);
   delay(2000);

   nbiot.init(MODEM_STREAM, 7);
	 nbiot.setDiag(DEBUG_STREAM);
   nbiot.setMinRSSI(-113); // original value (-93) was based on GSM we expect additional 20 dB extension

	 delay(2000);
	 while(!connectToNetwork());
   
	 initHumidityTemperature();
	 initPressureSensor();
	 initGPS();

	 digitalWrite(13, LOW);
 }

 bool connectToNetwork() {
   PrintStringAt("C:_",13,1);
   lcd.setCursor(15, 1);
   lcd.blink();
	 if (nbiot.connect("oceanconnect.t-mobile.nl", "172.16.14.22", "20416")) {
     lcd.noBlink();
		 PrintStringAt("Connect succes! ",0,0);
     PrintStringAt("C:Y",13,1);
     delay(2000);
		 return true;
	 }
	 else {
		 PrintStringAt("Failed 2 connect",0,0);
     PrintStringAt("C:N",13,1);
		 delay(2000);
		 return false;
	 }
 }

 void initHumidityTemperature() {
	 if (hts221.begin() == false)
	 {
		 DEBUG_STREAM.println("Error while retrieving WHO_AM_I byte...");
		 while (1);
	 }
 }

 void initPressureSensor() {
	 lps22hb.begin(0x5D);	// 

	 if (lps22hb.whoAmI() == false)
	 {
		 DEBUG_STREAM.println("Error while retrieving WHO_AM_I byte...");
	 }
 }

 void initGPS() {
	 sodaq_gps.init(6);
   String fixStr = "La"+String(lat) + "Lo"+String(lon);
   String last_fix_dt ="20000000000000"; //yymmddhhmmss dummy date
	 //sodaq_gps.setDiag(DEBUG_STREAM);
 }


 void loop()
 {
	 do_flash_led(13);
	 // Create the message
	 byte message[8+7];   //one for every 16 bits value and 2 for every 32 bits values?
	 uint16_t cursor = 0;
	 int16_t temperature;
	 int16_t humidity;
	 int16_t pressure;
   int8_t rssi;
   uint8_t ber; //dummy has no value


	 temperature = hts221.readTemperature() * 100;
	 DEBUG_STREAM.println("Temperature x100 : " + (String)temperature);
	 message[cursor++] = temperature >> 8;
	 message[cursor++] = temperature;

	 delay(100);

	 humidity = hts221.readHumidity() * 100;
	 DEBUG_STREAM.println("Humidity x100 : " + (String)humidity);
	 message[cursor++] = humidity >> 8;
	 message[cursor++] = humidity;

	 delay(100);

	 pressure = lps22hb.readPressure() ;
	 DEBUG_STREAM.println("Pressure: " + (String)pressure);
	 message[cursor++] = pressure >> 8;
	 message[cursor++] = pressure;

	 uint32_t start = millis();
	 uint32_t timeout = 1UL * 60 * 100; // 1 min timeout <<<<<< Change to 2 back ; 2UL * 60 * 1000

	 if (sodaq_gps.scan(true, timeout)) {

		 lat = sodaq_gps.getLat() * 100000;
		 lon = sodaq_gps.getLon() * 100000;
     last_fix_dt = sodaq_gps.getDateTimeString();
     yy = sodaq_gps.getYear();
     MM = sodaq_gps.getMonth();
     dd = sodaq_gps.getDay();
     hh = sodaq_gps.getHour();
     mm = sodaq_gps.getMinute();
     ss = sodaq_gps.getSecond();
     
     String fixStr = "La"+ String(lat).substring(0,6) + "Lo"+ String(lon).substring(0,6);
     PrintStringAt(fixStr,0,0);
	 }
	 else {
		 PrintStringAt("No Fix!         ",0,0);
	 } 
  
	 message[cursor++] = lat >> 24;
	 message[cursor++] = lat >> 16;
	 message[cursor++] = lat >> 8;
	 message[cursor++] = lat;

	
	 message[cursor++] = lon >> 24;
	 message[cursor++] = lon >> 16;
	 message[cursor++] = lon >> 8;
	 message[cursor++] = lon;

// start extension
   delay(100);
   
   message[cursor++] = yy >> 8;
   message[cursor++] = yy;
   message[cursor++] = MM >> 8;
   message[cursor++] = MM;
   message[cursor++] = dd >> 8;
   message[cursor++] = dd;
   message[cursor++] = hh >> 8;
   message[cursor++] = hh;
   message[cursor++] = mm >> 8;
   message[cursor++] = mm;
   message[cursor++] = ss >> 8;
   message[cursor++] = ss;

   delay(100);
   nbiot.getRSSIAndBER(&rssi, &ber);
   PrintStringAt(getRSSstr(rssi) ,0,1);
   att_rssi = rssi;
   message[cursor++] = att_rssi >> 8;
   message[cursor++] = att_rssi;
   delay(100);
  
// end extension  

	 // Send the message
   PrintStringAt("Sending message ",0,0);
   // Print the message we want to send to debug stream
   print_message(message,cursor);
   PrintStringAt("C:*",13,1);
	 nbiot.sendMessage(message, cursor);
   // Wait some time between messages
   // show some things while waiting
   delay(2000); // 1000 = 1 sec
   PrintStringAt("Delay.....        ",0,0);
   PrintStringAt("C:Y",13,1);
   delay(20000); // 1000 = 1 sec
   PrintStringAt("fix: " + last_fix_dt.substring(4),0,0); //skip the year
   nbiot.getRSSIAndBER(&rssi, &ber);
   PrintStringAt(getRSSstr(rssi) ,0,1);
   delay(20000); // 1000 = 1 sec
   nbiot.getRSSIAndBER(&rssi, &ber);
   PrintStringAt(getRSSstr(rssi) ,0,1);
   PrintStringAt(fixStr,0,0);
	 delay(20000); // 1000 = 1 sec
 }

 void do_flash_led(int pin)
 {
	 for (size_t i = 0; i < 2; ++i) {
		 delay(100);
		 digitalWrite(pin, LOW);
		 delay(100);
		 digitalWrite(pin, HIGH);
	 }
 }

void PrintStringAt(String Text,int col=0, int row=0)
{  
  lcd.setCursor(col, row);
  int i =0;
  while ( i < Text.length()) {
    lcd.print(Text[i]);
    i++;
  }
  DEBUG_STREAM.println(Text);
}

String getRSSstr(int8_t int_rssi) {
  
  if (int_rssi<-99) {
    return "RSSI:" + (String)int_rssi + " dB";
  }
  else {
    return "RSSI:" + (String)int_rssi + "  dB";
  } 
}

void print_message(byte l_message[],int l_cursor) {
   // Print the message we want to send
   // DEBUG_STREAM.println(message);
   for (int i = 0; i < l_cursor; i++) {
     if (l_message[i] < 16) {
       DEBUG_STREAM.print("0");
     }
     DEBUG_STREAM.print(l_message[i], HEX);
     if (i< l_cursor-1) {
       DEBUG_STREAM.print(":");
     }
   }
   DEBUG_STREAM.println();
}
