#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include<DHT.h>

# define DHTPIN 26
# define DHTTYPE DHT22

DHT dht(DHTPIN,DHTTYPE); // Object declaraion


void setup() {

  
  dht.begin();  // Iniialize sensor
  delay(2000); // to get accurate values

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.print("Temp : ");
  Serial.print(temp);
  Serial.print(" °C");
  Serial.print(" ");
  Serial.print("Humidity : ");
  Serial.print(humidity);
  Serial.println("%");

  delay(2000);
}