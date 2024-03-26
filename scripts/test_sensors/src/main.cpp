#include <Arduino.h>
#include<DHT.h>

# define DHTPIN 26
# define DHTTYPE DHT22

DHT dht(DHTPIN,DHTTYPE); // Object declaraion


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  dht.begin();  // Iniialize sensor
  delay(2000); // to get accurate values

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.print("Temp : ");
  Serial.print(temp);
  Serial.print(" °C");
  Serial.print("Humidity : ");
  Serial.print(humidity);
  Serial.println("%");
  delay(2000);

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}