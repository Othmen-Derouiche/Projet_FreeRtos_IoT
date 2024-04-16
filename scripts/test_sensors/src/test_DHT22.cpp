#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include<DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

# define DHTPIN 26
# define DHTTYPE DHT22

const byte DHT_SUCCESS = 0;        // Pas d'erreur
const byte DHT_TIMEOUT_ERROR = 1;  // Temps d'attente dépassé
const byte DHT_CHECKSUM_ERROR = 2; // Données reçues erronées

DHT dht(DHTPIN,DHTTYPE); // Object declaraion

void readDhtTask(void *pvParameters) {
  (void)pvParameters; 

  for (;;) {

    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();

    Serial.print("Temp : ");
    Serial.print(temp);
    Serial.print(" °C ");
    Serial.print("Humidity : ");
    Serial.print(humidity);
    Serial.println("%");

    vTaskDelay(pdMS_TO_TICKS(5000)); 
  }
}
void setup() {

  
  dht.begin();  // Iniialize sensor
  delay(2000); // to get accurate value
  Serial.begin(115200);

    xTaskCreate(readDhtTask, 
              "ReadDisplayTask", 
              4096, // Taille de la pile (en mots)
              NULL, // Paramètres de la tâche
              1, // Priorité de la tâche
              NULL); // Gestionnaire de tâches
}

void loop() {
  /*
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
  
  */
  
}