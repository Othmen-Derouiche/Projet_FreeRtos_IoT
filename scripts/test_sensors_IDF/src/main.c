#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define DISTANCE_TARGET 50 // Distance de détection cible en centimètres

volatile bool target_reached = false;
SemaphoreHandle_t distance_semaphore;

#define TRIGGER_GPIO 33
#define ECHO_GPIO 32

#define LED_GPIO 13
#define TXD_PIN 17
#define RXD_PIN 16
#define RX_BUF_SIZE 1024

#define COM_RXD_PIN 26
#define COM_TXD_PIN 27
#define COM_BUF_SIZE 1024

// Global varaibles : 
uint16_t ppm ;
double distance ;

// Tâche pour mesurer la distance et l'afficher
void distance_task(void *pvParameter) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); 

        gpio_set_level(TRIGGER_GPIO, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIGGER_GPIO, 0);

        // Attendre la réponse du capteur
        while (gpio_get_level(ECHO_GPIO) == 0) {}
        int64_t start = esp_timer_get_time();
        while (gpio_get_level(ECHO_GPIO) == 1) {}
        int64_t end = esp_timer_get_time();

        // Calculer la durée en microsecondes
        int64_t duration = end - start;

        // Calculer la distance en centimètres
        distance = (double)duration * 0.0343 / 2.0;

        // Afficher la distance mesurée
        printf("Distance: %.2f cm\n", distance);

        // Vérifier si la distance mesurée est proche de la distance cible
        if (distance < DISTANCE_TARGET ) {
            printf("Cible atteinte à %.2f cm!\n", distance);
            target_reached = true;
        }
        else {
            target_reached = false;
        }
    }
}
void signal_led(void *pvParameter){
    while (1) {
        if (target_reached) {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(3000)); 
            gpio_set_level(LED_GPIO, 0);
            target_reached = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay before checking again
    } 
}
//********************************************************************************************//
const uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};
//********************************************************************************************//
void init_uart1() {
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
//********************************************************************************************//
void init_uart2(){
    uart_driver_install(UART_NUM_2, COM_BUF_SIZE * 2, COM_BUF_SIZE * 2, 20, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, COM_TXD_PIN, COM_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);   
}
//********************************************************************************************//
void read_co2_concentration() {
    // Valeurs des bytes de la trame
  
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    uint8_t rx_buffer[9];
    uint8_t rx_bytes , tx_bytes;


    //uart_write_bytes(UART_NUM_1, "\xFF\x01\x99\x00\x00\x00\x13\x88\xCB", 9);
    // Send command
    tx_bytes = uart_write_bytes(UART_NUM_1,cmd, 9);

    // Wait for response
    //vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for the response to be available


    rx_bytes = uart_read_bytes(UART_NUM_1, rx_buffer, 9, 1000 / portTICK_PERIOD_MS);

    printf("Received %d bytes from UART\n", rx_bytes);
    for (int i = 0; i < rx_bytes; i++) {
        printf("%02X ", rx_buffer[i]);
    }
    printf("\n");

    // Check if enough bytes were received
    if (rx_bytes == 9) {
        // Calculate checksum
        uint8_t checksum = 0;
        for (int i = 1; i < 8; i++) {
            checksum += rx_buffer[i];
        }
        checksum = 0xFF - checksum;
        checksum += 1;
        // Verify checksum
        if (rx_buffer[8] == checksum) {
            // Calculate CO2 concentration
            uint16_t high_byte = rx_buffer[2];
            uint16_t low_byte = rx_buffer[3];
            ppm = (high_byte << 8) | low_byte;

            // Print concentration
    
            printf("CO2 concentration: %d ppm\n", ppm);
        } else {
            printf("Error: Incorrect checksum\n");
        }
    } else {
        printf("Error: Not enough bytes received\n");
    }

}
//********************************************************************************************//
void co2_task(void *pvParameters) {
    while (1) {
        read_co2_concentration();
        uart_write_bytes(UART_NUM_2, (char*)&ppm, sizeof(uint16_t));
        vTaskDelay(4000 / portTICK_PERIOD_MS); // Wait 4 seconds
    }
}
//********************************************************************************************//
void app_main() {

    gpio_config_t io_conf;
    // Configurer le pin Trigger comme output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TRIGGER_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configurer le pin Echo comme input
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ECHO_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure the pin as output
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // You can disable pull-up resistor
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // You can disable pull-down resistor
    io_conf.intr_type = GPIO_INTR_DISABLE; // You can enable interrupts if needed

    gpio_config(&io_conf);

    /********************************************************************************************/
    init_uart1();
    init_uart2();
    xTaskCreate(distance_task, "distance_task", 2048, NULL, 5, NULL);
    xTaskCreate(co2_task, "co2_task", 4096, NULL, 5, NULL);
    xTaskCreate(signal_led, "led_task", 2048, NULL, 5, NULL);

   /********************************************************************************************/

    

   /********************************************************************************************/
} 
