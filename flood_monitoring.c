#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/adc.h"

// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6

#define ADC_JOYSTICK_Y 26
#define ADC_JOYSTICK_X 27

#define ADC_JOYSTICK_Y_CHANNEL 0
#define ADC_JOYSTICK_X_CHANNEL 1

typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

QueueHandle_t xQueueJoystickData;

void gpio_irq_handler(uint gpio, uint32_t events);
void vJoystickTask(void *params);

int main()
{
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    adc_init();
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);

    stdio_init_all();
    sleep_ms(1000); // tempo para terminal abrir via USB

    xQueueJoystickData = xQueueCreate(10, sizeof(joystick_data_t)); // Cria a fila para os dados do joystick

   // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);

    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

void vJoystickTask(void *params)
{
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_Y);

    joystick_data_t joystick_data;

    while (true)
    {
        adc_select_input(ADC_JOYSTICK_Y_CHANNEL); // GPIO 26 = ADC0
        joystick_data.y_pos = adc_read();

        adc_select_input(ADC_JOYSTICK_X_CHANNEL); // GPIO 27 = ADC1
        joystick_data.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joystick_data, portMAX_DELAY); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(200));                        // 10 Hz de leitura
    }
}