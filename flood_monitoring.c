#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"

// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6

#define ADC_JOYSTICK_Y 26
#define ADC_JOYSTICK_X 27

#define ADC_JOYSTICK_Y_CHANNEL 0
#define ADC_JOYSTICK_X_CHANNEL 1

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define I2C_ADDRESS 0x3C

typedef struct
{
    uint16_t water_level; // Simula o nível da água no eixo X
    uint16_t rain_volume;  // Simula o volume de chuva no eixo Y
} joystick_data_t;

QueueHandle_t xQueueJoystickData;

bool alert_mode = false; // Variável global para o modo de alerta

void gpio_irq_handler(uint gpio, uint32_t events);
void vJoystickTask(void *params);
void vDisplayTask(void *params);

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
    xTaskCreate(vDisplayTask, "Display Task", 256, NULL, 1, NULL);

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
        joystick_data.rain_volume = adc_read();

        adc_select_input(ADC_JOYSTICK_X_CHANNEL); // GPIO 27 = ADC1
        joystick_data.water_level = adc_read();

        printf("Water Level: %d, Rain Volume: %d\n", joystick_data.water_level, joystick_data.rain_volume);

        if ((joystick_data.water_level >= 4095 * 0.7) || (joystick_data.rain_volume >= 4095 * 0.8)) {
            printf("Alert mode!\n");
            alert_mode = true; // Ativa o modo de alerta
        } else {
            alert_mode = false; // Desativa o modo de alerta
        }

        xQueueSend(xQueueJoystickData, &joystick_data, portMAX_DELAY); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(200));                        // 10 Hz de leitura
    }
}

void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, I2C_ADDRESS, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    joystick_data_t joystick_data;

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joystick_data, portMAX_DELAY) == pdTRUE)
        {
            char water_level_str[12];
            char rain_volume_str[12];
            snprintf(water_level_str, sizeof(water_level_str), "Water: %d", joystick_data.water_level);
            snprintf(rain_volume_str, sizeof(rain_volume_str), "Rain: %d", joystick_data.rain_volume);
            ssd1306_draw_string(&ssd, water_level_str, 2, 2);
            ssd1306_draw_string(&ssd, rain_volume_str, 2, 12);

            if (alert_mode)
            {
                ssd1306_draw_string(&ssd, "ALERT!", 2, 22);
            }
            else
            {
                ssd1306_draw_string(&ssd, "Safe  ", 2, 22);
            }

            ssd1306_send_data(&ssd);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a tela a cada 100ms
    }
}