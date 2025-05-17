#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

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

#define RED_LED_PIN 13

#define BUZZER_A_PIN 21
#define BUZZER_B_PIN 10
#define BUZZER_FREQUENCY 200

const float DIVIDER_PWM = 16.0;          
const uint16_t PERIOD = 4096;

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
void vRedLedTask(void *params);
void vBuzzerTask(void *params);
void pwm_init_buzzer(uint pin);
void play_note(uint pin, int frequency, int duration);

int main()
{
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    gpio_put(RED_LED_PIN, false); // Desliga o LED vermelho

    pwm_init_buzzer(BUZZER_A_PIN);
    pwm_init_buzzer(BUZZER_B_PIN);

    adc_init();
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);

    stdio_init_all();
    sleep_ms(1000); // tempo para terminal abrir via USB

    xQueueJoystickData = xQueueCreate(10, sizeof(joystick_data_t)); // Cria a fila para os dados do joystick

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 256, NULL, 1, NULL);
    xTaskCreate(vRedLedTask, "Red LED Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);

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
            int water_level_percentage = (joystick_data.water_level * 100) / 4095;
            int rain_volume_percentage = (joystick_data.rain_volume * 100) / 4095;
            printf("Water Level: %d%%, Rain Volume: %d%%\n", water_level_percentage, rain_volume_percentage);
            snprintf(water_level_str, sizeof(water_level_str), "Water: %d%%", water_level_percentage);
            snprintf(rain_volume_str, sizeof(rain_volume_str), "Rain: %d%%", rain_volume_percentage);
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

void vRedLedTask(void *params)
{
    while (true)
    {
        if (alert_mode)
        {
            gpio_put(RED_LED_PIN, true); // Liga o LED vermelho
        }
        else
        {
            gpio_put(RED_LED_PIN, false); // Desliga o LED vermelho
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 100ms
    }
}

void vBuzzerTask(void *params)
{
    while (true)
    {
        if (alert_mode)
        {
            play_note(BUZZER_A_PIN, 1000, 500);
            play_note(BUZZER_B_PIN, 2000, 500);
            play_note(BUZZER_A_PIN, 1843, 500);
            play_note(BUZZER_B_PIN, 2089, 500);
            play_note(BUZZER_A_PIN, 4023, 500);
            play_note(BUZZER_B_PIN, 2239, 500);
        }
        else
        {
            pwm_set_gpio_level(BUZZER_A_PIN, 0);
            pwm_set_gpio_level(BUZZER_B_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo antes de tocar novamente
    }
}

void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(pin, 0);
}

void play_note(uint pin, int frequency, int duration) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint32_t wrap = 4095;
    float divider = (float) clock_get_hz(clk_sys) / (frequency * (wrap + 1));
    pwm_set_clkdiv(slice_num, divider);
    uint16_t level = (uint16_t)(((wrap + 1) * 50) / 100); // 50% duty cycle
    pwm_set_gpio_level(pin, level);
    pwm_set_enabled(slice_num, true);

    sleep_ms(duration);

    pwm_set_enabled(slice_num, false);
}