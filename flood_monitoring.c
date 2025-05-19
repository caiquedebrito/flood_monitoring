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
#include "hardware/pio.h"
#include "ws2812.pio.h"

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

#define WS2812_PIN 7

#define LED_COUNT 25

const float DIVIDER_PWM = 16.0;          
const uint16_t PERIOD = 4096;

typedef struct
{
    uint16_t water_level; // Simula o nível da água no eixo X
    uint16_t rain_volume;  // Simula o volume de chuva no eixo Y
} joystick_data_t;

QueueHandle_t xQueueJoystickData;
QueueHandle_t xQueueAlertModeData;

struct pixel_t {
  uint8_t R, G, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

// Declaração do buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

const bool matrix_alert_draw[] = {
    10, 10, 10, 10, 10,
    10, 10, 0, 10, 10,
    10, 0, 10, 0, 10,
    10, 10, 0, 10, 10,
    10, 10, 10, 10, 10,
};

void gpio_irq_handler(uint gpio, uint32_t events);
void vJoystickTask(void *params);
void vDisplayTask(void *params);
void vRedLedTask(void *params);
void vBuzzerTask(void *params);
void vMatrixTask(void *params);
void pwm_init_buzzer(uint pin);
void play_note(uint pin, int frequency, int duration);
void npInit(uint pin);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();

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
    xQueueAlertModeData = xQueueCreate(1, sizeof(bool)); // Cria a fila para o modo de alerta

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 256, NULL, 1, NULL);
    xTaskCreate(vRedLedTask, "Red LED Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);
    xTaskCreate(vMatrixTask, "Matrix Task", 256, NULL, 1, NULL);

    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

/**
 * @brief Tarefa responsável por processar os comandos do joystick.
 *
 * Esta função implementa a lógica de controle associada às entradas do joystick,
 * sendo normalmente executada como uma tarefa em sistemas embarcados. 
 * Ela processa os dados provenientes do joystick e realiza as ações necessárias 
 * para atualizar o estado do sistema de monitoramento.
 *
 * @param params Ponteiro para os parâmetros da tarefa, que podem incluir configurações 
 *               iniciais ou dados relevantes ao processamento dos comandos do joystick.
 */
void vJoystickTask(void *params)
{
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_Y);

    joystick_data_t joystick_data;
    bool alert;

    while (true)
    {
        adc_select_input(ADC_JOYSTICK_Y_CHANNEL); // GPIO 26 = ADC0
        joystick_data.rain_volume = adc_read();

        adc_select_input(ADC_JOYSTICK_X_CHANNEL); // GPIO 27 = ADC1
        joystick_data.water_level = adc_read();

        printf("Water Level: %d, Rain Volume: %d\n", joystick_data.water_level, joystick_data.rain_volume);

        bool alerta = 
            (joystick_data.water_level >= 4095 * 0.7) ||
            (joystick_data.rain_volume  >= 4095 * 0.8);

        printf("Alert: %s\n", alerta ? "ON" : "OFF");

        xQueueOverwrite(xQueueAlertModeData, &alerta);

        xQueueSend(xQueueJoystickData, &joystick_data, portMAX_DELAY); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(200));                        // 10 Hz de leitura
    }
}

/**
 * @brief Função da tarefa responsável por atualizar o display.
 *
 * Esta função foi projetada para ser executada como uma tarefa dedicada ao gerenciamento
 * e atualização do display no sistema de monitoramento de enchentes. Ela processa os parâmetros
 * de entrada fornecidos à tarefa, os quais podem incluir configurações ou recursos compartilhados
 * necessários para o funcionamento do display.
 *
 * @param params Ponteiro para os parâmetros específicos da tarefa utilizados durante a execução.
 *
 * @note Esta função é destinada a ser utilizada como uma tarefa em um sistema operacional de tempo real (RTOS).
 */
/**
 * @brief Task function to handle display updates.
 *
 * This function is designed to run as a dedicated task for managing and updating the display
 * in the flood monitoring system. It processes the input parameters provided to the task,
 * which may include configuration settings or shared resources needed for display operation.
 *
 * @param params Pointer to task-specific parameters used during execution.
 *
 * @note This function is intended to be used as a task in a real-time operating system (RTOS)
 *       environment.
 */
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
    bool alert;

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

            if (xQueuePeek(xQueueAlertModeData, &alert, 0) == pdTRUE && alert)
            {
                ssd1306_draw_string(&ssd, "ALERTA!!!", 10, 22);
            }
            else
            {
                ssd1306_draw_string(&ssd, "Safe     ", 10, 22);
            }

            ssd1306_send_data(&ssd);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a tela a cada 100ms
    }
}

// Tarefa para controlar o LED vermelho
void vRedLedTask(void *params)
{
    bool alert;
    while (true)
    {
        if (xQueuePeek(xQueueAlertModeData, &alert, 0) == pdTRUE && alert) {
            gpio_put(RED_LED_PIN, true); // Liga o LED vermelho
        } else {
            gpio_put(RED_LED_PIN, false); // Desliga o LED vermelho
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 100ms
    }
}

// Tarefa para controlar o buzzer
void vBuzzerTask(void *params)
{
    bool alert;
    while (true)
    {
        if (xQueuePeek(xQueueAlertModeData, &alert, 0) == pdTRUE && alert)
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

// Tarefa para controlar a matriz de LEDs
void vMatrixTask(void *params)
{
    npInit(WS2812_PIN); // Inicializa a matriz de LEDs no pino 16

    bool alert;
    while (true)
    {
        if (xQueuePeek(xQueueAlertModeData, &alert, 0) == pdTRUE && alert) {
            for (uint i = 0; i < LED_COUNT; ++i) {
                if (matrix_alert_draw[i]) {
                    npSetLED(i, 255, 0, 0); // Define a cor vermelha para os LEDs
                } else {
                    npSetLED(i, 0, 0, 0); // Limpa o LED
                }
            }
        } else {
            npClear(); // Limpa a matriz de LEDs
        }
        npWrite(); // Atualiza a matriz de LEDs
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 100ms
    }
}

/**
 * @brief Inicializa o PWM para o buzzer.
 *
 * Configura o PWM no pino especificado para o controle do buzzer, possibilitando a ativação
 * de sinais sonoros conforme a aplicação.
 *
 * @param pin Número do pino utilizado para a configuração do PWM.
 */
void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(pin, 0);
}

/**
 * @brief Reproduz uma nota musical.
 *
 * Esta função envia um sinal para o pino especificado com a frequência e duração definidas,
 * permitindo assim a reprodução de uma nota.
 *
 * @param pin O pino de saída para emitir o sinal.
 * @param frequency A frequência da nota em Hertz.
 * @param duration A duração da nota em milissegundos.
 */
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

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

// Função para converter valores (0.0 a 1.0) em uma cor 32-bit (RGB)
uint32_t matrix_rgb(uint8_t intensity) {
    // Converte o valor de intensidade para 0-255
    unsigned char I = intensity * 255;
    // Retorna a cor no formato 0x00RRGGBB (usaremos apenas R, por exemplo)
    return (I << 16) | (I << 8) | I;
}