# Flood Monitoring

## Objetivo Geral
O projeto **Flood Monitoring** tem como objetivo construir um sistema embarcado de alerta de enchentes que simula, via joystick, os níveis de água e volume de chuva, exibindo informações em um display OLED, acionando sinalizações visuais e sonoras e permitindo entrada em modo de bootloader via botão dedicado.

---

## Descrição Funcional
- **Modo Normal**  
  - Lê continuamente as posições X e Y do joystick (nível de água e volume de chuva).  
  - Exibe no OLED: “Water: XX%” e “Rain: YY%”.  
  - LED RGB e buzzer permanecem desligados; a matriz de LEDs fica limpa.  
- **Modo Alerta**  
  - Ativado automaticamente quando  
    - Nível da água ≥ 70% **ou**  
    - Volume de chuva ≥ 80%.  
  - Indicações:  
    - **OLED** exibe “ALERTA!!!” em destaque.  
    - **LED RGB** (canal vermelho) acende continuamente.  
    - **Buzzer** toca sequência de notas.  
    - **Matriz de 5×5 WS2812** desenha símbolo de perigo em vermelho.  

---

## Requisitos de Hardware
- **Raspberry Pi Pico** ou outro RP2040 compatível  
- **Joystick** com duas saídas analógicas (e potenciômetros X/Y)  
- **Botão B** (GPIO 6) para BOOTSEL  
- **Display OLED 128×64** (SSD1306 via I2C)  
- **Matriz de LEDs WS2812** 5×5 (25 LEDs)  
- **LED RGB** (apenas canal vermelho usado, GPIO 13)  
- **Dois buzzers** piezoelétricos (GPIO 21 e GPIO 10)  

---

## Uso dos Periféricos da BitDogLab

### 1. Potenciômetro do Joystick  
- **Eixo X (ADC1, GPIO 27):** simula o nível de água (0–4095 → 0–100 %).  
- **Eixo Y (ADC0, GPIO 26):** simula o volume de chuva (0–4095 → 0–100 %).  
- Leitura em 10 Hz na task `vJoystickTask`.

### 2. Botões  
- **Botão B (GPIO 6):**  
  - Mapeado para entrar em BOOTSEL (modo USB-mass-storage).  
  - Configuração: `gpio_pull_up()`, `gpio_set_irq_enabled_with_callback(..., GPIO_IRQ_EDGE_FALL, true, ...)`.  
  - Ao pressionar, chama `reset_usb_boot(0,0)` no handler.

### 3. Display OLED  
- **Interface:** I2C1 (SDA→GPIO 14, SCL→GPIO 15), velocidade 400 kHz, endereço `0x3C`.  
- **Driver:** `lib/ssd1306`  
- **Task:** `vDisplayTask`  
  - Inicializa com `ssd1306_init()` e `ssd1306_config()`.  
  - A cada 100 ms:  
    - Recebe do RTOS queue (`xQueueJoystickData`) os valores de água e chuva.  
    - Converte em porcentagem.  
    - Desenha no display as strings “Water: XX%”, “Rain: YY%” e “ALERTA!!!” ou “Safe”.  
    - Chama `ssd1306_send_data()`.

### 4. Matriz de LEDs (WS2812)  
- **Pino:** GPIO 7  
- **PL:** PIO0 ou PIO1, usando o programa `ws2818b_program`.  
- **Pixels:** `LED_COUNT = 25` (5×5).  
- **Task:** `vMatrixTask`  
  - Em `npInit()`: carrega o programa no PIO, configura frequência 800 kHz e limpa buffer.  
  - No modo alerta: itera `matrix_alert_draw[]` para acender o desenho de perigo em vermelho.  
  - Senão: limpa (`npClear()`).  
  - Atualiza via `npWrite()` a cada 100 ms.

### 5. LED RGB  
- **Canal vermelho:** GPIO 13  
- **Task:** `vRedLedTask`  
  - Lê fila de alerta (`xQueueAlertModeData`).  
  - Acende (`gpio_put(13, true)`) se alerta; apaga caso contrário.  
  - Atualiza a cada 100 ms.

### 6. Buzzer  
- **Pinos:** BUZZER_A → GPIO 21, BUZZER_B → GPIO 10  
- **Init:** `pwm_init_buzzer(pin)` → configura slice e divisor para `BUZZER_FREQUENCY` (200 Hz)  
- **Task:** `vBuzzerTask`  
  - Se alerta: executa sequência de notas via `play_note(pin, freq, duration)` alternando A/B.  
  - Caso contrário: zera nível PWM (`pwm_set_gpio_level(pin, 0)`).  
  - Repetição a cada 1 s.

### 7. Interrupções & Debounce  
- A única interrupção é no **Botão B**, tratada por `gpio_set_irq_enabled_with_callback`.  
- **Debounce:** não implementado. Se for necessário, recomenda-se descartar eventos adicionais num período de ~200 ms dentro do handler ou usar um temporizador para filtragem.

---

## Instruções de Compilação e Upload

1. Instale o **Pico SDK** e o **FreeRTOS** conforme guia oficial.  
2. Coloque este código em `main.c` dentro do seu projeto CMake.  
3. No `CMakeLists.txt`, inclua as bibliotecas de ADC, I²C, PIO e PWM.  
4. Gere build e grave no Pico:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   picotool load flood_monitoring.uf2
