/*
 * ====================================================================
 * SISTEMA IoT PARA AUTOMA��O RESIDENCIAL- LarConectado 2.0 - RASPBERRY PI PICO W
 * ====================================================================
 * 
 * Descri��o: Sistema completo de automa��o residencial utilizando 
 * Raspberry Pi Pico W com conectividade Wi-Fi e protocolo MQTT.
 * 
 * Funcionalidades:
 * - Controle de ilumina��o de 5 c�modos via MQTT
 * - Sistema de alarme com sensores ultrass�nicos
 * - Display OLED para feedback visual
 * - Matriz de LEDs 5x5 representando os c�modos
 * - Sensores de luminosidade e proximidade
 * - Interface via joystick e bot�o
 * - Publica��o de dados dos sensores via MQTT
 * 
 * Autor: Mateus Moreira da Silva
 * Data: 02/06/2025
 * ====================================================================
 */

/* ========== INCLUDES - BIBLIOTECAS DO SISTEMA ========== */

// Bibliotecas padr�o do C
#include <stdio.h>               // Fun��es padr�o de entrada/sa�da
#include <string.h>              // Fun��es para manipula��o de strings
#include <stdlib.h>              // Aloca��o de mem�ria e outras utilidades

// Bibliotecas espec�ficas do Raspberry Pi Pico
#include "pico/stdlib.h"         // Biblioteca padr�o da Raspberry Pi Pico
#include "pico/cyw43_arch.h"     // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"      // Biblioteca para trabalhar com ID �nico da placa

// Bibliotecas de hardware
#include "hardware/gpio.h"       // Biblioteca de hardware de GPIO
#include "hardware/irq.h"        // Biblioteca de hardware de interrup��es
#include "hardware/adc.h"        // Biblioteca de hardware para convers�o ADC
#include "hardware/i2c.h"        // Interface I2C
#include "hardware/pio.h"        // Fun��es de I/O program�vel
#include "hardware/clocks.h"     // Fun��es de controle de clock

// Bibliotecas de rede e MQTT
#include "lwip/apps/mqtt.h"      // Biblioteca LWIP MQTT - conex�o MQTT
#include "lwip/apps/mqtt_priv.h" // Biblioteca para gera��o de conex�es
#include "lwip/dns.h"            // Biblioteca com suporte DNS
#include "lwip/altcp_tls.h"      // Biblioteca para conex�es seguras usando TLS

// Bibliotecas customizadas do projeto
#include "inc/ssd1306.h"         // Driver para display OLED
#include "inc/font.h"            // Defini��es de fontes para o display
#include "animacoes_led.pio.h"   // Programa PIO para anima��es de LED

/* ========== CONFIGURA��ES DE REDE ========== */

#define WIFI_SSID "*******"                  // Nome da rede Wi-Fi
#define WIFI_PASSWORD "*******"              // Senha da rede Wi-Fi
#define MQTT_SERVER "***********"            // Endere�o do broker MQTT
#define MQTT_USERNAME "******"               // Nome de usu�rio MQTT
#define MQTT_PASSWORD "***"                  // Senha do MQTT

/* ========== DEFINI��ES DE HARDWARE ========== */

// === Configura��o da matriz de LEDs ===
#define NUM_PIXELS 25           // N�mero de LEDs na matriz 5x5
#define matriz_leds 7           // Pino de sa�da para a matriz

// === Configura��o I2C para o display OLED ===
#define I2C_PORT i2c1           // Porta I2C utilizada
#define I2C_SDA 14              // Pino SDA (dados I2C)
#define I2C_SCL 15              // Pino SCL (clock I2C)
#define ENDERECO 0x3C           // Endere�o I2C do display
#define WIDTH 128               // Largura do display em pixels
#define HEIGHT 64               // Altura do display em pixels

// === Defini��o dos pinos dos LEDs RGB ===
#define LED_PIN CYW43_WL_GPIO_LED_PIN  // GPIO do chip CYW43 (LED onboard)
#define LED_BLUE_PIN 12                // GPIO12 - LED azul
#define LED_GREEN_PIN 11               // GPIO11 - LED verde
#define LED_RED_PIN 13                 // GPIO13 - LED vermelho

// === Pinos para o sensor ultrass�nico principal ===
#define TRIG_PIN 8              // Pino de trigger do sensor (controle de luzes)
#define ECHO_PIN 9              // Pino de echo do sensor (controle de luzes)

// === Pinos para o sensor ultrass�nico do alarme ===
#define TRIG_PIN_2 16           // Pino de trigger do sensor do alarme
#define ECHO_PIN_2 17           // Pino de echo do sensor do alarme

// === Outros componentes ===
#define BUZZER 21               // Pino do buzzer para alarme
#define ldr_pin 19              // Pino para o sensor de luz (LDR)

// === Pinos de entrada do joystick (ADC) ===
#define ADC_JOYSTICK_X 26       // Pino ADC para eixo X do joystick
#define ADC_JOYSTICK_Y 27       // Pino ADC para eixo Y do joystick

// === Bot�es ===
#define Botao_A 5               // Pino do bot�o A

/* ========== VARI�VEIS GLOBAIS ========== */

// === Controle do PIO para matriz de LEDs ===
PIO pio;                       // Controlador PIO
uint sm;                       // State Machine do PIO
uint contagem = 5;             // Contador para exibi��o na matriz
ssd1306_t ssd;                 // Estrutura do display OLED

// === Vari�veis de controle ===
int tv = 0;                    // Controle de estado da TV
int tv_alarme = 0;             // Controle de estado do alarme no display
uint Eixo_x_value, Eixo_Y_value; // Valores dos eixos do joystick

// === Estados dos dispositivos (ligado/desligado) ===
bool estado_led_sala = false;     // Estado da luz da sala
bool estado_led_cozinha = false;  // Estado da luz da cozinha
bool estado_led_quarto = false;   // Estado da luz do quarto
bool estado_led_banheiro = false; // Estado da luz do banheiro
bool estado_led_quintal = false;  // Estado da luz do quintal
bool estado_display = false;      // Estado do display (TV)
bool estado_alarme = false;       // Estado do sistema de alarme
bool Alarme_Acionado = false;     // Indica se o alarme foi acionado

/* ========== CONFIGURA��ES MQTT ========== */

// Defini��o da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // 'C' para Celsius ou 'F' para Fahrenheit
#endif

// Verifica��o se o servidor MQTT est� definido
#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// Inclus�o de certificado para autentica��o cliente-servidor (se definido)
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

// Tamanho m�ximo do t�pico MQTT
#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

// === Estrutura de dados do cliente MQTT ===
typedef struct {
    mqtt_client_t* mqtt_client_inst;                    // Inst�ncia do cliente MQTT
    struct mqtt_connect_client_info_t mqtt_client_info;  // Informa��es de conex�o
    char data[MQTT_OUTPUT_RINGBUF_SIZE];                // Buffer de dados
    char topic[MQTT_TOPIC_LEN];                         // Buffer do t�pico
    uint32_t len;                                       // Comprimento dos dados
    ip_addr_t mqtt_server_address;                      // Endere�o IP do servidor
    bool connect_done;                                  // Flag de conex�o estabelecida
    int subscribe_count;                                // Contador de assinaturas
    bool stop_client;                                   // Flag para parar o cliente
} MQTT_CLIENT_DATA_T;

/* ========== MACROS DE DEBUG ========== */

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf     // Debug habilitado
#else
#define DEBUG_printf(...)       // Debug desabilitado
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf      // Mensagens informativas
#endif

#ifndef ERROR_printf
#define ERROR_printf printf     // Mensagens de erro
#endif

/* ========== CONFIGURA��ES DE TEMPORIZA��O MQTT ========== */

#define TEMP_WORKER_TIME_S 10   // Intervalo de coleta de temperatura (segundos)
#define MQTT_KEEP_ALIVE_S 60    // Keep alive MQTT (segundos)

// === Configura��es de QoS (Quality of Service) ===
// QoS 0 - At most once (no m�ximo uma vez)
// QoS 1 - At least once (pelo menos uma vez)
// QoS 2 - Exactly once (exatamente uma vez)
#define MQTT_SUBSCRIBE_QOS 1    // QoS para assinaturas
#define MQTT_PUBLISH_QOS 1      // QoS para publica��es
#define MQTT_PUBLISH_RETAIN 0   // Reten��o de mensagens

// === Configura��es de Last Will and Testament ===
#define MQTT_WILL_TOPIC "/online"  // T�pico para status online
#define MQTT_WILL_MSG "0"          // Mensagem de desconex�o
#define MQTT_WILL_QOS 1            // QoS para will message

// === Nome do dispositivo ===
#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// === Configura��o de t�picos �nicos ===
// Definir como 1 para adicionar o nome do cliente aos t�picos
// Suporte para m�ltiplos dispositivos no mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/* ========== PROT�TIPOS DE FUN��ES - TEMPERATURA ========== */

/**
 * L� a temperatura interna do microcontrolador
 * @param unit Unidade de temperatura ('C' ou 'F')
 * @return Temperatura lida
 */
static float read_onboard_temperature(const char unit);

/* ========== PROT�TIPOS DE FUN��ES - MQTT ========== */

/**
 * Callback para requisi��o de publica��o
 */
static void pub_request_cb(__unused void *arg, err_t err);

/**
 * Gera o t�pico MQTT completo
 */
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

/**
 * Controla o LED onboard
 */
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

/**
 * Publica a temperatura no MQTT
 */
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

/**
 * Callback para requisi��o de assinatura
 */
static void sub_request_cb(void *arg, err_t err);

/**
 * Callback para cancelamento de assinatura
 */
static void unsub_request_cb(void *arg, err_t err);

/**
 * Gerencia assinaturas de t�picos
 */
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

/**
 * Callback para dados MQTT recebidos
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

/**
 * Callback para publica��es MQTT recebidas
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

/**
 * Worker para publica��o peri�dica de temperatura
 */
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

/**
 * Callback de conex�o MQTT
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

/**
 * Inicializa o cliente MQTT
 */
static void start_client(MQTT_CLIENT_DATA_T *state);

/**
 * Callback com resultado da resolu��o DNS
 */
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

/* ========== PROT�TIPOS DE FUN��ES - HARDWARE ========== */

/**
 * Inicializa os GPIOs dos LEDs e sensores
 */
void gpio_led_bitdog(void);

/**
 * Controla a matriz de LEDs representando os c�modos
 */
void ligar_luz(void);

/**
 * Controla o display OLED
 */
void ligar_display(void);

/**
 * Envia pulso para o sensor ultrass�nico principal
 */
void send_trigger_pulse(void);

/**
 * Envia pulso para o sensor ultrass�nico do alarme
 */
void send_trigger_pulse_2(void);

/**
 * Mede dist�ncia usando sensor ultrass�nico gen�rico
 * @param trigger_pin Pino de trigger
 * @param echo_pin Pino de echo
 * @return Dist�ncia em cent�metros
 */
float measure_distance_cm(uint trigger_pin, uint echo_pin);

/**
 * Controla LEDs frontais baseado em sensores
 */
void luz_frente_controlada(void);

/**
 * Sistema de alarme principal
 */
void Alarme(void);

/**
 * Gera som de alarme no buzzer
 */
void Som_Alarme(void);

/**
 * Handler de interrup��o dos bot�es
 */
void gpio_irq_handler(uint gpio, uint32_t events);

/**
 * Publica estados dos dispositivos via MQTT
 */
void publish_topicos_states(MQTT_CLIENT_DATA_T *state);

/**
 * Publica dados dos sensores via MQTT
 */
void publish_topicos(MQTT_CLIENT_DATA_T *state);

/* ========== FUN��O PRINCIPAL ========== */

/**
 * Fun��o principal do sistema
 * Inicializa todos os componentes e executa o loop principal
 */
int main(void) {

    // === INICIALIZA��O DO SISTEMA ===
    
    // Inicializa todas as bibliotecas stdio padr�o
    stdio_init_all();
    INFO_printf("mqtt client starting\n");

    // === INICIALIZA��O DO ADC ===
    
    // Inicializa o conversor ADC para leitura de temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);  // Habilita sensor de temperatura interno
    adc_select_input(4);                // Seleciona canal 4 (temperatura)

    // === CONFIGURA��O DO I2C PARA DISPLAY OLED ===
    
    // Inicializa I2C a 400kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  // Configura pino SDA
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);  // Configura pino SCL
    gpio_pull_up(I2C_SDA);                      // Pull-up no SDA
    gpio_pull_up(I2C_SCL);                      // Pull-up no SCL
    
    // === INICIALIZA��O DO DISPLAY OLED ===
    
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);  // Limpa o display
    ssd1306_send_data(&ssd);

    // === INICIALIZA��O DOS GPIOS ===
    
    gpio_led_bitdog();  // Inicializa LEDs e sensores

    // === CONFIGURA��O DO PIO PARA MATRIZ DE LEDS ===
    
    pio = pio0;  // Usa PIO0
    uint offset = pio_add_program(pio, &animacoes_led_program);  // Adiciona programa
    sm = pio_claim_unused_sm(pio, true);  // Reivindica state machine
    animacoes_led_program_init(pio, sm, offset, matriz_leds);  // Inicializa programa

    // === CONFIGURA��O DE INTERRUP��ES ===
    
    // Configura interrup��o no bot�o A (borda de descida)
    gpio_set_irq_enabled_with_callback(Botao_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // === INICIALIZA��O DO CLIENTE MQTT ===
    
    static MQTT_CLIENT_DATA_T state;  // Cria estrutura de dados do cliente

    // Inicializa a arquitetura do CYW43 (Wi-Fi)
    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

    // === GERA��O DE ID �NICO DO DISPOSITIVO ===
    
    // Obt�m identificador �nico da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    
    // Converte para min�sculas
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome �nico do cliente (ex: pico1234)
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    // === CONFIGURA��O DO CLIENTE MQTT ===
    
    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
    
    // Configura��o de autentica��o (se definida)
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif

    // Configura��o de Last Will and Testament
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    // === CONFIGURA��O TLS (SE HABILITADO) ===
    
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS habilitado
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    
    // Configura��o para autentica��o m�tua (servidor e cliente)
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(
        ca_cert, sizeof(ca_cert),
        client_key, sizeof(client_key), 
        NULL, 0, 
        client_cert, sizeof(client_cert)
    );
    
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // === CONEX�O WI-FI ===
    
    // Habilita modo station (cliente Wi-Fi)
    cyw43_arch_enable_sta_mode();
    
    // Conecta � rede Wi-Fi com timeout de 30 segundos
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    // === RESOLU��O DNS E CONEX�O MQTT ===
    
    // Faz requisi��o DNS para o endere�o IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se j� tem o endere�o, inicia o cliente imediatamente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS significa aguardar callback
        panic("dns request failed");
    }

    // === VARI�VEIS DE CONTROLE DE TEMPO ===
    
    static absolute_time_t last_publish_time = {0};  // �ltima publica��o de estados
    static absolute_time_t last_sensor_time = {0};   // �ltima publica��o de sensores

    // === LOOP PRINCIPAL ===
    
    // Loop executado enquanto estiver conectado ao MQTT
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        
        // Processa eventos de rede
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));

        // Verifica se est� conectado antes de publicar
        if (state.connect_done && mqtt_client_is_connected(state.mqtt_client_inst)) {
            
            // Publica estados dos LEDs a cada 5 segundos
            if (absolute_time_diff_us(last_publish_time, get_absolute_time()) > 5000000) {
                publish_topicos_states(&state);
                last_publish_time = get_absolute_time();
            }
            
            // Publica dados dos sensores a cada 2 segundos
            if (absolute_time_diff_us(last_sensor_time, get_absolute_time()) > 2000000) {
                publish_topicos(&state);
                last_sensor_time = get_absolute_time();
            }
        }

        // === EXECU��O DAS FUN��ES PRINCIPAIS ===
        
        ligar_display();         // Atualiza o display OLED
        Alarme();               // Verifica e processa o sistema de alarme
        luz_frente_controlada(); // Controla LEDs frontais baseado em sensores

        // Pequena pausa para reduzir uso da CPU
        sleep_ms(100);
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}

/* ========== IMPLEMENTA��O - FUN��ES DE TEMPERATURA ========== */

/**
 * L� a temperatura interna do microcontrolador Raspberry Pi Pico
 * Utiliza o sensor de temperatura integrado e f�rmula de calibra��o
 */
static float read_onboard_temperature(const char unit) {
    
    // Garante que o ADC est� configurado corretamente (inicializa��o �nica)
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_set_temp_sensor_enabled(true);
        adc_initialized = true;
    }

    // Seleciona o canal 4 (sensor de temperatura interno)
    adc_select_input(4);

    // Convers�o de 12 bits, assume valor m�ximo == ADC_VREF == 3.3V
    const float conversionFactor = 3.3f / (1 << 12);
    uint16_t raw = adc_read();          // L� valor bruto do ADC
    float adc = (float)raw * conversionFactor;  // Converte para voltagem
    
    // F�rmula de calibra��o fornecida pelo fabricante
    // Temperatura = 27�C - (Voltagem - 0.706V) / 0.001721V/�C
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    // Verifica��o de sanidade dos valores lidos
    if (tempC < -40.0f || tempC > 150.0f) {
        ERROR_printf("Leitura inv�lida! RAW=%d, Voltage=%.3fV\n", raw, adc);
        return -999.9f;  // Retorna valor de erro
    }

    // Converte para Fahrenheit se solicitado
    if (unit == 'F') {
        return tempC * 9.0f/5.0f + 32.0f;
    }
    return tempC;
}

/* ========== IMPLEMENTA��O - FUN��ES MQTT ========== */

/**
 * Callback executado ap�s requisi��o de publica��o MQTT
 * Trata erros de publica��o
 */
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

/**
 * Gera o t�pico MQTT completo
 * Adiciona o ID do cliente se MQTT_UNIQUE_TOPIC estiver habilitado
 */
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;  // Retorna o nome original sem modifica��o
#endif
}

/**
 * Publica os estados atuais de todos os dispositivos via MQTT
 * Executado periodicamente para manter sincroniza��o
 */
void publish_topicos_states(MQTT_CLIENT_DATA_T *state) {
    
    // === PUBLICA��O DOS ESTADOS DAS LUZES ===
    
    // Publica estado da luz da sala
    mqtt_publish(state->mqtt_client_inst, "/luz_sala/state", 
                estado_led_sala ? "On" : "Off", 
                estado_led_sala ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // Publica estado da luz da cozinha
    mqtt_publish(state->mqtt_client_inst, "/luz_cozinha/state", 
                estado_led_cozinha ? "On" : "Off", 
                estado_led_cozinha ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // Publica estado da luz do quarto
    mqtt_publish(state->mqtt_client_inst, "/luz_quarto/state", 
                estado_led_quarto ? "On" : "Off", 
                estado_led_quarto ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // Publica estado da luz do banheiro
    mqtt_publish(state->mqtt_client_inst, "/luz_banheiro/state", 
                estado_led_banheiro ? "On" : "Off", 
                estado_led_banheiro ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // Publica estado da luz do quintal
    mqtt_publish(state->mqtt_client_inst, "/luz_quintal/state", 
                estado_led_quintal ? "On" : "Off", 
                estado_led_quintal ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // === PUBLICA��O DO ESTADO DO DISPLAY (TV) ===
    
    mqtt_publish(state->mqtt_client_inst, "/display/state", 
                estado_display ? "On" : "Off", 
                estado_display ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    
    // === PUBLICA��O DO ESTADO DO ALARME ===
    
    mqtt_publish(state->mqtt_client_inst, "/alarme/state", 
                estado_alarme ? "On" : "Off", 
                estado_alarme ? 2 : 3, 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// FUN��O PARA PUBLICAR DADOS DOS SENSORES
// Esta fun��o � respons�vel por publicar o status de sensores e o estado de acionamento do alarme.
// Ela � chamada periodicamente para atualizar o broker MQTT sobre o status do sistema.
void publish_topicos(MQTT_CLIENT_DATA_T *state) {
    char sensor_data[32]; // Buffer n�o utilizado aqui, mas pode ser �til para outras publica��es.

    // Publica o estado de acionamento do alarme no t�pico "/alarme/acionado".
    // Se Alarme_Acionado for true, publica "VIOLACAO". Caso contr�rio, publica "SEM VIOLACAO".
    // O QoS (Quality of Service) � definido por MQTT_PUBLISH_QOS (provavelmente 0 ou 1).
    // O flag '0' indica que a mensagem n�o � retida pelo broker (ou seja, novos assinantes n�o recebem a �ltima mensagem,
    // apenas as que s�o publicadas a partir do momento da assinatura).
    // 'pub_request_cb' � o callback para verificar se a publica��o foi bem-sucedida.
    if (Alarme_Acionado) {
        mqtt_publish(state->mqtt_client_inst, "/alarme/acionado",
                     "VIOLACAO", 8, // "VIOLACAO" tem 8 caracteres
                     MQTT_PUBLISH_QOS, 0, pub_request_cb, state);
    } else {
        mqtt_publish(state->mqtt_client_inst, "/alarme/acionado",
                     "SEM VIOLACAO", 12, // "SEM VIOLACAO" tem 12 caracteres
                     MQTT_PUBLISH_QOS, 0, pub_request_cb, state);
    }
}

// Publicar temperatura
// Esta fun��o l� a temperatura interna do chip RP2040 e a publica no t�pico MQTT.
// Ela utiliza uma vari�vel est�tica 'old_temperature' para evitar publica��es redundantes
// se a temperatura n�o mudou, otimizando o tr�fego MQTT.
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature; // Armazena a �ltima temperatura publicada para compara��o
    const char *temperature_key = full_topic(state, "/temperature"); // Constr�i o nome completo do t�pico (com prefixo �nico se configurado)
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS); // L� a temperatura do sensor interno

    // Publica apenas se a temperatura atual for diferente da �ltima publicada
    if (temperature != old_temperature) {
        old_temperature = temperature; // Atualiza a �ltima temperatura
        char temp_str[16]; // Buffer para formatar a temperatura como string
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature); // Formata a temperatura com 2 casas decimais
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key); // Imprime no console para depura��o

        // Publica a temperatura no t�pico `/temperature`.
        // 'MQTT_PUBLISH_RETAIN' � usado aqui, o que significa que o broker reter� a �ltima mensagem publicada
        // para este t�pico, garantindo que novos assinantes recebam o valor mais recente imediatamente.
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

// Requisi��o de Assinatura - subscribe
// Este � o callback chamado ap�s uma solicita��o de assinatura (subscribe) ser processada pelo broker.
// Ele verifica por erros e incrementa um contador de assinaturas ativas.
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg; // Converte o argumento gen�rico para a estrutura de dados do cliente
    if (err != 0) {
        panic("subscribe request failed %d", err); // Em caso de erro na assinatura, causa um panic (erro fatal)
    }
    state->subscribe_count++; // Incrementa o contador de t�picos assinados
}

// Requisi��o para encerrar a assinatura
// Este � o callback chamado ap�s uma solicita��o de desassinatura (unsubscribe) ser processada pelo broker.
// Ele verifica por erros e decrementa o contador de assinaturas.
// Se o cliente solicitou para parar e n�o h� mais assinaturas ativas, ele se desconecta do broker.
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg; // Converte o argumento gen�rico para a estrutura de dados do cliente
    if (err != 0) {
        panic("unsubscribe request failed %d", err); // Em caso de erro na desassinatura, causa um panic
    }
    state->subscribe_count--; // Decrementa o contador de t�picos assinados
    assert(state->subscribe_count >= 0); // Garante que o contador nunca seja negativo

    // Se o cliente solicitou para parar (state->stop_client == true)
    // e todas as desassinaturas foram processadas (subscribe_count <= 0),
    // ent�o o cliente se desconecta do broker MQTT.
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// T�picos de assinatura
// Esta fun��o gerencia as opera��es de assinatura (subscribe) ou desassinatura (unsubscribe) para uma lista de t�picos.
// � uma fun��o auxiliar para evitar repeti��o de c�digo ao lidar com m�ltiplos t�picos.
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    // Define qual callback usar (sub_request_cb para subscribe, unsub_request_cb para unsubscribe)
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;

    // Chama mqtt_sub_unsub para cada t�pico na lista, com o QoS definido por MQTT_SUBSCRIBE_QOS.
    // O 'sub' booleano determina se a opera��o � de assinatura (true) ou desassinatura (false).
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/luz_sala"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/luz_cozinha"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/luz_quarto"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/luz_banheiro"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/luz_quintal"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/display"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/alarme"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT
// Este � o callback principal para o tratamento de mensagens MQTT recebidas.
// Ele � chamado ap�s 'mqtt_incoming_publish_cb' e cont�m o payload da mensagem.
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;

    // Ajusta o t�pico b�sico se MQTT_UNIQUE_TOPIC estiver habilitado (remove o ID do cliente do t�pico)
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic; // Se n�o for �nico, o t�pico � o que foi recebido
#endif
    
    // Copia os dados (payload) da mensagem para o buffer 'state->data' e garante que seja uma string terminada em nulo
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data); // Imprime a mensagem recebida para depura��o

    // L�gica para processar comandos baseada no t�pico recebido:

    // Controle da luz da sala
    if (strcmp(basic_topic, "/luz_sala") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_led_sala = true; // Define o estado da luz da sala como ligado
            ligar_luz(); // Atualiza a matriz de LEDs
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_led_sala = false; // Define o estado da luz da sala como desligado
            ligar_luz(); // Atualiza a matriz de LEDs
        }
        // Confirma a mudan�a publicando o novo estado no t�pico de estado "/luz_sala/state".
        // A mensagem � retida (MQTT_PUBLISH_RETAIN) para que novos assinantes saibam o estado atual.
        mqtt_publish(state->mqtt_client_inst, "/luz_sala/state",
                     estado_led_sala ? "On" : "Off", // Payload: "On" ou "Off"
                     estado_led_sala ? 2 : 3, // Tamanho do payload (2 para "On", 3 para "Off")
                     MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);

    }
    // Controle da luz da cozinha
    else if (strcmp(basic_topic, "/luz_cozinha") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_led_cozinha = true;
            ligar_luz();
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_led_cozinha = false;
            ligar_luz();
        }
        // (Nota: Faltou a publica��o do estado para "/luz_cozinha/state", "/luz_quarto/state", etc.
        // � uma boa pr�tica publicar o estado ap�s receber um comando para manter a sincronia).
    }
    // Controle da luz do quarto
    else if (strcmp(basic_topic, "/luz_quarto") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_led_quarto = true;
            ligar_luz();
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_led_quarto = false;
            ligar_luz();
        }
    }
    // Controle da luz do banheiro
    else if (strcmp(basic_topic, "/luz_banheiro") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_led_banheiro = true;
            ligar_luz();
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_led_banheiro = false;
            ligar_luz();
        }
    }
    // Controle da luz do quintal
    else if (strcmp(basic_topic, "/luz_quintal") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_led_quintal = true;
            ligar_luz();
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_led_quintal = false;
            ligar_luz();
        }
    }
    // Controle do display (simulando a TV)
    else if (strcmp(basic_topic, "/display") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_display = true;
            ligar_display(); // Atualiza o display OLED
        }
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_display = false;
            ligar_display(); // Atualiza o display OLED
        }
    }
    // Controle do alarme (ativa/desativa)
    else if (strcmp(basic_topic, "/alarme") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0){
            estado_alarme = true; // Ativa o alarme
            Alarme(); // Chama a fun��o Alarme para atualizar o estado e display
            // Esta condi��o "!estado_alarme" aqui parece ser um erro de l�gica, pois Alarme_Acionado s� deveria ser resetado se o alarme estivesse sendo desativado.
            // Se o comando "On" chegou, o alarme est� sendo LIGADO.
            if(!estado_alarme){ // -> Esta linha provavelmente deve ser removida ou a l�gica revisada.
                Alarme_Acionado = false;
            }
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0){
            estado_alarme = false; // Desativa o alarme
            Alarme(); // Chama a fun��o Alarme para atualizar o estado e display
            Alarme_Acionado = false; // Garante que o flag de acionamento seja resetado ao desativar
        }
    }
    // Comando para imprimir uma mensagem no console serial da Pico W
    else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", len, data); // Imprime o payload da mensagem
    }
    // Comando para publicar o tempo de atividade (uptime) do dispositivo
    else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        // Calcula o tempo de atividade em segundos e formata como string
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        // Publica o uptime no t�pico "/uptime". A mensagem � retida.
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
    // Comando para o cliente se desconectar e parar
    else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // Sinaliza que o cliente deve parar
        sub_unsub_topics(state, false); // Inicia o processo de desassinatura de todos os t�picos
    }
}

// Dados de entrada publicados
// Este � o primeiro callback acionado quando uma mensagem MQTT � recebida.
// Ele apenas copia o t�pico da mensagem para o buffer 'state->topic'.
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic)); // Copia o t�pico para o buffer do estado
}

// Publicar temperatura (fun��o agendada)
// Esta fun��o � um "worker" ass�ncrono que � agendado para ser executado periodicamente.
// Ele chama 'publish_temperature' e, em seguida, reagenda a si mesmo.
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data; // Obt�m o estado do cliente MQTT
    publish_temperature(state); // Chama a fun��o para publicar a temperatura
    // Reagenda o worker para ser executado novamente ap�s TEMP_WORKER_TIME_S milissegundos
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Conex�o MQTT
// Este � o callback chamado quando o status da conex�o MQTT com o broker muda.
// � crucial para gerenciar o ciclo de vida da conex�o.
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg; // Obt�m o estado do cliente MQTT
    if (status == MQTT_CONNECT_ACCEPTED) { // Conex�o bem-sucedida
        state->connect_done = true; // Seta o flag de conex�o conclu�da
        sub_unsub_topics(state, true); // Assina todos os t�picos de comando

        // Indica que o dispositivo est� online, publicando "1" no t�pico "will_topic" (definido como LWT)
        // Isso � feito para substituir a mensagem de "Last Will" com "0" por um "1" confirmando que o dispositivo est� ativo.
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Inicia o agendamento da publica��o de temperatura
        temperature_worker.user_data = state; // Associa o estado do cliente ao worker
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0); // Agenda para executar imediatamente
    } else if (status == MQTT_CONNECT_DISCONNECTED) { // Cliente desconectado
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server"); // Se a desconex�o ocorreu antes da primeira conex�o, � um erro fatal
        }
    }
    else {
        panic("Unexpected status"); // Trata outros status inesperados
    }
}

// Inicializar o cliente MQTT
// Esta fun��o aloca e configura uma nova inst�ncia do cliente MQTT e tenta iniciar a conex�o com o broker.
static void start_client(MQTT_CLIENT_DATA_T *state) {
    // Define a porta MQTT (TLS ou n�o TLS)
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT; // Porta padr�o 1883
    INFO_printf("Warning: Not using TLS\n"); // Alerta se TLS n�o estiver habilitado
#endif

    state->mqtt_client_inst = mqtt_client_new(); // Cria uma nova inst�ncia do cliente MQTT
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error"); // Erro se n�o conseguir criar a inst�ncia
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr))); // Imprime o IP do Pico W
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address)); // Imprime o IP do broker

    cyw43_arch_lwip_begin(); // Inicia uma se��o protegida para opera��es LWIP (rede)
    // Tenta conectar ao broker MQTT. Passa o callback de conex�o e as informa��es do cliente.
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error"); // Erro se a conex�o inicial falhar
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // Configura o hostname para TLS, se aplic�vel
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    // Define os callbacks para mensagens MQTT recebidas (primeiro o t�pico, depois os dados)
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end(); // Finaliza a se��o protegida LWIP
}

// Call back com o resultado do DNS
// Este � o callback chamado pela biblioteca LWIP quando a resolu��o de DNS do broker MQTT � conclu�da.
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) { // Se o endere�o IP foi resolvido com sucesso
        state->mqtt_server_address = *ipaddr; // Armazena o endere�o IP do broker
        start_client(state); // Inicia o cliente MQTT
    } else {
        panic("dns request failed"); // Se a resolu��o DNS falhar, causa um panic
    }
}

/* ========== FUN��ES DE HARDWARE ========== */

// Inicializa os pinos GPIO para os LEDs e sensores.
// Configura a dire��o (entrada/sa�da) e o estado inicial de todos os pinos de hardware.
void gpio_led_bitdog(void) {
    // Configura os LEDs RGB (para a luz frontal controlada) como sa�das e os desliga inicialmente.
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);

    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);

    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);

    // Configura o primeiro sensor ultrass�nico (para a luz frontal).
    // TRIG_PIN como sa�da (para enviar o pulso), ECHO_PIN como entrada (para receber a resposta).
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0); // Garante que o pino trigger esteja em LOW inicialmente

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Configura o segundo sensor ultrass�nico (para o alarme).
    // TRIG_PIN_2 como sa�da, ECHO_PIN_2 como entrada.
    gpio_init(TRIG_PIN_2);
    gpio_set_dir(TRIG_PIN_2, GPIO_OUT);
    gpio_put(TRIG_PIN_2, 0); // Garante que o pino trigger esteja em LOW inicialmente

    gpio_init(ECHO_PIN_2);
    gpio_set_dir(ECHO_PIN_2, GPIO_IN);

    // Inicializa��o do Buzzer como sa�da.
    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);

    // Configura o sensor de luz (LDR) como entrada.
    gpio_init(ldr_pin);
    gpio_set_dir(ldr_pin, GPIO_IN);

    // Configura��o do ADC (Conversor Anal�gico-Digital).
    // Inicializa o m�dulo ADC e configura os pinos do joystick (X e Y) para serem usados como entradas anal�gicas.
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_X); // ADC0
    adc_gpio_init(ADC_JOYSTICK_Y); // ADC1

    // Configura��o do Bot�o A como entrada com pull-up (para que o pino fique em HIGH quando n�o pressionado).
    gpio_init(Botao_A);
    gpio_set_dir(Botao_A, GPIO_IN);
    gpio_pull_up(Botao_A);
}

/* ========== FUN��ES DE CONTROLE ========== */

// Controla a matriz de LEDs baseada nos estados dos c�modos.
// Esta fun��o atualiza visualmente o status das luzes na matriz de LEDs 5x5.
void ligar_luz() {
    uint32_t luz_sala, luz_cozinha, luz_quarto, luz_banheiro, luz_quintal;

    // Define as cores (neste caso, cinza claro 0x4D4D4D00 para ligado, preto 0x00000000 para desligado)
    // para cada c�modo com base nas vari�veis booleanas de estado (estado_led_sala, etc.).
    luz_sala = estado_led_sala ? 0x4D4D4D00 : 0x00000000;
    luz_cozinha = estado_led_cozinha ? 0x4D4D4D00 : 0x00000000;
    luz_quarto = estado_led_quarto ? 0x4D4D4D00 : 0x00000000;
    luz_banheiro = estado_led_banheiro ? 0x4D4D4D00 : 0x00000000;
    luz_quintal = estado_led_quintal ? 0x4D4D4D00 : 0x00000000;

    // Itera por cada LED da matriz 5x5 (NUM_PIXELS = 25)
    for (int i = 0; i < NUM_PIXELS; i++) {
        uint32_t valor_led = 0; // Valor inicial para o LED (desligado)
        int linha = i / 5;      // Calcula a linha atual do LED (0 a 4)

        // Atribui a cor/estado do LED com base na linha (que representa um c�modo espec�fico).
        // As linhas s�o mapeadas inversamente (linha 4 = sala, linha 0 = quintal).
        if (linha == 4) {
            valor_led = luz_sala;
        } else if (linha == 3) {
            valor_led = luz_cozinha;
        } else if (linha == 2) {
            valor_led = luz_quarto;
        } else if (linha == 1) {
            valor_led = luz_banheiro;
        } else if (linha == 0) {
            valor_led = luz_quintal;
        } else {
            valor_led = 0x000000; // Por seguran�a, desliga LEDs em linhas n�o mapeadas
        }

        // Envia o valor (cor) para o LED atrav�s da interface PIO (Programable I/O).
        // 'pio_sm_put_blocking' envia o dado e bloqueia at� que o SM (State Machine) esteja pronto para aceitar mais dados.
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Controla o display OLED.
// Esta fun��o atualiza o conte�do exibido no display SSD1306 com base no estado da "TV" e do alarme.
void ligar_display() {
    bool cor = true; // Define a cor principal para o texto (branco no SSD1306 monocrom�tico)

    // Sempre limpa e desenha a moldura do display.
    ssd1306_fill(&ssd, !cor); // Preenche o display com preto (ou cor de fundo)
    ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor); // Moldura externa
    ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Moldura interna

    // L�gica para exibir o estado da "TELEVIS�O".
    if (estado_display) { // Se o estado da TV � LIGADO
        // Redesenha a moldura (redundante, pois j� foi feito acima, mas garante a atualiza��o)
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
        ssd1306_draw_string(&ssd, "TELEVISAO ", 30, 30); // Desenha "TELEVISAO"
        ssd1306_draw_string(&ssd, "LIGADA", 38, 40);     // Desenha "LIGADA"
        ssd1306_send_data(&ssd); // Envia os dados para o display
        tv = 1; // Flag para indicar que a TV foi ligada
    } else { // Se o estado da TV � DESLIGADO
        if (tv == 1){ // Verifica se a TV estava ligada antes para mostrar a mensagem de desligamento
            // Redesenha a moldura
            ssd1306_fill(&ssd, !cor);
            ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_draw_string(&ssd, "TELEVISAO ", 30, 30);
            ssd1306_draw_string(&ssd, "DESLIGADA", 28, 40);
            ssd1306_send_data(&ssd);
            sleep_ms(500); // Exibe a mensagem por 500ms
            ssd1306_fill(&ssd, !cor); // Limpa o display ap�s a mensagem
            ssd1306_send_data(&ssd);
            tv = 0; // Reseta o flag para evitar redesenhar "DESLIGADA" v�rias vezes
        }
    }

    // L�gica para exibir o estado do ALARME.
    if (estado_alarme){ // Se o alarme est� LIGADO (armado)
        // Redesenha a moldura
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
        ssd1306_draw_string(&ssd, "ALARME", 35, 30);
        ssd1306_draw_string(&ssd, "LIGADO", 35, 40);
        ssd1306_send_data(&ssd);
        tv_alarme = 1; // Flag para indicar que o alarme foi ligado

        // Se o alarme estiver ACIONADO (h� uma viola��o)
        if (Alarme_Acionado){
            // Redesenha a moldura
            ssd1306_fill(&ssd, !cor);
            ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_draw_string(&ssd, "ALARME", 35, 30);
            ssd1306_draw_string(&ssd, "ACIONADO", 28, 40); // Exibe "ACIONADO"
            ssd1306_send_data(&ssd);
        }
    } else { // Se o alarme est� DESLIGADO (desarmado)
        if (tv_alarme == 1){ // Verifica se o alarme estava ligado antes para mostrar a mensagem de desligamento
            // Redesenha a moldura
            ssd1306_fill(&ssd, !cor);
            ssd1306_rect(&ssd, 0, 0, 127, 63, cor, !cor);
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_draw_string(&ssd, "ALARME", 35, 30);
            ssd1306_draw_string(&ssd, "DESLIGADO", 28, 40);
            ssd1306_send_data(&ssd);
            sleep_ms(500); // Exibe a mensagem por 500ms
            ssd1306_fill(&ssd, !cor); // Limpa o display ap�s a mensagem
            ssd1306_send_data(&ssd);
            tv_alarme = 0; // Reseta o flag para evitar redesenhar "DESLIGADO" v�rias vezes
        }
    }
}

/* ========== FUN��ES DOS SENSORES ========== */

// Envia um pulso de trigger de 10 microssegundos para o primeiro sensor ultrass�nico (para os LEDs RGB frontais).
void send_trigger_pulse() {
    gpio_put(TRIG_PIN, 1); // Coloca o pino TRIG em HIGH
    sleep_us(10);          // Espera 10 microssegundos
    gpio_put(TRIG_PIN, 0); // Coloca o pino TRIG em LOW
}

// Envia um pulso de trigger de 10 microssegundos para o segundo sensor ultrass�nico (para o alarme).
void send_trigger_pulse_2() {
    gpio_put(TRIG_PIN_2, 1); // Coloca o pino TRIG_2 em HIGH
    sleep_us(10);            // Espera 10 microssegundos
    gpio_put(TRIG_PIN_2, 0); // Coloca o pino TRIG_2 em LOW
}

// Mede a dist�ncia em cent�metros usando um sensor ultrass�nico gen�rico (reutiliz�vel para ambos os sensores).
// Recebe os pinos de trigger e echo como argumentos.
float measure_distance_cm(uint trigger_pin, uint echo_pin) {
    // Sequ�ncia de pulso para iniciar a medi��o: LOW, HIGH por 10us, LOW.
    gpio_put(trigger_pin, 0);
    sleep_us(2);
    gpio_put(trigger_pin, 1);
    sleep_us(10);
    gpio_put(trigger_pin, 0);

    // Espera o pino ECHO ficar em HIGH (in�cio da onda de retorno).
    // O pulso ECHO � HIGH enquanto a onda sonora viaja e retorna.
    while (gpio_get(echo_pin) == 0); // Espera por HIGH

    absolute_time_t start = get_absolute_time(); // Marca o tempo de in�cio do pulso HIGH

    // Espera o pino ECHO voltar para LOW (fim da onda de retorno).
    while (gpio_get(echo_pin) == 1); // Espera por LOW

    absolute_time_t end = get_absolute_time(); // Marca o tempo de fim do pulso HIGH
    int64_t pulse_duration = absolute_time_diff_us(start, end); // Calcula a dura��o do pulso em microssegundos

    // Converte a dura��o do pulso para cent�metros.
    // A velocidade do som no ar � de aproximadamente 343 metros por segundo, ou 0.0343 cm por microssegundo.
    // Como o som percorre a dist�ncia duas vezes (ida e volta), a f�rmula � (dura��o * velocidade) / 2.
    // Ou, mais comumente, dura��o / 58.0 (onde 58 us/cm � uma constante derivada de 1/0.0343 * 2).
    float distance_cm = pulse_duration / 58.0;

    return distance_cm;
}

// Controla os LEDs RGB frontais (luz de entrada) baseado nas leituras do sensor ultrass�nico e LDR.
void luz_frente_controlada() {
    float dist1 = measure_distance_cm(TRIG_PIN, ECHO_PIN); // Mede a dist�ncia do primeiro sensor ultrass�nico

    // Aciona os LEDs RGB se:
    // 1. Um objeto estiver a menos de 15 cm de dist�ncia (dist1 < 15)
    // E
    // 2. Estiver escuro (gpio_get(ldr_pin) retorna 0, indicando baixa luminosidade)
    if ((dist1 < 15) && (!gpio_get(ldr_pin))) {
        gpio_put(LED_BLUE_PIN, 1);  // Liga o LED azul
        gpio_put(LED_GREEN_PIN, 1); // Liga o LED verde
        gpio_put(LED_RED_PIN, 1);   // Liga o LED vermelho
    } else {
        gpio_put(LED_BLUE_PIN, 0);  // Desliga o LED azul
        gpio_put(LED_GREEN_PIN, 0); // Desliga o LED verde
        gpio_put(LED_RED_PIN, 0);   // Desliga o LED vermelho
    }
}

/**
 * Gera um som de alerta no Buzzer.
 * Esta fun��o produz uma sequ�ncia de bipes para simular a sirene do alarme.
 */
void Som_Alarme() {
    const uint buzzer_pin = BUZZER; // Pino do buzzer
    const int freq = 2500;           // Frequ�ncia do som em Hz
    const int duration_ms = 80;      // Dura��o de cada bipe em milissegundos
    const int interval_ms = 50;      // Intervalo de sil�ncio entre os bipes em milissegundos

    int period_us = 1000000 / freq;   // Per�odo total da onda em microssegundos (1s / freq)
    int half_period_us = period_us / 2; // Metade do per�odo para gerar onda quadrada

    // Loop para produzir 8 bipes
    for (int i = 0; i < 8; i++) {
        uint32_t start_time = to_ms_since_boot(get_absolute_time()); // Marca o tempo de in�cio do bipe
        // Loop interno para manter o bipe ativo pela 'duration_ms'
        while (to_ms_since_boot(get_absolute_time()) - start_time < duration_ms) {
            gpio_put(buzzer_pin, 1);        // Liga o buzzer
            sleep_us(half_period_us);       // Espera metade do per�odo
            gpio_put(buzzer_pin, 0);        // Desliga o buzzer
            sleep_us(half_period_us);       // Espera a outra metade do per�odo
        }
        sleep_ms(interval_ms); // Pausa entre os bipes
    }
}

// Fun��o que verifica se houve viola��o por detec��o de objetos pr�ximos ou viola��o das portas, e aciona o alarme.
void Alarme(){
    ligar_display(); // Atualiza o display OLED com o status do alarme (Ligado/Desligado/Acionado)

    // Mede a dist�ncia do objeto ao segundo sensor ultrass�nico (para o alarme).
    float dist2 = measure_distance_cm(TRIG_PIN_2, ECHO_PIN_2);

    // Leitura dos valores anal�gicos do joystick (simulando a abertura de portas).
    adc_select_input(0); // Seleciona o canal ADC0 (Eixo X)
    Eixo_x_value = adc_read(); // L� o valor do Eixo X
    adc_select_input(1); // Seleciona o canal ADC1 (Eixo Y)
    Eixo_Y_value = adc_read(); // L� o valor do Eixo Y

    // L�gica para acionamento do alarme, APENAS se o alarme estiver ATIVADO (`estado_alarme` � true).
    if (estado_alarme){
        // Verifica a viola��o de "portas" atrav�s do joystick:
        // Se qualquer eixo do joystick estiver fora da faixa de 1800-2200 (considerando um valor central de ~2048),
        // isso � interpretado como uma "abertura de porta" ou "movimento do joystick".
        if (((Eixo_Y_value > 2200) || (Eixo_Y_value < 1800)) || ((Eixo_x_value > 2200) || (Eixo_x_value < 1800))) {
            Alarme_Acionado = true; // Define o flag de alarme acionado
        }
        // Verifica a detec��o de objetos pr�ximos pelo segundo sensor ultrass�nico:
        // Se um objeto estiver a menos de 15 cm, isso � interpretado como presen�a.
        if (dist2 < 15){
            Alarme_Acionado = true; // Define o flag de alarme acionado
        }

        // Se o alarme estiver acionado por qualquer uma das condi��es acima, chama a fun��o de som.
        if (Alarme_Acionado){
            Som_Alarme(); // Dispara a sirene do alarme
        }
    }
}

/* ========== HANDLER DE INTERRUP��O ========== */

// Tratamento das interrup��es dos bot�es.
// Esta fun��o � uma Rotina de Servi�o de Interrup��o (ISR) que � executada quando um evento GPIO (como o Bot�o A) ocorre.
// Ela implementa um debouncing de software para evitar leituras m�ltiplas de um �nico pressionamento.
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_time = 0; // Armazena o tempo do �ltimo evento processado
    uint32_t current_time = to_us_since_boot(get_absolute_time()); // Tempo atual em microssegundos

    // Debouncing de 300ms (300000 microssegundos):
    // S� processa o evento se o tempo desde o �ltimo evento for maior que 300ms.
    if (current_time - last_time > 300000) {
        last_time = current_time; // Atualiza o tempo do �ltimo evento processado

        // Se o GPIO que gerou a interrup��o foi o Botao_A e ele est� em LOW (pressionado, pois tem pull-up)
        if (gpio == Botao_A && !gpio_get(Botao_A)) {
            estado_alarme = !estado_alarme; // Alterna o estado do alarme (Liga/Desliga)

            // Se o alarme foi DESLIGADO (estado_alarme � false ap�s a altern�ncia),
            // redefine o flag de acionamento para false.
            if(!estado_alarme){
                Alarme_Acionado = false;
            }
        }
    }
}