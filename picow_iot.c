#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

#include "lwip/pbuf.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/rosc.h"

#include "hardware/uart.h"
#include "pico/binary_info.h"

// Microphone ADC pin used to control LED brightness using sound waves
#define ADC_PIN 26

// Wi-Fi credentials
// MQTT credentials and topic
#include "mqtt_info.h"


#define LED0 0
#define LED1 1
#define LED2 2
#define LED3 3
#define Led_Strip 16

#define DHT_PIN 14
#define MAX_TIMINGS 85

typedef struct
{
  float humidity;
  float temp_celsius;
} dht_reading;

float g_temperature;
float g_humidity;

// Track Wi-Fi and mqtt connection status
bool wifi_connected = false;
bool mqtt_connected = false;

uint16_t fade = 0;

uint16_t adc_raw;

/*-----------------------------------------------------------*/

void prvSetupHardware(void);
void wifi_task(void);
void read_from_dht(dht_reading *result);
void dht_task(void *pvParameters);
void second_core_code(void);
// Light strip
void on_pwm_wrap();
void lightStrip_Task(void *pvParameters);
// Mqtt
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void mqtt_sub_request_cb(void *arg, err_t err);
void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len);
static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void subscribe_topics(mqtt_client_t *client, void *arg);
// FreeRTOS hooks
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationIdleHook(void);

/*-----------------------------------------------------------*/

void prvSetupHardware(void)
{
    stdio_init_all();
    bi_decl(bi_program_description("Analog microphone")); // for picotool
    bi_decl(bi_1pin_with_name(ADC_PIN, "ADC input pin"));
    // Initialize GPIO for LEDs
    gpio_init(LED0);
    gpio_set_dir(LED0, GPIO_OUT);
    gpio_init(LED1);
    gpio_set_dir(LED1, GPIO_OUT);
    gpio_init(LED2);
    gpio_set_dir(LED2, GPIO_OUT);
    gpio_init(LED3);
    gpio_set_dir(LED3, GPIO_OUT);

    gpio_init(Led_Strip);
    gpio_set_dir(Led_Strip, GPIO_OUT);

    gpio_init(DHT_PIN);
}

/*-----------------------------------------------------------*/

void wifi_task(void)
{
    // Connect to Wi-Fi
    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();

    // Connect to the Wi-Fi network - loop until connected
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
    {
        printf("Attempting to connect...\n");
    }

    // Print a success message once connected
    printf("Connected to Wi-Fi\n");
    wifi_connected = true;
}

/*-----------------------------------------------------------*/

void dht_task(void *pvParameters)
{
  (void)pvParameters;

  dht_reading reading;

  while (1)
  {
    read_from_dht(&reading);
    float fahrenheit = (reading.temp_celsius * 9 / 5) + 32;
    printf("Humidity = %.1f%%, Temperature = %.1fC (%.1fF)\n",
           reading.humidity, reading.temp_celsius, fahrenheit);

    // Assign values to global variables
    g_temperature = reading.temp_celsius;
    g_humidity = reading.humidity;

    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

/*-----------------------------------------------------------*/

void read_from_dht(dht_reading *result)
{
  int data[5] = {0, 0, 0, 0, 0};
  uint last = 1;
  uint j = 0;
  gpio_set_dir(DHT_PIN, GPIO_OUT);
  gpio_put(DHT_PIN, 0);
  sleep_ms(20);
  gpio_set_dir(DHT_PIN, GPIO_IN);

  for (uint i = 0; i < MAX_TIMINGS; i++)
  {
    uint count = 0;
    while (gpio_get(DHT_PIN) == last)
    {
      count++;
      sleep_us(1);
      if (count == 255)
        break;
    }
    last = gpio_get(DHT_PIN);
    if (count == 255)
      break;

    if ((i >= 4) && (i % 2 == 0))
    {
      data[j / 8] <<= 1;
      if (count > 16)
        data[j / 8] |= 1;
      j++;
    }
  }

  if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
  {
    result->humidity = (float)((data[0] << 8) + data[1]) / 10;
    if (result->humidity > 100)
    {
      result->humidity = data[0];
    }
    result->temp_celsius = (float)(((data[2] & 0x7F) << 8) + data[3]) / 10;
    if (result->temp_celsius > 125)
    {
      result->temp_celsius = data[2];
    }
    if (data[2] & 0x80)
    {
      result->temp_celsius = -result->temp_celsius;
    }
  }
}


typedef struct MQTT_CLIENT_T_
{
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

// Perform initialisation
static MQTT_CLIENT_T *mqtt_client_init(void)
{
    MQTT_CLIENT_T *state = calloc(1, sizeof(MQTT_CLIENT_T));
    if (!state)
    {
        printf("failed to allocate state\n");
        return NULL;
    }
    state->received = 0;
    state->mqtt_client = mqtt_client_new();
    return state;
}

void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)callback_arg;
    printf("DNS query finished with resolved addr of %s.\n", ip4addr_ntoa(ipaddr));
    state->remote_addr = *ipaddr;
}

void run_dns_lookup(MQTT_CLIENT_T *state)
{
    printf("Running DNS query for %s.\n", MQTT_SERVER_HOST);

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(MQTT_SERVER_HOST, &(state->remote_addr), dns_found, state);
    cyw43_arch_lwip_end();

    if (err == ERR_ARG)
    {
        printf("failed to start DNS query\n");
        return;
    }

    if (err == ERR_OK)
    {
        printf("no lookup needed");
        return;
    }

    while (state->remote_addr.addr == 0)
    {
        cyw43_arch_poll();
        sleep_ms(1);
    }
}

u32_t data_in = 0;
u8_t buffer[1025];
u8_t data_len = 0;

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len)
{
    printf("mqtt_pub_start_cb: topic %s\n", topic);

    if (tot_len > 1024)
    {
        printf("Message length exceeds buffer size, discarding");
    }
    else
    {
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    if (data_in > 0)
    {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0)
        {
            buffer[data_len] = '\0'; // Ensure null termination for string operations
            printf("Message received: %s\n", buffer);

            // Check if the message is "All_ON" or "All_OFF"
            if (strcmp((char *)buffer, "All_ON") == 0)
            {
                gpio_put(LED0, 1);
                gpio_put(LED1, 1);
                gpio_put(LED2, 1);
                gpio_put(LED3, 1);
                gpio_put(Led_Strip, 1);
            }
            else if (strcmp((char *)buffer, "All_OFF") == 0)
            {
                gpio_put(LED0, 0);
                gpio_put(LED1, 0);
                gpio_put(LED2, 0);
                gpio_put(LED3, 0);
                gpio_put(Led_Strip, 0);
            }
            else if (strcmp((char *)buffer, "LED0_ON") == 0)
            {
                gpio_put(LED0, 1);
            }
            else if (strcmp((char *)buffer, "LED0_OFF") == 0)
            {
                gpio_put(LED0, 0);
            }
            else if (strcmp((char *)buffer, "LED1_ON") == 0)
            {
                gpio_put(LED1, 1);
            }
            else if (strcmp((char *)buffer, "LED1_OFF") == 0)
            {
                gpio_put(LED1, 0);
            }
            else if (strcmp((char *)buffer, "LED2_ON") == 0)
            {
                gpio_put(LED2, 1);
            }
            else if (strcmp((char *)buffer, "LED2_OFF") == 0)
            {
                gpio_put(LED2, 0);
            }
            else if (strcmp((char *)buffer, "LED3_ON") == 0)
            {
                gpio_put(LED3, 1);
            }
            else if (strcmp((char *)buffer, "LED3_OFF") == 0)
            {
                gpio_put(LED3, 0);
            }
            // Reset buffer for next message
            data_len = 0;
        }
    }
}


// Subscribe to the topics after connecting
static void subscribe_topics(mqtt_client_t *client, void *arg)
{
    sleep_ms(100);
    mqtt_subscribe(client, "absik/feeds/All_LEDs", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "absik/feeds/LED_0", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "absik/feeds/LED_1", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "absik/feeds/LED_2", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "absik/feeds/LED_3", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "absik/feeds/light_strip", 0, mqtt_sub_request_cb, arg);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    if (status != 0)
    {
        printf("Error during connection: err %d.\n", status);
    }
    else
    {
        printf("MQTT connected.\n");
        mqtt_connected = true;
    }
}

void mqtt_sub_request_cb(void *arg, err_t err)
{
    printf("mqtt_sub_request_cb: err %d\n", err);
}

err_t mqtt_connect(MQTT_CLIENT_T *state)
{
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = MQTT_CLIENT_ID;
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASS;
    ci.keep_alive = 60;
    ci.will_topic = DASHBOARDS;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr), MQTT_PORT, mqtt_connection_cb, state, client_info);

    if (err == ERR_OK)
    {
        // Successfully connected, now subscribe to topics
        subscribe_topics(state->mqtt_client, state);
    }
    else
    {
        printf("mqtt_connect return %d\n", err);
    }

    return err;
}

void publishToMqtt(MQTT_CLIENT_T *state)
{
    state->mqtt_client = mqtt_client_new();
    state->counter = 0;

    if (!state->mqtt_client)
    {
        printf("Failed to create mqtt client\n");
        return;
    }

    if (mqtt_connect(state) == ERR_OK)
    {
        mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb, mqtt_pub_data_cb, 0);
        while (true)
        {
            cyw43_arch_poll();
            if (mqtt_client_is_connected(state->mqtt_client))
            {

                char temp[128];
                sprintf(temp, "%.1f", g_temperature);

                char humidity[128];
                sprintf(humidity, "%.1f", g_humidity);
                err_t err;
                cyw43_arch_lwip_begin();
                err = mqtt_publish(state->mqtt_client, MQTT_TOPIC_TEMP, temp, strlen(temp), 0, 0, 0, state);
                mqtt_publish(state->mqtt_client, MQTT_TOPIC_HUM, humidity, strlen(humidity), 0, 0, 0, state);
                cyw43_arch_lwip_end();
                if (err != ERR_OK)
                {
                    printf("Publish err: %d\n", err);
                }
                else
                {
                    if (state->counter != 0)
                    {
                        printf("published %d\n", state->counter);
                    }
                    sleep_ms(10000); // 8-second delay
                    state->counter++;
                }
            }
        }
    }
}

void on_pwm_wrap()
{
    pwm_clear_irq(pwm_gpio_to_slice_num(Led_Strip));

    // Square the fade value to make the LED's brightness appear more linear
    pwm_set_gpio_level(Led_Strip,  adc_raw * adc_raw);
}

void lightStrip_Task(void *pvParameters)
{
    // Tell the LED pin that the PWM is in charge of its value
    gpio_set_function(Led_Strip, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(Led_Strip);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 4.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

    // Main loop
    while (true) {
        // Read ADC value
        adc_raw = adc_read(); // raw voltage from ADC

        // Update PWM level immediately in the main loop
        pwm_set_gpio_level(Led_Strip, adc_raw * adc_raw );

        // PWM handling logic if needed
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*-----------------------------------------------------------*/
// FreeRTOS hooks
void vApplicationMallocFailedHook(void)
{
    configASSERT((volatile void *)NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;
    configASSERT((volatile void *)NULL);
}

void vApplicationIdleHook(void)
{
    volatile size_t xFreeHeapSpace;
    xFreeHeapSpace = xPortGetFreeHeapSize();
    (void)xFreeHeapSpace;
}

/*-----------------------------------------------------------*/


void second_core_code(void)
{
    // Start the DHT task
    xTaskCreate(dht_task, "DHT Task", 2048, NULL, 2, NULL);

    xTaskCreate(lightStrip_Task, "lightStripTask", 2048, NULL, 1, NULL);
    // Start scheduler
    vTaskStartScheduler();
}

int main()
{
    // Initialize the hardware
    prvSetupHardware();
    multicore_launch_core1(second_core_code);

    if (!wifi_connected)
    {
        wifi_task();
    }

    // Wait for Wi-Fi connection before proceeding
    while (!wifi_connected)
    {
        sleep_ms(100);
    }

    if (wifi_connected & !mqtt_connected)
    {
        MQTT_CLIENT_T *state = mqtt_client_init();

        run_dns_lookup(state);

        publishToMqtt(state);

        cyw43_arch_deinit();
    }

    // Code should never reach here
    while (1)
    {
        // Handle errors
    }
}
