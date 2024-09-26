/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "driver/temperature_sensor.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"



static const char *TAG = "example";

#define MAX_APs 20

/*
typedef struct {
    char ssid[32];
    char password[64];
} wifi_credentials_t;
*/

/*
wifi_credentials_t wifi_credentials[] = {
    { "++", "++" },
    { "++t", "++" }
};
*/


/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;
static temperature_sensor_handle_t temp_handle;


float temperature_celsius = 0.0f;


static void blink_led(void)
{
    /* If the addressable LED is enabled */
        if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            int i = 0;
             while ( i < 25) {

                led_strip_set_pixel(led_strip, i, 20+i, 1+i, 1+i);
                i++;
                vTaskDelay(10);

                if (i > 4){
                    led_strip_set_pixel(led_strip, i - 5, 0, 0, 0);
                }
        
    
                led_strip_refresh(led_strip);
             }
            while (i != 4)
                {
                led_strip_set_pixel(led_strip, i-5, 20+i, 1+i, 1+i);
                led_strip_set_pixel(led_strip, i-1, 0, 0, 0);
                i--;
                led_strip_refresh(led_strip);
                 vTaskDelay(10);
            }

        } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);

        }
    
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 25, // at least one LED on board
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

static void configure_temperature(void)
{
    ESP_LOGI(TAG, "Example configured to read temperature sensor!");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(5, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
}

static void read_temperature(void)
{
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &temperature_celsius));
        ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
        vTaskDelay(1000);

}

// Event handler for Wi-Fi disconnect
static void on_wifi_disconnect(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_wifi_connect();
}

// Event handler for IP address acquisition
//Handler für die Verarbeitung von erfolgreichen IP-Zuweisungen nach der Verbindung.
static void on_got_ip(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

static void send_packet(){
    printf("send_packet hello");
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    // check sock >= 0 and handle error if check fails
    if (sock < 0) {
        printf("socket creation failed");
        return;
    }


    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));  // Zero out the structure
    dest_addr.sin_addr.s_addr = inet_addr(IP_A);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8888);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        printf("connect failed");
        close(sock);
        return;
    }

    char payload[50];
    sprintf(payload, "%.2f", temperature_celsius);
    size_t payloadLen = strlen(payload);

    err = send(sock, payload, payloadLen, 0);
    if (err < 0) {
        printf("send failed");
    }
    // check err >= 0 and handle error if check fails
    shutdown(sock, 0);
    close(sock);
    vTaskDelay(100);
}

void wifi_connect_to_network() {
    // Wi-Fi-Konfiguration
    //const char* ssid, const char* password

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_NAME,
            .password = PASSWORD,
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

   /*
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = -127;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    */


    // Wi-Fi-Verbindung herstellen
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

       
}

static void configure_wifi(void){

        ESP_LOGI(TAG, "Example configured to connect to Wi-Fi!");

        ESP_ERROR_CHECK(nvs_flash_init());
        //Initialisiert den TCP/IP-Stack
        ESP_ERROR_CHECK(esp_netif_init());
        //Erstellt die Standard-Ereignisschleife (Event Loop), die benötigt wird, um auf Systemereignisse zu reagieren.
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        //Stellt die Standardkonfiguration für die WLAN-Initialisierung bereit
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        //Initialisiert das WLAN mit dieser Konfiguration.
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        //Erstellt eine Netzwerkschnittstelle mit der Standardkonfiguration für den Station Mode (STA).
        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
        //Erstellt die Netzwerkschnittstelle für den Station Mode (STA) und wendet die Standardkonfiguration an.
        esp_netif_t *netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
        //Setzt die Standard-Ereignishandler für Wi-Fi Station Mode, z. B. zum Verarbeiten von Verbindungsereignissen.
        esp_wifi_set_default_wifi_sta_handlers();
        //Registriert einen Ereignishandler für Wi-Fi Station Mode, um auf Verbindungsereignisse zu reagieren.
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        /*
        for (int i = 0; i < sizeof(wifi_credentials) / sizeof(wifi_credentials[0]); i++) {
            wifi_connect_to_network(wifi_credentials[i].ssid, wifi_credentials[i].password);

            // Wait for a successful connection or check for errors
            esp_err_t event_ret;
            ip_event_got_ip_t* event;
            esp_event_loop_run(10000, &event_ret); // Example timeout of 10 seconds

            if (event_ret == ESP_OK) {
                ESP_LOGI(TAG, "Successfully connected to SSID: %s", wifi_credentials[i].ssid);
                break;
            }
        }
        */

} 

const char* auth_mode_type(wifi_auth_mode_t auth_mode) {
    switch (auth_mode) {
        case WIFI_AUTH_OPEN:
            return "Open";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA_WPA2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2_ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK:
            return "WPA3_PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK:
            return "WPA2_WPA3_PSK";
        case WIFI_AUTH_WAPI_PSK:
            return "WAPI_PSK";
        default:
            return "Unknown";
    }
}

void wifi_scan() {
     wifi_scan_config_t scan_config = {
    .ssid = 0,
    .bssid = 0,
    .channel = 0,
    .show_hidden = true
  };

  ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

  wifi_ap_record_t wifi_records[MAX_APs];

  uint16_t max_records = MAX_APs;
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&max_records, wifi_records));

  printf("Number of Access Points Found: %d\n", max_records);
  printf("\n");
  printf("               SSID              | Channel | RSSI |   Authentication Mode \n");
  printf("***************************************************************\n");
  for (int i = 0; i < max_records; i++)
    printf("%32s | %7d | %4d | %12s\n", (char *)wifi_records[i].ssid, wifi_records[i].primary, wifi_records[i].rssi, auth_mode_type(wifi_records[i].authmode));
  printf("***************************************************************\n");
}



void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_temperature();
    configure_wifi();
    wifi_connect_to_network();
    //wifi_connect();
    //wifi_scan();
    
    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        read_temperature();
        if (temperature_celsius) {
            send_packet();
        }
        ESP_LOGI(TAG, "Current temperature: %.2f°C", temperature_celsius);
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        printf("Hello, World!\n");
    }
}
