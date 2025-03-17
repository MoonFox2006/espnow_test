#include <string.h>
#include <assert.h>
#include <nvs_flash.h>
#include <esp_random.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_crc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define SERVER

#define ESPNOW_MODE     WIFI_MODE_STA
#define ESPNOW_IF       WIFI_IF_STA
#define ESPNOW_CHANNEL  0
//#define ESPNOW_CHANNEL  13
#define ESPNOW_RATE     WIFI_PHY_RATE_LORA_250K
//#define ESPNOW_RATE     WIFI_PHY_RATE_LORA_500K
#define ESPNOW_DATALEN  200

#ifdef SERVER
#define SEND_PERIOD     20000 // 20 ms. (50 Hz)
#define RECEIVE_PERIOD  1000000 // 1 sec.
#endif

typedef struct __attribute__((__packed__)) espnow_ack_t {
    uint32_t received;
    uint32_t skipped;
} espnow_ack_t;

#ifdef SERVER
typedef struct __attribute__((__packed__)) espnow_ackinfo_t {
    uint32_t timestamp;
    int8_t rssi;
    espnow_ack_t data;
} espnow_ackinfo_t;
#endif

static const char *TAG = "ESP-NOW";

#ifdef SERVER
static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static uint8_t broardcast_data[ESPNOW_DATALEN];
static esp_timer_handle_t send_timer = NULL;
static QueueHandle_t receive_queue = NULL;
//static SemaphoreHandle_t send_semaphore = NULL;
#else
static uint8_t server_mac[ESP_NOW_ETH_ALEN] = { 0 };
static volatile espnow_ack_t ack = {
    .received = 0,
    .skipped = 0
};
static SemaphoreHandle_t espnow_semaphore = NULL;
#if ESPNOW_CHANNEL == 0
static esp_timer_handle_t find_timer = NULL;
static volatile uint8_t channel = 13;
#endif
#endif

#if (ESPNOW_CHANNEL == 0) && defined(SERVER)
static uint8_t wifi_find_channel(void) {
    const wifi_scan_config_t cfg = {
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 0,
        .scan_time.active.max = 480, // 120
        .home_chan_dwell_time = 30
    };

    int16_t rssis[13];
    uint8_t aps[13];

    {
        wifi_ap_record_t rec;

        ESP_ERROR_CHECK(esp_wifi_scan_start(&cfg, true));
        memset(rssis, 0, sizeof(rssis));
        memset(aps, 0, sizeof(aps));
        while (esp_wifi_scan_get_ap_record(&rec) == ESP_OK) {
            ESP_LOGI(TAG, "\"%s\"\t%u%c\t%d", (char*)rec.ssid, rec.primary, rec.second == WIFI_SECOND_CHAN_ABOVE ? '+' : rec.second == WIFI_SECOND_CHAN_BELOW ? '-' : ' ', rec.rssi);
            rssis[rec.primary - 1] += rec.rssi;
            ++aps[rec.primary - 1];
            if (rec.second == WIFI_SECOND_CHAN_ABOVE) {
                if (rec.primary < 13) {
                    rssis[rec.primary - 1 + 1] += rec.rssi;
                    ++aps[rec.primary - 1 + 1];
                }
            } else if (rec.second == WIFI_SECOND_CHAN_BELOW) {
                if (rec.primary > 1) {
                    rssis[rec.primary - 1 - 1] += rec.rssi;
                    ++aps[rec.primary - 1 - 1];
                }
            }
        }
        esp_wifi_clear_ap_list();
    }
    {
        uint8_t result = 0;

        for (uint8_t i = 0; i < 13; ++i) {
            if (aps[i])
                rssis[i] /= aps[i];
            ESP_LOGI(TAG, "Channel %u RSSI %d", i + 1, rssis[i]);
        }
        for (uint8_t i = 1; i < 13; ++i) {
            if (rssis[i] < rssis[result])
                result = i;
        }
        ESP_LOGI(TAG, "WiFi channel %u selected", result + 1);
        return result + 1;
    }
}
#endif

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    {
        const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    }
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_MODE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_IF, WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_start());
#if ESPNOW_CHANNEL == 0
#ifdef SERVER
    {
        uint8_t channel = wifi_find_channel();

        ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    }
#else
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
#endif
#else
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
#endif
}

#ifdef SERVER
/*
static esp_err_t espnow_send(const uint8_t *data, size_t size) {
    esp_err_t result;

    result = esp_now_send(broadcast_mac, data, size);
    if (result == ESP_OK) {
        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (send_status != ESP_NOW_SEND_SUCCESS) {
                ESP_LOGE(TAG, "Send fail!");
                result = ESP_FAIL;
            }
        } else {
            ESP_LOGE(TAG, "Send timeout!");
            result = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGE(TAG, "Send error %d!", result);
    }
    return result;
}
*/

static void send_timer_cb(void *arg) {
    esp_err_t err;

    err = esp_now_send(broadcast_mac, broardcast_data, sizeof(broardcast_data));
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error broadcasting data (%d)!", err);
    ++(*(uint32_t*)broardcast_data);
}
#elif ESPNOW_CHANNEL == 0

static void find_timer_cb(void *arg) {
    esp_err_t err;

    if (channel > 1)
        --channel;
    else
        channel = 13;
    err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error changing WiFi channel (%d)!", err);
}
#endif

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS)
        ESP_LOGE(TAG, "Sending data fail!");
//    xSemaphoreGiveFromISR(send_semaphore, NULL);
}

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
#ifdef SERVER
    if (data_len == sizeof(espnow_ack_t)) {
        espnow_ackinfo_t ack;

        ack.timestamp = esp_now_info->rx_ctrl->timestamp;
        ack.rssi = esp_now_info->rx_ctrl->rssi;
        memcpy(&ack.data, data, sizeof(ack.data));
        xQueueSendFromISR(receive_queue, &ack, NULL);
    }
#else
    static uint32_t last_data = 0;

    if (data_len == ESPNOW_DATALEN) {
#if ESPNOW_CHANNEL == 0
        if (find_timer) {
            esp_timer_stop(find_timer);
            esp_timer_delete(find_timer);
            find_timer = NULL;
            ESP_LOGI(TAG, "Found server channel %u", channel);
        }
#endif

        if (! server_mac[0]) {
            esp_now_peer_info_t peer;
            esp_err_t err;

            memcpy(server_mac, esp_now_info->src_addr, sizeof(server_mac));
            memset(&peer, 0, sizeof(peer));
            memcpy(peer.peer_addr, server_mac, sizeof(peer.peer_addr));
//            peer.channel = 0; // ESPNOW_CHANNEL
            peer.ifidx = ESPNOW_IF;
            err = esp_now_add_peer(&peer);
            if (err == ESP_OK) {
                const esp_now_rate_config_t config = {
                    .phymode = WIFI_PHY_MODE_LR,
                    .rate = ESPNOW_RATE
                };

                err = esp_now_set_peer_rate_config(server_mac, (esp_now_rate_config_t*)&config);
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "Error setting server peer rate (%d)!", err);
            } else
                ESP_LOGE(TAG, "Error adding server peer (%d)!", err);
        } else if (memcmp(server_mac, esp_now_info->src_addr, sizeof(server_mac))) {
            ESP_LOGE(TAG, "Second server not allowed!");
        }

        ++ack.received;
        if (last_data) {
            ack.skipped += *(uint32_t*)&data[0] - last_data - 1;
        }
        last_data = *(uint32_t*)&data[0];
        xSemaphoreGiveFromISR(espnow_semaphore, NULL);
    }
#endif
}

static void espnow_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
#ifdef SERVER
    {
        esp_now_peer_info_t peer;

        memset(&peer, 0, sizeof(peer));
        memcpy(peer.peer_addr, broadcast_mac, sizeof(peer.peer_addr));
//        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = ESPNOW_IF;
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    }
    {
        const esp_now_rate_config_t config = {
            .phymode = WIFI_PHY_MODE_LR,
            .rate = ESPNOW_RATE
        };

        ESP_ERROR_CHECK(esp_now_set_peer_rate_config(broadcast_mac, (esp_now_rate_config_t*)&config));
    }
#endif
    {
        uint32_t ver;

        ESP_ERROR_CHECK(esp_now_get_version(&ver));
        ESP_LOGI(TAG, "ESP-NOW version: %lu", ver);
    }
}

void app_main(void) {
    {
        esp_err_t err = nvs_flash_init();

        if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
    }

#ifdef SERVER
//    send_semaphore = xSemaphoreCreateBinary();
//    assert(send_semaphore);

    receive_queue = xQueueCreate(32, sizeof(espnow_ackinfo_t));
    assert(receive_queue);

    {
        const esp_timer_create_args_t cfg = {
            .callback = send_timer_cb,
            .name = "send_timer"
        };

        ESP_ERROR_CHECK(esp_timer_create(&cfg, &send_timer));
    }
#else
    espnow_semaphore = xSemaphoreCreateBinary();
    assert(espnow_semaphore);
#endif

    wifi_init();
    espnow_init();

#ifdef SERVER
    uint32_t last_time = 0;
    uint32_t counter = 0;

    esp_fill_random(broardcast_data, sizeof(broardcast_data));
    *(uint32_t*)broardcast_data = 0;

    ESP_ERROR_CHECK(esp_timer_start_periodic(send_timer, SEND_PERIOD));

    ESP_LOGI(TAG, "Server started");
    while (1) {
        espnow_ackinfo_t ack;

        if (xQueueReceive(receive_queue, &ack, portMAX_DELAY) == pdTRUE) {
            ++counter;
            if (ack.timestamp - last_time >= RECEIVE_PERIOD) {
                ESP_LOGI(TAG, "%lu/%lu/%lu ACK/received/skipped, RSSI %d", counter, ack.data.received, ack.data.skipped, ack.rssi);
                last_time = ack.timestamp;
            }
        }
    }
#else
#if ESPNOW_CHANNEL == 0
    {
        const esp_timer_create_args_t cfg = {
            .callback = find_timer_cb,
            .name = "find_timer"
        };

        ESP_ERROR_CHECK(esp_timer_create(&cfg, &find_timer));
    }
    ESP_ERROR_CHECK(esp_timer_start_periodic(find_timer, 100000)); // 100 ms.
#endif

    ESP_LOGI(TAG, "Client started");
    while (1) {
        if (xSemaphoreTake(espnow_semaphore, portMAX_DELAY) == pdTRUE) {
            esp_err_t err;

            err = esp_now_send(server_mac, (uint8_t*)&ack, sizeof(ack));
            if (err != ESP_OK)
                ESP_LOGE(TAG, "Error sending ACK (%d)!", err);
        }
    }
#endif
}
