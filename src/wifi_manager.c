/*
 * This file is part of the ESP WiFi Manager project.
 * Copyright (C) 2019  Tido Klaassen <tido_wmngr@4gh.eu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */


#include <string.h>
#include <stdatomic.h>
#include <errno.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_wifi_types.h"
#include "esp_wifi.h"
#include "esp_wps.h"
#include "esp_err.h"
#include "nvs_flash.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "lwip/ip4.h"

#include "wifi_manager.h"
#include "kutils.h"
#include "kref.h"

static const char *TAG = "wifimngr";

#define WMNGR_NAMESPACE "esp_wmngr"

#define MAX_AP_CLIENTS  3
#define MAX_NUM_APS     32
#define SCAN_TIMEOUT    (60 * 1000 / portTICK_PERIOD_MS)
#define CFG_TIMEOUT     (60 * 1000 / portTICK_PERIOD_MS)
#define CFG_TICKS       (1000 / portTICK_PERIOD_MS)
#define CFG_DELAY       (100 / portTICK_PERIOD_MS)

struct scan_data_ref {
    struct kref ref_cnt;
    uint32_t status;
    struct scan_data data;
};

/*
 * This holds all the information needed to transition from the current
 * to the requested WiFi configuration. See handle_config_timer() and
 * update_wifi() on how to use this.
 */
struct wifi_cfg_state {
    SemaphoreHandle_t lock;
    TickType_t cfg_timestamp;
    enum wmngr_state state;
    struct wifi_cfg saved;
    struct wifi_cfg current;
    struct wifi_cfg new;
    TickType_t scan_timestamp;
    struct scan_data_ref *scan_ref;
};

const char *wmngr_state_names[wmngr_state_max] = {
    "Failed",
    "Connected",
    "Idle",
    "Update",
    "WPS Start",
    "WPS Active",
    "Connecting",
    "Disconnecting",
    "Fall Back"
};

static struct wifi_cfg_state cfg_state;

/* For keeping track of system events. */
#define BIT_TRIGGER             BIT0
#define BIT_STA_START           BIT1
#define BIT_STA_CONNECTED       BIT2
#define BIT_STA_GOT_IP          BIT3
#define BIT_AP_START            BIT4
#define BIT_SCAN_START          BIT5
#define BIT_SCAN_RUNNING        BIT6
#define BIT_SCAN_DONE           BIT7
#define BIT_WPS_SUCCESS         BIT8
#define BIT_WPS_FAILED          BIT9
#define BITS_WPS    (BIT_WPS_SUCCESS | BIT_WPS_FAILED)

static EventGroupHandle_t wifi_events = NULL;

static TimerHandle_t *config_timer = NULL;

static void handle_timer(TimerHandle_t timer);
static void event_handler(void* args, esp_event_base_t base,
                          int32_t id, void* data);
static esp_err_t get_saved_config(struct wifi_cfg *cfg);

/** Set configuration from compiled-in defaults.
 */
static void set_defaults(struct wifi_cfg *cfg)
{
    size_t len;

    memset(cfg, 0x0, sizeof(*cfg));
    cfg->is_default = true;
    cfg->mode = WIFI_MODE_APSTA;
   
    if(!(ip4addr_aton(CONFIG_WMNGR_AP_IP, &(cfg->ap_ip_info.ip)))){
        ESP_LOGE(TAG, "[%s] Invalid default AP IP: %s. "
                      "Using 192.168.4.1 instead.",
                      __func__, CONFIG_WMNGR_AP_IP);
        IP4_ADDR(&(cfg->ap_ip_info.ip), 192, 168, 4, 1);
    }

    if(!(ip4addr_aton(CONFIG_WMNGR_AP_MASK, &(cfg->ap_ip_info.netmask)))){
        ESP_LOGE(TAG, "[%s] Invalid default AP netmask: %s. "
                      "Using 255.255.255.0 instead.",
                      __func__, CONFIG_WMNGR_AP_MASK);
        IP4_ADDR(&(cfg->ap_ip_info.netmask), 255, 255, 255, 0);
    }

    if(!(ip4addr_aton(CONFIG_WMNGR_AP_GW, &(cfg->ap_ip_info.gw)))){
        ESP_LOGE(TAG, "[%s] Invalid default AP GW: %s. "
                      "Using 192.168.4.1 instead.",
                      __func__, CONFIG_WMNGR_AP_GW);
        IP4_ADDR(&(cfg->ap_ip_info.gw), 192, 168, 4, 1);
    }

    len = strlen(CONFIG_WMNGR_AP_SSID);
    if(len > 0 && len <= sizeof(cfg->ap.ap.ssid)){
        memcpy(cfg->ap.ap.ssid, CONFIG_WMNGR_AP_SSID, len);
    } else {
        ESP_LOGE(TAG, "[%s] Invalid default AP SSID: %s. "
                      "Using \"ESP WiFi Manager\" instead.",
                      __func__, CONFIG_WMNGR_AP_SSID);

        len = MIN(strlen("ESP WiFi Manager"), sizeof(cfg->ap.ap.ssid));
        memcpy(cfg->ap.ap.ssid, "ESP WiFi Manager", len);
    }
    cfg->ap.ap.ssid_len = len;
}

/* Free scan data, should only be called through kref_put(). */
static void free_scan_data(struct kref *ref)
{
    struct scan_data_ref *data;

    data = container_of(ref, struct scan_data_ref, ref_cnt);
    free(data->data.ap_records);
    free(data);
}

/** Fetch the latest AP scan data and make it available.
 * Fetch the latest set of AP scan results and make them available to the
 * users. The SCAN_RUNNING and SCAN_DONE flags will be cleared on success or
 * unrecoverable error.
 */
static void wifi_scan_done(void)
{
    uint16_t num_aps;
    struct scan_data_ref *old, *new;
    esp_err_t result;

    result = ESP_OK;
    new = NULL;

    /* cgiWifiSetup() must have been called prior to this point. */
    configASSERT(cfg_state.lock != NULL);

    /* Fetch number of APs found. Bail out early if there is nothing to get. */
    result = esp_wifi_scan_get_ap_num(&num_aps);
    if(result != ESP_OK || num_aps == 0){
        /* Something went seriously wrong, no point in trying again. */
        ESP_LOGI(TAG, "Scan error or empty scan result");
        xEventGroupClearBits(wifi_events, (BIT_SCAN_RUNNING | BIT_SCAN_DONE));
        goto on_exit;
    }

    /*
     * Limit number of records to fetch. Prevents possible DoS by tricking
     * us into allocating storage for a very large amount of scan results.
     */
    if(num_aps > MAX_NUM_APS){
        ESP_LOGI(TAG, "Limiting AP records to %d (Actually found %d)",
                 MAX_NUM_APS, num_aps);
        num_aps = MAX_NUM_APS;
    }

    /* Allocate and initialise memory for scan data and AP records. */
    new = calloc(1, sizeof(*new));
    if(new == NULL){
        ESP_LOGE(TAG, "Out of memory creating scan data");
        goto on_exit;
    }

    kref_init(&(new->ref_cnt)); // initialises ref_cnt to 1
    new->data.ap_records = calloc(num_aps, sizeof(*(new->data.ap_records)));
    if(new->data.ap_records == NULL){
        ESP_LOGE(TAG, "Out of memory for fetching records");
        goto on_exit;
    }

    /* Fetch actual AP scan data */
    new->data.tstamp = xTaskGetTickCount();
    new->data.num_records = num_aps;
    result = esp_wifi_scan_get_ap_records(&(new->data.num_records),
                                          new->data.ap_records);

    /*
     * Scan data has either been fetched or lost at this point, so
     * clear flags irregardless of returned status.
     */
    xEventGroupClearBits(wifi_events, (BIT_SCAN_RUNNING | BIT_SCAN_DONE));

    if(result != ESP_OK){
        ESP_LOGE(TAG, "Error getting scan results");
        goto on_exit;
    }

    ESP_LOGI(TAG, "Scan done: found %d APs", num_aps);

    /*
     * Make new scan data available.
     * The new data set will be assigned to the global pointer. Fetch
     * another reference so it will not be freed on function exit.
     */
    kref_get(&(new->ref_cnt));

    old = cfg_state.scan_ref;
    cfg_state.scan_ref = new;

    if(old != NULL){
        /*
         * Drop global reference to old data set so it will be freed
         * when the last connection using it gets closed.
         */
        esp_wmngr_put_scan(&(old->data));
    }

on_exit:
    /* Drop one reference to the new scan data. */
    if(new != NULL){
        esp_wmngr_put_scan(&(new->data));
    }
}

/** Start AP scan.
 */
static void wifi_scan_start(void)
{
    wifi_scan_config_t scan_cfg;
    EventBits_t events;
    wifi_mode_t mode;
    esp_err_t result;

    /*
     * Make sure we do not try to start a scan while the WiFi config is
     * in a transitional state. If we bail out here, the SCAN_START bit
     * will be kept set and the scan will start once the WiFi config has
     * settled down again.
     */
    if(cfg_state.state > wmngr_state_idle){
        ESP_LOGI(TAG, "[%s] WiFi connecting, not starting scan.",
                 __func__);
        goto on_exit;
    }

    /* WiFi config is in a stable state, clear the SCAN_START bit. */
    xEventGroupClearBits(wifi_events, BIT_SCAN_START);

    /* Check that we are in a suitable mode for scanning. */
    result =  esp_wifi_get_mode(&mode);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching WiFi mode.", __func__);
        goto on_exit;
    }

    if(mode != WIFI_MODE_APSTA && mode != WIFI_MODE_STA){
        ESP_LOGE(TAG, "[%s] Invalid WiFi mode for scanning.", __func__);
        goto on_exit;
    }

    events = xEventGroupGetBits(wifi_events);

    /* Finally, start a scan. Unless there is one running already. */
    if(!(events & (BIT_SCAN_RUNNING | BIT_SCAN_DONE))){
        ESP_LOGI(TAG, "[%s] Starting scan.", __func__);

        memset(&scan_cfg, 0x0, sizeof(scan_cfg));
        scan_cfg.show_hidden = true;
        scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;

        xEventGroupSetBits(wifi_events, BIT_SCAN_START);
        result = esp_wifi_scan_start(&scan_cfg, false);
        if(result == ESP_OK){
            ESP_LOGI(TAG, "[%s] Scan started.", __func__);
            xEventGroupSetBits(wifi_events, BIT_SCAN_RUNNING);
        } else {
            ESP_LOGE(TAG, "[%s] Starting AP scan failed.", __func__);
        }
    } else {
        ESP_LOGI(TAG, "[%s] Scan aleady running.", __func__);
    }

on_exit:
    return;
}

/** Read saved configuration from NVS.
 *
 * Read configuration from NVS and store it in the struct wifi_cfg.
 * @param[out] cfg Configuration read from NVS.
 * @return ESP_OK if valid configuration was found in NVS, ESP_ERR_* otherwise.
 */
static esp_err_t get_saved_config(struct wifi_cfg *cfg)
{
    nvs_handle handle;
    size_t len;
    uint32_t tmp;
    esp_err_t result;

    result = ESP_OK;

    memset(cfg, 0x0, sizeof(*cfg));

    result = nvs_open(WMNGR_NAMESPACE, NVS_READONLY, &handle);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] nvs_open() failed.", __func__);
        return result;
    }

    /* Read back the base type components of the struct wifi_cfg. */
    result = nvs_get_u32(handle, "mode", &(tmp));
    if(result != ESP_OK){
        goto on_exit;
    }
    cfg->mode = (wifi_mode_t) tmp;

    result = nvs_get_u32(handle, "sta_static", &tmp);
    if(result != ESP_OK){
        goto on_exit;
    }
    cfg->sta_static = (bool) tmp;

    result = nvs_get_u32(handle, "sta_connect", &tmp);
    if(result != ESP_OK){
        goto on_exit;
    }
    cfg->sta_connect = (bool) tmp;

    /*
     * The esp-idf types are stored as binary blobs. This is problematic
     * because their memory layout and padding might change between esp-idf
     * releases or by using a different toolchain or compiler options.
     * We do a very basic sanity check by comparing the types' sizes to the
     * records' lengths, but this is not guaranteed to catch every case.
     */

    len = sizeof(cfg->ap);
    result = nvs_get_blob(handle, "ap", &(cfg->ap), &len);
    if(result != ESP_OK || len != sizeof(cfg->ap)){
        result = (result != ESP_OK) ? result : ESP_ERR_NOT_FOUND;
        goto on_exit;
    }

    len = sizeof(cfg->sta);
    result = nvs_get_blob(handle, "sta", &(cfg->sta), &len);
    if(result != ESP_OK || len != sizeof(cfg->sta)){
        result = (result != ESP_OK) ? result : ESP_ERR_NOT_FOUND;
        goto on_exit;
    }

    len = sizeof(cfg->ap_ip_info);
    result = nvs_get_blob(handle, "ap_ip", &(cfg->ap_ip_info), &len);
    if(result != ESP_OK || len != sizeof(cfg->ap_ip_info)){
        result = (result != ESP_OK) ? result : ESP_ERR_NOT_FOUND;
        goto on_exit;
    }

    len = sizeof(cfg->sta_ip_info);
    result = nvs_get_blob(handle, "sta_ip", &(cfg->sta_ip_info), &len);
    if(result != ESP_OK || len != sizeof(cfg->sta_ip_info)){
        result = (result != ESP_OK) ? result : ESP_ERR_NOT_FOUND;
        goto on_exit;
    }

    len = sizeof(cfg->sta_dns_info);
    result = nvs_get_blob(handle, "sta_dns", &(cfg->sta_dns_info), &len);
    if(result != ESP_OK || len != sizeof(cfg->sta_dns_info)){
        result = (result != ESP_OK) ? result : ESP_ERR_NOT_FOUND;
        goto on_exit;
    }

on_exit:
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Reading config failed.", __func__);
    }

    nvs_close(handle);
    return result;
}

/** Save configuration to NVS.
 *
 * Store the wifi_cfg in NVS. The previously stored configuration will be
 * erased and not be recovered on error, so on return there will either be
 * a valid config or no config at all stored in the NVS.
 * This guarantees that the device is either reachable by the last valid
 * configuration or recoverable by the factory default settings.
 *
 * @param[in] cfg Configuration to be saved.
 * @return ESP_OK if configuration was saved, ESP_ERR_* otherwise.
 */
static esp_err_t save_config(struct wifi_cfg *cfg)
{
    nvs_handle handle;
    esp_err_t result;

    result = nvs_open(WMNGR_NAMESPACE, NVS_READWRITE, &handle);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] nvs_open() failed.", __func__);
        return result;
    }

    /*
     * Erase the previous config so that we can be sure that we do not end up
     * with a mix of the old and new in case of a power-fail.
     *
     * FIXME: We should use a two slot mechanism so that the old config will
     *        not be touched until the new one has been written successfully.
     */
    result = nvs_erase_all(handle);
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_commit(handle);
    if(result != ESP_OK){
        goto on_exit;
    }

    /* No point in saving the factory default settings. */
    if(cfg->is_default){
        result = ESP_OK;
        goto on_exit;
    }

    /*
     * Write all elements of the struct wifi_cfg individually. This gives
     * us a chance to extend it later without forcing the user into a
     * "factory reset" after a firmware update.
     */

    result = nvs_set_u32(handle, "mode", cfg->mode);
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_set_u32(handle, "sta_static", cfg->sta_static);
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_set_u32(handle, "sta_connect", cfg->sta_connect);
    if(result != ESP_OK){
        goto on_exit;
    }

    /* Store the esp-idf types as blobs. */
    /* FIXME: we should also store them component-wise. */
    result = nvs_set_blob(handle, "ap", &(cfg->ap), sizeof(cfg->ap));
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_set_blob(handle, "sta", &(cfg->sta), sizeof(cfg->sta));
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_set_blob(handle, "ap_ip", &(cfg->ap_ip_info),
                            sizeof(cfg->ap_ip_info));
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_set_blob(handle, "sta_ip", &(cfg->sta_ip_info),
                            sizeof(cfg->sta_ip_info));
    if(result != ESP_OK){
        goto on_exit;
    }

    result = nvs_set_blob(handle, "sta_dns", &(cfg->sta_dns_info),
                            sizeof(cfg->sta_dns_info));
    if(result != ESP_OK){
        goto on_exit;
    }

on_exit:
    if(result != ESP_OK){
        /* we do not want to leave a half-written config lying around. */
        ESP_LOGE(TAG, "[%s] Writing config failed.", __func__);
        (void) nvs_erase_all(handle);
    }

    (void) nvs_commit(handle);
    nvs_close(handle);

    return result;
}

/* Helper function to check if WiFi is connected in station mode. */
static bool sta_connected(void)
{
    EventBits_t events;

    events = xEventGroupGetBits(wifi_events);

    return !!(events & BIT_STA_CONNECTED);
}

/* Helper function to set WiFi configuration from struct wifi_cfg. */
static esp_err_t set_wifi_cfg(struct wifi_cfg *cfg)
{
    unsigned int idx;
    esp_err_t result;

    ESP_LOGD(TAG, "[%s] Called.", __FUNCTION__);

    /*
     * FIXME: we should check for errors. OTOH, this is also used
     *        for the fall-back mechanism, so aborting on error is
     *        probably a bad idea.
     */

    memmove(&cfg_state.current, cfg, sizeof(*cfg));

    result = esp_wifi_restore();
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_wifi_restore(): %d %s",
                 __func__, result, esp_err_to_name(result));
    }

    result = esp_wifi_set_mode(cfg->mode);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_wifi_set_mode(): %d %s",
                 __func__, result, esp_err_to_name(result));
    }

    if(cfg->mode == WIFI_MODE_APSTA || cfg->mode == WIFI_MODE_AP){
        cfg->ap.ap.max_connection = MAX_AP_CLIENTS;
        result = esp_wifi_set_config(WIFI_IF_AP, &(cfg->ap));
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] esp_wifi_set_config() AP: %d %s",
                     __func__, result, esp_err_to_name(result));
        }
    }

    if(cfg->mode == WIFI_MODE_APSTA || cfg->mode == WIFI_MODE_STA){
        result = esp_wifi_set_config(WIFI_IF_STA, &(cfg->sta));
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] esp_wifi_set_config() STA: %d %s",
                     __func__, result, esp_err_to_name(result));
        }
        if(cfg->sta_static){
            (void) tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
            for(idx = 0; idx < ARRAY_SIZE(cfg->sta_dns_info); ++idx){
                if(ip_addr_isany_val(cfg->sta_dns_info[idx].ip)){
                    continue;
                }

                result = tcpip_adapter_set_dns_info(TCPIP_ADAPTER_IF_STA,
                                                    idx,
                                                    &(cfg->sta_dns_info[idx]));
                if(result != ESP_OK){
                    ESP_LOGE(TAG, "[%s] Setting DNS server IP failed.", 
                            __func__);
                    goto on_exit;
                }
            }
        } else {
            (void) tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);
        }
    }

    result = esp_wifi_start();
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_wifi_start(): %d %s",
                 __func__, result, esp_err_to_name(result));
    }

    if(cfg->sta_connect
       && (   cfg->mode == WIFI_MODE_STA
           || cfg->mode == WIFI_MODE_APSTA))
    {
        result = esp_wifi_connect();
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] esp_wifi_connect(): %d %s",
                     __func__, result, esp_err_to_name(result));
        }
    }

on_exit:
    return result;
}

/* Helper to store current WiFi configuration into a struct wifi_cfg. */
static esp_err_t get_wifi_cfg(struct wifi_cfg *cfg)
{
    tcpip_adapter_dhcp_status_t dhcp_status;
    unsigned int idx;
    esp_err_t result;

    result = ESP_OK;
    memset(cfg, 0x0, sizeof(*cfg));

    cfg->sta_connect = sta_connected();

    result = esp_wifi_get_mode(&(cfg->mode));
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching WiFi mode.", __func__);
        goto on_exit;
    }

    result = esp_wifi_get_config(WIFI_IF_STA, &(cfg->sta));
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching STA config.", __func__);
        goto on_exit;
    }

    result = tcpip_adapter_dhcpc_get_status(WIFI_IF_STA, &dhcp_status);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching DHCP status.", __func__);
        goto on_exit;
    }

    if(dhcp_status == TCPIP_ADAPTER_DHCP_STOPPED){
        cfg->sta_static = 1;
        for(idx = 0; idx < ARRAY_SIZE(cfg->sta_dns_info); ++idx){
            result = tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA,
                                                idx,
                                                &(cfg->sta_dns_info[idx]));
            if(result != ESP_OK){
                ESP_LOGE(TAG, "[%s] Getting DNS server IP failed.", 
                         __func__);
                goto on_exit;
            }

        }
    }

    result = esp_wifi_get_config(WIFI_IF_AP, &(cfg->ap));
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching AP config.", __func__);
        goto on_exit;
    }

on_exit:
    return result;
}

/* Helper function to update the STA connect setting of the current config */
static esp_err_t set_connect(bool connect)
{
    struct wifi_cfg cfg;
    esp_err_t result;

    result = esp_wmngr_get_cfg(&cfg);
    if(result != ESP_OK){
        goto on_exit;
    }

    if(cfg.mode != WIFI_MODE_APSTA && cfg.mode != WIFI_MODE_STA){
        result = ESP_ERR_INVALID_STATE;
        goto on_exit;
    }

    cfg.sta_connect = connect;

    result = esp_wmngr_set_cfg(&cfg);

on_exit:
    return result;
}

/*
 * This function is called from the config_timer and handles all WiFi
 * configuration changes. It takes its information from the global
 * cfg_state struct and tries to set the WiFi configuration to the one
 * found in the "new" member. If things go wrong, it will try to fall
 * back to the configuration found in "saved". This should minimise
 * the risk of users locking themselves out of the device by setting
 * wrong WiFi credentials in STA-only mode.
 *
 * This function will keep triggering itself until it reaches a "stable"
 * (idle, connected, failed) state in cfg_state.state.
 *
 * cfg_state must not be modified without first obtaining the cfg_state.lock
 * mutex and then checking that cfg_state.state is in a stable state.
 * To set a new configuration, just store the current config to .saved,
 * update .new to the desired config, set .state to wmngr_state_update
 * and start the config_timer.
 * To connect to an AP with WPS, save the current state, set .state
 * to wmngr_state_wps_start and start the config_timer.
 */
static void handle_wifi(TimerHandle_t timer)
{
    bool connected;
    wifi_mode_t mode;
    esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);
    TickType_t now, delay;
    EventBits_t events;
    esp_err_t result;

    ESP_LOGD(TAG, "[%s] Called.\n", __FUNCTION__);

    /*
     * If we can not get the config state lock, we try to reschedule the
     * timer. If that also fails, we are SOL...
     * Maybe we should trigger a reboot.
     */
    if(xSemaphoreTake(cfg_state.lock, 0) != pdTRUE){
        if(xTimerChangePeriod(config_timer, CFG_DELAY, CFG_DELAY) != pdPASS){
            ESP_LOGE(TAG, "[%s] Failure to get config lock and change timer.",
                     __func__);
            /* FIXME: should we restart the device? */
        }
        return;
    }

    ESP_LOGD(TAG, "[%s] Called. State: %s",
             __func__, wmngr_state_names[cfg_state.state]);

    /* If delay gets set later, the timer will be re-scheduled on exit. */
    delay = 0;

    /* Gather various information about the current system state. */
    connected = sta_connected();
    events = xEventGroupGetBits(wifi_events);
    now = xTaskGetTickCount();

    result = esp_wifi_get_mode(&mode);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching WiFi mode.", __func__);
        cfg_state.state = wmngr_state_failed;
        goto on_exit;
    }

    switch(cfg_state.state){
    case wmngr_state_wps_start:
        ESP_LOGI(TAG, "[%s] Starting WPS.", __func__);
        /*
         * Try connecting to AP with WPS. First, tear down any connection
         * we might currently have.
         */
        result = get_wifi_cfg(&cfg_state.new);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] WPS start: Error getting current config.",
                     __func__);
            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
            goto on_exit;
        }

        memset(&cfg_state.new.sta, 0x0, sizeof(cfg_state.new.sta));
        cfg_state.new.mode = WIFI_MODE_APSTA;
        cfg_state.new.sta_connect = false;

        result = set_wifi_cfg(&cfg_state.new);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] WPS start: Error setting temp config.",
                     __func__);
            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
            goto on_exit;
        }

        /* Clear previous results and start WPS. */
        xEventGroupClearBits(wifi_events, BITS_WPS);
        result = esp_wifi_wps_enable(&config);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] esp_wifi_wps_enable() failed: %d %s",
                     __func__, result, esp_err_to_name(result));
            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
            goto on_exit;
        }

        result = esp_wifi_wps_start(0);
        if(result != ESP_OK){
            ESP_LOGE(TAG, "[%s] esp_wifi_wps_start() failed: %d %s",
                     __func__, result, esp_err_to_name(result));
            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
            goto on_exit;
        }

        /* WPS is running, set time stamp and transition to next state. */
        cfg_state.cfg_timestamp = now;
        cfg_state.state = wmngr_state_wps_active;
        delay = CFG_TICKS;
        break;
    case wmngr_state_wps_active:
        /* WPS is running. Check for events and timeout. */
        if(events & BIT_WPS_SUCCESS){
            /* WPS succeeded. Disable WPS and use the received credentials *\
             * to connect to the AP by transitioning to the updating state.*/
            ESP_LOGI(TAG, "[%s] WPS success.", __func__);
            result = esp_wifi_wps_disable();
            if(result != ESP_OK){
                ESP_LOGE(TAG, "[%s] wifi wps disable: %d %s",
                        __func__, result, esp_err_to_name(result));
            }

            /*
             * Get received STA config, then force APSTA mode, set
             * connect flag and trigger update.
             */
            get_wifi_cfg(&cfg_state.new);
            cfg_state.new.mode = WIFI_MODE_APSTA;
            cfg_state.new.sta_connect = true;
            cfg_state.state = wmngr_state_update;
            delay = CFG_DELAY;
        } else if(time_after(now, (cfg_state.cfg_timestamp + CFG_TIMEOUT))
                  || (events & BIT_WPS_FAILED))
        {
            /* Failure or timeout. Trigger fall-back to the previous config. */
            ESP_LOGI(TAG, "[%s] WPS failed, restoring saved config.",
                     __func__);

            result = esp_wifi_wps_disable();
            if(result != ESP_OK){
                ESP_LOGE(TAG, "[%s] wifi wps disable: %d %s",
                        __func__, result, esp_err_to_name(result));
            }

            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
        } else {
            /* Still waiting. Set up next check. */
            delay = CFG_TICKS;
        }
        break;
    case wmngr_state_update:
        ESP_LOGI(TAG, "[%s] Setting new configuration.", __func__);
        /* Start changing WiFi to new configuration. */
        (void) esp_wifi_scan_stop();
        (void) esp_wifi_disconnect();
        result = set_wifi_cfg(&(cfg_state.new));
        if(result != ESP_OK){
            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
            goto on_exit;
        }

        if(cfg_state.new.mode == WIFI_MODE_AP || !cfg_state.new.sta_connect){
            /* AP-only mode or not connecting, we are done. */
            cfg_state.state = wmngr_state_idle;
        } else {
            /* System should now connect to the AP. */
            cfg_state.cfg_timestamp = now;
            cfg_state.state = wmngr_state_connecting;
            delay = CFG_TICKS;
        }
        break;
    case wmngr_state_connecting:
        /* We are waiting for a connection to an AP. */
        if(connected){
            /* We have a connection! \o/ */
            ESP_LOGI(TAG, "[%s] Established connection to AP.", __func__);
            cfg_state.state = wmngr_state_connected;
            result = save_config(&cfg_state.new);
            if(result != ESP_OK){
                ESP_LOGE(TAG, "[%s] Saving config failed.", __func__);
            }
        } else if(time_after(now, (cfg_state.cfg_timestamp + CFG_TIMEOUT))){
            /*
             * Timeout while waiting for connection. Try falling back to the
             * saved configuration.
             */
            ESP_LOGI(TAG, "[%s] Timed out waiting for connection to AP.",
                        __func__);
            cfg_state.state = wmngr_state_fallback;
            delay = CFG_DELAY;
        } else {
            /* Twiddle our thumbs and keep waiting for the connection.  */
            delay = CFG_TICKS;
        }
        break;
    case wmngr_state_disconnecting:
        break;
    case wmngr_state_fallback:
        /* Something went wrong, try going back to the previous config. */
        ESP_LOGI(TAG, "[%s] Falling back to previous configuration.",
                    __func__);
        (void) esp_wifi_disconnect();
        (void) set_wifi_cfg(&(cfg_state.saved));
        cfg_state.state = wmngr_state_failed;
        break;
    case wmngr_state_connected:
        if(!connected){
            /*
             * We should be connected, but are not. Change into update state
             * so current configuration gets re-applied.
             */
            ESP_LOGI(TAG, "[%s] Connection to AP lost, retrying.", __func__);
            cfg_state.state = wmngr_state_update;
            delay = CFG_DELAY;
        }
        break;
    case wmngr_state_idle:
    case wmngr_state_failed:
        break;
    default:
        ESP_LOGE(TAG, "[%s] Illegal state: 0x%x", __func__, cfg_state.state);
        cfg_state.state = wmngr_state_failed;
    }

    if(cfg_state.state <= wmngr_state_idle){
        if(events & BIT_SCAN_START){
            wifi_scan_start();
        } else if(events & BIT_SCAN_DONE){
            wifi_scan_done();
        }

        /* Check the SCAN bits and re-schedule if necessary. */
        events = xEventGroupGetBits(wifi_events);
        if(events & (BIT_SCAN_START | BIT_SCAN_DONE)){
            delay = CFG_DELAY;
        }
    }

on_exit:
    xSemaphoreGive(cfg_state.lock);

    if(delay > 0){
        /* We are in a transitional state, re-arm the timer. */
        if(xTimerChangePeriod(config_timer, delay, CFG_DELAY) != pdPASS){
            cfg_state.state = wmngr_state_failed;
        }
    }

    ESP_LOGD(TAG, "[%s] Leaving. State: %s delay: %d",
             __func__, wmngr_state_names[cfg_state.state], delay);

    return;
}

static void handle_timer(TimerHandle_t timer)
{
    ESP_LOGD(TAG, "[%s] Called.\n", __FUNCTION__);
#if defined(CONFIG_WMNGR_TASK)
    /* Reset timer to regular tick rate and trigger the task. */
    (void) xTimerChangePeriod(timer, CFG_TICKS, CFG_DELAY);
    xEventGroupSetBits(wifi_events, BIT_TRIGGER);
#else
    handle_wifi(timer);
#endif
}

/*
 * Update state information from system events. This function must be
 * called from the main event handler to keep this module updated about
 * the current system state.
 */
static void event_handler(void* args, esp_event_base_t base,
                          int32_t id, void* data)
{
    EventBits_t old, new;
    wifi_event_sta_scan_done_t *scan_data;

    if(base != WIFI_EVENT && base != IP_EVENT){
        ESP_LOGE(TAG, "[%s] Got event for wrong base.", __func__);
        goto on_exit;
    }

    old = xEventGroupGetBits(wifi_events);

    if(base == WIFI_EVENT){
        switch(id){
        case WIFI_EVENT_SCAN_DONE:
            scan_data = (wifi_event_sta_scan_done_t *) data;
            if(scan_data->status == ESP_OK){
                xEventGroupSetBits(wifi_events, BIT_SCAN_DONE);
            }
            xEventGroupClearBits(wifi_events, BIT_SCAN_START);
            break;
        case WIFI_EVENT_STA_START:
            xEventGroupSetBits(wifi_events, BIT_STA_START);
            break;
        case WIFI_EVENT_STA_STOP:
            xEventGroupClearBits(wifi_events, BIT_STA_START);
            break;
        case WIFI_EVENT_STA_CONNECTED:
            xEventGroupSetBits(wifi_events, BIT_STA_CONNECTED);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(wifi_events, BIT_STA_CONNECTED);
            break;
        case WIFI_EVENT_AP_START:
            xEventGroupSetBits(wifi_events, BIT_AP_START);
            break;
        case WIFI_EVENT_AP_STOP:
            xEventGroupClearBits(wifi_events, BIT_AP_START);
            break;
        case WIFI_EVENT_STA_WPS_ER_SUCCESS:
            xEventGroupSetBits(wifi_events, BIT_WPS_SUCCESS);
            break;
        case WIFI_EVENT_STA_WPS_ER_FAILED:
        case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
        case WIFI_EVENT_STA_WPS_ER_PIN:
            xEventGroupSetBits(wifi_events, BIT_WPS_FAILED);
            break;
        default:
            break;
        }
    }

    if(base == IP_EVENT){
        switch(id){
        case IP_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_events, BIT_STA_GOT_IP);
            break;
        case IP_EVENT_STA_LOST_IP:
            xEventGroupClearBits(wifi_events, BIT_STA_GOT_IP);
            break;
        default:
            break;
        }
    }

    new = xEventGroupGetBits(wifi_events);

    if(old != new){
#if defined(CONFIG_WMNGR_TASK)
        xEventGroupSetBits(wifi_events, BIT_TRIGGER);
#else
        if(xTimerChangePeriod(config_timer, CFG_DELAY, CFG_DELAY) != pdPASS){
            cfg_state.state = wmngr_state_failed;
        }
#endif
    }

on_exit:
    return;
}

#if defined(CONFIG_WMNGR_TASK)
void esp_wmngr_task(void *pvParameters)
{
    do{
        /* Wait for and clear timer bit */
        (void) xEventGroupWaitBits(wifi_events, BIT_TRIGGER,
                                   true, false, portMAX_DELAY);

        xEventGroupClearBits(wifi_events, BIT_TRIGGER);

        handle_wifi(config_timer);
    } while(1);
}
#endif // defined(CONFIG_WMNGR_TASK)

/*****************************************************************************\ 
 *  API functions                                                            *
\*****************************************************************************/

/** Initialise the WiFi Manager.
 *
 * Calling this function will initialise the WiFi Manger. It must be called
 * after initialising the NVS, default event loop, and TCP adapter and before
 * calling any other esp_wmngr function.
 *
 * @return ESP_OK on success, ESP_ERR_* otherwise.
 */
esp_err_t esp_wmngr_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    BaseType_t status;
    esp_err_t result;

    configASSERT(wifi_events == NULL);
    configASSERT(cfg_state.lock == NULL);
    configASSERT(config_timer == NULL);

    result = ESP_OK;
    memset(&cfg_state, 0x0, sizeof(cfg_state));
    cfg_state.state = wmngr_state_idle;

    wifi_events = xEventGroupCreate();
    if(wifi_events == NULL){
        ESP_LOGE(TAG, "Unable to create event group.");
        result = ESP_ERR_NO_MEM;
        goto on_exit;
    }

    cfg_state.lock = xSemaphoreCreateMutex();
    if(cfg_state.lock == NULL){
        ESP_LOGE(TAG, "Unable to create state lock.");
        result = ESP_ERR_NO_MEM;
        goto on_exit;
    }

    result = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &event_handler, NULL);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_event_handler_register() failed", __func__);
        goto on_exit;
    }

    result = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                        &event_handler, NULL);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_event_handler_register() failed", __func__);
        goto on_exit;
    }

    /*
     * Restore saved WiFi config or fall back to compiled-in defaults.
     * Setting state to update will trigger applying this config.
     */
    set_defaults(&cfg_state.saved);
    result = get_saved_config(&cfg_state.new);
    if(result != ESP_OK){
        ESP_LOGI(TAG, "[%s] No saved config found, setting defaults",
                 __func__);
        set_defaults(&cfg_state.new);
    }
    cfg_state.state = wmngr_state_update;

    tcpip_adapter_init();

    result = esp_wifi_init(&cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_wifi_init() failed", __func__);
        goto on_exit;
    }

    result = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] esp_wifi_set_storage() failed", __func__);
        goto on_exit;
    }

#if defined(CONFIG_WMNGR_TASK)
    config_timer = xTimerCreate("WMngr_Timer",
                              CFG_TICKS,
                              pdTRUE, NULL, handle_timer);
#else
    config_timer = xTimerCreate("WMngr_Timer",
                              CFG_TICKS,
                              pdFALSE, NULL, handle_timer);
#endif /* defined(CONFIG_WMNGR_TASK) */

    if(config_timer == NULL){
        ESP_LOGE(TAG, "[%s] Failed to create config validation timer",
                 __func__);
        result = ESP_ERR_NO_MEM;
        goto on_exit;
    }

    status = xTimerStart(config_timer, CFG_TICKS);
    if(status != pdPASS){
        ESP_LOGE(TAG, "[%s] Starting config timer failed.", __func__);
        result = ESP_ERR_NO_MEM;
        goto on_exit;
    }

#if defined(CONFIG_WMNGR_TASK)
    status = xTaskCreate(&esp_wmngr_task, "WMngr_Task", 
                        CONFIG_WMNGR_TASK_STACK,
                        NULL,
                        CONFIG_WMNGR_TASK_PRIO,
                        NULL);
    if(status != pdPASS){
        ESP_LOGE(TAG, "[%s] Creating WiFi Manager task failed.", __func__);
        result = ESP_ERR_NO_MEM;
    }
#endif

on_exit:
    if(result != ESP_OK){
        if(wifi_events != NULL){
            vEventGroupDelete(wifi_events);
            wifi_events = NULL;
        }

        if(cfg_state.lock != NULL){
            vSemaphoreDelete(cfg_state.lock);
            cfg_state.lock = NULL;
        }

        if(config_timer != NULL){
            xTimerDelete(config_timer, 0);
            config_timer = NULL;
        }
    }

    return result;
}

/** Set a new WiFi Manager configuration.
 *
 * This function is used to set a new WiFi Manager configuration. The current
 * configuration is backed up and an asynchronous update process is triggered.
 *
 * If setting the new configuration succeeds, the state reported by
 * #esp_wmngr_get_state will change to #wmngr_state_connected (in STA or
 * APSTA mode) or #wmngr_state_idle (in AP mode).
 * If the new configuration fails, the device will revert to the previous
 * configuration and set the state to #wmngr_state_failed.
 *
 * @param[in] new New WiFi Manager configuration to be set.
 * @return ESP_OK if update was triggered, ESP_ERR_* otherwise.
 */
esp_err_t esp_wmngr_set_cfg(struct wifi_cfg *new)
{
    bool connected;
    bool update;
    esp_err_t result;

    if(xSemaphoreTake(cfg_state.lock, CFG_DELAY) != pdTRUE){
        ESP_LOGE(TAG, "[%s] Error taking mutex.", __func__);
        return ESP_ERR_TIMEOUT;
    }

    if(cfg_state.state > wmngr_state_idle){
        ESP_LOGI(TAG, "[%s] WiFi change in progress.", __func__);
        result = ESP_ERR_INVALID_STATE;
        goto on_exit;
    }

    result = ESP_OK;

    /* Save current configuration for fall-back. */
    result = get_wifi_cfg(&(cfg_state.saved));
    if(result != ESP_OK){
        ESP_LOGI(TAG, "[%s] Error fetching current WiFi config.",
                 __func__);
        goto on_exit;
    }

    /* Clear station configuration if we are not connected to an AP. */
    connected = sta_connected();
    if(!connected){
        memset(&(cfg_state.saved.sta), 0x0, sizeof(cfg_state.saved.sta));
    }

    memmove(&(cfg_state.new), new, sizeof(cfg_state.new));
    cfg_state.new.is_default = false;
    update = false;

    /*
     * Do some naive checks to see if the new configuration is an actual
     * change. Should be more thorough by actually comparing the elements.
     */
    if(cfg_state.new.mode != cfg_state.saved.mode){
        update = true;
    }

    if((new->mode == WIFI_MODE_AP || new->mode == WIFI_MODE_APSTA)
       && memcmp(&(cfg_state.new.ap), &(cfg_state.saved.ap),
                    sizeof(cfg_state.new.ap)))
    {
        update = true;
    }

    if((new->mode == WIFI_MODE_STA || new->mode == WIFI_MODE_APSTA)
       && memcmp(&(cfg_state.new.sta), &(cfg_state.saved.sta),
                    sizeof(cfg_state.new.sta)))
    {
        update = true;
    }

    /*
     * If new config is different, trigger asynchronous update. This gives
     * the httpd some time to send out the reply before possibly tearing
     * down the connection.
     */
    if(update == true){
        cfg_state.state = wmngr_state_update;
        if(xTimerChangePeriod(config_timer, CFG_DELAY, CFG_DELAY) != pdPASS){
            cfg_state.state = wmngr_state_failed;
            result = ESP_ERR_TIMEOUT;
            goto on_exit;
        }
    }

on_exit:
    xSemaphoreGive(cfg_state.lock);
    return result;
}

/** Get current WiFi Manager configuration.
 * @param[out] cfg Pointer to a #wifi_cfg struct the current configuration
 *             will be copied into.
 * @return ESP_OK on success, ESP_ERR_* otherwise.
 */
esp_err_t esp_wmngr_get_cfg(struct wifi_cfg *cfg)
{
    esp_err_t result;

    if(xSemaphoreTake(cfg_state.lock, CFG_DELAY) != pdTRUE){
        ESP_LOGE(TAG, "[%s] Error taking mutex.", __func__);
        return ESP_ERR_TIMEOUT;
    }

    if(cfg_state.state > wmngr_state_idle){
        ESP_LOGI(TAG, "[%s] WiFi change in progress.", __func__);
        result = ESP_ERR_INVALID_STATE;
        goto on_exit;
    }

    memmove(cfg, &cfg_state.current, sizeof(*cfg));
    result = ESP_OK;

on_exit:
    xSemaphoreGive(cfg_state.lock);
    return result;
}

/** Connect to AP with WPS.
 *
 * Trigger a connection attemp to an AP using WPS. Can only be used if
 * device is in a stable state (idle, connected, failed).
 * @return ESP_OK if WPS is started, ESP_ERR_* otherwise.
 */
esp_err_t esp_wmngr_start_wps(void)
{
    struct wifi_cfg cfg;
    esp_err_t result;

    result = ESP_OK;

    /* Make sure we are not in the middle of setting a new WiFi config. */
    if(xSemaphoreTake(cfg_state.lock, CFG_DELAY) != pdTRUE){
        ESP_LOGE(TAG, "[%s] Error taking mutex.", __func__);
        return ESP_ERR_TIMEOUT;
    }

    if(cfg_state.state > wmngr_state_idle){
        ESP_LOGI(TAG, "[%s] WiFi change in progress.", __func__);
        result = ESP_ERR_INVALID_STATE;
        goto on_exit;
    }

    ESP_LOGI(TAG, "[%s] Starting WPS.", __func__);

    /* Save current config for fall-back. */
    result = get_wifi_cfg(&cfg);
    if(result != ESP_OK){
        ESP_LOGE(TAG, "[%s] Error fetching WiFi config.", __func__);
        goto on_exit;
    }

    memmove(&cfg_state.saved, &cfg, sizeof(cfg_state.saved));
    cfg_state.state = wmngr_state_wps_start;

    if(xTimerChangePeriod(config_timer, CFG_DELAY, CFG_DELAY) != pdTRUE){
        cfg_state.state = wmngr_state_failed;
    }

on_exit:
    xSemaphoreGive(cfg_state.lock);
    return result;
}

/** Start AP scan.
 *
 * Calling this function will trigger a scan for available APs. Scanning
 * will start as soon as the device is in a stable state (idle, connected,
 * failed).
 * Once the scan has completed, the acquired data can be fetched by calling
 * #esp_wmngr_get_scan.
 *
 * @return ESP_OK on success, ESP_ERR_* otherwise.
 */
esp_err_t esp_wmngr_start_scan(void)
{
    esp_err_t result;

    result = ESP_OK;
    xEventGroupSetBits(wifi_events, (BIT_SCAN_START | BIT_TRIGGER));

#if !defined(CONFIG_WMNGR_TASK)
    if(xTimerChangePeriod(config_timer, CFG_DELAY, CFG_DELAY) != pdPASS){
        cfg_state.state = wmngr_state_failed;
        result = ESP_FAIL;
    }
#endif

    return result;
}

/** Get a pointer to a set of AP scan data.
 *
 * Fetches a reference counted pointer to the latest set of AP scan
 * data. Caller must at some point release the data by calling
 * #esp_wmngr_put_scan.
 *
 * @return Pointer to a #scan_data or NULL
 */
struct scan_data *esp_wmngr_get_scan(void)
{
    struct scan_data *data;

    configASSERT(cfg_state.lock != NULL);

    data = NULL;
    if(cfg_state.lock == NULL || cfg_state.scan_ref == NULL){
        goto on_exit;
    }

    if(xSemaphoreTake(cfg_state.lock, CFG_DELAY) == pdTRUE){
        data = &(cfg_state.scan_ref->data);
        kref_get(&(cfg_state.scan_ref->ref_cnt));
        xSemaphoreGive(cfg_state.lock);
    }

on_exit:
    return data;
}

/** Drop a reference to a scan data set, possibly freeing it.
 * @param[in] data Reference to scan data set.
 * @return Void
 */
void esp_wmngr_put_scan(struct scan_data *data)
{
    struct scan_data_ref *data_ref;

    configASSERT(data != NULL);

    data_ref = container_of(data, struct scan_data_ref, data);
    kref_put(&(data_ref->ref_cnt), free_scan_data);
}

/** Query current connection status.
 * @return true if device is connected to AP, false otherwise
 */
bool esp_wmngr_is_connected(void)
{
    return sta_connected();
}

/** Connect to currently configured AP.
 * @return ESP_OK on success, ESP_ERR_* otherwise 
 */
esp_err_t esp_wmngr_connect(void)
{
    return set_connect(true);
}

/** Disconnect from currently configured AP.
 * @return ESP_OK on success, ESP_ERR_* otherwise 
 */
esp_err_t esp_wmngr_disconnect(void)
{
    return set_connect(false);
}

/** Fetch current WiFI Manager state.
 * @return Current state #wmngr_state
 */
enum wmngr_state esp_wmngr_get_state(void)
{
    return cfg_state.state;
}

/** Check if a valid configuration is stored in NVS.
 * @return true if valid config is found, false otherwise.
 */
bool esp_wmngr_nvs_valid(void)
{
    struct wifi_cfg cfg;
    esp_err_t result;

    result = get_saved_config(&cfg);

    return result == ESP_OK;
}
