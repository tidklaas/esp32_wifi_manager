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

#ifndef ESP_WIFI_MANAGER_H
#define ESP_WIFI_MANAGER_H

/** @file */

#include <stdbool.h>
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "tcpip_adapter.h"

/** A set of AP scan data. */
struct scan_data {
    TickType_t tstamp;              //!< Timestamp in FreeRTOS ticks at creation
    wifi_ap_record_t *ap_records;   //!< Array of AP data entries
    uint16_t num_records;           //!< Number of AP entries 
};

/** States used during WiFi (re)configuration. */
enum wmngr_state {
    /* "stable" states */
    wmngr_state_failed,         //!< Connection to AP failed
    wmngr_state_connected,      //!< Device is connected to AP
    wmngr_state_idle,           //!< Device is in AP mode, no STA config set

    /* transitional states */
    wmngr_state_update,         //!< New configuration has been set
    wmngr_state_wps_start,      //!< WPS has been triggered by user
    wmngr_state_wps_active,     //!< WPS is running
    wmngr_state_connecting,     //!< Device is trying to connect to AP
    wmngr_state_disconnecting,  //!< Disconnect from AP has been triggered
    wmngr_state_fallback,       //!< Connection failed, falling back to previous config
    wmngr_state_max,            //!< Number of states
};

/** Array of strings describing current state. */
extern const char *wmngr_state_names[wmngr_state_max];

/* Holds complete WiFi config for both STA and AP, the mode and whether       *\
\* the WiFi should connect to an AP in STA or APSTA mode.                     */
struct wifi_cfg {
    bool is_default;    /*!< True if this is the factory default config. */
    wifi_mode_t mode;   /*!< WiFi mode (AP, AP+STA, STA) */
    wifi_config_t ap;   /*!< Configuration of the AP component. */
    tcpip_adapter_ip_info_t ap_ip_info;
                        /*!< The IP address of the AP interface. */
    wifi_config_t sta;  /*!< Configuration of the STA component. */
    bool sta_static;    /*!< True if STA interface should use static IP and DNS 
                             configuration. When false, DHCP will be used. */
    tcpip_adapter_ip_info_t sta_ip_info;
                        /*!< The IP address of the STA interface in static mode.*/
    tcpip_adapter_dns_info_t sta_dns_info[TCPIP_ADAPTER_DNS_MAX];
                        /*!< IP addresses of DNS servers to use in static IP mode. */
    bool sta_connect;   /*!< True if device should connect to AP in STA mode. */
};

esp_err_t esp_wmngr_init(void);
esp_err_t esp_wmngr_start_scan(void);
struct scan_data *esp_wmngr_get_scan(void);
void esp_wmngr_put_scan(struct scan_data *data);
esp_err_t esp_wmngr_set_cfg(struct wifi_cfg *cfg);
esp_err_t esp_wmngr_get_cfg(struct wifi_cfg *cfg);
esp_err_t esp_wmngr_start_wps(void);
bool esp_wmngr_is_connected(void);
esp_err_t esp_wmngr_connect(void);
esp_err_t esp_wmngr_disconnect(void);
enum wmngr_state esp_wmngr_get_state(void);
bool wmngr_nvs_valid(void);

#endif // ESP_WIFI_MANAGER_H
