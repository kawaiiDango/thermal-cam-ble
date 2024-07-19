/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_BLEPRPH_
#define H_BLEPRPH_

#include "nimble/ble.h"
#include "modlog/modlog.h"
#include "host/ble_hs.h"
#include "config.h"
#ifdef __cplusplus
extern "C"
{
#endif

    struct status_t
    {
        float t_a;
        float battery_voltage;
        float free_heap_k;
    };

    extern uint16_t gatt_svr_chr_temps_frame_val_handle;
    extern QueueHandle_t temps_frame_queue;
    extern QueueHandle_t status_queue;
    extern float temps_frames[MAX_Q_ITEMS][NUM_TEMPS_PIXELS];
    extern bool bt_inited;

/** GATT server. */
#define GATT_SVR_SVC_CAM_UUID BLE_UUID128_INIT(0xe5, 0xdd, 0x35, 0x41, 0xe7, 0xdf, 0x4f, 0x55, 0xb8, 0xe1, 0x17, 0x4e, 0xd8, 0x5c, 0x18, 0x38)
#define GATT_SVR_CHR_TEMPS_FRAME_UUID BLE_UUID128_INIT(0x62, 0x56, 0x98, 0x68, 0xa1, 0xee, 0x57, 0x8a, 0x63, 0x40, 0x88, 0x81, 0xa5, 0xaa, 0x55, 0xbe)
#define GATT_SVR_USR_DESC_UUID BLE_UUID16_INIT(0x2901)
#define GATT_SVR_TEMPS_FRAME_FPS_DESC_UUID BLE_UUID128_INIT(0x94, 0x80, 0xec, 0x47, 0x31, 0x79, 0xc0, 0x96, 0x5f, 0x4f, 0x2e, 0xfe, 0x64, 0xc8, 0x54, 0x9c)
#define GATT_SVR_CHR_TEST_UUID BLE_UUID128_INIT(0x7b, 0x26, 0x1c, 0xb9, 0x18, 0x69, 0xfd, 0xaf, 0x28, 0x44, 0xa7, 0x7d, 0x3d, 0xa2, 0x6f, 0xf9);
/* Device Information configuration */
#define DIS_COMPANY_IDENTIFIER_ESPRESSIF 0x02E5
#define DIS_PRODUCT_IDENTIFIER 0x0101
#define DIS_PRODUCT_VERSION 0x0101

#define ADV_APPEARANCE 0x0300                   // generic thermometer
#define MIN_REQUIRED_MBUF 2 * NUM_TEMPS_PACKETS /* Assuming payload of 500Bytes and each mbuf can take 292Bytes.  */

    void bt_host_init(void);
    void bt_io_init(void);

    void bleprph_advertise(void);
    int bleprph_gap_event(struct ble_gap_event *event, void *arg);
    void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
    int gatt_svr_init(void);
    int set_battery_level(uint8_t battery_level);
    bool add_notify_conn_handle(uint16_t conn_handle);
    bool remove_notify_conn_handle(uint16_t conn_handle);
    bool at_least_one_subscribed();
    void print_addr(const void *addr);

#ifdef __cplusplus
}
#endif

#endif
