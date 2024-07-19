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

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "bleprph.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/bas/ble_svc_bas.h"
#include "services/dis/ble_svc_dis.h"
#include "led_indicator.h"
#include "prefs.h"
#include "my_utils.h"

#define MAX_CONNS MYNEWT_VAL(BLE_MAX_CONNECTIONS)
#define TAG "GATT_SVR"

static const ble_uuid128_t gatt_svr_svc_cam_uuid = GATT_SVR_SVC_CAM_UUID;

ble_uuid128_t gatt_svr_temps_frame_chr_uuid = GATT_SVR_CHR_TEMPS_FRAME_UUID;
ble_uuid128_t gatt_svr_test_chr_uuid = GATT_SVR_CHR_TEST_UUID;

uint16_t gatt_svr_chr_temps_frame_val_handle;
uint16_t gatt_svr_chr_test_val_handle;

long test_val = 0;

QueueHandle_t temps_frame_queue;
QueueHandle_t status_queue;
// declare large var on stack, to avoid heap depletion
float temps_frames[MAX_Q_ITEMS][NUM_TEMPS_PIXELS];
uint16_t conn_handles[MAX_CONNS];
uint8_t last_refresh_rate = 2;

/* Characteristic User Description */
static const ble_uuid128_t gatt_svr_temps_frame_fps_desc_uuid = GATT_SVR_TEMPS_FRAME_FPS_DESC_UUID;
static const ble_uuid16_t gatt_svr_usr_dsc_uuid = GATT_SVR_USR_DESC_UUID;

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svc_test_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt,
                                void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Camera Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_cam_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){{
                                                           .uuid = &gatt_svr_temps_frame_chr_uuid.u,
                                                           .access_cb = gatt_svc_access,
                                                           .descriptors = (struct ble_gatt_dsc_def[]){
                                                               {.uuid = &gatt_svr_usr_dsc_uuid.u,
                                                                .att_flags = BLE_ATT_F_READ,
                                                                .access_cb = gatt_svc_access,
                                                                .arg = &gatt_svr_temps_frame_chr_uuid.u},
                                                               {.uuid = &gatt_svr_temps_frame_fps_desc_uuid.u, .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE, .access_cb = gatt_svc_access, .arg = &gatt_svr_temps_frame_chr_uuid.u},
                                                               {
                                                                   0, /* No more descriptors in this characteristic */
                                                               }},
                                                           .flags = BLE_GATT_CHR_F_NOTIFY,
                                                           .val_handle = &gatt_svr_chr_temps_frame_val_handle,
                                                       },
                                                       {
                                                           .uuid = &gatt_svr_test_chr_uuid.u,
                                                           .access_cb = gatt_svc_test_access,
                                                           .descriptors = NULL,
                                                           .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
                                                           .val_handle = &gatt_svr_chr_test_val_handle,
                                                       },
                                                       {
                                                           0, /* No more characteristics in this service. */
                                                       }},
    },

    {
        0, /* No more services. */
    },
};

static int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                          void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

/**
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid;
    const ble_uuid_t *chr_uuid;
    int rc;

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            MODLOG_DFLT(INFO, "Descriptor read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        }
        else
        {
            MODLOG_DFLT(DEBUG, "Descriptor read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->dsc->uuid;
        chr_uuid = (const ble_uuid_t *)arg;

        if (ble_uuid_cmp(uuid, &gatt_svr_usr_dsc_uuid.u) == 0)
        {

            const char *desc_val;
            if (ble_uuid_cmp(chr_uuid, &gatt_svr_temps_frame_chr_uuid.u) == 0)
                desc_val = "Temperatures Frame";
            else
                goto unknown;

            rc = os_mbuf_append(ctxt->om, desc_val, strlen(desc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (ble_uuid_cmp(uuid, &gatt_svr_temps_frame_fps_desc_uuid.u) == 0)
        {
            float refreshRateHz = refreshRateToHz(prefs.refreshRate);
            rc = os_mbuf_append(ctxt->om, &refreshRateHz, sizeof(refreshRateHz));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        goto unknown;
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            MODLOG_DFLT(INFO, "Descriptor write; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        }
        else
        {
            struct ble_gap_conn_desc desc;
            rc = ble_gap_conn_find(conn_handle, &desc);
            if (rc != 0 || !(desc.sec_state.bonded && desc.sec_state.encrypted))
            {
                MODLOG_DFLT(ERROR, "Unauthed client tried to write, ignoring\n");
                return 0;
            }

            MODLOG_DFLT(DEBUG, "Descriptor write by NimBLE stack; attr_handle=%d\n", attr_handle);
        }
        uuid = ctxt->dsc->uuid;
        chr_uuid = (const ble_uuid_t *)arg;

        if (ble_uuid_cmp(uuid, &gatt_svr_temps_frame_fps_desc_uuid.u) == 0)
        {
            uint16_t len;
            float refreshRateHz;
            rc = gatt_svr_write(ctxt->om, sizeof(refreshRateHz), sizeof(refreshRateHz), &refreshRateHz, &len);

            if (rc != 0)
                return rc;

            prefs.refreshRate = hzToRefreshRate(refreshRateHz);

            return 0;
        }

        goto unknown;
    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     */

    MODLOG_DFLT(ERROR, "Unknown characteristic/descriptor/op; op=%d conn_handle=%d attr_handle=%d\n",
                ctxt->op, conn_handle, attr_handle);

    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svc_test_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt,
                                void *arg)
{
    bool is_test_chr = ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_test_chr_uuid.u) == 0;
    int rc;

    if (is_test_chr)
    {
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);

        rc = os_mbuf_append(ctxt->om, &test_val,
                            sizeof test_val);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

void test_notify_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        test_val = random();

        ble_gatts_chr_updated(gatt_svr_chr_test_val_handle);
    }
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op)
    {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                           "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

/* This function sends temperature frame notifications to the client */
static void notify_task(void *arg)
{
    uint8_t temps_frame_idx = 0;
    struct status_t last_status;
    uint8_t payload[NUM_NOTIFY_BYTES_LAST_PACKET] = {0};
    int16_t *part_seq = (int16_t *)payload;
    int16_t *part_data = part_seq + 1;
    int part_data_idx;
    uint16_t part_data_len;
    int rc;
    struct os_mbuf *om;

    while (1)
    {
        if (!at_least_one_subscribed())
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);

        if (xQueueReceive(temps_frame_queue, &temps_frame_idx, 5000 / portTICK_PERIOD_MS) != pdTRUE)
        {
            ESP_LOGE(TAG, "Did not receive temps frame from queue.");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        // notify all conn_handles
        for (int conn_handle_idx = 0; conn_handle_idx < MAX_CONNS; conn_handle_idx++)
        {
            if (conn_handles[conn_handle_idx] == BLE_HS_CONN_HANDLE_NONE)
                continue;

            uint16_t conn_handle = conn_handles[conn_handle_idx];

            /* Send the temps frame in NUM_NOTIFY_PACKETS parts */

            for (int16_t part = 0; part < NUM_NOTIFY_PACKETS; part++)
            {
                bool is_last_part = part == NUM_NOTIFY_PACKETS - 1;

                *part_seq = part;

                // convert temps_frames[temps_frame_idx] (float32) to signed int16 and multiply by 100
                // to retain 2 decimal places and reduce the payload size
                for (part_data_idx = 0; part_data_idx < NUM_TEMPS_PIXELS / NUM_NOTIFY_PACKETS; part_data_idx++)
                {
                    part_data[part_data_idx] = temps_frames[temps_frame_idx][part * NUM_TEMPS_PIXELS / NUM_NOTIFY_PACKETS + part_data_idx] * 100;
                }

                if (is_last_part)
                {
                    // add status data to last packet
                    xQueueReceive(status_queue, &last_status, 0); // do not wait

                    part_data[part_data_idx++] = last_status.t_a * 100;
                    part_data[part_data_idx++] = last_status.battery_voltage * 100;
                    part_data[part_data_idx++] = last_status.free_heap_k * 100;

                    part_data_len = NUM_NOTIFY_BYTES_LAST_PACKET;
                }
                else
                {
                    part_data_len = NUM_NOTIFY_BYTES_PER_PACKET;
                }

                om = ble_hs_mbuf_from_flat(payload, part_data_len);
                if (om == NULL)
                {
                    /* Memory not available for mbuf */
                    ESP_LOGE(TAG, "No MBUFs available from pool, removing connection handle %d", conn_handle);
                    remove_notify_conn_handle(conn_handle);
                    continue;
                }

                rc = ble_gatts_notify_custom(conn_handle, gatt_svr_chr_temps_frame_val_handle, om);

                if (rc != 0)
                {
                    ESP_LOGE(TAG, "Error sending notification; rc = %d", rc);

                    if (!at_least_one_subscribed())
                        continue;

                    if (rc == BLE_HS_ENOTCONN || rc == BLE_HS_EREJECT)
                        remove_notify_conn_handle(conn_handle);

                    // do not set if disconnected
                    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
                        current_indicator_state = INDICATOR_TRANSFER_FAILED;

                    // xSemaphoreGive(notify_sem);
                    /* Most probably error is because we ran out of mbufs (rc = 6),
                     * increase the mbuf count/size from menuconfig. Though
                     * inserting delay is not good solution let us keep it
                     * simple for time being so that the mbufs get freed up
                     * (?), of course assumption is we ran out of mbufs */
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    current_indicator_state = INDICATOR_TRANSFER_SUCCESS;
                }
            }
        }
    }
}

int set_battery_level(uint8_t battery_level)
{
    return ble_svc_bas_battery_level_set(battery_level);
}

void init_notify_conn_handles()
{
    for (int i = 0; i < MAX_CONNS; i++)
    {
        conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
    }
}

bool add_notify_conn_handle(uint16_t conn_handle)
{
    for (int i = 0; i < MAX_CONNS; i++)
    {
        if (conn_handles[i] == conn_handle)
            return true;
    }

    for (int i = 0; i < MAX_CONNS; i++)
    {
        if (conn_handles[i] == BLE_HS_CONN_HANDLE_NONE)
        {
            conn_handles[i] = conn_handle;
            return true;
        }
    }

    return false;
}

bool remove_notify_conn_handle(uint16_t conn_handle)
{
    for (int i = 0; i < MAX_CONNS; i++)
    {
        if (conn_handles[i] == conn_handle)
        {
            conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
            return true;
        }
    }

    return false;
}

bool at_least_one_subscribed()
{
    for (int i = 0; i < MAX_CONNS; i++)
    {
        if (conn_handles[i] != BLE_HS_CONN_HANDLE_NONE)
        {
            return true;
        }
    }
    return false;
}

int gatt_svr_init(void)
{
    int rc;

    init_notify_conn_handles();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_bas_init();
    ble_svc_dis_init();

    // windows implicitly queries DIS on bonding

    ble_svc_dis_manufacturer_name_set(DEVICE_NAME);
    ble_svc_dis_model_number_set("MLX90640");
    ble_svc_dis_serial_number_set("1234567890");
    ble_svc_dis_hardware_revision_set("1.0.0");
    ble_svc_dis_firmware_revision_set("1.0.0");
    ble_svc_dis_software_revision_set("1.0.0");
    ble_svc_dis_system_id_set("1234567890");

    const char *pnp_id = "\x01\xe5\x02\x01\x01\x01\x01";
    ble_svc_dis_pnp_id_set(pnp_id);

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    /* Initialize Notify Task */
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 10, NULL);
    xTaskCreate(test_notify_task, "test_notify_task", 2048, NULL, 3, NULL);

    return 0;
}

void print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = (const uint8_t *)addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}