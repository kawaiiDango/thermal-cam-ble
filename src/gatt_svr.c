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
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/bas/ble_svc_bas.h"
#include "bleprph.h"
#include "led_indicator.h"
#include "prefs.h"

/*** Maximum number of characteristics with the notify flag ***/
#define MAX_NOTIFY 5
#define TAG_GATT_SVR "GATT_SVR"

static const ble_uuid128_t gatt_svr_svc_cam_uuid = GATT_SVR_SVC_CAM_UUID;

ble_uuid128_t gatt_svr_temps_frame_chr_uuid = GATT_SVR_CHR_TEMPS_FRAME_UUID;
ble_uuid128_t gatt_svr_status_chr_uuid = GATT_SVR_CHR_STATUS_UUID;

static const char *manuf_name = DEVICE_NAME;
static const char *model_num = DEVICE_NAME;

uint16_t gatt_svr_chr_temps_frame_val_handle;
uint16_t gatt_svr_chr_status_val_handle;

time_t last_status_notify_time = 0;

QueueHandle_t temps_frame_queue;
QueueHandle_t status_queue;
// declare large var on stack, to avoid heap depletion
float temps_frames[MAX_Q_ITEMS][NUM_TEMPS_PIXELS];
uint8_t last_refresh_rate = 2;

/* Characteristic User Description */
static const ble_uuid128_t gatt_svr_temps_frame_fps_desc_uuid = GATT_SVR_TEMPS_FRAME_FPS_DESC_UUID;
static const ble_uuid16_t gatt_svr_usr_dsc_uuid = GATT_SVR_USR_DESC_UUID;
static const ble_uuid16_t gatt_svr_manufacturer_name_uuid = GATT_MANUFACTURER_NAME_UUID;
static const ble_uuid16_t gatt_svr_model_number_uuid = GATT_MODEL_NUMBER_UUID;
static const ble_uuid16_t gatt_svr_device_info_svc_uuid = GATT_DEVICE_INFO_UUID;

static int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_chr_def ble_gatt_chr_device_info[] = {
    {
        /* Characteristic: * Manufacturer name */
        .uuid = &gatt_svr_manufacturer_name_uuid.u,
        .access_cb = gatt_svr_chr_access_device_info,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        /* Characteristic: Model number string */
        .uuid = &gatt_svr_model_number_uuid.u,
        .access_cb = gatt_svr_chr_access_device_info,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        0, /* No more characteristics in this service */
    },
};

static const struct ble_gatt_chr_def ble_gatt_chr_cam[] = {{
                                                               /*** This characteristic can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                                                               .uuid = &gatt_svr_temps_frame_chr_uuid.u,
                                                               .access_cb = gatt_svc_access,
                                                               .descriptors = (struct ble_gatt_dsc_def[]){{.uuid = &gatt_svr_usr_dsc_uuid.u, .att_flags = BLE_ATT_F_READ, .access_cb = gatt_svc_access, .arg = &gatt_svr_temps_frame_chr_uuid.u}, {.uuid = &gatt_svr_temps_frame_fps_desc_uuid.u, .att_flags = BLE_ATT_F_READ | BLE_ATT_ACCESS_OP_WRITE, .access_cb = gatt_svc_access, .arg = &gatt_svr_temps_frame_chr_uuid.u}, {
                                                                                                                                                                                                                                                                                                                                                                                                                                     0, /* No more descriptors in this characteristic */
                                                                                                                                                                                                                                                                                                                                                                                                                                 }},
                                                               .flags = BLE_GATT_CHR_F_NOTIFY,
                                                               .val_handle = &gatt_svr_chr_temps_frame_val_handle,
                                                           },
                                                           {
                                                               /*** This characteristic can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                                                               .uuid = &gatt_svr_status_chr_uuid.u,
                                                               .access_cb = gatt_svc_access,
                                                               .descriptors = (struct ble_gatt_dsc_def[]){{.uuid = &gatt_svr_usr_dsc_uuid.u, .att_flags = BLE_ATT_F_READ, .access_cb = gatt_svc_access, .arg = &gatt_svr_status_chr_uuid.u}, {
                                                                                                                                                                                                                                                 0, /* No more descriptors in this characteristic */
                                                                                                                                                                                                                                             }},
                                                               .flags = BLE_GATT_CHR_F_NOTIFY,
                                                               .val_handle = &gatt_svr_chr_status_val_handle,
                                                           },
                                                           {
                                                               0, /* No more characteristics in this service. */
                                                           }};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Device Information */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_device_info_svc_uuid.u,
        .characteristics = ble_gatt_chr_device_info,
    },
    {
        /*** Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_cam_uuid.u,
        .characteristics = ble_gatt_chr_cam,
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
            else if (ble_uuid_cmp(chr_uuid, &gatt_svr_status_chr_uuid.u) == 0)
                desc_val = "Status";
            else
                goto unknown;

            rc = os_mbuf_append(ctxt->om, desc_val, strlen(desc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (ble_uuid_cmp(uuid, &gatt_svr_temps_frame_fps_desc_uuid.u) == 0)
        {
            rc = os_mbuf_append(ctxt->om, &prefs.refreshRate, sizeof(prefs.refreshRate));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        goto unknown;
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
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

        if (ble_uuid_cmp(uuid, &gatt_svr_temps_frame_fps_desc_uuid.u) == 0)
        {
            uint16_t len;
            rc = gatt_svr_write(ctxt->om, sizeof(prefs.refreshRate), sizeof(prefs.refreshRate), &prefs.refreshRate, &len);

            if (rc != 0)
                return rc;

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

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        MODLOG_DFLT(INFO, "gatt_svr_chr_access_device_info; conn_handle=%d attr_handle=%d\n",
                    conn_handle, attr_handle);
    }

    if (uuid == gatt_svr_model_number_uuid.value)
    {
        rc = os_mbuf_append(ctxt->om, model_num, strlen(model_num));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (uuid == gatt_svr_manufacturer_name_uuid.value)
    {
        rc = os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
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
    struct status_t status;
    char status_payload[64] = {0};                           /* Data payload */
    uint8_t temps_payload[NUM_TEMPS_BYTES_PER_PACKET] = {0}; /* Data payload */
    int16_t *part_seq = (int16_t *)temps_payload;
    int16_t *part_data = part_seq + 1;
    int rc;
    struct os_mbuf *om;

    while (1)
    {
        if (!temps_frame_subscribed)
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);

        if (xQueueReceive(temps_frame_queue, &temps_frame_idx, 5000 / portTICK_PERIOD_MS) != pdTRUE)
        {
            ESP_LOGE(TAG_GATT_SVR, "Did not receive temps frame from queue.");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        /* Send the temps frame in NUM_TEMPS_PACKETS parts */

        for (int16_t part = 0; part < NUM_TEMPS_PACKETS; part++)
        {
            *part_seq = part;

            // convert temps_frames[temps_frame_idx] (float32) to signed int16 and multiply by 100
            // to retain 2 decimal places and reduce the payload size
            for (int j = 0; j < NUM_TEMPS_PIXELS / NUM_TEMPS_PACKETS; j++)
            {
                part_data[j] = temps_frames[temps_frame_idx][part * NUM_TEMPS_PIXELS / NUM_TEMPS_PACKETS + j] * 100;
            }

            om = ble_hs_mbuf_from_flat(temps_payload, sizeof(temps_payload));
            if (om == NULL)
            {
                /* Memory not available for mbuf */
                ESP_LOGE(TAG_GATT_SVR, "No MBUFs available from pool, give up..");
                continue;
            }

            rc = ble_gatts_notify_custom(conn_handle, gatt_svr_chr_temps_frame_val_handle, om);

            if (rc != 0)
            {
                ESP_LOGE(TAG_GATT_SVR, "Error while sending notification; rc = %d", rc);

                if (!temps_frame_subscribed)
                    continue;

                // do not set if disconnected
                if (rc != BLE_HS_ENOTCONN && conn_handle != BLE_HS_CONN_HANDLE_NONE)
                    current_indicator_state = INDICATOR_TRANSFER_FALED;

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

        // send status notification if needed
        if (!status_subscribed)
            continue;

        if (xQueueReceive(status_queue, &status, 0) != pdTRUE) // do not wait
        {
            continue;
        }

        sprintf(status_payload, "Free heap:%.2fK | Battery:%.2fV | Ta: %.2f°C", status.free_heap_k, status.battery_voltage, status.t_a);

        om = ble_hs_mbuf_from_flat(status_payload, strlen(status_payload));
        if (om == NULL)
        {
            /* Memory not available for mbuf */
            ESP_LOGE(TAG_GATT_SVR, "No MBUFs available from pool, skipping status notification.");
            continue;
        }

        rc = ble_gatts_notify_custom(conn_handle, gatt_svr_chr_status_val_handle, om);

        if (rc != 0)
        {
            ESP_LOGE(TAG_GATT_SVR, "Error while sending notification; rc = %d", rc);
            /* Most probably error is because we ran out of mbufs (rc = 6),
             * increase the mbuf count/size from menuconfig. Though
             * inserting delay is not good solution let us keep it
             * simple for time being so that the mbufs get freed up
             * (?), of course assumption is we ran out of mbufs */
        }

        // lithium ion battery voltage range is 3.6V to 4.2V
        uint8_t battery_level = (status.battery_voltage - 3.6) / (4.2 - 3.6) * 100;
        ble_svc_bas_battery_level_set(battery_level);
    }
}

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_bas_init();
    // ble_svc_dis_init();

    // if (rc != 0)
    // {
    //     return rc;
    // }

    // rc = ble_svc_dis_manufacturer_name_set("Melexis");
    // if (rc != 0)
    // {
    //     return rc;
    // }

    // rc = ble_svc_dis_model_number_set("MLX90640");
    // if (rc != 0)
    // {
    //     return rc;
    // }

    // rc = ble_svc_dis_serial_number_set("1234567890");
    // if (rc != 0)
    // {
    //     return rc;
    // }

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

    /* Create a counting semaphore for Notification. Can be used to track
     * successful notification txmission. Optimistically take some big number
     * for counting Semaphore */
    // notify_sem = xSemaphoreCreateCounting(100, 0);

    /* Initialize Notify Task */
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 10, NULL);

    return 0;
}

void print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = (const uint8_t *)addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}