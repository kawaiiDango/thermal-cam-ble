#include "bleprph.h"
#include <services/gap/ble_svc_gap.h>
#include <host/ble_hs_adv.h>
#include <host/ble_gap.h>
#include <esp_err.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include "led_indicator.h"
#include <esp_bt.h>

static SemaphoreHandle_t pair_button_semaphore;
bool temps_frame_subscribed = false;
bool status_subscribed = false;
uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

#if MYNEWT_VAL(BLE_POWER_CONTROL)
static struct ble_gap_event_listener power_control_event_listener;
#endif

/**
 * Logs information about a connection to the console.
 */
static void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                      "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
void bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    // fields.uuids16 = (ble_uuid16_t[]){
    //     GATT_SVR_ADV_SVC_UUID,
    // };

    // fields.num_uuids16 = 1;
    // fields.uuids16_is_complete = 1;

    fields.appearance = ADV_APPEARANCE;
    fields.appearance_is_present = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    adv_params.itvl_min = BLE_GAP_ADV_ITVL;
    adv_params.itvl_max = BLE_GAP_ADV_ITVL;

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N9);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }

    current_indicator_state = INDICATOR_BLE_ADVERTISING;
}

#if MYNEWT_VAL(BLE_POWER_CONTROL)
static void bleprph_power_control(uint16_t conn_handle)
{
    int rc;

    rc = ble_gap_read_remote_transmit_power_level(conn_handle, 0x01); // Attempting on LE 1M phy
    assert(rc == 0);

    rc = ble_gap_set_transmit_power_reporting_enable(conn_handle, 0x1, 0x1);
    assert(rc == 0);
}

static int bleprph_gap_power_event(struct ble_gap_event *event, void *arg)
{

    switch (event->type)
    {
    case BLE_GAP_EVENT_TRANSMIT_POWER:
        MODLOG_DFLT(INFO, "Transmit power event : status=%d conn_handle=%d reason=%d "
                          "phy=%d power_level=%x power_level_flag=%d delta=%d",
                    event->transmit_power.status,
                    event->transmit_power.conn_handle,
                    event->transmit_power.reason,
                    event->transmit_power.phy,
                    event->transmit_power.transmit_power_level,
                    event->transmit_power.transmit_power_level_flag,
                    event->transmit_power.delta);
        return 0;

    case BLE_GAP_EVENT_PATHLOSS_THRESHOLD:
        MODLOG_DFLT(INFO, "Pathloss threshold event : conn_handle=%d current path loss=%d "
                          "zone_entered =%d",
                    event->pathloss_threshold.conn_handle,
                    event->pathloss_threshold.current_path_loss,
                    event->pathloss_threshold.zone_entered);
        return 0;

    default:
        return 0;
    }
}
#endif

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
int bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        rc = ble_att_set_preferred_mtu(PREFERRED_MTU_VALUE);
        if (rc != 0)
        {
            MODLOG_DFLT(ERROR, "Failed to set preferred MTU; rc = %d", rc);
        }

        if (event->connect.status == 0)
        {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);

            conn_handle = event->connect.conn_handle;
        }
        MODLOG_DFLT(INFO, "\n");

        if (event->connect.status != 0)
        {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            /* Connection failed; resume advertising. */
            bleprph_advertise();
        }
        else
        {
            current_indicator_state = INDICATOR_CONNECTED;
        }

        rc = ble_hs_hci_util_set_data_len(event->connect.conn_handle,
                                          LL_PACKET_LENGTH,
                                          LL_PACKET_TIME);
        if (rc != 0)
        {
            MODLOG_DFLT(ERROR, "Set packet length failed");
        }

#if MYNEWT_VAL(BLE_POWER_CONTROL)
        bleprph_power_control(event->connect.conn_handle);
        ble_gap_event_listener_register(&power_control_event_listener,
                                        bleprph_gap_power_event, NULL);
#else
        MODLOG_DFLT(ERROR, "Power control not enabled");
#endif
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        temps_frame_subscribed = false;
        status_subscribed = false;

        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        /* Connection terminated; resume advertising. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);

        if (event->adv_complete.reason == BLE_HS_ETIMEOUT)
        {

            fflush(stdout);
            /* Advertising timed out; deep sleep. */
            esp_deep_sleep_start();
        }

        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        // MODLOG_DFLT(INFO, "notify_tx event; conn_handle=%d attr_handle=%d "
        //                   "status=%d is_indication=%d",
        //             event->notify_tx.conn_handle,
        //             event->notify_tx.attr_handle,
        //             event->notify_tx.status,
        //             event->notify_tx.indication);

        // if ((event->notify_tx.status == 0) ||
        //     (event->notify_tx.status == BLE_HS_EDONE))
        // {
        //     /* Send new notification i.e. give Semaphore. By definition,
        //      * sending new notifications should not be based on successful
        //      * notifications sent, but let us adopt this method to avoid too
        //      * many `BLE_HS_ENOMEM` errors because of continuous transfer of
        //      * notifications.XXX */
        //     // xSemaphoreGive(notify_sem);
        // }

        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                          "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);

        // if (event->subscribe.cur_notify)
        // {
        //     rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        //     bleprph_print_conn_desc(&desc);

        //     if (rc != 0 || !(desc.sec_state.bonded && desc.sec_state.authenticated && desc.sec_state.encrypted))
        //     {
        //         MODLOG_DFLT(ERROR, "Unauthed client tried to subscribe");
        //         return BLE_ERR_INSUFFICIENT_SEC;
        //     }
        // }

        if (event->subscribe.attr_handle == gatt_svr_chr_temps_frame_val_handle)
        {
            temps_frame_subscribed = event->subscribe.cur_notify;
        }
        else if (event->subscribe.attr_handle == gatt_svr_chr_status_val_handle)
        {
            status_subscribed = event->subscribe.cur_notify;
        }

        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        MODLOG_DFLT(INFO, "repeat pairing; conn_handle=%d cur_authenticated=%d "
                          "peer_addr=",
                    event->repeat_pairing.conn_handle,
                    event->repeat_pairing.cur_authenticated);

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:

        MODLOG_DFLT(WARN, "BLE_GAP_EVENT_PASSKEY_ACTION");

        struct ble_sm_io pkey = {0};

        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP)
        {
            MODLOG_DFLT(WARN, "Passkey on device's display: %" PRIu32, event->passkey.params.numcmp);

            // set the pair LED to on
            current_indicator_state = INDICATOR_BLE_BONDING_REQ;

            // reset pair_button_semaphore, have to press the button after the passkey action is started
            xSemaphoreTake(pair_button_semaphore, 0);

            pkey.action = event->passkey.params.action;
            if (xSemaphoreTake(pair_button_semaphore, PAIR_BUTTON_TIMEOUT) == pdTRUE)
            {
                pkey.numcmp_accept = 1;
            }
            else
            {
                pkey.numcmp_accept = 0;
                MODLOG_DFLT(ERROR, "Timeout! Rejecting the key");
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        }
        else
        {
            MODLOG_DFLT(ERROR, "Passkey action not supported");
        }

        // set the pair LED to off
        current_indicator_state = INDICATOR_BLE_ADVERTISING;

        return 0;

    case BLE_GAP_EVENT_AUTHORIZE:
        MODLOG_DFLT(INFO, "authorize event: conn_handle=%d attr_handle=%d is_read=%d",
                    event->authorize.conn_handle,
                    event->authorize.attr_handle,
                    event->authorize.is_read);

        /* The default behaviour for the event is to reject authorize request */
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;

#if MYNEWT_VAL(BLE_POWER_CONTROL)
    case BLE_GAP_EVENT_TRANSMIT_POWER:
        MODLOG_DFLT(INFO, "Transmit power event : status=%d conn_handle=%d reason=%d "
                          "phy=%d power_level=%x power_level_flag=%d delta=%d",
                    event->transmit_power.status,
                    event->transmit_power.conn_handle,
                    event->transmit_power.reason,
                    event->transmit_power.phy,
                    event->transmit_power.transmit_power_level,
                    event->transmit_power.transmit_power_level_flag,
                    event->transmit_power.delta);
        return 0;

    case BLE_GAP_EVENT_PATHLOSS_THRESHOLD:
        MODLOG_DFLT(INFO, "Pathloss threshold event : conn_handle=%d current path loss=%d "
                          "zone_entered =%d",
                    event->pathloss_threshold.conn_handle,
                    event->pathloss_threshold.current_path_loss,
                    event->pathloss_threshold.zone_entered);
        return 0;
#endif
    }

    return 0;
}

static void IRAM_ATTR pair_button_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(pair_button_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void bt_io_init(void)
{

    // setup the pair button
    pair_button_semaphore = xSemaphoreCreateBinary();

    gpio_config_t io_conf_pair_button = {
        .pin_bit_mask = 1ULL << PAIR_BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_pair_button));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)PAIR_BUTTON_PIN, pair_button_isr, NULL));
}