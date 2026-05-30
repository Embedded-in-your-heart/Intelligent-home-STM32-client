/**
  ******************************************************************************
  * @file    App/sensor.c
  * @brief   BlueNRG-MS GAP / advertising for the Intelligent-home STM32 client.
  *
  *          Advertises as "HOME-XXXX" where XXXX is the last 2 bytes of the
  *          static random BD_ADDR (hex). Routes incoming HCI events to GATT
  *          callbacks in gatt_db.c.
  ******************************************************************************
  */

#include <stdint.h>
#include "sensor.h"
#include "gatt_db.h"
#include "bluenrg_def.h"
#include "bluenrg_conf.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_aci_const.h"
#include "hci_le.h"
#include "hci_const.h"

/* Advertising interval: 100–200 ms in 0.625 ms units. */
#define ADV_INTERVAL_MIN_UNITS  160
#define ADV_INTERVAL_MAX_UNITS  320

/* Globals shared with app_bluenrg_ms.c and gatt_db.c -------------------------*/
extern uint8_t bdaddr[BDADDR_SIZE];
extern uint8_t bnrg_expansion_board;

__IO uint8_t  set_connectable        = 1;
__IO uint16_t connection_handle      = 0;
__IO uint8_t  notification_enabled   = FALSE;
__IO uint32_t connected              = FALSE;

/* Private prototypes --------------------------------------------------------*/
static void GAP_DisconnectionComplete_CB(void);
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);

/* Public functions ----------------------------------------------------------*/

void Set_DeviceConnectable(void)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t local_name[10];
    tBleStatus ret;

    /* Build "HOME-XXXX" preceded by the AD type byte. Total length 10. */
    local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
    local_name[1] = 'H';
    local_name[2] = 'O';
    local_name[3] = 'M';
    local_name[4] = 'E';
    local_name[5] = '-';
    local_name[6] = hex[(bdaddr[1] >> 4) & 0xF];
    local_name[7] = hex[ bdaddr[1]       & 0xF];
    local_name[8] = hex[(bdaddr[0] >> 4) & 0xF];
    local_name[9] = hex[ bdaddr[0]       & 0xF];

    hci_le_set_scan_resp_data(0, NULL);

    ret = aci_gap_set_discoverable(ADV_DATA_TYPE,
                                   ADV_INTERVAL_MIN_UNITS,
                                   ADV_INTERVAL_MAX_UNITS,
                                   STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                   sizeof(local_name), local_name,
                                   0, NULL, 0, 0);

    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_discoverable failed: 0x%02x\n", ret);
    } else {
        PRINTF("Advertising as HOME-%02X%02X\n", bdaddr[1], bdaddr[0]);
    }
}

void user_notify(void *pData)
{
    hci_uart_pckt  *hci_pckt   = pData;
    hci_event_pckt *event_pckt = (hci_event_pckt *)hci_pckt->data;

    if (hci_pckt->type != HCI_EVENT_PKT) return;

    switch (event_pckt->evt) {

    case EVT_DISCONN_COMPLETE:
        GAP_DisconnectionComplete_CB();
        break;

    case EVT_LE_META_EVENT: {
        evt_le_meta_event *evt = (void *)event_pckt->data;
        if (evt->subevent == EVT_LE_CONN_COMPLETE) {
            evt_le_connection_complete *cc = (void *)evt->data;
            GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
    }

    case EVT_VENDOR: {
        evt_blue_aci *blue_evt = (void *)event_pckt->data;
        switch (blue_evt->ecode) {

        case EVT_BLUE_GATT_READ_PERMIT_REQ: {
            evt_gatt_read_permit_req *pr = (void *)blue_evt->data;
            Read_Request_CB(pr->attr_handle);
            break;
        }

        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED: {
            /* IDB05A1/IDB05A2 layout has an extra `offset` field before
             * att_data; IDB04A1 does not. Dispatch on the variant. */
            if (bnrg_expansion_board == IDB05A1) {
                evt_gatt_attr_modified_IDB05A1 *am =
                    (evt_gatt_attr_modified_IDB05A1 *)blue_evt->data;
                Attribute_Modified_CB(am->attr_handle,
                                      am->data_length,
                                      am->att_data);
            } else {
                evt_gatt_attr_modified_IDB04A1 *am =
                    (evt_gatt_attr_modified_IDB04A1 *)blue_evt->data;
                Attribute_Modified_CB(am->attr_handle,
                                      am->data_length,
                                      am->att_data);
            }
            break;
        }
        }
        break;
    }
    }
}

/* GAP callbacks -------------------------------------------------------------*/

static void GAP_DisconnectionComplete_CB(void)
{
    connected             = FALSE;
    connection_handle     = 0;
    notification_enabled  = FALSE;
    set_connectable       = TRUE;  /* re-enter advertising */
    PRINTF("Disconnected\n");
}

static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
    connected         = TRUE;
    connection_handle = handle;

    PRINTF("Connected to %02X:%02X:%02X:%02X:%02X:%02X\n",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}
