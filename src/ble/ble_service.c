#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>
#include "myble_lbs.h"
#include <zephyr/logging/log.h>

#include <nfc/ndef/le_oob_rec.h>

#include "ble_service.h"
#include <zephyr/pm/device.h>

#define NFC_BUFFER_SIZE 1024
#define SYSTEM_OFF_DELAY 5

static struct bt_le_oob oob_local;
static uint8_t tk_local[NFC_NDEF_LE_OOB_REC_TK_LEN];

LOG_MODULE_REGISTER(BLE_SERVICE, LOG_LEVEL_DBG);

static struct k_work adv_work;

static void key_generation_work_handler(struct k_work *work);
static K_WORK_DEFINE(key_generate_work, key_generation_work_handler);

/*variable that holds callback for MTU negotiation */
static struct bt_gatt_exchange_params exchange_params;

/*Forward declaration of exchange_func(): */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY),
    800, 801, NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }
    else
    {
        LOG_INF("Advertising started");
    }
}

static void update_phy(struct bt_conn *conn)
{
    int err;
    const struct bt_conn_le_phy_param preferred_phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    err = bt_conn_le_phy_update(conn, &preferred_phy);
    if (err)
    {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

/* STEP 10 - Define the function to update the connection's data length */
static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err)
    {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

/* STEP 11.1 - Define the function to update the connection's MTU */
static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err)
    {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{

    if (err)
    {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    dk_set_led_on(DK_LED2);
    LOG_INF("Connected");

    /*Update PHY to BT_GAP_LE_PHY_2M*/
    update_phy(conn);

    /*Connection Info*/
    struct bt_conn_info info;
    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        LOG_ERR("bt_conn_get_info() returned %d", err);
        return;
    }
    double connection_interval = info.le.interval * 1.25; // in ms
    uint16_t supervision_timeout = info.le.timeout * 10;  // in ms
    LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, info.le.latency, supervision_timeout);
    /*Data_Length & MTU*/
    k_sleep(K_MSEC(1000)); // Delay added to avoid link layer collisions.
    update_data_length(conn);
    update_mtu(conn);
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err)
    {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3; // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    dk_set_led_off(DK_LED2);
    LOG_INF("Disconnected (reason %u)", reason);
}

static void recycled_cb(void)
{
    LOG_INF("Connection recycled, restarting advertising...");
    ble_start_advertising();
}

void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval * 1.25; // in ms
    uint16_t supervision_timeout = timeout * 10;  // in ms
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}

void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    // PHY Updated
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M)
    {
        LOG_INF("PHY updated. New PHY: 1M");
    }
    else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M)
    {
        LOG_INF("PHY updated. New PHY: 2M");
    }
    else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8)
    {
        LOG_INF("PHY updated. New PHY: Long Range");
    }
}

void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len = info->tx_max_len;
    uint16_t tx_time = info->tx_max_time;
    uint16_t rx_len = info->rx_max_len;
    uint16_t rx_time = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err)
    {
        printk("Security changed: %s level %u\n", addr, level);
    }
    else
    {
        printk("Security failed: %s level %u err %d %s\n", addr, level, err,
               bt_security_err_to_str(err));
    }
}

static struct bt_conn_cb connection_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .security_changed = security_changed,
    .recycled = recycled_cb,
    .le_param_updated = on_le_param_updated,
    .le_phy_updated = on_le_phy_updated,
    .le_data_len_updated = on_le_data_len_updated,
};

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

static void legacy_tk_value_set(struct bt_conn *conn)
{
    int err;

    err = bt_le_oob_set_legacy_tk(conn, tk_local);
    if (err)
    {
        printk("Failed to set local TK (err %d)\n", err);
    }
}

static void lesc_oob_data_set(struct bt_conn *conn, struct bt_conn_oob_info *info)
{
    int err;
    struct bt_conn_info conn_info;
    char addr[BT_ADDR_LE_STR_LEN];

    if (info->lesc.oob_config != BT_CONN_OOB_LOCAL_ONLY)
    {
        printk("Unsupported OOB request\n");
        return;
    }

    err = bt_conn_get_info(conn, &conn_info);
    if (err)
    {
        printk("Failed to get conn info (err %d)\n", err);
        return;
    }

    if (bt_addr_le_cmp(conn_info.le.local, &oob_local.addr) != 0)
    {
        bt_addr_le_to_str(conn_info.le.local, addr, sizeof(addr));
        printk("No OOB data available for local %s", addr);
        bt_conn_auth_cancel(conn);
        return;
    }

    err = bt_le_oob_set_sc_data(conn, &oob_local.le_sc_data, NULL);
    if (err)
    {
        printk("Failed to set OOB SC local data (err %d)\n", err);
    }
}

static void oob_data_request(struct bt_conn *conn, struct bt_conn_oob_info *info)
{
    if (info->type == BT_CONN_OOB_LE_SC)
    {
        printk("LESC OOB data requested\n");
        lesc_oob_data_set(conn, info);
    }

    if (info->type == BT_CONN_OOB_LE_LEGACY)
    {
        printk("Legacy TK value requested\n");
        legacy_tk_value_set(conn);
    }
}

static const struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel,
    .oob_data_request = oob_data_request};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing completed: %s, bonded: %d\n", addr, bonded);

    /* Generate new pairing keys if pairing procedure succeed. */
    k_work_submit(&key_generate_work);
}
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing failed conn: %s, reason %d %s\n", addr, reason,
           bt_security_err_to_str(reason));

    /* Generate new pairing keys if pairing procedure failed. */
    k_work_submit(&key_generate_work);
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
};

int pairing_key_generate(void)
{
    int err;

    err = bt_le_oob_get_local(BT_ID_DEFAULT, &oob_local);
    if (err)
    {
        printk("Failed to get local oob data (err %d)\n", err);
        return err;
    }

    err = bt_rand(tk_local, sizeof(tk_local));
    if (err)
    {
        printk("Failed to generate random TK value (err %d)\n", err);
    }

    return err;
}

/*Device Bonding*/
static void key_generation_work_handler(struct k_work *work)
{
    pairing_key_generate();
}

void ble_start_advertising(void)
{
    k_work_submit(&adv_work);
}

int ble_service_init(void)
{
    k_work_init(&adv_work, adv_work_handler);
    bt_conn_cb_register(&connection_callbacks);

    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    return 0;
}

int init_ble_service(void)
{
    int err;

    k_work_init(&adv_work, adv_work_handler);
    bt_conn_cb_register(&connection_callbacks);

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        err = settings_load();
        if (err)
        {
            printk("Failed to load settings (err %d)\n", err);
            return 0;
        }
    }

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err)
    {
        printk("Failed to register authorization info callbacks (err %d)\n", err);
        return 0;
    }

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err)
    {
        printk("Failed to register authorization callbacks (err %d)\n", err);
        return 0;
    }

    err = pairing_key_generate();
    if (err)
    {
        printk("Failed to generate pairing keys (err %d)\n", err);
        return 0;
    }

    LOG_INF("BLE SERVICE initialized");
    return 0;
}
