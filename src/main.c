/* main.c - Application main entry point */

/*
* Copyright (c) 2016 Intel Corporation
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

// Analog-to-Digital header
#include "generic_sensor_adc.h"

// LED blink header
#include "generic_led.h"

// Bluetooth libraries
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

#define SENSOR_1_NAME				"Sensor 1"

/* Sensor Internal Update Interval [miliseconds] */
#define SENSOR_1_UPDATE_IVAL            100
// #define SENSOR_2_UPDATE_IVAL         100
// #define SENSOR_3_UPDATE_IVAL         60

/* error definitions */
#define ERR_WRITE_REJECT                0x80
#define ERR_COND_NOT_SUPP               0x81

/* Trigger Setting conditions */
#define TRIGGER_INACTIVE                0x00
#define FIXED_TIME_INTERVAL             0x01
#define NO_LESS_THAN_SPECIFIED_TIME     0x02
#define VALUE_CHANGED                   0x03
#define LESS_THAN_REF_VALUE             0x04
#define LESS_OR_EQUAL_TO_REF_VALUE      0x05
#define GREATER_THAN_REF_VALUE          0x06
#define GREATER_OR_EQUAL_TO_REF_VALUE   0x07
#define EQUAL_TO_REF_VALUE              0x08
#define NOT_EQUAL_TO_REF_VALUE          0x09


int blink_red_led_flag = 1;
int blink_blue_led_flag = 0;

static uint64_t time, last_time;

/* Custom Service Variables
Randomly generated UUID:  a7ea14cf-7778-43ba-ab86-1d6e136a2e9e
Base UUID Generic Sensor: a7ea14cf-0000-43ba-ab86-1d6e136a2e9e
https://www.guidgenerator.com/online-guid-generator.aspx
*/
static struct bt_uuid_128 BT_UUID_GENERIC_SENSOR_SERVICE = BT_UUID_INIT_128(
    0x9e, 0x2e, 0x6a, 0x13, 0x6e, 0x1d, 0x86, 0xab,
    0xba, 0x43, 0x10, 0x00, 0xcf, 0x14, 0xea, 0xa7);

static struct bt_uuid_128 BT_UUID_GENERIC_SENSOR_CHARACTERISTIC = BT_UUID_INIT_128(
    0x9e, 0x2e, 0x6a, 0x13, 0x6e, 0x1d, 0x86, 0xab,
    0xba, 0x43, 0x11, 0x00, 0xcf, 0x14, 0xea, 0xa7);

static struct bt_uuid_128 BT_UUID_GS_MEASUREMENT = BT_UUID_INIT_128(
    0x9e, 0x2e, 0x6a, 0x13, 0x6e, 0x1d, 0x86, 0xab,
    0xba, 0x43, 0x12, 0x00, 0xcf, 0x14, 0xea, 0xa7);
    
static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    printk("read_u16\n");

    const uint16_t *u16 = attr->user_data;
    uint16_t value = sys_cpu_to_le16(*u16);

    printk("Size of data: %d\n", sizeof(value));

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
                            sizeof(value));
}

// Sensing Service Declaration
struct measurement {
    uint16_t  flags; /* Reserved for Future Use */
    uint8_t   sampling_func;
    uint32_t  meas_period;
    uint32_t  update_interval;
    uint8_t   application;
    uint8_t   meas_uncertainty;
};

struct generic_sensor {
    int16_t sensor_values[3];
    
    /* Valid Range */
    int16_t lower_limit;
    int16_t upper_limit;

    /* ES trigger setting - Value Notification condition */
    uint8_t condition;
    union {
        uint32_t milliseconds;
        int16_t ref_val; /* Reference temperature */
    };

    struct measurement meas;
};

int16_t values[3];

static bool notify_enabled;
static struct generic_sensor sensor_1 = {
        .sensor_values = {0, 0, 0},
        .lower_limit = -10000,
        .upper_limit = 10000,
        .condition = FIXED_TIME_INTERVAL,
        .meas.sampling_func = 0x00,
        .meas.meas_period = 0x01,
        .meas.update_interval = SENSOR_1_UPDATE_IVAL,
        .meas.application = 0x1c,
        .meas.meas_uncertainty = 0x04,
};

static void gs_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                uint16_t value)
{
    printk("gs_ccc_cfg_changed\n");
    printk("Value received: %d\n", value);
    notify_enabled = value == BT_GATT_CCC_NOTIFY;
}

struct read_es_measurement_rp {
    uint16_t flags; /* Reserved for Future Use */
    uint8_t sampling_function;
    uint8_t measurement_period[3];
    uint8_t update_interval[3];
    uint8_t application;
    uint8_t measurement_uncertainty;
} __packed;

static ssize_t read_gs_measurement(struct bt_conn *conn,
                const struct bt_gatt_attr *attr, void *buf,
                uint16_t len, uint16_t offset)
{
    printk("read_gs_measurement\n");
    const struct measurement *value = attr->user_data;
    struct read_es_measurement_rp rsp;

    rsp.flags = sys_cpu_to_le16(value->flags);
    rsp.sampling_function = value->sampling_func;
    sys_put_le24(value->meas_period, rsp.measurement_period);
    sys_put_le24(value->update_interval, rsp.update_interval);
    rsp.application = value->application;
    rsp.measurement_uncertainty = value->meas_uncertainty;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &rsp,
                sizeof(rsp));
}

static ssize_t read_value_valid_range(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr, void *buf,
                    uint16_t len, uint16_t offset)
{
    printk("read_value_valid_range\n");
    const struct generic_sensor *sensor = attr->user_data;
    uint16_t tmp[] = {sys_cpu_to_le16(sensor->lower_limit),
            sys_cpu_to_le16(sensor->upper_limit)};

    return bt_gatt_attr_read(conn, attr, buf, len, offset, tmp,
                sizeof(tmp));
}

struct es_trigger_setting_milliseconds {
    uint8_t condition;
    uint8_t millisec[3];
} __packed;

struct es_trigger_setting_reference {
    uint8_t condition;
    int16_t ref_val;
} __packed;

static ssize_t read_value_trigger_setting(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr,
                    void *buf, uint16_t len,
                    uint16_t offset)
{
    printk("read_value_trigger_setting\n");
    const struct generic_sensor *sensor = attr->user_data;

    switch (sensor->condition) {
    /* Operand N/A */
    case TRIGGER_INACTIVE:
        __fallthrough;
    case VALUE_CHANGED:
        return bt_gatt_attr_read(conn, attr, buf, len, offset,
                    &sensor->condition,
                    sizeof(sensor->condition));
    /* Milli seconds */
    case FIXED_TIME_INTERVAL:
        __fallthrough;
    case NO_LESS_THAN_SPECIFIED_TIME: {
            struct es_trigger_setting_milliseconds rp;

            rp.condition = sensor->condition;
            sys_put_le24(sensor->milliseconds, rp.millisec);

            return bt_gatt_attr_read(conn, attr, buf, len, offset,
                        &rp, sizeof(rp));
        }
    /* Reference temperature */
    default: {
            struct es_trigger_setting_reference rp;

            rp.condition = sensor->condition;
            rp.ref_val = sys_cpu_to_le16(sensor->ref_val);

            return bt_gatt_attr_read(conn, attr, buf, len, offset,
                        &rp, sizeof(rp));
        }
    }
}

static bool check_condition(uint8_t condition, int16_t *old_val, int16_t *new_val,
                int16_t ref_val)
{
    printk("check_condition\n");
    switch (condition) {
    case TRIGGER_INACTIVE:
        return false;
    case FIXED_TIME_INTERVAL:
                return true;
    case NO_LESS_THAN_SPECIFIED_TIME:
        /* TODO: Check time requirements */
        return false;
    case VALUE_CHANGED:
        return new_val[0] != old_val[0];
    case LESS_THAN_REF_VALUE:
        return new_val[0] < ref_val;
    case LESS_OR_EQUAL_TO_REF_VALUE:
        return new_val[0] <= ref_val;
    case GREATER_THAN_REF_VALUE:
        return new_val[0] > ref_val;
    case GREATER_OR_EQUAL_TO_REF_VALUE:
        return new_val[0] >= ref_val;
    case EQUAL_TO_REF_VALUE:
        return new_val[0] == ref_val;
    case NOT_EQUAL_TO_REF_VALUE:
        return new_val[0] != ref_val;
    default:
        return false;
    }
}

static void update_sensor_values(struct bt_conn *conn,
                const struct bt_gatt_attr *chrc,
                struct generic_sensor *sensor)
{
    // printk("update_sensor_values\n");

    // printk("Size of data: %d\n", sizeof(values));

    generic_sensor_adc_multi_sample(values);
    
    bool notify = check_condition(sensor->condition,
                    sensor->sensor_values, values,
                    sensor->ref_val);

    // printk("Condition: %s", notify?"true\n":"false\n");

    /* Update flow value */
    sensor->sensor_values[0] = values[0];
    sensor->sensor_values[1] = values[1];
    sensor->sensor_values[2] = values[2];

    /* Trigger notification if conditions are met */
    if (notify) {
        // values[0] = sys_cpu_to_le16(sensor->sensor_values[0]);
        // values[1] = sys_cpu_to_le16(sensor->sensor_values[1]);
        // values[2] = sys_cpu_to_le16(sensor->sensor_values[2]);

        // printk("Size of data: %d\n", sizeof(values));

        bt_gatt_notify(conn, chrc, &values, sizeof(values));
    }

        // printk("Value: %06d\n", value);
}

BT_GATT_SERVICE_DEFINE(gss_svc,
    BT_GATT_PRIMARY_SERVICE(&BT_UUID_GENERIC_SENSOR_SERVICE),

    /*  Sensor 1 */
    BT_GATT_CHARACTERISTIC(&BT_UUID_GENERIC_SENSOR_CHARACTERISTIC.uuid,
                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_READ,
                read_u16, NULL, &sensor_1.sensor_values),
    BT_GATT_CUD(SENSOR_1_NAME, BT_GATT_PERM_READ),
    BT_GATT_DESCRIPTOR(&BT_UUID_GS_MEASUREMENT.uuid, BT_GATT_PERM_READ,
            read_gs_measurement, NULL, &sensor_1.meas),
    BT_GATT_CUD(SENSOR_1_NAME, BT_GATT_PERM_READ),
    BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
            read_value_valid_range, NULL, &sensor_1),
    BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING,
            BT_GATT_PERM_READ, read_value_trigger_setting,
            NULL, &sensor_1),
    BT_GATT_CCC(gs_ccc_cfg_changed,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /*  Sensor 2 */
    /*  Removed */
);

static void update_sensor_data(void)
{
    static uint8_t i;

    if (!(i % SENSOR_1_UPDATE_IVAL)) {
        // time = k_uptime_get();
        update_sensor_values(NULL, &gss_svc.attrs[2], &sensor_1);
        // last_time = k_uptime_get();
        // printk("Time passed: %lli ms\n", last_time - time);
    }

    if (!(i % 100U)) {
        i = 0U; // unsigned int
    }

    i++;
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
            BT_UUID_16_ENCODE(BT_UUID_ESS_VAL),
            BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err 0x%02x)\n", err);
    } else {
        printk("Connected\n");
        blink_red_led_flag = 0;
        red_led_on();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
    blink_red_led_flag = 1;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(void)
{
    int err;
    printk("Bluetooth initialized\n");
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

static void bas_notify(void)
{
    uint8_t battery_level = bt_bas_get_battery_level();

    battery_level--;

    if (!battery_level) {
        battery_level = 100U;
    }

    bt_bas_set_battery_level(battery_level);
}

void main(void)
{
    int err;

    err = generic_led_init();
    if (err) {
        printk("LED error! (err %d)\n", err);
        return;
    }

    err = generic_sensor_adc_init();
    if (err) {
        printk("ADC error! (err %d)\n", err);
        return;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_ready();
    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    static int i = 0;
    while (1) {
        k_sleep(K_MSEC(1));

        /* Update sensor data */
        if (notify_enabled) {

            update_sensor_data();

        }

        /* Battery level simulation */
        bas_notify();

        if (i == 0){
            if(blink_red_led_flag){
                red_led_blink();
            }
        }
        i++;
        if (i >= 1000) {
            i=0;
        }
    }
}
