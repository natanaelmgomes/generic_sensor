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
#define SENSOR_1_UPDATE_IVAL                100
// #define SENSOR_2_UPDATE_IVAL             100
// #define SENSOR_3_UPDATE_IVAL             60

/* ESS error definitions */
#define ESS_ERR_WRITE_REJECT                0x80
#define ESS_ERR_COND_NOT_SUPP               0x81

/* ESS Trigger Setting conditions */
#define ESS_TRIGGER_INACTIVE                0x00
#define ESS_FIXED_TIME_INTERVAL             0x01
#define ESS_NO_LESS_THAN_SPECIFIED_TIME     0x02
#define ESS_VALUE_CHANGED                   0x03
#define ESS_LESS_THAN_REF_VALUE             0x04
#define ESS_LESS_OR_EQUAL_TO_REF_VALUE      0x05
#define ESS_GREATER_THAN_REF_VALUE          0x06
#define ESS_GREATER_OR_EQUAL_TO_REF_VALUE   0x07
#define ESS_EQUAL_TO_REF_VALUE              0x08
#define ESS_NOT_EQUAL_TO_REF_VALUE          0x09

int red_led = 0;
int blue_led = 1;

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const uint16_t *u16 = attr->user_data;
    uint16_t value = sys_cpu_to_le16(*u16);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
                            sizeof(value));
}

// Sensing Service Declaration
struct es_measurement {
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

    struct es_measurement meas;
};

//struct humidity_sensor {
//	int16_t humid_value;

//	struct es_measurement meas;
//};

static bool notify_enabled;
static struct generic_sensor sensor_1 = {
		.sensor_values = {0, 0, 0},
		.lower_limit = -10000,
		.upper_limit = 10000,
		.condition = ESS_FIXED_TIME_INTERVAL,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_1_UPDATE_IVAL,
		.meas.application = 0x1c,
		.meas.meas_uncertainty = 0x04,
};

static void flow_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
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

static ssize_t read_es_measurement(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr, void *buf,
				   uint16_t len, uint16_t offset)
{
	const struct es_measurement *value = attr->user_data;
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

static ssize_t read_flow_valid_range(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr, void *buf,
				     uint16_t len, uint16_t offset)
{
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

static ssize_t read_temp_trigger_setting(struct bt_conn *conn,
					 const struct bt_gatt_attr *attr,
					 void *buf, uint16_t len,
					 uint16_t offset)
{
	const struct generic_sensor *sensor = attr->user_data;

	switch (sensor->condition) {
	/* Operand N/A */
	case ESS_TRIGGER_INACTIVE:
		__fallthrough;
	case ESS_VALUE_CHANGED:
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
					 &sensor->condition,
					 sizeof(sensor->condition));
	/* Milli seconds */
	case ESS_FIXED_TIME_INTERVAL:
		__fallthrough;
	case ESS_NO_LESS_THAN_SPECIFIED_TIME: {
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
	switch (condition) {
	case ESS_TRIGGER_INACTIVE:
		return false;
	case ESS_FIXED_TIME_INTERVAL:
                return true;
	case ESS_NO_LESS_THAN_SPECIFIED_TIME:
		/* TODO: Check time requirements */
		return false;
	case ESS_VALUE_CHANGED:
		return new_val[0] != old_val[0];
	case ESS_LESS_THAN_REF_VALUE:
		return new_val[0] < ref_val;
	case ESS_LESS_OR_EQUAL_TO_REF_VALUE:
		return new_val[0] <= ref_val;
	case ESS_GREATER_THAN_REF_VALUE:
		return new_val[0] > ref_val;
	case ESS_GREATER_OR_EQUAL_TO_REF_VALUE:
		return new_val[0] >= ref_val;
	case ESS_EQUAL_TO_REF_VALUE:
		return new_val[0] == ref_val;
	case ESS_NOT_EQUAL_TO_REF_VALUE:
		return new_val[0] != ref_val;
	default:
		return false;
	}
}

static void update_sensor_values(struct bt_conn *conn,
			       const struct bt_gatt_attr *chrc, int16_t *value,
			       struct generic_sensor *sensor)
{
	bool notify = check_condition(sensor->condition,
				      sensor->sensor_values, value,
				      sensor->ref_val);

	/* Update flow value */
	sensor->sensor_values[0] = value[0];
    sensor->sensor_values[1] = value[1];
    sensor->sensor_values[2] = value[2];

	/* Trigger notification if conditions are met */
	if (notify) {
		value[0] = sys_cpu_to_le16(sensor->sensor_values[0]);
        value[1] = sys_cpu_to_le16(sensor->sensor_values[1]);
        value[2] = sys_cpu_to_le16(sensor->sensor_values[2]);

		bt_gatt_notify(conn, chrc, &value, sizeof(value));
	}

        // printk("Flow: %06d\n", value);
}

BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	/* Flow Sensor 1 */
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_1.sensor_values),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_1.meas),
	BT_GATT_CUD(SENSOR_1_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
			   read_flow_valid_range, NULL, &sensor_1),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING,
			   BT_GATT_PERM_READ, read_temp_trigger_setting,
			   NULL, &sensor_1),
	BT_GATT_CCC(flow_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void simulate(void)
{
    static uint8_t i;

    if (!(i % SENSOR_1_UPDATE_IVAL)) {
        static int16_t *values;
        update_sensor_values(NULL, &ess_svc.attrs[2], values, &sensor_1);
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
        red_led = 1;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
    red_led = 0;
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

        /* Flow simulation */
        if (notify_enabled) {
            simulate();
        }

        /* Battery level simulation */
        bas_notify();
        
        if (i == 0){
            generic_led_blink(red_led, blue_led);
        }
        i++;
        if (i >= 400) {
            i=0;
        }
    }
}
