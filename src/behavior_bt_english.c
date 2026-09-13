/* SPDX-License-Identifier: MIT */

#define DT_DRV_COMPAT zmk_behavior_bt_english

#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <drivers/behavior.h>
#include <dt-bindings/zmk/keys.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ENGLISH_USAGE ZMK_HID_USAGE_ID(LANG2)
#define RETRY_DELAY K_MSEC(100)

BUILD_ASSERT(ENGLISH_USAGE <= ZMK_HID_KEYBOARD_MAX_USAGE,
             "LANG2 には HKRO または NKRO extended report が必要です");
BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP) && CONFIG_NUM_METAIRQ_PRIORITIES == 0,
             "通知開始の順序保証には単一 CPU と MetaIRQ なしの構成が必要です");

/* 接続の参照を保持し、切断後に同じ slot を別の接続へ流用しない。 */
struct connection_state {
    struct bt_conn *conn;
    struct zmk_hid_keyboard_report_body normal;
    bool release_pending;
    uint32_t generation;
    bool security_requested;
    bool security_attempted;
    bool security_failed;
    uint32_t security_generation;
    atomic_val_t security_failures_seen;
};

static struct connection_state connections[CONFIG_BT_MAX_CONN];
/* Bluetooth callback から report_lock を待たずに失敗を伝える。 */
static atomic_t security_failures[CONFIG_BT_MAX_CONN];
static const struct bt_gatt_attr *keyboard_attr;
static const struct bt_gatt_attr *keyboard_decl;
static bool initialized;
static struct k_spinlock request_lock;
static struct {
    uint32_t generation;
    uint8_t profile;
    bool pending;
    bool selecting;
} request;

/* notify が buffer を待つことがあるため、system workqueue からは送信しない。 */
static struct k_work_q english_queue;
K_THREAD_STACK_DEFINE(english_stack, CONFIG_ZMK_BLE_THREAD_STACK_SIZE);
K_MUTEX_DEFINE(report_lock);
static void process_pending(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(english_work, process_pending);

int __real_bt_gatt_notify_cb(struct bt_conn *conn, struct bt_gatt_notify_params *params);

static void wake_worker(void) {
    if (initialized) {
        k_work_reschedule_for_queue(&english_queue, &english_work, K_NO_WAIT);
    }
}

static bool connected(struct bt_conn *conn) {
    struct bt_conn_info info;
    return bt_conn_get_info(conn, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED;
}

static struct connection_state *state_for(struct bt_conn *conn) {
    uint8_t index = bt_conn_index(conn);
    if (index >= ARRAY_SIZE(connections)) {
        return NULL;
    }
    struct connection_state *state = &connections[index];
    if (state->conn == NULL) {
        state->conn = bt_conn_ref(conn);
        state->security_failures_seen = atomic_get(&security_failures[index]);
    }
    return state->conn == conn ? state : NULL;
}

static void complete_request(uint32_t generation) {
    k_spinlock_key_t key = k_spin_lock(&request_lock);
    if (request.generation == generation) {
        request.pending = false;
    }
    k_spin_unlock(&request_lock, key);
}

/* 現行 HOG の callback なし通知を直列化する。他の呼出しはそのまま渡す。 */
int __wrap_bt_gatt_notify_cb(struct bt_conn *conn, struct bt_gatt_notify_params *params) {
    if (!conn || !keyboard_attr || params->uuid || params->func || params->user_data ||
        k_current_get() == &k_sys_work_q.thread ||
        (params->attr != keyboard_attr && params->attr != keyboard_decl) ||
        params->len != sizeof(struct zmk_hid_keyboard_report_body)) {
        return __real_bt_gatt_notify_cb(conn, params);
    }

    k_mutex_lock(&report_lock, K_FOREVER);
    int err = __real_bt_gatt_notify_cb(conn, params);
    if (!err) {
        struct connection_state *state = state_for(conn);
        if (state) {
            memcpy(&state->normal, params->data, sizeof(state->normal));
            /* 通常 report の受理で、一時的な英数押下の所有も終わる。 */
            if (state->release_pending) {
                complete_request(state->generation);
                state->release_pending = false;
            }
        }
    }
    k_mutex_unlock(&report_lock);
    return err;
}

struct attribute_search {
    bool hid_service;
    const struct bt_gatt_attr *declaration;
    const struct bt_gatt_attr *report;
};

static uint8_t find_keyboard_report(const struct bt_gatt_attr *attr, uint16_t handle,
                                    void *user_data) {
    struct attribute_search *search = user_data;
    if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) ||
        !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY)) {
        search->hid_service = !bt_uuid_cmp(attr->user_data, BT_UUID_HIDS);
        search->report = NULL;
        search->declaration = NULL;
    } else if (search->hid_service && !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
        const struct bt_gatt_chrc *chrc = attr->user_data;
        search->declaration =
            !bt_uuid_cmp(chrc->uuid, BT_UUID_HIDS_REPORT) &&
                    (chrc->properties & BT_GATT_CHRC_NOTIFY)
                ? attr
                : NULL;
        search->report = NULL;
    } else if (search->declaration && !bt_uuid_cmp(attr->uuid, BT_UUID_HIDS_REPORT)) {
        search->report = attr;
    } else if (search->report && !bt_uuid_cmp(attr->uuid, BT_UUID_HIDS_REPORT_REF) &&
               attr->read) {
        uint8_t reference[2];
        if (attr->read(NULL, attr, reference, sizeof(reference), 0) == sizeof(reference) &&
            reference[0] == ZMK_HID_REPORT_ID_KEYBOARD && reference[1] == 1) {
            keyboard_attr = search->report;
            keyboard_decl = search->declaration;
            return BT_GATT_ITER_STOP;
        }
    }
    return BT_GATT_ITER_CONTINUE;
}

static bool add_english(struct zmk_hid_keyboard_report_body *report) {
    /* OS の修飾付きショートカットにせず、修飾キーの解放を待つ。 */
    if (report->modifiers) {
        return false;
    }
#if IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_NKRO)
    uint8_t mask = BIT(ENGLISH_USAGE % 8);
    if (report->keys[ENGLISH_USAGE / 8] & mask) {
        return false;
    }
    report->keys[ENGLISH_USAGE / 8] |= mask;
    return true;
#else
    int empty = -1;
    for (int i = 0; i < ARRAY_SIZE(report->keys); i++) {
        if (report->keys[i] == ENGLISH_USAGE) {
            return false;
        }
        if (!report->keys[i]) {
            empty = i;
        }
    }
    if (empty < 0) {
        return false;
    }
    report->keys[empty] = ENGLISH_USAGE;
    return true;
#endif
}

static bool ready(struct connection_state *state) {
    if (!connected(state->conn)) {
        return false;
    }
    if (bt_conn_get_security(state->conn) < BT_SECURITY_L2) {
        atomic_val_t failures = atomic_get(&security_failures[bt_conn_index(state->conn)]);
        if (failures != state->security_failures_seen) {
            state->security_failures_seen = failures;
            state->security_requested = false;
            state->security_failed = true;
        }

        k_spinlock_key_t key = k_spin_lock(&request_lock);
        if (!request.selecting &&
            request.profile == zmk_ble_profile_index(bt_conn_get_dst(state->conn)) &&
            request.generation != state->security_generation) {
            state->security_generation = request.generation;
            state->security_requested = false;
            state->security_attempted = false;
            state->security_failed = false;
        }
        k_spin_unlock(&request_lock, key);

        /* 認証は選択 1 回につき 1 回まで。失敗後は明示的な再選択で再試行する。 */
        if (!state->security_attempted && !state->security_failed) {
            state->security_attempted = true;
            int err = bt_conn_set_security(state->conn, BT_SECURITY_L2);
            state->security_requested = !err || err == -EALREADY;
            state->security_failed = !state->security_requested;
        }
        return false;
    }
    return bt_gatt_is_subscribed(state->conn, keyboard_attr, BT_GATT_CCC_NOTIFY);
}

static int notify_report(struct connection_state *state,
                         const struct zmk_hid_keyboard_report_body *report) {
    struct bt_gatt_notify_params params = {
        .attr = keyboard_attr,
        .data = report,
        .len = sizeof(*report),
    };
    return __real_bt_gatt_notify_cb(state->conn, &params);
}

static bool release_english(struct connection_state *state) {
    if (!state->release_pending) {
        return true;
    }
    if (!ready(state) || notify_report(state, &state->normal)) {
        return false;
    }
    state->release_pending = false;
    complete_request(state->generation);
    return true;
}

static void process_pending(struct k_work *work) {
    bool retry = false;
    k_mutex_lock(&report_lock, K_FOREVER);
    for (int i = 0; i < ARRAY_SIZE(connections); i++) {
        struct connection_state *state = &connections[i];
        if (!state->conn) {
            continue;
        }
        if (!connected(state->conn)) {
            bt_conn_unref(state->conn);
            memset(state, 0, sizeof(*state));
        } else if (!release_english(state)) {
            retry = true;
        }
    }

    k_spinlock_key_t key = k_spin_lock(&request_lock);
    uint32_t generation = request.generation;
    uint8_t profile = request.profile;
    bool pending = request.pending && !request.selecting;
    k_spin_unlock(&request_lock, key);

    if (pending && zmk_ble_active_profile_index() != profile) {
        complete_request(generation);
        pending = false;
    }
    struct zmk_endpoint_instance endpoint = zmk_endpoints_selected();
    if (pending && endpoint.transport == ZMK_TRANSPORT_BLE &&
        endpoint.ble.profile_index == profile) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn) {
            struct connection_state *state = state_for(conn);
            if (state && zmk_ble_profile_index(bt_conn_get_dst(conn)) == profile &&
                !state->release_pending && ready(state)) {
                struct zmk_hid_keyboard_report_body report = state->normal;
                /*
                 * この単一 CPU では最終確認から notify の入口まで選択処理を挟ませない。
                 * notify 内で buffer 待ちに入ると通常どおり他 thread が実行される。
                 * spinlock は notify より前に解放し、開始後の切替は旧 conn で後始末する。
                 */
                k_sched_lock();
                key = k_spin_lock(&request_lock);
                bool current = request.pending && !request.selecting &&
                               request.generation == generation;
                k_spin_unlock(&request_lock, key);
                endpoint = zmk_endpoints_selected();
                bool sent = current && zmk_ble_active_profile_index() == profile &&
                            endpoint.transport == ZMK_TRANSPORT_BLE &&
                            endpoint.ble.profile_index == profile && add_english(&report) &&
                            !notify_report(state, &report);
                if (sent) {
                    state->generation = generation;
                    state->release_pending = true;
                }
                k_sched_unlock();
                if (sent) {
                    release_english(state);
                }
            }
            bt_conn_unref(conn);
            /* CCC の変化に公開 callback がないため、接続中だけ低頻度で確認する。 */
            key = k_spin_lock(&request_lock);
            retry |= request.pending;
            k_spin_unlock(&request_lock, key);
        }
    }
    k_mutex_unlock(&report_lock);

    if (retry) {
        k_work_schedule_for_queue(&english_queue, &english_work, RETRY_DELAY);
    }
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    if (binding->param1 >= ZMK_BLE_PROFILE_COUNT) {
        return -ERANGE;
    }
    k_spinlock_key_t key = k_spin_lock(&request_lock);
    request.generation++;
    request.profile = binding->param1;
    request.pending = true;
    request.selecting = true;
    k_spin_unlock(&request_lock, key);
    int err = zmk_ble_prof_select(binding->param1);
    key = k_spin_lock(&request_lock);
    request.selecting = false;
    request.pending = !err;
    k_spin_unlock(&request_lock, key);
    wake_worker();
    return err;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static int endpoint_listener(const zmk_event_t *event) {
    wake_worker();
    return ZMK_EV_EVENT_BUBBLE;
}

static void connection_changed(struct bt_conn *conn, uint8_t reason) { wake_worker(); }

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    uint8_t index = bt_conn_index(conn);
    if (err && index < ARRAY_SIZE(security_failures)) {
        atomic_inc(&security_failures[index]);
    }
    wake_worker();
}

static struct bt_conn_cb connection_callbacks = {
    .connected = connection_changed,
    .disconnected = connection_changed,
    .security_changed = security_changed,
};

static int english_init(void) {
    struct attribute_search search = {0};
    bt_gatt_foreach_attr(1, UINT16_MAX, find_keyboard_report, &search);
    if (!keyboard_attr) {
        LOG_ERR("英数送信用 keyboard input report が見つかりません");
        return -ENOENT;
    }
    k_work_queue_start(&english_queue, english_stack, K_THREAD_STACK_SIZEOF(english_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, NULL);
    bt_conn_cb_register(&connection_callbacks);
    initialized = true;
    return 0;
}

SYS_INIT(english_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(bt_english, endpoint_listener);
ZMK_SUBSCRIPTION(bt_english, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(bt_english, zmk_endpoint_changed);

static const struct behavior_driver_api driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &driver_api);

#else

int __real_bt_gatt_notify_cb(struct bt_conn *conn, struct bt_gatt_notify_params *params);
int __wrap_bt_gatt_notify_cb(struct bt_conn *conn, struct bt_gatt_notify_params *params) {
    return __real_bt_gatt_notify_cb(conn, params);
}

#endif
