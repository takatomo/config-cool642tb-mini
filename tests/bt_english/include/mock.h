#pragma once

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#define DT_HAS_COMPAT_STATUS_OKAY(x) 1
#define IS_ENABLED(x) (x)
#define CONFIG_BT_MAX_CONN 6
#define CONFIG_ZMK_BLE_THREAD_STACK_SIZE 1024
#define CONFIG_ZMK_BLE_THREAD_PRIORITY 5
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#define CONFIG_ZMK_LOG_LEVEL 0
#define CONFIG_SMP 0
#define CONFIG_NUM_METAIRQ_PRIORITIES 0
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define BIT(x) (1U << (x))
#define BUILD_ASSERT(c, m) _Static_assert(c, m)
#define ZMK_HID_USAGE_ID(x) ((x) & 0xffff)
#define LANG2 0x70091
#define ZMK_HID_KEYBOARD_MAX_USAGE 255
#define ZMK_HID_REPORT_ID_KEYBOARD 1
#define ZMK_BLE_PROFILE_COUNT 5
#define LOG_MODULE_DECLARE(...)
#define LOG_ERR(...) do {} while (0)
#define ZMK_BEHAVIOR_OPAQUE 0
#define ZMK_EV_EVENT_BUBBLE 0
#define K_FOREVER -1
#define K_NO_WAIT 0
#define K_MSEC(x) (x)

typedef atomic_int atomic_t;
typedef int atomic_val_t;
static atomic_val_t atomic_get(const atomic_t *value) { return atomic_load(value); }
static atomic_val_t atomic_inc(atomic_t *value) { return atomic_fetch_add(value, 1); }
static void atomic_set(atomic_t *value, atomic_val_t next) { atomic_store(value, next); }

struct zmk_hid_keyboard_report_body {
    uint8_t modifiers;
    uint8_t _reserved;
#if CONFIG_ZMK_HID_REPORT_TYPE_NKRO
    uint8_t keys[32];
#else
    uint8_t keys[6];
#endif
};

struct k_thread { int id; };
struct k_work { int unused; };
struct k_work_q { struct k_thread thread; };
struct k_work_delayable { struct k_work work; void (*handler)(struct k_work *); };
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
struct k_mutex { pthread_mutex_t native; int depth; };
static struct k_work_q k_sys_work_q;
static struct k_thread worker_thread;
static _Thread_local struct k_thread *mock_thread = &worker_thread;
static _Thread_local void (*before_lock)(void);
static _Thread_local int sched_depth, spin_depth;
static _Thread_local void (*before_sched_lock)(void);
static _Thread_local void (*after_spin_unlock)(void);
static _Thread_local void (*deferred_preemption)(void);
static _Thread_local void (*preempt_before_notify)(void);
static int scheduled_delay;
#define K_MUTEX_DEFINE(name) static struct k_mutex name = {.native = PTHREAD_MUTEX_INITIALIZER}
#define K_THREAD_STACK_DEFINE(name, size) static char name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)
#define K_WORK_DELAYABLE_DEFINE(name, fn) static struct k_work_delayable name = {.handler = fn}
static int k_mutex_lock(struct k_mutex *m, int timeout) {
    assert(mock_thread != &k_sys_work_q.thread);
    if (before_lock) { before_lock(); }
    assert(pthread_mutex_lock(&m->native) == 0);
    m->depth++;
    return 0;
}
static void k_mutex_unlock(struct k_mutex *m) {
    assert(m->depth > 0);
    m->depth--;
    assert(pthread_mutex_unlock(&m->native) == 0);
}
static void run_preemption(void (*fn)(void)) {
    struct k_thread *previous_thread = mock_thread;
    int previous_depth = sched_depth;
    mock_thread = &k_sys_work_q.thread;
    sched_depth = 0;
    fn();
    sched_depth = previous_depth;
    mock_thread = previous_thread;
}
static void preempt_or_defer(void (*fn)(void)) {
    if (sched_depth) {
        assert(deferred_preemption == NULL);
        deferred_preemption = fn;
    } else {
        run_preemption(fn);
    }
}
/* notify 内で thread が待機すると、scheduler lock 中でも他 thread が動く。 */
static void mock_thread_blocked(void) {
    if (deferred_preemption) {
        void (*fn)(void) = deferred_preemption;
        deferred_preemption = NULL;
        run_preemption(fn);
    }
}
static void k_sched_lock(void) {
    if (before_sched_lock) {
        void (*fn)(void) = before_sched_lock;
        before_sched_lock = NULL;
        preempt_or_defer(fn);
    }
    sched_depth++;
}
static void k_sched_unlock(void) {
    assert(sched_depth > 0);
    if (--sched_depth == 0) { mock_thread_blocked(); }
}
static k_spinlock_key_t k_spin_lock(struct k_spinlock *l) { spin_depth++; return 0; }
static void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key) {
    assert(spin_depth > 0);
    spin_depth--;
    if (after_spin_unlock) {
        void (*fn)(void) = after_spin_unlock;
        after_spin_unlock = NULL;
        preempt_or_defer(fn);
    }
}
static struct k_thread *k_current_get(void) { return mock_thread; }
static int k_work_reschedule_for_queue(struct k_work_q *q, struct k_work_delayable *w, int d) {
    scheduled_delay = d;
    return 0;
}
static int k_work_schedule_for_queue(struct k_work_q *q, struct k_work_delayable *w, int d) {
    if (scheduled_delay < 0) { scheduled_delay = d; }
    return 0;
}
static void k_work_queue_start(struct k_work_q *q, char *stack, size_t size, int pri, void *cfg) {}

typedef int bt_security_t;
enum bt_security_err { BT_SECURITY_ERR_SUCCESS, BT_SECURITY_ERR_AUTH_FAIL };
#define BT_SECURITY_L2 2
#define BT_CONN_STATE_CONNECTED 1
struct bt_conn { int index, profile, refs, state, security; bool subscribed; };
struct bt_conn_info { int state; };
struct bt_conn_cb {
    void (*connected)(struct bt_conn *, uint8_t);
    void (*disconnected)(struct bt_conn *, uint8_t);
    void (*security_changed)(struct bt_conn *, bt_security_t, enum bt_security_err);
};
static struct bt_conn peers[CONFIG_BT_MAX_CONN];
static int security_calls;
static int security_error;
static int active_profile;
static int selected_transport;
static int selected_profile;
static int returned_peer;
static struct bt_conn *bt_conn_ref(struct bt_conn *c) { c->refs++; return c; }
static void bt_conn_unref(struct bt_conn *c) { assert(c->refs > 0); c->refs--; }
static uint8_t bt_conn_index(const struct bt_conn *c) { return c->index; }
static int bt_conn_get_info(const struct bt_conn *c, struct bt_conn_info *i) {
    i->state = c->state;
    return 0;
}
static const int *bt_conn_get_dst(const struct bt_conn *c) { return &c->profile; }
static int bt_conn_get_security(const struct bt_conn *c) { return c->security; }
static int bt_conn_set_security(struct bt_conn *c, bt_security_t l) {
    security_calls++;
    return security_error;
}
static void bt_conn_cb_register(struct bt_conn_cb *cb) {}
static int zmk_ble_active_profile_index(void) { return active_profile; }
static struct bt_conn *zmk_ble_active_profile_conn(void) {
    struct bt_conn *c = &peers[returned_peer < 0 ? active_profile : returned_peer];
    return c->state ? bt_conn_ref(c) : NULL;
}
static int zmk_ble_profile_index(const int *p) { return *p; }
static int zmk_ble_prof_select(uint8_t p) {
    active_profile = p;
    selected_profile = p;
    return 0;
}
enum zmk_transport { ZMK_TRANSPORT_USB, ZMK_TRANSPORT_BLE };
struct zmk_endpoint_instance { int transport; struct { int profile_index; } ble; };
static struct zmk_endpoint_instance zmk_endpoints_selected(void) {
    return (struct zmk_endpoint_instance){selected_transport, {selected_profile}};
}

struct bt_uuid { int val; };
static const struct bt_uuid uuid_primary = {1}, uuid_secondary = {2}, uuid_chrc = {3},
                            uuid_hids = {4}, uuid_report = {5}, uuid_ref = {6}, uuid_other = {7};
#define BT_UUID_GATT_PRIMARY (&uuid_primary)
#define BT_UUID_GATT_SECONDARY (&uuid_secondary)
#define BT_UUID_GATT_CHRC (&uuid_chrc)
#define BT_UUID_HIDS (&uuid_hids)
#define BT_UUID_HIDS_REPORT (&uuid_report)
#define BT_UUID_HIDS_REPORT_REF (&uuid_ref)
#define BT_GATT_CHRC_NOTIFY 16
#define BT_GATT_CCC_NOTIFY 1
#define BT_GATT_ITER_STOP 0
#define BT_GATT_ITER_CONTINUE 1
struct bt_gatt_attr {
    const struct bt_uuid *uuid;
    void *user_data;
    ssize_t (*read)(struct bt_conn *, const struct bt_gatt_attr *, void *, uint16_t, uint16_t);
};
struct bt_gatt_chrc { const struct bt_uuid *uuid; int properties; };
struct bt_gatt_notify_params {
    const struct bt_uuid *uuid;
    const struct bt_gatt_attr *attr;
    const void *data;
    uint16_t len;
    void (*func)(struct bt_conn *, void *);
    void *user_data;
};
static int bt_uuid_cmp(const struct bt_uuid *a, const struct bt_uuid *b) { return a->val - b->val; }
static bool bt_gatt_is_subscribed(struct bt_conn *c, const struct bt_gatt_attr *a, uint16_t type) {
    if (preempt_before_notify) {
        after_spin_unlock = preempt_before_notify;
        preempt_before_notify = NULL;
    }
    return c->subscribed;
}
static void bt_gatt_foreach_attr(uint16_t start, uint16_t end,
                               uint8_t (*func)(const struct bt_gatt_attr *, uint16_t, void *),
                               void *data);

struct device { int unused; };
struct zmk_behavior_binding { uint32_t param1; };
struct zmk_behavior_binding_event { int unused; };
struct behavior_driver_api {
    int (*binding_pressed)(struct zmk_behavior_binding *, struct zmk_behavior_binding_event);
    int (*binding_released)(struct zmk_behavior_binding *, struct zmk_behavior_binding_event);
};
typedef int zmk_event_t;
#define ZMK_LISTENER(...)
#define ZMK_SUBSCRIPTION(...)
#define BEHAVIOR_DT_INST_DEFINE(...)
#define SYS_INIT(...)
