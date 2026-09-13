/* 本体を mock API と一緒にコンパイルし、送信先と report の変化を検証する。 */
#include "mock.h"
#include "../../src/behavior_bt_english.c"

static uint8_t keyboard_reference[] = {1, 1}, consumer_reference[] = {2, 1};
static struct bt_gatt_chrc report_chrc = {BT_UUID_HIDS_REPORT, BT_GATT_CHRC_NOTIFY};
static ssize_t read_reference(struct bt_conn *c, const struct bt_gatt_attr *a, void *buf,
                              uint16_t len, uint16_t offset) {
    memcpy(buf, a->user_data, 2);
    return 2;
}
static struct bt_gatt_attr attributes[] = {
    {BT_UUID_GATT_PRIMARY, (void *)&uuid_other, NULL},
    {BT_UUID_GATT_CHRC, &report_chrc, NULL},
    {BT_UUID_HIDS_REPORT, NULL, NULL},
    {BT_UUID_HIDS_REPORT_REF, keyboard_reference, read_reference},
    {BT_UUID_GATT_PRIMARY, (void *)BT_UUID_HIDS, NULL},
    {BT_UUID_GATT_CHRC, &report_chrc, NULL},
    {BT_UUID_HIDS_REPORT, NULL, NULL},
    {BT_UUID_HIDS_REPORT_REF, consumer_reference, read_reference},
    {BT_UUID_GATT_CHRC, &report_chrc, NULL},
    {BT_UUID_HIDS_REPORT, NULL, NULL},
    {&uuid_other, NULL, NULL},
    {BT_UUID_HIDS_REPORT_REF, keyboard_reference, read_reference},
};
static void bt_gatt_foreach_attr(uint16_t start, uint16_t end,
                               uint8_t (*fn)(const struct bt_gatt_attr *, uint16_t, void *),
                               void *data) {
    for (int i = 0; i < ARRAY_SIZE(attributes); i++) {
        if (fn(&attributes[i], i + 1, data) == BT_GATT_ITER_STOP) { break; }
    }
}

struct notification { struct bt_conn *conn; struct zmk_hid_keyboard_report_body body; };
static struct notification notifications[64];
static int notification_count, attempts, fail_attempt;
static struct bt_gatt_notify_params last_params;
static void (*notify_hook)(void);
int __real_bt_gatt_notify_cb(struct bt_conn *c, struct bt_gatt_notify_params *p) {
    assert(spin_depth == 0);
    last_params = *p;
    attempts++;
    mock_thread_blocked();
    if (attempts == fail_attempt) { return -ENOMEM; }
    if (notify_hook) { void (*hook)(void) = notify_hook; notify_hook = NULL; hook(); }
    assert(notification_count < ARRAY_SIZE(notifications));
    notifications[notification_count].conn = c;
    memcpy(&notifications[notification_count++].body, p->data, p->len);
    return 0;
}

static void reset(void) {
    memset(connections, 0, sizeof(connections));
    memset(&request, 0, sizeof(request));
    memset(peers, 0, sizeof(peers));
    for (int i = 0; i < ARRAY_SIZE(peers); i++) {
        atomic_set(&security_failures[i], 0);
        peers[i] = (struct bt_conn){.index = i, .profile = i, .refs = 1, .state = 1,
                                    .security = 2, .subscribed = true};
    }
    active_profile = selected_profile = 0;
    selected_transport = ZMK_TRANSPORT_BLE;
    returned_peer = -1;
    security_calls = attempts = notification_count = fail_attempt = 0;
    security_error = 0;
    notify_hook = NULL;
    scheduled_delay = -1;
    mock_thread = &worker_thread;
    sched_depth = spin_depth = 0;
    before_sched_lock = after_spin_unlock = deferred_preemption = preempt_before_notify = NULL;
    report_lock.depth = 0;
    assert(english_init() == 0);
    assert(keyboard_attr == &attributes[9]);
    assert(keyboard_decl == &attributes[8]);
}
static void select_profile(int p) {
    struct zmk_behavior_binding binding = {.param1 = p};
    assert(on_pressed(&binding, (struct zmk_behavior_binding_event){0}) == 0);
}
static void step(void) {
    scheduled_delay = -1;
    process_pending(&english_work.work);
    assert(sched_depth == 0 && spin_depth == 0);
}
static bool has_key(const struct zmk_hid_keyboard_report_body *body, uint8_t usage) {
#if CONFIG_ZMK_HID_REPORT_TYPE_NKRO
    return body->keys[usage / 8] & BIT(usage % 8);
#else
    for (int i = 0; i < ARRAY_SIZE(body->keys); i++) {
        if (body->keys[i] == usage) { return true; }
    }
    return false;
#endif
}
static void set_key(struct zmk_hid_keyboard_report_body *body, uint8_t usage) {
#if CONFIG_ZMK_HID_REPORT_TYPE_NKRO
    body->keys[usage / 8] |= BIT(usage % 8);
#else
    for (int i = 0; i < ARRAY_SIZE(body->keys); i++) {
        if (!body->keys[i]) { body->keys[i] = usage; return; }
    }
#endif
}
static void normal_report(int p, struct zmk_hid_keyboard_report_body *body) {
    struct bt_gatt_notify_params params = {.attr = keyboard_attr, .data = body, .len = sizeof(*body)};
    assert(__wrap_bt_gatt_notify_cb(&peers[p], &params) == 0);
}

static void test_wait_and_once(void) {
    reset();
    peers[1].state = 0;
    select_profile(1);
    step();
    assert(notification_count == 0 && scheduled_delay == -1 && request.pending);
    peers[1].state = 1;
    peers[1].security = 1;
    connection_changed(&peers[1], 0);
    step();
    assert(notification_count == 0 && scheduled_delay == 100 && security_calls == 1);
    step();
    assert(security_calls == 1);
    peers[1].security = 2;
    peers[1].subscribed = false;
    step();
    assert(notification_count == 0 && scheduled_delay == 100);
    peers[1].subscribed = true;
    step();
    assert(notification_count == 2 && !request.pending);
    assert(notifications[0].conn == &peers[1]);
    assert(has_key(&notifications[0].body, ENGLISH_USAGE));
    assert(!has_key(&notifications[1].body, ENGLISH_USAGE));
    step();
    assert(notification_count == 2 && scheduled_delay == -1);

    reset();
    peers[0].security = 1;
    security_error = -ENOMEM;
    select_profile(0);
    step();
    assert(security_calls == 1 && !connections[0].security_requested);
    security_error = 0;
    step();
    assert(security_calls == 1 && !connections[0].security_requested);
    select_profile(0);
    step();
    assert(security_calls == 2 && connections[0].security_requested);
}

static void test_security_failure_reselection(void) {
    reset();
    peers[0].security = 1;
    select_profile(0);
    step();
    assert(security_calls == 1 && connections[0].security_requested);
    security_changed(&peers[0], 1, BT_SECURITY_ERR_AUTH_FAIL);
    step();
    assert(!connections[0].security_requested && connections[0].security_failed);
    for (int i = 0; i < 5; i++) { step(); }
    assert(security_calls == 1 && notification_count == 0 && request.pending);
    select_profile(0);
    step();
    assert(security_calls == 2 && connections[0].security_requested);
    assert(!connections[0].security_failed);
    peers[0].security = 2;
    security_changed(&peers[0], 2, BT_SECURITY_ERR_SUCCESS);
    step();
    assert(notification_count == 2 && !request.pending);
}

static int attempts_when_selected;
static void select_at_send_boundary(void) {
    attempts_when_selected = attempts;
    select_profile(3);
}

static void test_selection_before_notify_entry(void) {
    reset();
    select_profile(0);
    attempts_when_selected = -1;
    before_sched_lock = select_at_send_boundary;
    step();
    assert(attempts_when_selected == 0 && notification_count == 0 && request.pending);
    step();
    assert(notification_count == 2 && !request.pending);
    assert(notifications[0].conn == &peers[3] && notifications[1].conn == &peers[3]);
}

static void test_selection_after_final_check(void) {
    reset();
    select_profile(0);
    attempts_when_selected = -1;
    /* 旧実装で競合した最終 current 判定の spin_unlock 直後に選択を要求する。 */
    preempt_before_notify = select_at_send_boundary;
    step();
    /* 選択の実行は notify の入口を過ぎた buffer 待ちまで繰り越される。 */
    assert(attempts_when_selected == 1 && notification_count == 2 && request.pending);
    assert(notifications[0].conn == &peers[0] && notifications[1].conn == &peers[0]);
    assert(!has_key(&notifications[1].body, ENGLISH_USAGE));
    step();
    assert(notification_count == 4 && !request.pending);
    assert(notifications[2].conn == &peers[3] && notifications[3].conn == &peers[3]);
}

static void test_target_and_last_wins(void) {
    reset();
    select_profile(0);
    select_profile(4);
    selected_transport = ZMK_TRANSPORT_USB;
    step();
    assert(notification_count == 0 && request.pending);
    selected_transport = ZMK_TRANSPORT_BLE;
    selected_profile = 3;
    step();
    assert(notification_count == 0);
    selected_profile = 4;
    returned_peer = 3;
    step();
    assert(notification_count == 0);
    returned_peer = -1;
    step();
    assert(notification_count == 2 && notifications[0].conn == &peers[4]);
    select_profile(2);
    active_profile = 1;
    step();
    assert(notification_count == 2 && !request.pending);
}

static void test_preserve_normal_input(void) {
    reset();
    struct zmk_hid_keyboard_report_body normal = {0};
    set_key(&normal, 4);
    set_key(&normal, 5);
    normal_report(0, &normal);
    select_profile(0);
    step();
    assert(notification_count == 3);
    assert(notifications[1].body.modifiers == normal.modifiers);
    assert(has_key(&notifications[1].body, 4) && has_key(&notifications[1].body, 5));
    assert(memcmp(&notifications[2].body, &normal, sizeof(normal)) == 0);
    assert(memcmp(&connections[0].normal, &normal, sizeof(normal)) == 0);

    set_key(&normal, ENGLISH_USAGE);
    normal_report(0, &normal);
    select_profile(0);
    step();
    assert(notification_count == 4 && request.pending);
    memset(&normal, 0, sizeof(normal));
    normal_report(0, &normal);
    step();
    assert(notification_count == 7 && !request.pending);
}

static void test_wait_for_modifiers(void) {
    reset();
    struct zmk_hid_keyboard_report_body normal = {.modifiers = 0x23};
    set_key(&normal, 4);
    normal_report(0, &normal);
    select_profile(0);
    step();
    assert(notification_count == 1 && request.pending && scheduled_delay == 100);
    assert(connections[0].normal.modifiers == 0x23);
    normal.modifiers = 0;
    normal_report(0, &normal);
    step();
    assert(notification_count == 4 && !request.pending);
    assert(notifications[2].body.modifiers == 0);
    assert(has_key(&notifications[2].body, 4));
    assert(memcmp(&notifications[3].body, &normal, sizeof(normal)) == 0);
}

static pthread_mutex_t arrival_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t arrival_condition = PTHREAD_COND_INITIALIZER;
static pthread_t normal_thread;
static bool normal_arrived;
static struct zmk_hid_keyboard_report_body arriving_report;

static void announce_normal_arrival(void) {
    assert(pthread_mutex_lock(&arrival_lock) == 0);
    normal_arrived = true;
    assert(pthread_cond_signal(&arrival_condition) == 0);
    assert(pthread_mutex_unlock(&arrival_lock) == 0);
}

static void *send_arriving_report(void *arg) {
    before_lock = announce_normal_arrival;
    normal_report(0, &arriving_report);
    return NULL;
}

static void arrive_during_press(void) {
    assert(pthread_mutex_lock(&arrival_lock) == 0);
    assert(pthread_create(&normal_thread, NULL, send_arriving_report, NULL) == 0);
    while (!normal_arrived) {
        assert(pthread_cond_wait(&arrival_condition, &arrival_lock) == 0);
    }
    assert(pthread_mutex_unlock(&arrival_lock) == 0);
}

static void test_concurrent_normal_report(void) {
    reset();
    memset(&arriving_report, 0, sizeof(arriving_report));
    arriving_report.modifiers = 2;
    set_key(&arriving_report, 5);
    normal_arrived = false;
    select_profile(0);
    notify_hook = arrive_during_press;
    step();
    assert(pthread_join(normal_thread, NULL) == 0);
    assert(notification_count == 3);
    assert(has_key(&notifications[0].body, ENGLISH_USAGE));
    assert(!has_key(&notifications[1].body, ENGLISH_USAGE));
    assert(memcmp(&notifications[2].body, &arriving_report, sizeof(arriving_report)) == 0);
    assert(memcmp(&connections[0].normal, &arriving_report, sizeof(arriving_report)) == 0);
}

static void test_retry_and_cleanup(void) {
    reset();
    select_profile(1);
    fail_attempt = 1;
    step();
    assert(notification_count == 0 && request.pending);
    fail_attempt = 3;
    step();
    assert(notification_count == 1 && connections[1].release_pending);
    assert(has_key(&notifications[0].body, ENGLISH_USAGE));
    select_profile(2);
    fail_attempt = 0;
    step();
    assert(notification_count == 4 && !request.pending);
    assert(notifications[1].conn == &peers[1]);
    assert(!has_key(&notifications[1].body, ENGLISH_USAGE));
    assert(notifications[2].conn == &peers[2] && notifications[3].conn == &peers[2]);

    reset();
    select_profile(0);
    fail_attempt = 2;
    step();
    struct zmk_hid_keyboard_report_body normal = {.modifiers = 8};
    set_key(&normal, 6);
    normal_report(0, &normal);
    assert(!connections[0].release_pending && !request.pending);
    step();
    assert(notification_count == 2);
    assert(memcmp(&notifications[1].body, &normal, sizeof(normal)) == 0);

    reset();
    select_profile(0);
    fail_attempt = 2;
    step();
    peers[0].subscribed = false;
    select_profile(1);
    step();
    assert(notification_count == 3 && connections[0].release_pending && !request.pending);
    assert(notifications[1].conn == &peers[1]);
    peers[0].subscribed = true;
    step();
    assert(notification_count == 4 && notifications[3].conn == &peers[0]);
    assert(!connections[0].release_pending);
}

static void select_during_press(void) { select_profile(3); }
static void test_change_during_press_and_disconnect(void) {
    reset();
    select_profile(0);
    notify_hook = select_during_press;
    step();
    assert(notification_count == 2 && request.pending);
    assert(notifications[0].conn == &peers[0] && notifications[1].conn == &peers[0]);
    step();
    assert(notification_count == 4 && notifications[2].conn == &peers[3] && !request.pending);

    reset();
    struct zmk_hid_keyboard_report_body old_normal = {0};
    set_key(&old_normal, 4);
    normal_report(1, &old_normal);
    select_profile(1);
    fail_attempt = 3;
    step();
    peers[1].state = 0;
    step();
    assert(connections[1].conn == NULL && peers[1].refs == 1 && request.pending);
    peers[1].state = 1;
    fail_attempt = 0;
    step();
    assert(notification_count == 4 && !request.pending);
    assert(!has_key(&notifications[2].body, 4));
    assert(connections[1].normal.modifiers == 0);
}

static void dummy_callback(struct bt_conn *c, void *data) {}
static void test_passthrough_and_validation(void) {
    reset();
    struct zmk_hid_keyboard_report_body body = {.modifiers = 7};
    struct bt_gatt_notify_params params = {
        .attr = &attributes[6], .data = &body, .len = sizeof(body),
        .func = dummy_callback, .user_data = &body,
    };
    assert(__wrap_bt_gatt_notify_cb(&peers[0], &params) == 0);
    assert(memcmp(&last_params, &params, sizeof(params)) == 0);
    assert(connections[0].conn == NULL);
    params.attr = keyboard_attr;
    mock_thread = &k_sys_work_q.thread;
    assert(__wrap_bt_gatt_notify_cb(&peers[0], &params) == 0);
    assert(last_params.func == dummy_callback && last_params.user_data == &body);
    fail_attempt = attempts + 1;
    assert(__wrap_bt_gatt_notify_cb(&peers[0], &params) == -ENOMEM);
    mock_thread = &worker_thread;
    struct zmk_behavior_binding invalid = {.param1 = 256};
    assert(on_pressed(&invalid, (struct zmk_behavior_binding_event){0}) == -ERANGE);
    assert(!request.pending);
    assert(on_released(&invalid, (struct zmk_behavior_binding_event){0}) == 0);
    assert(!request.pending);
}

static void test_rollover(void) {
#if !CONFIG_ZMK_HID_REPORT_TYPE_NKRO
    reset();
    struct zmk_hid_keyboard_report_body body = {.keys = {4, 5, 6, 7, 8, 9}};
    normal_report(0, &body);
    select_profile(0);
    step();
    assert(notification_count == 1 && request.pending);
    body.keys[5] = 0;
    normal_report(0, &body);
    step();
    assert(notification_count == 4 && !request.pending);
#endif
}

int main(void) {
    test_wait_and_once();
    test_security_failure_reselection();
    test_selection_before_notify_entry();
    test_selection_after_final_check();
    test_target_and_last_wins();
    test_preserve_normal_input();
    test_wait_for_modifiers();
    test_concurrent_normal_report();
    test_retry_and_cleanup();
    test_change_during_press_and_disconnect();
    test_passthrough_and_validation();
    test_rollover();
    puts("Bluetooth 英数: 12 組のテストが合格しました");
}
