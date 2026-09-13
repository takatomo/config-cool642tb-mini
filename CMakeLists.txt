if(CONFIG_ZMK_BLE AND ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
  target_sources(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/behavior_bt_english.c)
  zephyr_link_libraries(-Wl,--wrap=bt_gatt_notify_cb)
endif()
