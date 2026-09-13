#!/bin/sh
set -eu
test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/bt-english-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
for nkro in 0 1; do
    cc -std=c11 -pthread -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function \
        -Wno-unused-const-variable -Wno-sign-compare \
        -fsanitize=address,undefined -DCONFIG_ZMK_HID_REPORT_TYPE_NKRO="$nkro" \
        -I "$test_dir/include" "$test_dir/test_bt_english.c" -o "$build_dir/test-$nkro"
    "$build_dir/test-$nkro"
done
