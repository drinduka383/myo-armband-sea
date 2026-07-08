#!/usr/bin/env bash

run() {
  echo "$ $*"
  "$@" 2>&1 || true
  echo
}

run uname -a
run python3 --version
run bluetoothctl --version
run rfkill list
run systemctl status bluetooth --no-pager
run lsusb

echo '$ serial ports'
shopt -s nullglob
ports=(/dev/ttyACM* /dev/ttyUSB*)
((${#ports[@]})) && printf '%s\n' "${ports[@]}" || echo '(none visible)'
echo

run id
