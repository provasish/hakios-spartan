#!/usr/bin/env bash
# HakiOS on the realme GT Neo 3T (spartan): the tester's script for Linux and macOS.
#
#   ./test-spartan.sh 1     stage 1: U-Boot alone
#   ./test-spartan.sh 2     stage 2: the loader and the open kernel's hello
#
# The phone in fastboot mode (power off, then power + volume down), on USB. Everything runs from RAM: nothing is
# flashed, and the phone goes back to Android at the end. If ABL refuses an image and changes the active slot (OPLUS ABL
# does that), the script puts the slot back. Logs go to logs/<time>-stage<N>/.
set -u
stage=${1:-}
case "$stage" in 1|2) ;; *) echo "usage: $0 1|2"; exit 1 ;; esac
here=$(cd "$(dirname "$0")" && pwd)
FASTBOOT=${FASTBOOT:-fastboot}
out="$here/logs/$(date +%H%M%S)-stage$stage"
mkdir -p "$out"

fb() { { echo "> fastboot $*"; "$FASTBOOT" "$@" 2>&1; } | tee -a "$out/fastboot.txt" | sed 1d; }
getvar() { fb getvar "$1" | sed -n "s/^$1:[[:space:]]*//p" | head -1 | tr -d '\r'; }
wait_for() { # wait_for <product> <seconds>
	local i
	for ((i = 0; i < $2; i += 2)); do
		case "$(getvar product)" in *"$1"*) return 0 ;; esac
		sleep 2
	done
	return 1
}
say() { echo "$*" | tee -a "$out/summary.txt"; }

fb devices | grep -q fastboot || { say "No phone in fastboot mode: power off, hold power + volume down, connect USB."; exit 1; }
product=$(getvar product)
[ "$product" = kona ] || { say "This phone says product '$product', not 'kona' (Snapdragon 870). Stopping."; exit 1; }
[ "$(getvar unlocked)" = yes ] || { say "The bootloader is locked: fastboot boot will not work. Stopping."; exit 1; }
slot=$(getvar current-slot)
say "ABL: product kona, unlocked, active slot '$slot'."

put_slot_back() {
	# OPLUS ABL marks the active slot unbootable and switches when it cannot start an image
	sleep 3
	[ "$(getvar product)" = kona ] || return
	local now bad
	now=$(getvar current-slot); bad=$(getvar "slot-unbootable:$slot")
	if [ "$now" != "$slot" ] || [ "$bad" = yes ]; then
		say "ABL refused the image and changed the slots (now '$now', '$slot' unbootable: $bad): setting '$slot' back."
		fb set_active "$slot" >/dev/null
		say "Slot now: $(getvar current-slot), unbootable:$(getvar "slot-unbootable:$slot")"
	fi
}

if [ "$stage" = 1 ]; then
	say "Stage 1: U-Boot alone."
	r=$(fb boot "$here/stage1-u-boot/hakios-u-boot.img")
	echo "$r" | grep -Eq 'Booting +OKAY' || { say "ABL did not start it: $r"; put_slot_back; exit 2; }
	say "ABL started it. Waiting for U-Boot on USB..."
	wait_for realme-spartan 40 || { say "U-Boot did not answer on USB. Look at the screen, take a photo, then hold power 15 s for Android."; exit 3; }
	say "U-Boot answers: $(getvar version-bootloader)"
	fb oem console > "$out/u-boot-console.txt"
	say "U-Boot's log: $out/u-boot-console.txt"
	say 'Now: is there text on the screen ending in "HakiOS: waiting for the host"? Take a photo.'
	read -r -p "Press Enter to send the phone back to Android "
	fb reboot >/dev/null
	say "Done: the phone goes back to Android. Send the whole folder $out"
	exit 0
fi

say "Stage 2: U-Boot, the HakiOS loader, the open kernel and hello."
r=$(fb boot "$here/stage2-hello/xnu-usb.img")
echo "$r" | grep -Eq 'Booting +OKAY' || { say "ABL did not start it: $r"; put_slot_back; exit 2; }
say "Waiting for the HakiOS loader on USB..."
wait_for xnuloader 60 || { say "The loader did not answer on USB. Take a photo of the screen, then hold power 15 s for Android."; exit 3; }
fb oem log >/dev/null
fb get_staged "$out/loader-before.txt" >/dev/null
say "Loader up: version $(getvar version). Its log: loader-before.txt (look for \"platform spartan\")."

fb stage "$here/stage2-hello/run-payload.bin" | grep -q OKAY || { say "Sending the kernel failed."; fb reboot >/dev/null; exit 4; }
fb oem run >/dev/null
say "The kernel is running (it hands back after about 8 s)..."
result=
for ((i = 0; i < 120; i += 2)); do
	sleep 2
	result=$(getvar result)
	case "$result" in ""|*running*) ;; *) break ;; esac
done
case "$result" in ""|*running*)
	say "No answer from the loader: the kernel hung. Photo of the screen, then hold power 15 s for Android."
	exit 5 ;;
esac
say "Result: $result"
fb oem log >/dev/null
fb get_staged "$out/run-log.txt" >/dev/null
screen=$(getvar screen)
fb oem screenshot >/dev/null
fb get_staged "$out/screen.raw" >/dev/null
echo "width height bytes-per-row: $screen (raw 32-bit pixels)" > "$out/screen.txt"
if tr -d '\0' < "$out/run-log.txt" 2>/dev/null | grep -q 'Hello from user space on Darwin'; then
	say 'SUCCESS: "Hello from user space on Darwin!" is in the log.'
else
	say "No hello in the log: send the folder."
fi
fb reboot >/dev/null
say "Done: the phone goes back to Android. Send the whole folder $out"
