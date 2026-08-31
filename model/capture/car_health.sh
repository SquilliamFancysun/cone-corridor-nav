#!/usr/bin/env bash
# What happened to the car. Run it after an unexplained drop off the network.
#
#   ssh robocar 'bash -s' < car_health.sh          # from the laptop
#   ./car_health.sh                                 # on the car
#
# The question this answers is narrow and it is the one that matters: did the
# machine REBOOT, or did it stay up and lose the network? Those have nothing to
# do with each other, and every remedy for one is wasted on the other.
#
#   reboot + kernel log that just STOPS      -> power was cut. Brownout under
#                                               load, or a battery protection
#                                               circuit tripping. Look at
#                                               get_throttled below.
#   reboot + "Under-voltage detected!"       -> same, and now it is on record
#   reboot + oom-killer / watchdog           -> memory or a hang, not power
#   no reboot + brcmfmac/wlan disconnects    -> wifi only. The machine was fine
#                                               and ssh was not.
#
# Read `uptime` against the timestamps: an uptime shorter than the gap you
# observed means it came back on its own, which is a power event, not a hang.
set -u

section() { printf '\n=== %s\n' "$1"; }

section "identity and uptime"
  cat /proc/device-tree/model 2>/dev/null | tr -d '\0'; echo
  uname -sr
  uptime
  echo "booted at: $(who -b 2>/dev/null | awk '{print $3, $4}')"
  echo "now      : $(date)"

section "power: has this board ever browned out?"
  # Bit 0 = under-voltage NOW, bit 16 = under-voltage HAS OCCURRED since boot.
  # 0x0 is the only clean answer. This is the single most useful line here.
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd get_throttled
    echo "  0x0=clean  bit0=undervolt now  bit16=undervolt occurred"
    echo "  bit1/17=freq capped  bit2/18=throttled  bit3/19=soft temp limit"
    vcgencmd measure_volts 2>/dev/null
    vcgencmd measure_temp 2>/dev/null
  else
    echo "  vcgencmd not present"
  fi
  for f in /sys/class/hwmon/hwmon*/in0_lcrit_alarm; do
    [ -e "$f" ] && echo "  $f = $(cat "$f")  (1 = undervoltage alarm latched)"
  done
  echo "  soc temp: $(( $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0) / 1000 )) C"

section "reboot history: clean shutdown or power loss?"
  # A reboot with no matching 'shutdown' entry before it was not orderly.
  last -x --time-format=iso reboot shutdown 2>/dev/null | head -12 \
    || last -x reboot shutdown 2>/dev/null | head -12

section "boots journald knows about"
  journalctl --list-boots --no-pager 2>/dev/null | tail -8 \
    || echo "  journald has no persistent storage (see the note at the end)"

section "the end of the PREVIOUS boot -- what the kernel said before it died"
  # If this stops mid-sentence with nothing alarming, power was cut. That
  # absence IS the finding; there is no log entry for losing the supply.
  journalctl -b -1 -k --no-pager 2>/dev/null | tail -25 \
    || echo "  no previous boot retained"

section "errors from the previous boot"
  journalctl -b -1 -p err..alert --no-pager 2>/dev/null | tail -25 \
    || echo "  no previous boot retained"

section "undervoltage / OOM / watchdog / thermal, all boots"
  journalctl -k --no-pager 2>/dev/null \
    | grep -iE 'under-voltage|voltage normalis|throttl|oom-killer|Out of memory|killed process|watchdog|thermal|hwmon' \
    | tail -25 || echo "  none found"

section "wifi: did the link drop without the machine going down?"
  journalctl --no-pager 2>/dev/null \
    | grep -iE 'brcmfmac|wlan0|wpa_supplicant|dhcpcd|deauth|disconnect|link is not ready|association' \
    | tail -25 || echo "  none found"
  iwconfig wlan0 2>/dev/null | sed -n '1,6p'
  echo "  power save: $(iw wlan0 get power_save 2>/dev/null || echo unknown)"

section "memory now"
  free -h
  echo "  swap in use above is the thing to watch while torch is loaded"

section "USB: is the camera drawing through a hub?"
  lsusb 2>/dev/null | grep -iE 'movidius|luxonis|intel|myriad' || echo "  no OAK-D enumerated right now"

section "note"
  echo "  If 'boots journald knows about' shows only one boot, the journal is"
  echo "  volatile and the previous boot is gone. Make it persistent BEFORE the"
  echo "  next run or the next drop is just as blind:"
  echo "      sudo mkdir -p /var/log/journal && sudo systemd-tmpfiles --create --prefix /var/log/journal"
  echo "      sudo systemctl restart systemd-journald"
