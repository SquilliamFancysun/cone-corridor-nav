# Field network runbook (v2, "lorelei")

_How the laptop reaches the v2 car in the field, and what to do when it can't.
v1 ran on a phone hotspot, which moved the car's IP between sessions and stalled
every SSH connect for 5 s on an IPv6/mDNS lookup. v2 has its own router, and the
car's address is fixed._

| Piece | What it is | Address |
|---|---|---|
| Router | GL.iNet GL-MT3000 (Beryl AX), firmware 4.8.1, USB-C 5 V / 3 A | `192.168.148.1` (admin UI) |
| Wi-Fi | SSID `lorelei`, WPA2, both bands; 5 GHz pinned to ch 149 | DHCP pool `.100–.249` |
| Car | Raspberry Pi 5, hostname `lorelei`, Pi OS Lite 64-bit (trixie) | `192.168.148.221` (reserved) |
| Rescue cable | Pi `eth0`, direct to the laptop | link-local IPv6, or `10.0.148.2` |
| Internet | iPhone **USB-tethered** into the router's USB-A port | optional |

The Wi-Fi password is not in this repo.

---

## 1. Bring-up

1. Power the router from the power bank. It takes about 40 s to boot.
2. Power the car. The Pi joins `lorelei` by itself, about 20 s after boot.
3. Join `lorelei` on the laptop, then `ssh lorelei`. The alias `ssh robocar-v2`
   reaches the same machine over the same connection.

The router never needs internet for driving. The internet uplink is only for
`pip`/`apt`/`git` on the car.

## 2. Internet in the field

Plug the iPhone into the router's USB-A port, turn on **Settings → Personal
Hotspot → Allow Others to Join** (iOS won't tether over USB without it), and
tap **Trust** if the phone asks. The router picks the uplink up on its own.

**Do not use the router's wireless Repeater mode to reach the hotspot.** When
the hotspot appears or disappears, the router restarts *both* of its radios,
and every client, including the car, drops off Wi-Fi for as long as that
takes. With USB tethering, plugging or unplugging the phone costs about 14 s of
internet and nothing else.

## 3. When the car is unreachable

| Symptom | Check |
|---|---|
| `ssh lorelei` hangs | Is the laptop still on `lorelei`? macOS jumps to other known networks when the router blips, and does not come back on its own. |
| Laptop is on `lorelei`, the car isn't answering | Router power-cycled? The Pi takes ~20 s to notice and ~5 s to rejoin once the router is back (about 1 min total). |
| Still nothing | Use the rescue cable (below). |

**Rescue cable.** Plug an ethernet cable from the laptop's USB adapter
straight into the Pi, then run `ssh lorelei-eth`. It needs no router, no Wi-Fi
and no setup on the laptop. It rides IPv6 link-local, derived from the Pi's
`eth0` MAC. The `%en5` in the ssh config is the macOS name of the USB adapter;
if yours differs, check with `ifconfig | grep -B4 169.254`. For IPv4 instead,
give the laptop `10.0.148.1/24` and use `10.0.148.2`.

## 4. How it is configured

**Router** (factory reset, then):
- LAN `192.168.148.1/24`, IPv6 off, AdGuard / GoodCloud / guest Wi-Fi off, AP
  isolation **off** (it would block laptop ↔ car traffic).
- 2.4 GHz on a fixed channel at 20 MHz, for range.
- 5 GHz on channel 149 at 80 MHz. Auto can land on radar-shared (DFS)
  channels, which add a ~60 s wait at every router boot. Dynamic Bandwidth is
  off.
- A DHCP reservation for the Pi's `wlan0` MAC `88:a2:9e:7b:05:99`.

**Pi** (NetworkManager; profiles live as netplan YAML in `/etc/netplan/90-NM-*.yaml`):
- `lorelei` Wi-Fi profile: `autoconnect-priority 100`, `autoconnect-retries 0`
  (retry forever), power save off, IPv6 off.
- No other Wi-Fi networks are saved. That is deliberate: NetworkManager does not
  move back to `lorelei` while it is connected elsewhere, so a phone hotspot saved
  as a fallback would strand the car off the network.
- `eth0`: static `10.0.148.2/24` with no gateway, plus link-local IPv6. The
  static address stops NetworkManager cycling the port while it hunts for DHCP
  on a direct cable. That cycling is what made the old rescue path flap.
- The `pi` user has passwordless sudo (`/etc/sudoers.d/010_pi-nopasswd`); login
  is SSH-key only.

**Laptop** (macOS):
- Wi-Fi sits above the USB ethernet adapter in the service order. Otherwise, a
  cable into a router with no internet takes over the laptop's default route.
- Auto-Join is off for the iPhone hotspot, so the laptop stays on `lorelei`.
